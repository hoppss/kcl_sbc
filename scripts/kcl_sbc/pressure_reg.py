#!/usr/bin/env python

import roslib
roslib.load_manifest('kcl_sbc')
import rospy
import sys
from std_msgs.msg import Float64 
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
import numpy as np
from pycomedi.device import Device
from pycomedi.channel import AnalogChannel
from pycomedi.constant import SUBDEVICE_TYPE, AREF, UNIT
from kcl_sbc.cfg import P_RegConfig as ConfigType
from kcl_sbc.msg import PressureControl


class ComediIO():
    def __init__(self):
        dev=rospy.get_param('~comedi_dev','/dev/comedi0')
        chan_out=rospy.get_param('~comedi_out_chan',0)
        chan_in=rospy.get_param('~comedi_in_chan',0)
        self.device=Device(dev)
        self.device.open()
        subdev_out = self.device.find_subdevice_by_type(SUBDEVICE_TYPE.ao)
        subdev_in = self.device.find_subdevice_by_type(SUBDEVICE_TYPE.ai)
        self.channel_out=subdev_out.channel(chan_out, factory=AnalogChannel, aref=AREF.ground)
        self.channel_in=subdev_in.channel(chan_in, factory=AnalogChannel, aref=AREF.ground)
        self.channel_out.range=self.channel_out.find_range(unit=UNIT.volt,min=0,max=10)
        self.channel_in.range=self.channel_in.find_range(unit=UNIT.volt,min=1,max=5)
        self.max=self.channel_in.get_maxdata()
        self.conv_out=self.channel_out.get_converter()
        self.conv_in=self.channel_in.get_converter()
        
    def send(self,v_out):
        self.channel_out.data_write(self.conv_out.from_physical(v_out))               
    def read(self):
        #print(self.channel_in.data_read_delayed(nano_sec=10e3))
        return self.conv_in.to_physical(self.channel_in.data_read_delayed(nano_sec=10e3))    
        

class Pressure_reg():
    def __init__(self):
        self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)
        self.pub = rospy.Publisher("pressure",PressureControl )
        self.com_io=ComediIO();
     

    # Create a callback function for the dynamic reconfigure server.
    def reconfigure(self, config, level):
        self.freq = config["frequency"]
        self.mean = config["mean"]
        self.std = config["std_dev"]
        return config
    
    def callback_read(self,event):
        self.msg.v_in=self.com_io.read()
        self.msg.header.stamp=rospy.Time.now()
        self.msg.p_measure=(self.msg.v_in-1.0)/2.52*0.5        
        self.pub.publish(self.msg)
    
    def run(self):
        self.msg = PressureControl()
        rospy.Timer(rospy.Duration(0.01),self.callback_read,oneshot=False)
        
        while not rospy.is_shutdown():
            rate=rospy.Rate(self.freq)        
            self.msg.header.stamp=rospy.Time.now()
            v_demand= self.std*np.random.randn()+self.mean
            v_demand=np.min(np.array([v_demand,10]))
            v_demand=np.max(np.array([v_demand,0]))            
                                 
            self.msg.v_out=v_demand
            self.msg.p_demand=v_demand/10.0 * 0.5
            self.com_io.send(v_demand)            
            self.pub.publish(self.msg)
            rate.sleep()           
            
            
            
# Main function.
if __name__ == '__main__':
    rospy.init_node('pressure_reg')
    try:
        ne = Pressure_reg()
        ne.run()
    except rospy.ROSInterruptException: pass
