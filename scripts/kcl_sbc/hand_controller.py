#!/usr/bin/env python

import roslib
roslib.load_manifest('kcl_sbc')
import rospy 
from pyHand_api import *
import numpy as np
from std_msgs.msg import Float64


class BHand_controller():   
    
    def __init__(self):
        rospy.init_node('bhand_controller', anonymous = False)
        print("init")
        self.hand = pyHand('/dev/pcan1')
        ret=self.hand.initialize()    
        if(ret != True):
            print("merda")
        
     
        ret=self.hand.set_mode(HAND_GROUP, 'VEL')
        rospy.sleep(2.0)
        ret=self.hand.init_hand();
        if(ret != True):
            print("merda2")  
        rospy.Subscriber("hand_command", Float64, self.callback)

        print("GO!")
        rospy.spin();        
        
            
            
            
    def callback(self,data):
        print(data.data)
        self.hand.set_velocity(FINGER1, self.hand.rad_to_enc(0.01*data.data))
        
    def main(self):
        print("joao")
        

if __name__ == "__main__":
    bhc=BHand_controller();
    bhc.main()