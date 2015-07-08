#!/usr/bin/env python

import roslib
roslib.load_manifest('kcl_sbc')
import rospy 
import numpy as np
from std_msgs.msg import Float64
import tf
from tf import TransformListener
from sensor_msgs.msg import JointState
from kcl_sbc.msg import SBC_Output, PressureControl
from sr_grasp_msgs.msg import KCL_ContactStateStamped
from geometry_msgs.msg import PointStamped, Point




class Collector():
    def __init__(self):
        rospy.init_node('collector', anonymous = False)                
        self.lis=TransformListener();
        self.data_out=SBC_Output();
        rospy.Subscriber("/joint_states", JointState, self.j_callback)
        rospy.Subscriber("/finger1/ContactState", KCL_ContactStateStamped, self.f_callback1)
        rospy.Subscriber("/finger2/ContactState", KCL_ContactStateStamped, self.f_callback2)
        rospy.Subscriber("/pressure", PressureControl, self.p_callback)        
        rospy.Subscriber("/prob_fail", Float64, self.prob_callback)
        self.publisher=rospy.Publisher("sbc_data", SBC_Output)
        self.point1=PointStamped()
        self.point2=PointStamped()
        self.rate=rospy.Rate(20);

    def getParams(self):     
        self.data_out.D_Gain=rospy.get_param("/bhand_pid/d_gain")     
        self.data_out.F_ref_pid=rospy.get_param("/bhand_pid/f_ref")
        self.data_out.I_Gain=rospy.get_param("/bhand_pid/i_gain")
        self.data_out.P_Gain=rospy.get_param("/bhand_pid/p_gain")
        self.data_out.freq=rospy.get_param("/pressure_reg/frequency")
        self.data_out.Beta=rospy.get_param("/bhand_sbc/beta")
        self.data_out.Delta=rospy.get_param("/bhand_sbc/delta")
        self.data_out.Eta=rospy.get_param("/bhand_sbc/eta")
        self.data_out.F_ref_sbc=rospy.get_param("/bhand_sbc/f_ref")
        
    def j_callback(self,data):
        self.joints=data;
        self.data_out.effort1=data.effort[1]
        self.data_out.effort2=data.effort[0]
        
    def f_callback1(self,data):                    
        self.data_out.Fn1=data.Fnormal;            
        ft=np.array([data.tangential_force.x,data.tangential_force.y,data.tangential_force.z])
        self.data_out.Ft1=np.sqrt(ft.dot(ft));

        self.point1=PointStamped();
        self.point1.header=data.header;
        self.point1.point=data.contact_position;
                    
        
    def f_callback2(self,data):
        self.data_out.Fn2=data.Fnormal;
        ft=np.array([data.tangential_force.x,data.tangential_force.y,data.tangential_force.z])
        self.data_out.Ft2=np.sqrt(ft.dot(ft));  
        
        self.point2=PointStamped();
        self.point2.header=data.header;
        self.point2.point=data.contact_position;

        
    def p_callback(self,data):
        self.data_out.p_demand=data.p_demand;
        self.data_out.p_measure=data.p_measure;
    def prob_callback(self,data):
        self.data_out.Pfailure=data.data;
        
    def transform_it(self,data):
        if(data.header.frame_id):
            #data.header.stamp=rospy.Time.now();
            if(self.lis.canTransform("base_link",data.header.frame_id,data.header.stamp) or True):
                #print(rospy.Time.now())     
                data.header.stamp=data.header.stamp-rospy.Duration(0.02)           
                #point=self.lis.transformPoint("base_link", data)
                try:    
                    #self.lis.waitForTransform("base_link", data.header.frame_id, data.header.stamp, rospy.Duration(1))
                  #  print(rospy.Time.now())                
                    self.point=self.lis.transformPoint("base_link", data)
                    return True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logwarn("TF problem 2")                    
                    pass
            else:
                rospy.logwarn("Cannot Transform")
        else:
            print(data.header.frame_id)        
        return False
    
    def get_distance(self,point1,point2):  
        d=np.array([point1.x-point2.x, point1.y-point2.y, point1.z-point2.z])
        return np.sqrt(d.dot(d));
  
        
        
    def send_it(self):
        while not rospy.is_shutdown():            
            self.data_out.header.stamp=rospy.Time.now();
            self.getParams()            
            
            got_it=self.transform_it(self.point1);
            if(got_it):
                self.data_out.contact1=self.point.point
            got_it=self.transform_it(self.point2);
            if(got_it):
                self.data_out.contact2=self.point.point                   
            self.data_out.distance=self.get_distance(self.data_out.contact1,self.data_out.contact2)*100
            self.publisher.publish(self.data_out);
            self.rate.sleep();           


if __name__ == "__main__":
    colector=Collector()   
#   rospy.spin();      
    print("Joao") 
    rospy.sleep(1)
    colector.send_it(); 
