#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy
from pprint import pprint
import std_msgs.msg
import sensor_msgs.msg
                                                                                                       
class JointStateReader(object):                                                                        
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """  
    def callback_on_obj(self, data):
        self.joints = {}
        for counter in range(0,len(data.name)):  
            self.joints[data.name[counter]] = data.position[counter]


    def __init__(self):
        sub = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.callback_on_obj)
        #rospy.spin()                                                                                

    


    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                                                       
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """  
        if name in self.joints:
            return self.joints[name]   
        
        else:                                                      
            return None                                                                                       
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """ 
                                     
        return [self.joints[x] if x in self.joints else 0 for x in names]