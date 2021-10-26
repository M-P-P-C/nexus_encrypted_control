#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

import pandas as pd
import numpy as np

import sys #used to get arguments into the code
import os
import rospkg

class data_listener:

    def __init__(self):

        rospy.init_node('listener_new', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        self.df = pd.DataFrame()

        run_n = int(sys.argv[1])

        rospack = rospkg.RosPack()
        PATH = rospack.get_path('nexhom') #This gets the path from the shown ROS package
        self.FILEPATH = os.path.join(PATH+'/saved_variables', 'Saved_Data_'+str(run_n)+'.csv')

        self.begin = np.float64([rospy.get_time()])

        self.time_log = []

                
        self.time = []
        
        self.mu = []
        self.error1 = []
        self.error2 = []
        self.robot = []

        self.my_data = [[],[],[]]
        self.my_err1 = [[],[],[]]
        self.my_err2 = [[],[],[]]
        
        rospy.Subscriber('n_1/mu_hat_dec', Floats, self.callback, callback_args=(1))
        rospy.Subscriber('n_2/mu_hat_dec', Floats, self.callback, callback_args=(2))
        rospy.Subscriber('n_3/mu_hat_dec', Floats, self.callback, callback_args=(3))

        rospy.Subscriber('n_1/z_values', numpy_msg(Floats), self.callback_z, callback_args=(1))
        rospy.Subscriber('n_2/z_values', numpy_msg(Floats), self.callback_z, callback_args=(2))
        rospy.Subscriber('n_3/z_values', numpy_msg(Floats), self.callback_z, callback_args=(3))

        while not rospy.is_shutdown():
            self.store_df(run_n)
        


    def callback(self, data, args):
        
        self.my_data[args] = data.data

        #self.mu = []

    def callback_z(self, data, args):
        
        self.my_data_z = data.data

        #self.my_err1[args-1].append(self.my_data_z[0])
        #self.robot = []




    def store_df(self, run_n):
        

        self.now = np.float64([rospy.get_time()])
        self.time = np.float64([self.now-self.begin])
        self.time_log = np.append(self.time_log, self.time)
        self.old = self.now

        print self.time

        if self.time < 15:

            self.robot.append(1)
            self.mu = []
            self.error1 = []
            self.error2.append(self.my_err1)
            self.robot = []

            

        else:

            self.df["robot"]=self.robot
            self.df["error1"]=self.error1
            self.df["error2"]=self.error2

            rospy.signal_shutdown("Time is over")
    

    def shutdown(self):

        self.df.to_csv(self.FILEPATH)

        rospy.sleep(5)


if __name__ == '__main__':
    data_listener()
    rospy.spin()