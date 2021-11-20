#!/usr/bin/env python

import sys
import os

import rospy
import rospkg
from std_msgs.msg import String, MultiArrayDimension, Int32, Int64MultiArray
from rospy.numpy_msg import numpy_msg

import numpy as np
import csv

from pymomorphic import pymomorphic_py2 as pymh #Change to pymomorphic_py3 to use with python3


from rospy_tutorials.msg import Floats



class Controller:
    '''This version of the controller does not use Hector's formation control law, as Suzane's research
    only makes use of the error in desired distances for the controller'''

    ''' NOTE: this script requires the dataprocessing node to be running first, as well as an input
        argument, an example of how to properly run this code in the terminal is as following:

        rosrun nexus_encrypted_control controller_N_hom_encr_formation_filter_one.py "1" 
        
        where "1" is the value assigned to the robot
        '''
    
    def __init__(self):
        
        # Get input argument and save it as a string (e.g. n_1)
        self.name='n_'+str(int(sys.argv[1])) 

        # controller variables
        self.d = float(0.8)
        self.d_x = np.sqrt((self.d**(2))/2)
        self.d_y = np.sqrt((self.d**(2))/2)
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        self.c = np.float32(0.5)
        self.U_old = np.array([0, 0])
        self.U_oldd = np.array([0, 0])
        self.mu_hat_log =[]

        # Prepare time variables
        self.time = np.float64([])
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])

        # Prepare shutdown
        rospy.on_shutdown(self.shutdown)
    
        # prepare Log arrays
        self.E1_log = np.array([])
        self.E2_log = np.array([])
        self.E3_log = np.array([])
        self.E4_log = np.array([])
        self.E5_log = np.array([])
        self.E6_log = np.array([])
        self.Un = np.float32([])
        self.U_log = np.array([])
        self.time = np.float64([])
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
        self.k = 0

        self.mu_hat = [0, 0, 0]
        self.o = 1
        
        # Prepare publishers
        self.pub_controller = rospy.Publisher(self.name+'/cmd_vel_enc', String, queue_size=1)
        self.pub_controller_estimator = rospy.Publisher(self.name+'/mu_hat_enc_tot', String, queue_size=1)
        self.pub_mu = rospy.Publisher(self.name+'/mu_hat_enc', String, queue_size=1)

        # Initialize encrypted operator
        self.enc_vars_subscriber = rospy.Subscriber(self.name+'/encryption_vars', String, callback = self.recover_encryption_vars)

        # Prepare subscribers
        rospy.Subscriber(self.name+'/enc_error', String, callback = self.recover_error)
        rospy.Subscriber(self.name+'/enc_rec_z', String, callback = self.recover_z)
        rospy.Subscriber(self.name+'/enc_mu', String, callback = self.recover_mu)
        rospy.Subscriber(self.name+'/enc_dt', String, callback = self.recover_DT)
        rospy.Subscriber(self.name+'/enc_x_and_y', String, callback = self.controller)

        # subscribe to controller_variables
        rospy.Subscriber('/controller_variables', numpy_msg(Floats), self.update_controller_variables)

        # Motion parameters
        self.x_dot = np.float32(0)
        self.y_dot = np.float32(0)
        self.r_dot = np.float32(0)
        self.mu_x = self.x_dot*np.array([0, -1, 0, -1, 0])
        self.mut_x = self.x_dot*np.array([0, 1, 0, 1, 0])
        self.mu_y = self.y_dot*np.array([-1, 0, 0, 0, 1])
        self.mut_y = self.y_dot*np.array([1, 0, 0, 0, -1])
        self.mu_r = self.r_dot*np.array([-1, -1, 0, 1, -1])
        self.mut_r = self.r_dot*np.array([1, 1, 0, -1, 1])
        
        self.mu = self.mu_x+self.mu_y+self.mu_r
        self.mut = self.mut_x+self.mut_y+self.mut_r


    def n(self, data):   #This is used to extract the value of n (i.e. the number of robots the agent detected, published from the from the dataprocessor node)
        
        if not rospy.is_shutdown():
            self.n=data.data

    def recover_encryption_vars(self, data):

        if not rospy.is_shutdown():
            enc_vars = pymh.recvr_pub_ros_str(data.data)
            
            p_enc = enc_vars[0]
            L_enc = enc_vars[1]
            r_enc = enc_vars[2]
            N_enc = enc_vars[3]

            self.my_op = pymh.HOM_OP(p_enc, L_enc, r_enc, N_enc)

            self.mu_hat = [0]*(N_enc+1) #initialize mu_hat with N+1 size

            self.enc_vars_subscriber.unregister() #Once the data has been obtained the subscriber is stopped

    def recover_error(self, data):
        
        if not rospy.is_shutdown():
            e = data.data
            self.e2 = pymh.recvr_pub_ros_str(e)

    def recover_z(self, data): 

        if not rospy.is_shutdown():
            z = data.data
            self.z_rec = pymh.recvr_pub_ros_str(z)

    def recover_mu(self, data): 

        if not rospy.is_shutdown():
            mu = data.data
            self.mu = pymh.recvr_pub_ros_str(mu)

    def recover_DT(self, data): 

        if not rospy.is_shutdown():
            DT_arr = data.data
            self.FG_s_enc = pymh.recvr_pub_ros_str(DT_arr)

    def update_controller_variables(self, data):
        ''' Update controller variables '''

        if not rospy.is_shutdown():
            # Assign data 
            self.controller_variables = data.data

            # Safe variables
            self.running = np.float32(self.controller_variables[0])
            self.d = np.float32(self.controller_variables[1])
            self.dd = np.float32(self.controller_variables[2])
            self.c = np.float32(self.controller_variables[3])
            self.x_dot = np.float32(self.controller_variables[4])
            self.y_dot = np.float32(self.controller_variables[5])
            self.r_dot = np.float32(self.controller_variables[6])
            
            # Calculate mu
            self.mu_x = self.x_dot*np.array([0, -1, 0, -1, 0])
            self.mut_x = self.x_dot*np.array([0, 1, 0, 1, 0])
            self.mu_y = self.y_dot*np.array([-1, 0, 0, 0, 1])
            self.mut_y = self.y_dot*np.array([1, 0, 0, 0, -1])
            self.mu_r = self.r_dot*np.array([-1, -1, 0, 1, -1])
            self.mut_r = self.r_dot*np.array([1, 1, 0, -1, 1])
            
            self.mu = self.mu_x+self.mu_y+self.mu_r
            self.mut = self.mut_x+self.mut_y+self.mut_r
        
    def controller(self, data):
        ''' Calculate U based on z_values and save error velocity in log arrays '''
        if self.pub_mu.get_num_connections() > 0: #To avoid the acumulation of operations on self.mu_hat we wait for the decryption to work
            
            rospy.loginfo("-------------------------------------------")
            rospy.loginfo("Controller of Nexus: %s", int(sys.argv[1]))

            # Input for controller
            xy=data.data
            
            self.xy = pymh.recvr_pub_ros_str(xy)


            # Control law
            #U = self.c*BbDz.dot(Dzt).dot(Ed) #+ (Ab.dot(z)).reshape((2, 1))

            #print self.Hom_mul([self.Hom_mul([list(BbDz[0][0])], [list(Dzt.astype(int))])], [list((Ed.astype(int)))])

            #U =  self.Hom_mul([self.Hom_mul([BbDz[0][0]], Dzt[0])], Ed[0])


            X=[]
            X2=[0]

            Y=[]
            Y2=[0]

            from operator import add
                
            X.append(map(add, self.xy[0], self.z_rec[0]))
            Y.append(map(add, self.xy[1], self.z_rec[0]))


            Z = [X[0],Y[0]]

            Z2 = [X2,Y2]

            # Publish X and Y Velocities
            Z_str = pymh.prep_pub_ros_str(Z) 
            self.pub_controller.publish(String(Z_str))


            # Publish X and Y Velocities of Estimating part of controller
            Z2_str = pymh.prep_pub_ros_str(Z2) 
            self.pub_controller_estimator.publish(String(Z2_str))

            
            # Update Time variables
            self.now = np.float64([rospy.get_time()])
            self.time = np.float64([self.now-self.old])
            self.time_log = np.append(self.time_log, self.time)
            self.old = self.now


    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Encrypted_Controller_"+str(int(sys.argv[1]))+"...")

        rospy.loginfo('Shutting Down')
        rospy.sleep(1)




if __name__ == '__main__':
    
    try:
        rospy.init_node('controller_enc_'+str(int(sys.argv[1])), anonymous=False)
        r = rospy.Rate(10)
        Controller()
        rospy.loginfo("Cloud Processing Working")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller node_"+str(int(sys.argv[1]))+" terminated.")  
        pass
 


