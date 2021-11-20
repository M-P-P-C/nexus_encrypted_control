#!/usr/bin/env python  

import sys
import os

import rospy
import rospkg
from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg

import numpy as np
import csv


from pymomorphic import pymomorphic_py2 as pymh #Change to pymomorphic_py3 to use with python3



class Controller:
    ''' The controller uses the interagent distances to determine the desired velocity of the Nexus '''

    ''' NOTE: this script requires the dataprocessing node to be running first, as well as an input
        argument, an example of how to properly run this code in the terminal is as following:
        
        rosrun lasmulticontrol3 controller_N_encr_formation_est_v3.py "1" 
        
        where "1" is the value assigned to the robot
        '''
    
    def __init__(self):

        # Get input argument and save it as a string (e.g. n_1)
        self.name='n_'+str(int(sys.argv[1])) 

        # controller variables
        self.running = np.float32(1)
        self.d = int(0.8)
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        self.c = np.float32(0.5)
        self.U_old = np.array([0, 0])
        self.U_oldd = np.array([0, 0])


        '''# Motion parameters
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
        self.mut = self.mut_x+self.mut_y+self.mut_r'''
                
        # prepare Log arrays
        self.E1_log = np.array([])
        self.E2_log = np.array([])
        self.E3_log = np.array([])
        self.E4_log = np.array([])
        self.E5_log = np.array([])
        self.E6_log = np.array([])
        self.Un = np.float32([])
        self.U_log = np.array([])
        self.k = 0
        self.mu_hat_log =[]
        self.DT_log =[]

        # Prepare time variables
        self.time = np.float64([])
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
        

        # Prepare shutdown
        rospy.on_shutdown(self.shutdown)
        
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

        # Initialize Decryption for debugging
        '''
        self.decrypt_init()
        #'''

        # subscribe to controller_variables
        #rospy.Subscriber('/controller_variables', numpy_msg(Floats), self.update_controller_variables)


    def n(self, data):   #This is used to extract the value of n (i.e. the number of robots the agent detected, published from the from the dataprocessor node)
        
        if not rospy.is_shutdown():
            self.n = data.data

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


        #print "PRINTING MULTIPILICATION CONTROLLER: " + str([self.mu_hat, self.mu2[0]])
        
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
        
        # self.pub_controller_estimator.get_num_connections() > 0

        if self.pub_mu.get_num_connections() > 0: #To avoid the acumulation of operations on self.mu_hat we wait for the decryption to work

            rospy.loginfo("-------------------------------------------")
            rospy.loginfo("Controller of Nexus: %s", int(sys.argv[1]))

            xy=data.data
            
            self.xy = pymh.recvr_pub_ros_str(xy)

            # Calculate new mu_hat
            self.mu_hat = self.my_op.hom_mul_mat(self.FG_s_enc, [self.mu_hat, self.mu[0]])[0] #if constantly running this function will constantly add to self.mu_hat

            X=[]
            X2=[]

            Y=[]
            Y2=[]


            # [X; Y] x mu_hat
            xy_by_mu_x = self.my_op.hom_multiply([self.mu_hat], [self.xy[0][0]])
            xy_by_mu_y = self.my_op.hom_multiply([self.mu_hat], [self.xy[1][0]])
            xy_by_mu = [xy_by_mu_x[0], xy_by_mu_y[0]]


            # [X; Y] x Error
            xy_err = self.my_op.hom_mul_mat(self.xy, self.e2)

            # Calculate final X and Y velocities (For both sections of the controller (U = Controller + Estimator))

            # [Xe; Ye] x D_z_tilde
            X = self.my_op.hom_multiply([xy_err[0]] , self.z_rec[0])
            Y = self.my_op.hom_multiply([xy_err[1]] , self.z_rec[1])

            X2 = (self.my_op.hom_multiply([xy_by_mu[0]] , self.z_rec[0]))
            Y2 = (self.my_op.hom_multiply([xy_by_mu[1]] , self.z_rec[0])) 

            # Assemble an array with the X and Y velocities 
            Z = [X,Y]
            Z2 = [X2,Y2]    

            # Publish mu_hat 
            rospy.loginfo("%s + %s", self.my_key.decrypt(self.mu_hat)[0], self.my_key.decrypt(self.mu[0])[0])
            mu_str = pymh.prep_pub_ros_str(self.mu_hat)
            self.pub_mu.publish(String(mu_str))

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

    def decrypt_init(self): #This function is here only for debugging purposes

        p_enc = 10**13
        L_enc = 10**4
        r_enc = 10**1   # Error Injection
        N_enc = 5       # Key Length

        self.my_key = pymh.KEY(p_enc, L_enc, r_enc, N_enc, seed = 20)


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
        


