#!/usr/bin/env python

import sys
import os

import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int64, Float32MultiArray, String, Float32, MultiArrayDimension

import numpy as np
import csv #to read private key from csv file
import copy
import random

from pymomorphic import pymomorphic_py2 as pymh #Change to pymomorphic_py3 to use with python3 


class HomEncrypt:
    '''This encrypts the data recieved and outputs the homomorphically encrypted data in form of a matrix'''
    
    def __init__(self):    #The "self" structure replaces "env" from Kim's paper
        
        # Get input argument and save it as a string (e.g. n_1)
        self.name='n_'+str(int(sys.argv[1])) 

        #Define desired distance for agents
        self.d = 0.8
        if int(sys.argv[1]) == 1: #Used to set an agent with a constant mismatch (for estimator)
            self.d = 0.8

        # Initialize encryption
        self.p_enc = 10**13
        self.L_enc = 10**4
        self.r_enc = 10**1   # Error Injection
        self.N_enc = 5       # Key Length

        self.my_key = pymh.KEY(self.p_enc, self.L_enc, self.r_enc, self.N_enc, seed = 20)

        self.mu_dec = 0 # initialize mu that will be received from decryption node


        # Initialize some variables
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
        self.E1_log = []
        self.E2_log = []
        self.z_log = []
        self.DT = self.now-self.old

        # Prepare Publishing of array with DT
        self.DT = 0.1 #(self.now-self.old)
        self.scal_DT = 100
        self.FG_s = [[1, int(self.DT*self.scal_DT)]] 
        self.FG_s_enc =  self.my_key.enc_2_mat(self.FG_s) 
        self.FG_s_enc_str = pymh.prep_pub_ros_str(self.FG_s_enc)

        # Prepare shutdown
        rospy.on_shutdown(self.shutdown)

        # Prepare publishers
        self.pub_e = rospy.Publisher(self.name+'/enc_error', String, queue_size=1)
        self.pub_z = rospy.Publisher(self.name+'/enc_rec_z', String, queue_size=1)
        self.pub_mu = rospy.Publisher(self.name+'/enc_mu', String, queue_size=1)
        self.pub_xy = rospy.Publisher(self.name+'/enc_x_and_y', String, queue_size=1)
        self.pub_DT = rospy.Publisher(self.name+'/enc_dt', String, queue_size=1)
        self.pub_scal = rospy.Publisher(self.name+'/scaling', String, queue_size=1)
        self.pub_scalDT = rospy.Publisher(self.name+'/scaling_DT', Int64, queue_size=1)
        self.pub_encryption_variables = rospy.Publisher(self.name+'/encryption_vars', String, queue_size=1)
        self.pub_secret_key = rospy.Publisher(self.name+'/secret_key', String, queue_size=1) #secret_key for decryption script (encryption and decryption should occur in the same script to avoid this)

        #Specific for rlms filter
        self.pub_noise = rospy.Publisher(self.name+'/noise', numpy_msg(Float32MultiArray), queue_size=1)

        # Prepare subscribers
        rospy.Subscriber(self.name+'/mu_hat_dec', Float32, callback = self.mu_dec_callback)
        rospy.Subscriber(self.name+'/z_values', numpy_msg(Float32MultiArray), callback = self.encrypt_callback, queue_size=1)




    def mu_dec_callback(self, data):

        if not rospy.is_shutdown():
            self.mu_dec = data.data
        

    def encrypt_callback(self, data):

        if not rospy.is_shutdown():
            
            rospy.loginfo("--------------------------------------------------")
            rospy.loginfo("Encryption of Nexus: %s", int(sys.argv[1]))

            # Publish encryption variables
            encryption_vars_topub = pymh.prep_pub_ros_str([self.p_enc, self.L_enc, self.r_enc, self.N_enc])
            self.pub_encryption_variables.publish(encryption_vars_topub)

            # Publish secret_key for decryption
            secret_key_topub = pymh.prep_pub_ros_str(self.my_key.secret_key.tolist())
            self.pub_secret_key.publish(secret_key_topub)

            z_values = data.data

            robots = len(z_values)/3

            z_values = z_values.reshape((robots, 3))

            z_reciprocal=np.zeros((robots,1))
            for i in range(robots):
                z_reciprocal[i] = z_values[i][0]**(-1)

            z_values = np.append(z_values, z_reciprocal, axis=1)


            ########## filter computation and publishing

            rospy.loginfo("actual u_x: {}".format(0.5*(z_values[0][1]-0.8)))

            noise = np.zeros((robots, 2), dtype = np.float32)

            for agent in range(robots):
                for j in [1,2]:
                    noise[agent][j-1] = round(z_values[agent][j]) - z_values[agent][j] + np.random.randint(low=0, high =3)
                    z_values[agent][j] = z_values[agent][j] + noise[agent][j-1]

            # All of this to publish a multidimensional array...
            noise_multiarray = Float32MultiArray()
            noise_multiarray.data = noise.flatten()
            noise_multiarray.layout.dim.append(MultiArrayDimension())
            noise_multiarray.layout.dim.append(MultiArrayDimension())
            noise_multiarray.layout.dim[0].size = noise.shape[0]
            noise_multiarray.layout.dim[1].size = noise.shape[1]
            noise_multiarray.layout.dim[0].label = "xy"
            noise_multiarray.layout.dim[1].label = "agents"
            noise_multiarray.layout.dim[0].stride = noise.shape[0]*noise.shape[1]
            noise_multiarray.layout.dim[1].stride = noise.shape[0]
                
            self.pub_noise.publish(noise_multiarray)

            rospy.loginfo("noise: {}".format(noise))
            
            ##########


            z_values[:, 0] = z_values[:, 0] - self.d #substract desired distance from first column of array

            # Calculating and Publishing mu before scaling to avoid large jumps in the estimator
            mu = [int(0)] 

            # Uncomment the following to set a different mu for specific agents
            '''if int(sys.argv[1]) == 4:
                mu = [int(0)] '''

            rospy.loginfo("Expected Estimator Velocity: [%s, %s]", self.mu_dec*z_values[0][1]*z_values[0][3], self.mu_dec*z_values[0][2]*z_values[0][3])             
            
            scaling_mu = 10000
            mu = 2*(z_values[0][0]-self.mu_dec) * scaling_mu

            rospy.loginfo("%s = %s - %s \n", int(mu), z_values[0][0], self.mu_dec)
            rospy.loginfo("Received mu: %s \n", self.mu_dec)
            rospy.loginfo("New mu: %s \n", int(mu))
            
            #Bounding mu_dec to avoid large fluctuations (debug)
            '''if self.mu_dec > 100: 
                self.mu_dec = 100
            elif self.mu_dec < -100:
                self.mu_dec = -100'''
            
            mu_ciph = self.my_key.encrypt([int(mu)])

            mu_topub = pymh.prep_pub_ros_str(mu_ciph)
            self.pub_mu.publish(String(mu_topub))            


            # Scaling of data
            scal1 = 4 #scaling of error
            scal2 = 2 #scaling of x
            scal3 = scal2 #scaling of y must be same as x
            scal4 = 3 #scaling of reciprocal of z

            low = abs(z_values) < 0.00001 #This is to take away values that are too small
            
            z_values[low] = 0 #Make sure low values are 0

            scal1 = self.my_key.log_scaling(z_values[:, 0], scal1)
            scal2 = self.my_key.log_scaling(z_values[:, 1], scal2)
            scal3 = self.my_key.log_scaling(z_values[:, 2], scal3)
            scal4 = self.my_key.log_scaling(z_values[:, 3], scal4)

            z_values[:, 0] = scal1[0]
            z_values[:, 1] = scal2[0]
            z_values[:, 2] = scal3[0]
            z_values[:, 3] = scal4[0]

            z_values = z_values.astype(int) #Turn all values into integers before encrypting otherwise it causes errors

            scal1 = scal1[1]
            scal2 = scal2[1]
            scal3 = scal3[1]
            scal4 = scal4[1]

            scaling = scal1*scal2*scal4

            rospy.loginfo("z_values after scaling:\n %s", z_values)
            rospy.loginfo("Total Scaling: %i", scaling)
            
            scaling_values = pymh.prep_pub_ros_str([scal1, scal2, scal4, self.scal_DT, scaling_mu])

            self.pub_scal.publish(scaling_values)
            self.pub_scalDT.publish(self.scal_DT)

            # Publish Error
            err_ciph = []

            for i in range(robots): #Encryption of error (z-d) with Enc1

                m = [z_values[i][0]] 

                ciphertext = self.my_key.encrypt(m) 

                err_ciph.append(ciphertext[0])

            error_topub = pymh.prep_pub_ros_str(err_ciph) #Pymomorphic must be updated to output only lists

            self.pub_e.publish(String(error_topub))

            
            # Publish reciprocal of z
            zr_ciph = [[]]*robots
            
            for i in range(robots):

                m = [z_values[i][3]] 

                zr_ciph[i] = self.my_key.encrypt2(m) 
                
            rec_z_topub = pymh.prep_pub_ros_str(zr_ciph)

            self.pub_z.publish(String(rec_z_topub))
            


            # Publish x and y distances
            x_y = [[row[i] for row in z_values] for i in range(1,3)] #[[x1,x2,x3],[y1,y2,y3]]

            x_y_enc = self.my_key.enc_2_mat(x_y)

            x_y_enc_topub = pymh.prep_pub_ros_str(x_y_enc)
            
            self.pub_xy.publish(String(x_y_enc_topub))
            

            # Publish DT array
            self.pub_DT.publish(self.FG_s_enc_str)


            # Update Time variables
            self.now = np.float64([rospy.get_time()])
            '''self.time = np.float64([self.now-self.begin])
            self.time_log = np.append(self.time_log, self.time)'''
            print_time= self.now-self.old
            #rospy.loginfo("Time between runs: %s", print_time) #Creates stuttering in the printing on the console
            self.old = self.now            


        else:
            self.shutdown()



    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''

        rospy.loginfo("Stopping Encryption_"+str(int(sys.argv[1]))+"...")

        rospy.loginfo('Shutting Down')
        rospy.sleep(1)


if __name__ == '__main__':
    
    try:
        rospy.init_node('enc_'+str(int(sys.argv[1])), anonymous=False)
        r = rospy.Rate(10)
        HomEncrypt()
        rospy.loginfo("Encryption Working")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Encryption node_"+str(int(sys.argv[1]))+" terminated.")
        pass

