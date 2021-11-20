#!/usr/bin/env python

import sys
import os

import rospy
import rospkg
from std_msgs.msg import Int64, Float32, Float32MultiArray, String, MultiArrayDimension
from geometry_msgs.msg import Twist
from rospy.numpy_msg import numpy_msg

import csv
import numpy as np

from pymomorphic import pymomorphic_py2 as pymh #Change to pymomorphic_py3 to use with python3


class HomDecrypt:
    def __init__(self):    #The "self" structure replaces "env" from Kim's paper
        
        # Get input argument and save it as a string (e.g. n_1)
        self.name='n_'+str(int(sys.argv[1])) 

        # Initialize some variables
        self.time = np.float64([])
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
        self.mu_hat_log = []

        self.velocity = Twist()

        # Prepare shutdown
        rospy.on_shutdown(self.shutdown)

        # Prepare publishers
        self.pub = rospy.Publisher(self.name+'/cmd_vel', Twist, queue_size=1)
        self.pub_mu = rospy.Publisher(self.name+'/mu_hat_dec', Float32, queue_size=1)

        # Initialize decryption
        self.secret_key_subscriber = rospy.Subscriber(self.name+'/secret_key', String, callback = self.recover_secret_key)
        self.enc_vars_subscriber = rospy.Subscriber(self.name+'/encryption_vars', String, callback = self.recover_encryption_vars)

        # Prepare subscribers
        rospy.Subscriber(self.name+'/scaling', String, self.scaling)
        rospy.Subscriber(self.name+'/scaling_DT', Int64, self.scalDT_rec)
        rospy.Subscriber(self.name+'/mu_hat_enc', String, self.mu_rec)
        rospy.Subscriber(self.name+'/mu_hat_enc_tot', String, self.mu_rec_tot)
        rospy.Subscriber(self.name+'/cmd_vel_enc', String, self.Dec)
        
        rospy.Subscriber(self.name+'/noise', numpy_msg(Float32MultiArray), callback = self.recover_noise)

        #Filter related
        self.b = np.zeros((2), dtype = np.float32) #(neighbours, [x,y])
        self.P = np.ones((2), dtype = np.float32)
        self.G = np.zeros((2), dtype = np.float32)
        self.z_filt = np.zeros((2), dtype = np.float32)

        #self.P1 = 1
        #self.P2 = 1
        #self.z_filt = np.zeros(2)



    def scaling(self, data):

        scal = data.data
        self.scaling_list = pymh.recvr_pub_ros_str(scal)
    
    def scalDT_rec(self, data):

        self.scal_DT = data.data

    def recover_noise(self, data):

        self.noise = data.data.reshape((data.layout.dim[0].size, data.layout.dim[1].size))

        self.z_filt[0] = self.noise[0][0] * self.b[0]
        self.z_filt[1] = self.noise[0][1] * self.b[1]

        #for agent in range(data.layout.dim[1].size):
        #    self.z_filt[agent] = np.matmul(self.noise[agent], self.b[agent])

        
    def recover_secret_key(self, data):

        if not rospy.is_shutdown():
            
            self.secret_key = np.array(pymh.recvr_pub_ros_str(data.data), dtype = object)

    def recover_encryption_vars(self, data):

        if not rospy.is_shutdown():
            enc_vars = pymh.recvr_pub_ros_str(data.data)
            
            p_enc = enc_vars[0]
            L_enc = enc_vars[1]
            r_enc = enc_vars[2]
            N_enc = enc_vars[3]

            self.my_key = pymh.KEY(p_enc, L_enc, r_enc, N_enc, secret_key_set = self.secret_key)

            self.enc_vars_subscriber.unregister() #Once the data has been obtained the subscriber is stopped

    def mu_rec(self,data):

        if not rospy.is_shutdown():

            mu = data.data

            self.mu = pymh.recvr_pub_ros_str(mu)


    def mu_rec_tot(self,data):

        if not rospy.is_shutdown():

            xy_mu = data.data

            self.xy_mu = pymh.recvr_pub_ros_str(xy_mu)


    def Dec(self, data):

        if not rospy.is_shutdown():

            rospy.loginfo("-------------------------------------------")
            rospy.loginfo("Decryption of Nexus: "+str(int(sys.argv[1])))

            encr = data.data
            encrypted_data = pymh.recvr_pub_ros_str(encr)

            lamb = 0.999
            self.now = np.float64([rospy.get_time()])
            self.DT = 1#((self.now-self.old))

            # Prepare the scaling values [scaling_error, scaling_x, scaling_z_reciprocal, scaling_DT, scaling_(err-mu)]
            #S = self.scaling_list[0]*self.scaling_list[1]*self.scaling_list[2]
            #S_mu = self.scaling_list[1]*self.scaling_list[2]*self.scaling_list[3]*self.scaling_list[4]
            #s1 = self.scaling_list[3]*self.scaling_list[4]

            S = self.scaling_list[1]

            # Decrypt X and Y velocities
            decr = []

            for c in encrypted_data:
                plaintext = self.my_key.decrypt(c)
                decr.append(0.7 * (float(plaintext[0])/S))

            '''
            # Decrypt X and Y velocities of Estimating factor
            self.decr_mu = []

            for c in self.xy_mu:
                plaintext = self.my_key.decrypt(c)
                self.decr_mu.append((float(plaintext[0])/(S_mu))) #multiplied by self.scal_DT because of the multiplication done to DT
            

            # Decrypt new mu to send to encryption node
            for c in [self.mu]:
                plaintext = self.my_key.decrypt(c)
                self.mu_dec = (float(plaintext[0])/s1)
            '''

            u_filt = np.array(decr) - self.z_filt

            final_velocity = self.DT * u_filt

            #'''
            rospy.loginfo("z_filt: {}".format(self.z_filt))
            rospy.loginfo("u_filt: {}".format(u_filt))
            rospy.loginfo("Agent Velocity: {}".format(final_velocity))
            rospy.loginfo("self.b : {}".format(self.b))
            rospy.loginfo("self.P : {}".format(self.P))

            #Update Filter Parameters
            pi_x = self.P[0] * self.noise[0][0]
            pi_y = self.P[1] * self.noise[0][1]

            self.G[0] = pi_x/(lamb + self.noise[0][0] * pi_x)
            self.G[1] = pi_y/(lamb + self.noise[0][1] * pi_y)

            self.b[0] = self.b[0] + self.G[0] * u_filt[0]
            self.b[1] = self.b[1] + self.G[1] * u_filt[1]

            self.P[0] = lamb**(-1)*self.P[0]-lamb**(-1)*self.G[0]*self.noise[0][0]*self.P[0]
            self.P[1] = lamb**(-1)*self.P[1]-lamb**(-1)*self.G[1]*self.noise[0][1]*self.P[1]
            #'''

            ''' #Attempt to make filter work with multiple neighbours
            pi_X = self.P[0] * self.noise[0]
            pi_Y = self.P[1] * self.noise[1]

            self.G[0] = pi_X/(lamb + self.noise[0] * pi_X)
            self.G[1] = pi_Y/(lamb + self.noise[1] * pi_Y)

            self.b[0] = self.b[0] + self.G[0] * u_filt[0]
            self.b[1] = self.b[1] + self.G[1] * u_filt[1]

            self.P[0] = (lamb**(-1)) * self.P[0] - (lamb**(-1)) * self.G[0] * self.noise[0] * self.P[0]
            self.P[1] = (lamb**(-1)) * self.P[1] - (lamb**(-1)) * self.G[1] * self.noise[1] * self.P[1]
            '''
            
            '''
            final_velocity = [0, 0]

            final_velocity[0] = (decr[0] - self.decr_mu[0]) #Adding the estimated velocity and the controller velocity
            final_velocity[1] = (decr[1] - self.decr_mu[1])

            rospy.loginfo("Final Velocity = Controller Velocity - Estimator Velocity")
            rospy.loginfo("%s = %s - %s", str(['%.5f' % n for n in final_velocity]), str(['%.5f' % n for n in decr]), str(['%.5f' % n for n in self.decr_mu]))
            rospy.loginfo("\n")

            v_max = 0.04
            v_min = 0.001
            for i in range(len(decr)):          
                if final_velocity[i] > v_max:
                    final_velocity[i] = v_max
                elif decr[i] < -v_max:
                    final_velocity[i] = -v_max
                elif -v_min < decr[i] < v_min:
                    final_velocity[i] = 0
            '''
            
            self.velocity.linear.x = final_velocity[0]
            self.velocity.linear.y = final_velocity[1]

            # Publish decrypted and downscaled mu_hat
            #self.pub_mu.publish(self.mu_dec)

            # Publish velocity for nexus agent
            self.pub.publish(self.velocity) #the data is: [ANGLE, x dist, y dist]


            self.now = np.float64([rospy.get_time()])
            self.time = np.float64([self.now-self.begin])
            #rospy.loginfo(self.time)
            self.time_log = np.append(self.time_log, self.time)
            self.old = self.now

        else:
            self.shutdown() 

    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Decryption_"+str(int(sys.argv[1]))+"...")

        self.velocity = Twist()
        self.pub.publish(self.velocity) #set velocity to 0 to avoid drift

        rospy.loginfo("\n")

        rospy.loginfo("-----------------------------------------")

        rospy.loginfo("Velocity of Agent set to 0\n")
        
        
        rospy.loginfo('Shutting Down')
        rospy.sleep(1)

if __name__ == '__main__':
    
    try:
        rospy.init_node('dec_'+str(int(sys.argv[1])), anonymous=False)
        r = rospy.Rate(10)
        HomDecrypt()
        rospy.loginfo("Decryption Working")
        rospy.spin() 
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Decryption node_"+str(int(sys.argv[1]))+" terminated.")  
        pass

