#!/usr/bin/env python

import sys
import os

import rospy
import rospkg
from std_msgs.msg import Int64, Float32, Float32MultiArray, String
from geometry_msgs.msg import Twist

import csv
import numpy as np

from pymomorphic import pymomorphic_py2 as pymh #Change to pymomorphic_py3 to use with python3


class Hom_decrypt:

    def __init__(self):
        
        # Get input argument and save it as a string (e.g. n_1)
        self.name='n_'+str(int(sys.argv[1])) 

        # Initialize some variables
        self.time = np.float64([])
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
        self.mu_hat_log = []
        
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


        self.velocity = Twist()


    def scaling(self, data):

        scal = data.data
        self.scaling_list = pymh.recvr_pub_ros_str(scal)
    
    def scalDT_rec(self, data):

        self.scal_DT = data.data

    def recover_secret_key(self, data):

        if not rospy.is_shutdown():
            
            self.secret_key = np.array(pymh.recvr_pub_ros_str(data.data), dtype = object)
            
            #self.secret_key_subscriber.unregister() #Once the data has been obtained the subscriber is stopped

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

            # Prepare the scaling values [scaling_error, scaling_x, scaling_z_reciprocal, scaling_DT, scaling_(err-mu)]
            S = self.scaling_list[0]*self.scaling_list[1]*self.scaling_list[2]
            S_mu = self.scaling_list[1]*self.scaling_list[2]*self.scaling_list[3]*self.scaling_list[4]
            s1 = self.scaling_list[3]*self.scaling_list[4]

            # Decrypt X and Y velocities
            decr = []

            for c in encrypted_data:
                plaintext = self.my_key.decrypt(c)
                decr.append((float(plaintext[0])/S))

            # Decrypt X and Y velocities of Estimating factor
            self.decr_mu = []

            for c in self.xy_mu:
                plaintext = self.my_key.decrypt(c)
                self.decr_mu.append((float(plaintext[0])/(S_mu))) #multiplied by self.scal_DT because of the multiplication done to DT

            # Decrypt new mu to send to encryption node
            for c in [self.mu]:
                plaintext = self.my_key.decrypt(c)
                self.mu_dec = (float(plaintext[0])/s1)
       


            #if int(sys.argv[1]) == 1:
            #    self.decr_mu[0] = 0
            #    self.decr_mu[1] = 0

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

            
            self.velocity.linear.x = final_velocity[0]
            self.velocity.linear.y = final_velocity[1]

            # Publish decrypted and downscaled mu_hat
            self.pub_mu.publish(self.mu_dec)

            # Publish velocity for nexus agent
            self.pub.publish(self.velocity) #the data is: [ANGLE, x dist, y dist]
            

            

            
            #print ("FINAL VELOCITY X: "+str(self.velocity.linear.x)+"\n")
            #print ("FINAL VELOCITY Y: "+str(self.velocity.linear.y)+"\n")
            #print ("Decr_mu: "+ str(self.decr_mu))
            #print ("mu : "+str(self.mu))

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

        #rospack = rospkg.RosPack()
        #PATH = rospack.get_path('nexus_encrypted_control') #This gets the path from the shown ROS package
        #FILEPATH = os.path.join(PATH+'/saved_variables', 'Decryption_'+self.name+'.csv')

        rospy.loginfo("\n")

        rospy.loginfo("-----------------------------------------")

        rospy.loginfo("Velocity of Agent set to 0\n")
        
        
        rospy.loginfo('Shutting Down')
        rospy.sleep(1)

if __name__ == '__main__':
    
    try:
        rospy.init_node('dec_'+str(int(sys.argv[1])), anonymous=False)
        r = rospy.Rate(10)
        Hom_decrypt()
        rospy.loginfo("Decryption Working")
        rospy.spin() 
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Decryption node_"+str(int(sys.argv[1]))+" terminated.")  
        pass

