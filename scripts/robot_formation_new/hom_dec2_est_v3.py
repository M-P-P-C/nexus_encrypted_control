#!/usr/bin/env python

import sys
import os

import rospy
import rospkg
from std_msgs.msg import Int64, Float32, Float32MultiArray, String
from geometry_msgs.msg import Twist

import csv
import numpy as np

from pymomorphic3 import pymomorphic_py2 as pymh2 #Change to pymomorphic_py3 to use with python3


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
        rospy.Subscriber(self.name+'/scaling', Int64, self.scaling)
        rospy.Subscriber(self.name+'/scaling1', Int64, self.scal1_rec)
        rospy.Subscriber(self.name+'/mu_hat_enc', String, self.mu_rec)
        rospy.Subscriber(self.name+'/mu_hat_enc_tot', String, self.mu_rec_tot)
        rospy.Subscriber(self.name+'/cmd_vel_enc', String, self.Dec)


        self.velocity = Twist()


    def scaling(self, data):

        self.scal = data.data
    
    def scal1_rec(self, data):

        self.scal1 = data.data

    def recover_secret_key(self, data):

        if not rospy.is_shutdown():
            
            self.secret_key = np.array(pymh2.recvr_pub_ros_str(data.data), dtype = object)
            
            self.secret_key_subscriber.unregister() #Once the data has been obtained the subscriber is stopped

    def recover_encryption_vars(self, data):

        if not rospy.is_shutdown():
            enc_vars = pymh2.recvr_pub_ros_str(data.data)
            
            p_enc = enc_vars[0]
            L_enc = enc_vars[1]
            r_enc = enc_vars[2]
            N_enc = enc_vars[3]

            self.my_key = pymh2.KEY(p_enc, L_enc, r_enc, N_enc, secret_key_set = self.secret_key)

            self.enc_vars_subscriber.unregister() #Once the data has been obtained the subscriber is stopped



    def mu_rec(self,data):

        while not rospy.is_shutdown():

            mu = data.data

            self.mu = pymh2.recvr_pub_ros_str(mu)

        else:
            self.shutdown() 


    def mu_rec_tot(self,data):

        while not rospy.is_shutdown():
            xy_mu = data.data

            self.xy_mu = pymh2.recvr_pub_ros_str(xy_mu)

        else:
            self.shutdown() 


    def Dec(self, data):

        if not rospy.is_shutdown():
            rospy.loginfo("-------------------------------------------")
            rospy.loginfo("Decryption of Nexus: "+str(int(sys.argv[1])))

            encr = data.data


            cc = pymh2.recvr_pub_ros_str(encr)           


            decr = []

            for c in cc:
                plaintext = self.my_key.decrypt(c)
                decr.append((float(plaintext[0][0])/self.scal))

            self.decr_mu = []

            for c in self.xy_mu:
                plaintext = self.my_key.decrypt(c)
                self.decr_mu.append((float(plaintext[0])/(self.scal*1000))) #multiplied by 1000 because of the multiplication done in the controller for DT



            try:
                for c in [self.mu]:
                    plaintext = self.my_key.decrypt(c)
                    self.mu_dec = (float(plaintext[0])/(1000))
            except:
                for c in [self.mu]:
                    plaintext = self.my_key.decrypt(c)
                    self.mu_dec = (float(plaintext)/(1000))
       

            #rospy.loginfo("Controller Velocity "+ str(decr))
            #rospy.loginfo("Estimator Velocity: "+ str(self.decr_mu))

            #if int(sys.argv[1]) == 1:
            #    self.decr_mu[0] = 0
            #    self.decr_mu[1] = 0

            decr[0] = (decr[0] - self.decr_mu[0]) #Adding the estimated velocity and the controller velocity
            decr[1] = (decr[1] - self.decr_mu[1])

            '''for c in cc:
                s=0
                s = sk[:]
                
                s.insert(0, 1)


                dot=[0]
                aa = 1
                dot[0] = [sum(i[0] * i[1] for i in zip(c,s))]
                            
                
                #dot = [sum(i[0] * i[1] for i in zip(c, s))]  #dot multiplication of c and s

                plain = self.Mod(dot[0], self.L*self.p)

                plaintext = float(plain[0])/self.L

                plaintext = int(round(plaintext))
                #print(plaintext)
                decr.append(float(plaintext)/self.scal)''' #PLAINTEXT must be turn into float otherwise it is an integer and does not provide decimals when divided

            #self.tosend = Float64MultiArray() #store list into array that can be published for ROS
            #print decr
            #self.tosend.data = decr


            v_max = 0.04
            v_min = 0.01 #lower than this results in weird jittering once agents converge
            for i in range(len(decr)):          
                if decr[i] > v_max:
                    decr[i] = v_max
                elif decr[i] < -v_max:
                    decr[i] = -v_max
                elif -v_min < decr[i] < v_min:
                    decr[i] = 0
                #elif -v_min < decr[i]+self.U_old[i]+self.U_oldd[i] < v_min : # preventing shaking 
                #    decr[i] = 0

            
            self.velocity.linear.x = decr[0]
            self.velocity.linear.y = decr[1]

            # Publish velocity for nexus agent
            self.pub.publish(self.velocity) #the data is: [ANGLE, x dist, y dist]
            
            # Publish decrypted and downscaled mu_hat
            self.pub_mu.publish(self.mu_dec) 
            rospy.loginfo(self.mu_dec)
            

            
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

        rospack = rospkg.RosPack()
        PATH = rospack.get_path('nexhom') #This gets the path from the shown ROS package
        FILEPATH = os.path.join(PATH+'/saved_variables', 'Decryption_'+self.name+'.csv')

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

