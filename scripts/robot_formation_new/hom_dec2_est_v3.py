#!/usr/bin/env python

import sys
import os

import rospy
import rospkg
from std_msgs.msg import Int64, Float32, Float32MultiArray, String
from geometry_msgs.msg import Twist

import csv
import numpy as np

from pymomorphic3 import pymomorphic_py2 as pymh2


class Hom_decrypt:
    def __init__(self):    #The "self" structure replaces "env" from Kim's paper
        
        # Get input argument and save it as a string (e.g. n_1)
        self.name='n_'+str(int(sys.argv[1])) 

        self.my_key = pymh2.KEY(p = 10**13, L = 10**4, r=10**1, N = 5)


        self.DUMDUM = 1


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


    def mu_rec(self,data):

        while not rospy.is_shutdown():

            mu = data.data

            self.mu = pymh2.recvr_pub_ros_str(mu)

        else:
            self.shutdown() 


    def mu_rec_tot(self,data):

        #if self.running:
        xy_mu = data.data

        self.xy_mu = pymh2.recvr_pub_ros_str(xy_mu)

        '''self.decr_mu = []

        for c in self.xy_mu:
            plaintext = pymorph.dec_hom(self.p, self.L, self.sk, c)
            self.decr_mu.append(0.5*(float(plaintext[0])/(self.scal*100))) #multiplied by 100 because of the multiplication done in the controller for DT
        
        print ("FINAL VELOCITY X Before Limit: "+str(self.decr_mu[0]))  
        print ("FINAL VELOCITY Y Before Limit: "+str(self.decr_mu[1])+"\n")

        #print ("Decrypted Message: "+str(decr)+"\n")
        v_max = 0.05
        v_min = 0.005
        for i in range(len(self.decr_mu)):          
            if self.decr_mu[i] > v_max:
                self.decr_mu[i] = v_max
            elif self.decr_mu[i] < -v_max:
                self.decr_mu[i] = -v_max
            elif -v_min < self.decr_mu[i] < v_min:
                self.decr_mu[i] = 0
            #elif -v_min < decr[i]+self.U_old[i]+self.U_oldd[i] < v_min : # preventing shaking 
            #    decr[i] = 0

        
        self.velocity.linear.x = self.decr_mu[0]
        self.velocity.linear.y = self.decr_mu[1]

        print ("FINAL VELOCITY X MU: "+str(self.decr_mu[0]))'''

        #self.pub.publish(self.velocity)


    def Dec(self, data):

        if not rospy.is_shutdown():
            print("-------------------------------------------")
            print("Decryption of Nexus: "+str(int(sys.argv[1])))

            encr = data.data

            #sizevec = data.layout.dim[0].size #size of each vector

            #cc=[]
            #for i in range(0, len(encr), sizevec):
            #    cc.append(list(encr[i:sizevec+i]))

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
       
        


            #print ("Controller Velocity "+ str(decr))
            #print ("Estimator Velocity: "+ str(self.decr_mu))

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

            
            #print ("FINAL VELOCITY X Before Limit: "+str(decr[0]))
            #print ("FINAL VELOCITY Y Before Limit: "+str(decr[1])+"\n")

            #print ("Decrypted Message: "+str(decr)+"\n")
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

            self.pub.publish(self.velocity) #the data is: [ANGLE, x dist, y dist]
            self.pub_mu.publish([self.mu_dec]) 

            
            #print ("FINAL VELOCITY X: "+str(self.velocity.linear.x)+"\n")
            #print ("FINAL VELOCITY Y: "+str(self.velocity.linear.y)+"\n")
            #print ("Decr_mu: "+ str(self.decr_mu))
            #print ("mu : "+str(self.mu))

            self.now = np.float64([rospy.get_time()])
            self.time = np.float64([self.now-self.begin])
            print(self.time)
            self.time_log = np.append(self.time_log, self.time)
            self.old = self.now

            self.mu_hat_log.append(self.mu_dec/self.scal)

            '''rospack = rospkg.RosPack()
            #PATH = os.path.dirname(os.path.abspath(__file__)) #This gets path from wherever the terminal used to launch the file was
            PATH = rospack.get_path('nexhom') #This gets the path from the shown ROS package
            FILEPATH = os.path.join(PATH+'/saved_variables', 'Decryption_'+self.name+'.csv')
            with open(FILEPATH, "a") as output:
                    
                #np.savetxt(output, np.array([self.mu/self.scal]), delimiter=",")
                #np.savetxt(output, np.array([self.mu/self.scal]), delimiter=",")

                self.writer = csv.writer(output, delimiter=',')
                self.writer.writerow([self.mu/self.scal])
                self.writer.writerow(self.time)'''
                        

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


        if self.DUMDUM == 1:
            self.DUMDUM=2
            with open(FILEPATH, "a") as output:
                
                #np.savetxt("err"+self.name+".csv", (np.asarray(self.E_log)), delimiter=",")
                        
                self.writer = csv.writer(output, delimiter=',')
                self.writer.writerow(self.mu_hat_log)
                #self.writer.writerow(list(z_values_old_noscal[:, 0]+0.8))
                #self.writer.writerow([self.z_log[1]])
                self.writer.writerow(self.time_log)

        print("\n")

        print("-----------------------------------------")

        print("Velocity of Agent set to 0\n")
        
        
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

