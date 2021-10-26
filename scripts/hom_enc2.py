#!/usr/bin/env python

import numpy as np
import rospy
import sys
from geometry_msgs.msg import Twist
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int64, Float64, Int64MultiArray	
from rospy_tutorials.msg import Floats

from std_msgs.msg import MultiArrayDimension

from rospy.numpy_msg import numpy_msg

import rospkg

import csv #to read private key from csv file

import random

from pymomorphic import pymorph
        

#CHECK A ROBOT INDIVIDUALLY FOR ERRORS IN CALCULATIONS DUE TO LOST MEMORY USING PYMOMORPHIC

#FIGURE THE "SMART" SCALING

#REDUCE USE OF LONG?

class hom_encrypt:
    '''This encrypts the data recieved and outputs the homomorphically encrypted data in form of a matrix'''
    
    #APPLY SCALING TO MAKE NUMBERS LARGER, OTHERWISE WHEN DECRYPTED THEY ARE 0s AND 1s
    
    def __init__(self):    #The "self" structure replaces "env" from Kim's paper
        
        pymorph.variables_define(p = 10**10, L = 10**4, r=10**1, N = 2)

        var= pymorph.variables_import() #get variables

        self.p=var.p
        self.L=var.L
        self.q=var.q
        self.r=var.r
        self.N=var.N

        pymorph.key_generate(self.q, self.L, self.N, int(sys.argv[1]))

        self.sk = pymorph.key_import(int(sys.argv[1])) #get seret key
        
        self.scal = 10000 #adjust scaling for larger integers

        self.name='n_'+str(int(sys.argv[1]))

        self.running = True

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber(self.name+'/z_values', numpy_msg(Floats), self.enc)

        # prepare publisher
        #self.pub = rospy.Publisher('/encrypted_data', Twist, queue_size=1)
        #self.pub = rospy.Publisher('/encrypted_data', Int64MultiArray, queue_size=1)
        self.pub = rospy.Publisher(self.name+'/encrypted_data', Int64MultiArray, queue_size=1)

        self.pub_scal = rospy.Publisher(self.name+'/scaling', Int64, queue_size=1)

        # subscribe to data topic (this gathers the values that need to be encrypted)
        #rospy.Subscriber(self.name+'/key',  Int64MultiArray)




    def enc(self, data):

        if self.running:
            
            #write check to make sure multiplication is not above threshold

            z_values = data.data.copy()

            robots = len(z_values)/3

            z_values = z_values.reshape((robots, 3))

            '''laser_data=[]
            for i in range(0, len(encr), len(z_values)):
                laser_data.append(encr[i:sizevec+i])'''
            
            self.d = 0.8

            vvv=np.zeros((robots,1))
            for i in range(robots):
                vvv[i] = z_values[i][0]**(-1)

            z_values = np.append(z_values, vvv, axis=1)


            z_values[:, 0] = z_values[:, 0] - self.d #substract desired distance from first column of array

            #print (z_values[0]*z_values[1]*z_values[3])

            #z_values = z_values * self.scal
            #print z_values
            scal1 = 1
            scal2 = 1
            scal3 = 1
            scal4 = 1

            low = abs(z_values) < 0.0001 #This is to take away values that are too small
            
            z_values[low] = 100


            while np.any(abs(z_values[:, 0])<=100):

                #z_values = (np.ma.masked_where(abs(z_values) > 100, z_values)*10).data
                
                z_values[:, 0] *= 10

                scal1 *= 10 

                #print z_values[:, 0]

            while np.any(abs(z_values[:, 1])<=100):
                
                #z_values = (np.ma.masked_where(abs(z_values) > 100, z_values)*10).data
                
                z_values[:, 1] *= 10

                scal2 *= 10 
            
            while np.any(abs(z_values[:, 2])<=100):
                
                #z_values = (np.ma.masked_where(abs(z_values) > 100, z_values)*10).data
                
                z_values[:, 2] *= 10

                scal3 *= 10 
            
            while np.any(abs(z_values[:, 3])<=100):
                
                #z_values = (np.ma.masked_where(abs(z_values) > 100, z_values)*10).data
                
                z_values[:, 3] *= 10

                scal4 *= 10 

            #low = abs(z_values) == 100 #This is to take away values that are too small
            z_values[low] = 1

            if scal2 > scal3:
                scal5 = scal2
                aa=scal2/scal3
                z_values[:, 2] *= aa
            elif scal3 > scal2:
                scal5 = scal3
                aa=scal3/scal2
                z_values[:, 2] *= aa
            else:
                scal5=scal2

            scaling = scal1*scal4*scal5

            '''z_values[:, 0] = z_values[:, 0] * 10000

            z_values[:, 1] = z_values[:, 1] * 1000

            z_values[:, 2] = z_values[:, 2] * 1000

            z_values[:, 3] = z_values[:, 3] * 100

            scaling = 10000*1000*100'''

            #z_values, scaling = pymorph.smart_scaling(z_values, 100) #CHECK WHY THE SMART SCALING FUNCTION IS NOT WORKING PROPERLY

            res = []

            z_values = z_values.astype(int) #Turn all values into integers before encrypting otherwise it causes innacuracy

            
            for i in range(robots):

                m = [z_values[i][0]] 

                ciphertext = pymorph.enc_2(self.p, self.L, self.q, self.r, self.N, self.sk, m) 

                [res.append(ciphertext[i]) for i in range(0, len(ciphertext))] #Enc2 outputs a large list of lists, this appends every list individually


            for i in range(robots):

                m = [z_values[i][1]] 
                
                ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, m)

                res.append(ciphertext[0])


            for i in range(robots):

                m = [z_values[i][2]] 
                
                ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, m)

                res.append(ciphertext[0])

            
            for i in range(robots):

                m = [z_values[i][3]] 

                ciphertext = pymorph.enc_2(self.p, self.L, self.q, self.r, self.N, self.sk, m) 

                [res.append(ciphertext[i]) for i in range(0, len(ciphertext))] #Enc2 outputs a large list of lists, this appends every list individually

            
            '''m = [z_values[3]]

            #m=[int(2)* self.scal]

            ciphertext = pymorph.enc_2(self.p, self.L, self.q, self.r, self.N, self.sk, m) #self.Enc2(m)

            [res.append(ciphertext[i]) for i in range(0, len(ciphertext))] 
            #res[0].append(ciphertext)'''


            self.tosend = Int64MultiArray() #store list into array that can be published for ROS
            flat_list = [item for sublist in res for item in sublist]
            self.tosend.data = flat_list #np.concatenate(res) #res
            self.tosend.layout.dim.append(MultiArrayDimension())
            self.tosend.layout.dim.append(MultiArrayDimension())
            self.tosend.layout.dim[0].label = "number of vectors"
            self.tosend.layout.dim[0].size = len(res)
            self.tosend.layout.dim[1].label = "length"
            self.tosend.layout.dim[1].size= len(res[0])
            #self.tosend.layout.dim= [3, 6]
            
            #print self.tosend

            self.pub.publish(self.tosend)

            print "data from Nexus: "+str(int(sys.argv[1]))
            print scaling
            self.pub_scal.publish(scaling)

            print z_values

            #print (z_values[0]*z_values[1]*z_values[3])

            #print (z_values[0]*z_values[1]*z_values[3])/(self.scal*10000)

            #self.pub.publish(np.asarray(res[0]))

        else:
            self.shutdown()



    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Encryption_"+str(int(sys.argv[1]))+"...")
        self.running = False

        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('enc_'+str(int(sys.argv[1])), anonymous=False)
    hom_encrypt()
    print "Encryption Working"
    rospy.spin() 

