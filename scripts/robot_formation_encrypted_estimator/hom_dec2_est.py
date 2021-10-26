#!/usr/bin/env python

import sys
import numpy as np
import rospy
from std_msgs.msg import Int64, Int64MultiArray, Float64MultiArray

import csv

import rospkg

from geometry_msgs.msg import Twist

from pymomorphic import pymorph



class hom_decrypt:
    def __init__(self):    #The "self" structure replaces "env" from Kim's paper
        
        var = pymorph.variables_import()

        self.p=var.p
        self.L=var.L
        self.q=var.q
        self.r=var.r
        self.N=var.N
        
        #self.scal = 100000000 #scaling of numbers to get values smaller than 1

        self.name='n_'+str(int(sys.argv[1]))

        self.running = True

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber(self.name+'/scaling', Int64, self.scaling)


        rospy.Subscriber(self.name+'/cmd_vel_enc', Int64MultiArray, self.Dec)


        # prepare publisher
        self.pub = rospy.Publisher(self.name+'/cmd_vel', Twist, queue_size=1)
        self.velocity = Twist()


    def scaling(self, data):

        if self.running < 10:
            self.scal = data.data

        elif 10 < self.running < 1000:
            self.shutdown()


    def Dec(self, data):

        if self.running:

            encr = data.data
            sizevec = data.layout.dim[0].size #size of each vector

            cc=[]
            for i in range(0, len(encr), sizevec):
                cc.append(list(encr[i:sizevec+i]))

            sk = pymorph.key_import(int(sys.argv[1])) #get secret key


            decr = []

            for c in cc:
                plaintext = pymorph.dec_hom(self.p, self.L, sk, [c])
                decr.append(0.5*(float(plaintext[0][0])/self.scal))


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

            print decr
            v_max = 0.2
            v_min = 0.01
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

            print self.velocity

        else:
            self.shutdown() 

    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Decryption_"+str(int(sys.argv[1]))+"...")
        self.running = False
        print "decryption stopped"
        #self.velocity = Twist()
        #self.pub.publish(self.velocity)
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('dec_'+str(int(sys.argv[1])), anonymous=False)
    hom_decrypt()
    print "Decryption Working"
    rospy.spin() 

