#!/usr/bin/env python

import sys
import numpy as np
import rospy
from std_msgs.msg import Int64, Int64MultiArray, Float64MultiArray

import csv

import rospkg

from geometry_msgs.msg import Twist
from rospy_tutorials.msg import Floats


from pymomorphic import pymorph

from rospy.numpy_msg import numpy_msg



class hom_decrypt:
    def __init__(self):    #The "self" structure replaces "env" from Kim's paper
        
        var = pymorph.variables_import()

        self.p=var.p
        self.L=var.L
        self.q=var.q
        self.r=var.r
        self.N=var.N

        self.b1 = 0
        self.b2 = 0
        self.P1 = 1
        self.P2 = 1
        self.z_filt = np.zeros(2)
        
        #self.scal = 100000000 #scaling of numbers to get values smaller than 1

        self.name='n_'+str(int(sys.argv[1]))

        self.running = True

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber(self.name+'/scaling', Int64, self.scaling)


        rospy.Subscriber(self.name+'/scaling1', Int64, self.scal1_rec)

        rospy.Subscriber(self.name+'/cmd_vel_enc', Int64MultiArray, self.Dec)

        rospy.Subscriber(self.name+'/mu_hat_enc', Int64MultiArray, self.mu_rec)

        rospy.Subscriber(self.name+'/mu_hat_enc_tot', Int64MultiArray, self.mu_rec_tot)

        rospy.Subscriber(self.name+'/noise', numpy_msg(Floats), self.recover_noise)

        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])

        # prepare publisher
        self.pub = rospy.Publisher(self.name+'/cmd_vel', Twist, queue_size=1)

        self.pub_mu = rospy.Publisher(self.name+'/mu_hat_dec', Floats, queue_size=1)

        self.velocity = Twist()




    def scaling(self, data):

        if self.running < 10:
            self.scal = data.data

        elif 10 < self.running < 1000:
            self.shutdown()

    def recover_noise(self, data):

        self.noise = data.data

        self.z_filt[0] = self.noise[0] * self.b1

        self.z_filt[1] = self.noise[1] * self.b2
        
    
    def scal1_rec(self, data):


        self.scal1 = data.data



    def mu_rec(self,data):
        mu = data.data

        self.mu = pymorph.recvr_pub_ros(self.q, self.N, mu)


    def mu_rec_tot(self,data):
        xy_mu = data.data

        self.xy_mu = pymorph.recvr_pub_ros(self.q, self.N, xy_mu)


    def Dec(self, data):

        if self.running:

            encr = data.data
            sizevec = data.layout.dim[0].size #size of each vector

            lamb = 0.999

            self.now = np.float64([rospy.get_time()])
            self.DT = ((self.now-self.old))

            cc=[]
            for i in range(0, len(encr), 3):
                cc.append(list(encr[i:3+i]))

            sk = pymorph.key_import(int(sys.argv[1])) #get secret key


            decr = []
            decr_mu = []

            


            for c in cc:
                plaintext = pymorph.dec_hom(self.p, self.L, sk, [c])
                decr.append(0.6*(float(plaintext[0][0])/10000))

            print "decrypted :", decr


            #self.z_filt2 = np.array([[0,0]])
            #self.z_filt2[0,0]= self.z_filt[0,0]+self.z_filt[0,2]
            #self.z_filt2[0,1]= self.z_filt[0,1]+self.z_filt[0,3]
            u_filt = np.array(decr)- self.z_filt

            self.robot =  self.DT*u_filt
            print "u_filt : ", u_filt
            print "self.robot : ", self.robot

            '''for c in self.xy_mu:
                plaintext = pymorph.dec_hom(self.p, self.L, sk, [c])
                #decr.append(0.011*(float(plaintext[0][0])/self.scal))
                decr_mu.append((float(plaintext[0][0]))/(self.scal*100))

            print decr_mu

            for c in self.mu:
                plaintext = pymorph.dec_hom(self.p, self.L, sk, [c])
                #decr.append(0.011*(float(plaintext[0][0])/self.scal))
                self.mu = (float(plaintext[0][0])/(100))
            
            self.pub_mu.publish([self.mu])

            print ("mu : "+str(self.mu))'''


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

            pi1 = self.P1 * self.noise[0]

            pi2 = self.P2 * self.noise[1]

            print "self.P : ", self.P1

            print "self.b : ", self.b1

            G=np.array([0,0], dtype = np.float32)

            G[0] = pi1/(lamb + self.noise[0]*pi1)

            G[1] = pi2/(lamb + self.noise[0]*pi2)

            #self.b = self.b + G*u_filt

            self.b1 = self.b1+ G[0]*u_filt[0]

            self.b2 = self.b2+ G[1]*u_filt[1]

            #self.P = lamb**(-1)*self.P-lamb**(-1)*G*self.noise*self.P

            self.P1 = lamb**(-1)*self.P1-lamb**(-1)*G[0]*self.noise[0]*self.P1
            self.P2 = lamb**(-1)*self.P2-lamb**(-1)*G[1]*self.noise[1]*self.P2

            #print decr
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

            
            self.velocity.linear.x = self.robot[0]
            self.velocity.linear.y = self.robot[1]

            #self.pub.publish(self.velocity) #the data is: [ANGLE, x dist, y dist]

            

            print self.velocity


            self.now = np.float64([rospy.get_time()])
            self.old = self.now

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

