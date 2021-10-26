#!/usr/bin/env python

import numpy as np
import rospy
import sys
from geometry_msgs.msg import Twist
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int64, Float64, Int64MultiArray	
from rospy_tutorials.msg import Floats

from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import String

from rospy.numpy_msg import numpy_msg

import rospkg

import csv #to read private key from csv file
import os

import random

from pymomorphic import pymorph
        

#CHECK A ROBOT INDIVIDUALLY FOR ERRORS IN CALCULATIONS DUE TO LOST MEMORY USING PYMOMORPHIC

#FIGURE THE "SMART" SCALING

#REDUCE USE OF LONG?

class hom_encrypt:
    '''This encrypts the data recieved and outputs the homomorphically encrypted data in form of a matrix'''
    
    #APPLY SCALING TO MAKE NUMBERS LARGER, OTHERWISE WHEN DECRYPTED THEY ARE 0s AND 1s
    
    def __init__(self):    #The "self" structure replaces "env" from Kim's paper
        
        pymorph.variables_define(p = 10**12, L = 10**4, r=10**1, N = 10)

        var= pymorph.variables_import() #get variables

        self.p=var.p
        self.L=var.L
        self.q=var.q
        self.r=var.r
        self.N=var.N

        pymorph.key_generate(self.q, self.L, self.N, int(sys.argv[1]))

        self.sk = pymorph.key_import(int(sys.argv[1])) #get seret key
        
        self.scal = 1000 #adjust scaling for larger integers

        self.name='n_'+str(int(sys.argv[1]))

        self.running = True

        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber(self.name+'/mu_hat_dec', Floats, self.mu_dec)

        self.mu_dec = 0
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
	self.E_log = []
        self.DT = self.now-self.old

        rospy.Subscriber(self.name+'/z_values', numpy_msg(Floats), self.enc)

        self.o=1
        # prepare publisher
        #self.pub = rospy.Publisher('/encrypted_data', Twist, queue_size=1)
        #self.pub = rospy.Publisher('/encrypted_data', Int64MultiArray, queue_size=1)
        self.pub_e = rospy.Publisher(self.name+'/enc_error', Int64MultiArray, queue_size=1)
        self.pub_z = rospy.Publisher(self.name+'/enc_rec_z', Int64MultiArray, queue_size=1)
        self.pub_mu = rospy.Publisher(self.name+'/enc_mu', Int64MultiArray, queue_size=1)
        self.pub_xy = rospy.Publisher(self.name+'/enc_x_and_y', Int64MultiArray, queue_size=1)

        #ADD PUBLISHERS FOR FG_s AND ZEROS_MATRIX


        self.pub_scal = rospy.Publisher(self.name+'/scaling', Int64, queue_size=1)
        self.pub_scal1 = rospy.Publisher(self.name+'/scaling1', Int64, queue_size=1)

        # subscribe to data topic (this gathers the values that need to be encrypted)
        #rospy.Subscriber(self.name+'/key',  Int64MultiArray)


    def mu_dec(self, data):
        self.mu_dec = data.data[0]

        if self.o == 1:
            self.mu_dec = 0
            self.o = 2
        


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

            if int(sys.argv[1]) == 1: 
                self.d = 0.85

            vvv=np.zeros((robots,1))
            for i in range(robots):
                vvv[i] = z_values[i][0]**(-1)

            z_values = np.append(z_values, vvv, axis=1)


            z_values[:, 0] = z_values[:, 0] - self.d #substract desired distance from first column of array

            import copy
            z_values_old_noscal = copy.deepcopy(z_values)
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

            while np.any(abs(z_values[:, 1])<=10):
                
                #z_values = (np.ma.masked_where(abs(z_values) > 100, z_values)*10).data
                
                z_values[:, 1] *= 10

                scal2 *= 10 
            
            while np.any(abs(z_values[:, 2])<=10):
                
                #z_values = (np.ma.masked_where(abs(z_values) > 100, z_values)*10).data
                
                z_values[:, 2] *= 10

                scal3 *= 10 
            
            while np.any(abs(z_values[:, 3])<=100):
                
                #z_values = (np.ma.masked_where(abs(z_values) > 100, z_values)*10).data
                
                z_values[:, 3] *= 10

                scal4 *= 10 

            #low = abs(z_values) == 100 #This is to take away values that are too small
            
            z_values[low] = 0 #Make sure this value is 0 in all codes

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

            
            '''for i in range(robots): #Encryption of error (z-d)

                m = [z_values[i][0]] 

                ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, m) 

                [res.append(ciphertext[i]) for i in range(0, len(ciphertext))] #Enc2 outputs a large list of lists, this appends every list individually'''


            for i in range(robots): #Encryption of error (z-d) in Enc1

                m = [z_values[i][0]] 

                ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, m) 

                res.append(ciphertext[0])


            error_topub = pymorph.prep_pub_ros(self.q, self.N, res)

            if int(sys.argv[1]) == 1: #initalize mu
                mu = [0] 
            if int(sys.argv[1]) == 2:
                mu = [0] 
            if int(sys.argv[1]) == 3:
                mu = [0] 
            if int(sys.argv[1]) == 4:
                mu = [0] 
            
            mu = 2*(z_values[0][0]-int(self.mu_dec))

            ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, [mu])

            mu_topub = pymorph.prep_pub_ros(self.q, self.N, ciphertext)


            self.now = np.float64([rospy.get_time()])
            self.DT = 0.1#(self.now-self.old)
            print ("z_values[0][0]"" : "+str(z_values[0][0]))
            #print ("mu_dec_moded : "+str(int(self.mu_dec*scal1)))
            print ("mu_dec : "+str(self.mu_dec))
            print ("mu_hat_dot : "+str(mu))

            #mu = int(mu*self.DT*100) #add a min value where it's rounded to 0

            print ("self.DT : "+str(self.DT))
            print ("scal1 : "+str(scal1))
            #print ("mu_hat_dot_by_DT : "+str(mu))




            #res.append(ciphertext[0])



            '''for i in range(robots): #Encryption of x distance?

                m = [z_values[i][1]] 
                
                ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, m)

                res.append(ciphertext[0])


            for i in range(robots): #Encryption of y distance?

                m = [z_values[i][2]] 
                
                ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, m)

                res.append(ciphertext[0])'''

            res = []
            
            for i in range(robots): #Encryption of reciprocal of z

                m = [z_values[i][3]] 

                ciphertext = pymorph.enc_2(self.p, self.L, self.q, self.r, self.N, self.sk, m) 

                [res.append(ciphertext[i]) for i in range(0, len(ciphertext))] #Enc2 outputs a large list of lists, this appends every list individually

            rec_z_topub = pymorph.prep_pub_ros(self.q, self.N, res)
            


            s_1 = 2**12

            G= [int(round(self.DT*scal1))]

            #FG_s = [[1, G]] 

            #FG_s_enc =  pymorph.enc_2_mat(self.p, self.L, self.q, self.r, self.N, self.sk, FG_s) #should make a function in pymomorphic to send and recover shapes of matrices through ros
            
            #ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, G) #check that reencrypting this all the time is not a problem

            #res.append(ciphertext[0])


            x_y = [[row[i] for row in z_values] for i in range(1,3)] #[[x1,x2,x3],[y1,y2,y3]]

            x_y_enc = pymorph.enc_2_mat(self.p, self.L, self.q, self.r, self.N, self.sk, x_y)

            x_y_enc_topub = pymorph.prep_pub_ros(self.q, self.N, x_y_enc)

            #add line to append to res

            #I NEED TO ENCRYPT THE FG_ENC MATRIX HERE TO THEN BE ABLE TO MULTIPLY TO OBTAIN MU IN THE CONTROLLER

            #I ALSO NEED TO DECIDE IF I CAN SUBSTRACT THE MU_HAT AFTER ENCRYPTING THE ERROR, OR SHOULD I ENCRYPT IT BEFORE

            #ALSO IF AND HOW I CAN IMPLEMENT THE "SPLITM" FUNCTION FROM MATLAB HERE (I THINK I JUST NEED TO ENCRYPT FG)

            #ADD A ENC1 TO ENC2 AND VICEVERSA TO PYMOMORPHIC?

            FG = [[1, 0.1]]
            FG_s = [[1, int(self.DT*100)]]
            
            '''m = [z_values[3]]

            #m=[int(2)* self.scal]

            ciphertext = pymorph.enc_2(self.p, self.L, self.q, self.r, self.N, self.sk, m) #self.Enc2(m)

            [res.append(ciphertext[i]) for i in range(0, len(ciphertext))] 
            #res[0].append(ciphertext)'''

            
            #flat_list = [item for sublist in res for item in sublist]


            #self.tosend
            
            '''self.tosend.layout.dim.append(MultiArrayDimension())
            self.tosend.layout.dim.append(MultiArrayDimension())
            self.tosend.layout.dim[0].label = "number of vectors"
            self.tosend.layout.dim[0].size = len(res) #-2 adjusts for the addition of the mu and G
            self.tosend.layout.dim[1].label = "length"
            self.tosend.layout.dim[1].size= len(res[0])'''
            #self.tosend.layout.dim= [3, 6]
            
            #print self.tosend

            self.tosend = Int64MultiArray() #store list into array that can be published for ROS
            self.tosend.data = error_topub #np.concatenate(res) #res
            
            #str_err = pymorph.prep_pub_ros_str(error_topub)
            #print ("error_topub : "+str_err)
            self.pub_e.publish(self.tosend)
            #self.pub_e.publish(String(str_err))

            #print ("z_topub : "+str(rec_z_topub))
            self.tosend_z = Int64MultiArray() #store list into array that can be published for ROS
            self.tosend_z.data = rec_z_topub 
            self.pub_z.publish(self.tosend_z)
            
            #print ("MU_topub : "+str(mu_topub))
            self.tosend_mu = Int64MultiArray() #store list into array that can be published for ROS
            self.tosend_mu.data = mu_topub 
            self.pub_mu.publish(self.tosend_mu)

            self.tosend_xy = Int64MultiArray() #store list into array that can be published for ROS
            self.tosend_xy.data = x_y_enc_topub 
            self.pub_xy.publish(self.tosend_xy)

            
            print "data from Nexus: "+str(int(sys.argv[1]))
            print scaling
            self.pub_scal.publish(scaling)

            self.pub_scal1.publish(scal1)


            print ("z_values" + str(z_values))
            print ("z_values no scale" + str(z_values_old_noscal))

            #print (z_values[0]*z_values[1]*z_values[3])

            #print (z_values[0]*z_values[1]*z_values[3])/(self.scal*10000)

            #self.pub.publish(np.asarray(res[0]))

            self.now = np.float64([rospy.get_time()])
            self.time = np.float64([self.now-self.begin])
            self.time_log = np.append(self.time_log, self.time)
            self.old = self.now
		
            self.E_log.append(z_values[:, 0] - 0.8)

        else:
            self.shutdown()



    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Encryption_"+str(int(sys.argv[1]))+"...")
        self.running = False

        rospack = rospkg.RosPack()
        #PATH = os.path.dirname(os.path.abspath(__file__)) #This gets path from wherever the terminal used to launch the file was
        PATH = rospack.get_path('nexhom') #This gets the path from the shown ROS package
        FILEPATH = os.path.join(PATH+'/saved_variables', 'err'+self.name+'.csv')
        with open(FILEPATH, "w") as output:
                
            np.savetxt("err"+self.name+".csv", (np.asarray(self.E_log)), delimiter=",")
                       
            self.writer = csv.writer(output, delimiter=',')
            self.writer.writerow([self.E_log])
        
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('enc_'+str(int(sys.argv[1])), anonymous=False)
    hom_encrypt()
    print "Encryption Working"
    rospy.spin() 

