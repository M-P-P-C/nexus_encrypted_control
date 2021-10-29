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

from pymomorphic3 import pymomorphic_py2 as pymh2

import copy
        

#CHECK A ROBOT INDIVIDUALLY FOR ERRORS IN CALCULATIONS DUE TO LOST MEMORY USING PYMOMORPHIC

#REDUCE USE OF LONG?

class hom_encrypt:
    '''This encrypts the data recieved and outputs the homomorphically encrypted data in form of a matrix'''
        
    def __init__(self):
        
        # Get input argument and save it as a string (e.g. n_1)
        self.name='n_'+str(int(sys.argv[1])) 

        #Define desired distance for agents
        self.d = 0.8
        if int(sys.argv[1]) == 1: 
            self.d = 0.8 #Change desired distance for an agent for estimator

        # Initialize encryption
        self.my_key = pymh2.KEY(p = 10**13, L = 10**4, r=10**1, N = 90)


        self.DUMDUM = 1
        self.o=1

        # Initialize some variables
        self.mu_dec = 0
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
        self.E1_log = []
        self.E2_log = []
        self.z_log = []
        self.DT = self.now-self.old

        # Prepare shutdown
        rospy.on_shutdown(self.shutdown)
        
        # Prepare publishers
        self.pub_e = rospy.Publisher(self.name+'/enc_error', String, queue_size=1)
        self.pub_z = rospy.Publisher(self.name+'/enc_rec_z', String, queue_size=1)
        self.pub_mu = rospy.Publisher(self.name+'/enc_mu', String, queue_size=1)
        self.pub_xy = rospy.Publisher(self.name+'/enc_x_and_y', String, queue_size=1)
        self.pub_scal = rospy.Publisher(self.name+'/scaling', Int64, queue_size=1)
        self.pub_scal1 = rospy.Publisher(self.name+'/scaling1', Int64, queue_size=1)

        # Prepare subscribers
        rospy.Subscriber(self.name+'/mu_hat_dec', Floats, callback = self.mu_dec_callback)
        rospy.Subscriber(self.name+'/z_values', numpy_msg(Floats), callback = self.encrypt_callback, queue_size=1)

        #ADD PUBLISHERS FOR FG_s AND ZEROS_MATRIX


    def mu_dec_callback(self, data):

        while not rospy.is_shutdown():
            self.mu_dec = data.data[0]

            if self.o == 1:
                self.mu_dec = 0
                self.o = 2
        
        else:
            self.shutdown()

    def encrypt_callback(self, data):

        if not rospy.is_shutdown():

            #print("--------------------------------------------------")
            #rospy.loginfo("Encryption of Nexus: "+str(int(sys.argv[1])))
            
            z_values = data.data.copy()

            robots = len(z_values)/3

            z_values = z_values.reshape((robots, 3))

            '''laser_data=[]
            for i in range(0, len(encr), len(z_values)):
                laser_data.append(encr[i:sizevec+i])'''
            
 
            z_reciprocal=np.zeros((robots,1))
            for i in range(robots):
                z_reciprocal[i] = z_values[i][0]**(-1)

            z_values = np.append(z_values, z_reciprocal, axis=1)


            z_values[:, 0] = z_values[:, 0] - self.d #substract desired distance from first column of array ##THIS LINE SHOHULD B UNCOMMENTED
            #print(z_values[:, 0] - self.d)

            z_values_old_noscal = copy.deepcopy(z_values)
            
            #print (z_values[0]*z_values[1]*z_values[3])

            #z_values = z_values * self.scal
            #print z_values

            #specify the desired significant figures of each variables
            scal1 = 4 #scaling of error
            scal2 = 2 #scaling of x
            scal3 = scal2 #scaling of y must be same as x
            scal4 = 3 #scaling of reciprocal of z

            low = abs(z_values) < 0.00001 #This is to take away values that are too small (values below approc 0.01 cause jitter)
            
            z_values[low] = 0 #Make sure low values are 0

            scal1 = pymorph.log_scaling(z_values[:, 0], scal1)
            scal2 = pymorph.log_scaling(z_values[:, 1], scal2)
            scal3 = pymorph.log_scaling(z_values[:, 2], scal3)
            scal4 = pymorph.log_scaling(z_values[:, 3], scal4)

            z_values[:, 0] = scal1[0]
            z_values[:, 1] = scal2[0]
            z_values[:, 2] = scal3[0]
            z_values[:, 3] = scal4[0]


            #print(z_values[:, 0])

            #z_values[:, 0] = z_values[:, 0] - int(self.d*(scal1[1])) #TRYING TO SCALE THE DESIRED DISTANCE TO AVOID HAVING TO SCALE THE ERROR AS IT GETS CLOSE TO 0

            #print(z_values[:, 0])

            '''
            while np.any(abs(z_values[:, 0])<=100):

                #z_values = (np.ma.masked_where(abs(z_values) > 100, z_values)*10).data
                
                z_values[:, 0] *= 10

                scal1 *= 10 

                print z_values[:, 0]

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
            '''
            #low = abs(z_values) == 100 #This is to take away values that are too small
            
            #z_values[low] = 0 #Make sure this value is 0 in all codes


            scal5=scal2

            z_values = z_values.astype(int) #Turn all values into integers before encrypting otherwise it causes innacuracy

            #scal1 = 10**scal1
            #scal2 = 10**scal2
            #scal3 = 10**scal3
            #scal4 = 10**scal4
            #scal5 = 10**scal5

            scal1 = scal1[1]
            scal2 = scal2[1]
            scal3 = scal3[1]
            scal4 = scal4[1]
            scal5 = scal5[1]

            scaling = scal1*scal4*scal5

            
            #print "Total Scaling: "+str(scaling)
            self.pub_scal.publish(scaling)

            self.pub_scal1.publish(scal1)

            #print ("z_values no scale" + str(z_values_old_noscal))
            #print ("z_values" + str(z_values))
            
            
            

            if int(sys.argv[1]) == 1: #initalize mu
                mu = [int(0)] 
            if int(sys.argv[1]) == 2:
                mu = [int(0)] 
            if int(sys.argv[1]) == 3:
                mu = [int(0)] 
            if int(sys.argv[1]) == 4:
                mu = [int(0)] 

            #print ("mu_dec BEFORE : "+str(self.mu_dec))
            #print ("mu_dec BEFORE Round: "+str(round(self.mu_dec)))
            #print ("mu_dec BEFORE Round and INT: "+str(round(self.mu_dec)))   \

            #print ("Expected Estimator Velocity: "+str([self.mu_dec*z_values[0][1]*z_values[0][3], self.mu_dec*z_values[0][2]*z_values[0][3]]))             
            
            mu = 2*(z_values[0][0]-int(round(self.mu_dec)))


            if self.mu_dec==0: #avoids starting mu without first starting the sim
                mu= 0
                
            if self.mu_dec > 100: #Bounding mu_dec to avoid large fluctuations
                self.mu_dec = 100

            if self.mu_dec < -100:
                self.mu_dec = -100
            

            mu_ciph = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, [mu])

            mu_topub = pymorph.prep_pub_ros_str(mu_ciph)
            self.pub_mu.publish(String(mu_topub))

            #self.now = np.float64([rospy.get_time()])
            #self.DT = 0.1#(self.now-self.old)
            
            #print ("z_values[0][0]"" : "+str(z_values[0][0]))
            #print ("mu_dec_moded : "+str(int(self.mu_dec*scal1)))
            #print ("mu_dec AFTER : "+str(self.mu_dec))
            #print ("mu : "+str(mu))
            #print ("mu DIFFERENCE : "+str(mu-self.mu_dec))

            #mu = int(mu*self.DT*100) #add a min value where it's rounded to 0

            #print ("self.DT : "+str(self.DT))
            #print ("scal1 : "+str(scal1))
            #print ("mu_hat_dot_by_DT : "+str(mu))

            #z_values, scaling = pymorph.smart_scaling(z_values, 100) #CHECK WHY THE SMART SCALING FUNCTION IS NOT WORKING PROPERLY


            '''for i in range(robots): #Encryption of error (z-d)

                m = [z_values[i][0]] 

                ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, m) 

                [res.append(ciphertext[i]) for i in range(0, len(ciphertext))] #Enc2 outputs a large list of lists, this appends every list individually'''

            err_ciph = []

            for i in range(robots): #Encryption of error (z-d) in Enc1

                m = [z_values[i][0]] 

                ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, m) 

                err_ciph.append(ciphertext[0])

            error_topub = pymorph.prep_pub_ros_str(err_ciph)

            self.pub_e.publish(String(error_topub))





            zr_ciph = [[]]*robots
            
            for i in range(robots): #Encryption of reciprocal of z

                m = [z_values[i][3]] 

                zr_ciph[i] = pymorph.enc_2(self.p, self.L, self.q, self.r, self.N, self.sk, m) 
                
                #[res.append(ciphertext[i]) for i in range(0, len(ciphertext))] #Enc2 outputs a large list of lists, this appends every list individually

            rec_z_topub = pymorph.prep_pub_ros_str(zr_ciph)

            self.pub_z.publish(String(rec_z_topub))
            


            #s_1 = 2**12

            #G= [int(round(self.DT*scal1))]

            #FG_s = [[1, G]] 

            #FG_s_enc =  pymorph.enc_2_mat(self.p, self.L, self.q, self.r, self.N, self.sk, FG_s) #should make a function in pymomorphic to send and recover shapes of matrices through ros
            
            #ciphertext = pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, G) #check that reencrypting this all the time is not a problem

            #res.append(ciphertext[0])


            x_y = [[row[i] for row in z_values] for i in range(1,3)] #[[x1,x2,x3],[y1,y2,y3]]

            x_y_enc = pymorph.enc_2_mat(self.p, self.L, self.q, self.r, self.N, self.sk, x_y)

            x_y_enc_topub = pymorph.prep_pub_ros_str(x_y_enc)
            
            self.pub_xy.publish(String(x_y_enc_topub))
            

            #add line to append to res

            #I NEED TO ENCRYPT THE FG_ENC MATRIX HERE TO THEN BE ABLE TO MULTIPLY TO OBTAIN MU IN THE CONTROLLER

            #I ALSO NEED TO DECIDE IF I CAN SUBSTRACT THE MU_HAT AFTER ENCRYPTING THE ERROR, OR SHOULD I ENCRYPT IT BEFORE

            #ALSO IF AND HOW I CAN IMPLEMENT THE "SPLITM" FUNCTION FROM MATLAB HERE (I THINK I JUST NEED TO ENCRYPT FG)

            #ADD A ENC1 TO ENC2 AND VICEVERSA TO PYMOMORPHIC?

            #FG = [[1, 0.1]]
            #FG_s = [[1, int(self.DT*100)]]
            
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

            #self.tosend = Int64MultiArray() #store list into array that can be published for ROS
            #self.tosend.data = error_topub #np.concatenate(res) #res
            
            
            #print ("error_topub : "+str_err)
            #self.pub_e.publish(self.tosend)



            #print ("z_topub : "+str(rec_z_topub))
            #self.tosend_z = Int64MultiArray() #store list into array that can be published for ROS
            #self.tosend_z.data = rec_z_topub 
            #self.pub_z.publish(self.tosend_z)


            
            #print ("MU_topub : "+str(mu_topub))
            #self.tosend_mu = Int64MultiArray() #store list into array that can be published for ROS
            #self.tosend_mu.data = mu_topub 
            #self.pub_mu.publish(self.tosend_mu)
            

            #self.tosend_xy = Int64MultiArray() #store list into array that can be published for ROS
            #self.tosend_xy.data = x_y_enc_topub 
            #self.pub_xy.publish(self.tosend_xy)
            

            




            #print (z_values[0]*z_values[1]*z_values[3])

            #print (z_values[0]*z_values[1]*z_values[3])/(self.scal*10000)

            #self.pub.publish(np.asarray(res[0]))

            self.now = np.float64([rospy.get_time()])
            '''self.time = np.float64([self.now-self.begin])
            self.time_log = np.append(self.time_log, self.time)'''
            print_time= self.now-self.old
            print("ACTUAL TIME BETWEEN RUNS: " + str(print_time)) #Creates stuttering in the printing on the console
            self.old = self.now
		
            '''self.E1_log.append(z_values_old_noscal[0][0])
            self.E2_log.append(z_values_old_noscal[1][0])'''
            #self.z_log.append(list(z_values_old_noscal[:, 0]+0.8))

            '''rospack = rospkg.RosPack()
            PATH = rospack.get_path('nexhom') #This gets the path from the shown ROS package
            FILEPATH = os.path.join(PATH+'/saved_variables', 'Encrypter'+self.name+'.csv')
            with open(FILEPATH, "a") as output:
                
                #np.savetxt("err"+self.name+".csv", (np.asarray(self.E_log)), delimiter=",")
                       
                self.writer = csv.writer(output, delimiter=',')
                self.writer.writerow(list(z_values_old_noscal[:, 0]))
                self.writer.writerow(list(z_values_old_noscal[:, 0]+0.8))
                #self.writer.writerow([self.z_log[1]])
                self.writer.writerow(self.time)'''

        else:
            self.shutdown()



    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Encryption_"+str(int(sys.argv[1]))+"...")

        '''rospack = rospkg.RosPack()
        PATH = rospack.get_path('nexhom') #This gets the path from the shown ROS package
        FILEPATH = os.path.join(PATH+'/saved_variables', 'Encryption_'+self.name+'.csv')

        if self.DUMDUM == 1:
            self.DUMDUM=2
            with open(FILEPATH, "a") as output:
                
                #np.savetxt("err"+self.name+".csv", (np.asarray(self.E_log)), delimiter=",")
                        
                self.writer = csv.writer(output, delimiter=',')
                self.writer.writerow(self.E1_log)
                self.writer.writerow(self.E2_log)
                #self.writer.writerow(list(z_values_old_noscal[:, 0]+0.8))
                #self.writer.writerow([self.z_log[1]])
                self.writer.writerow(self.time_log)'''


        
        rospy.sleep(1)


if __name__ == '__main__':
    rospy.init_node('enc_'+str(int(sys.argv[1])), anonymous=False)
    r = rospy.Rate(10)
    hom_encrypt()
    print("Encryption Working")
    rospy.spin() 

