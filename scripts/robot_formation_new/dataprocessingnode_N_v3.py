#!/usr/bin/env python

from __future__ import division

import sys
import os

import rospy
import rospkg
from std_msgs.msg import Int32, Float32MultiArray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan

from math import *
import numpy as np


class Determine_z_values:
    ''' Determines the inter-agent distance values and publishes it to z_values topic

    In the init state (k=0) the robots are assumed to be in there approximate initial position.
    After recognizing, they are assigned their proper robot number and tracking is started.

    The proper number derived from there topology, see the picture in the thesis of Johan Siemonsma.
    See the code at the and if the initial state code.

    In the tracking state the z_values are published.
    
    Laser scanner angle zero is in the backward direction wrt the robot as used here.
    We use angles (0:360) as follows:

    '''
  
    def __init__(self):
        ''' Initiate self and subscribe to /scan topic '''
        
        # Get input argument and save it as a string (e.g. n_1)
        self.name = 'n_'+str(int(sys.argv[1]))

        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])
        self.time_log = np.array([])
        self.begin = np.float64([rospy.get_time()])

        self.E1_log = []
        self.E2_log = []

        # Desired distance - used for sending if no z is found or if the dataprocessingnode is shutdown: 
        # robot not influenced if one z not found
        self.d = np.float32(0.8)

        if int(sys.argv[1]) == 1: #Used to set an agent with a constant mismatch (for estimator)
            self.d = np.float32(0.85)

        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d))) #calculate diagonal for square formation

        # set min and max values for filtering ranges in meters during initiation 
        self.min_range = 0.25
        self.max_range = 1.5

        # Prepare shutdown
        rospy.on_shutdown(self.shutdown)
                        
        # Prepare publishers
        self.pub_z_val = rospy.Publisher(self.name +'/z_values', numpy_msg(Float32MultiArray), queue_size=1) #Publish the distances and angles to neighboring agents
        self.pub_n = rospy.Publisher(self.name +'/agents', Int32 , queue_size=1) #Publish the amount of neighboring agents
        
        # Prepare subscribers
        rospy.Subscriber('/'+self.name+'/hokuyo_points', LaserScan, callback = self.calculate_z)

       
        np.set_printoptions(precision=2) #Limit amount of decimals in numpy console prints
    
    def calculate_z(self, msg):
        ''' Calculate the z_values from the scan data '''       
        
        if not rospy.is_shutdown():
            
            rospy.loginfo('-----------------------------------------')
            rospy.loginfo("Data from Nexus: "+str(int(sys.argv[1])))

            self.ranges= np.asarray(msg.ranges)
                 
            # Save the angles (hits) of the robots in seperate arrays
            z_a = np.where((self.ranges >= self.min_range) & (self.ranges <= self.max_range))[0] #The zero at the end is to access the first value of the tuple created by "np.where", so z_a is just an array

            n = 1
                
            for i in range(len(z_a)-1):  # Calculate the number of neighboring robots

                if (z_a[i] - z_a[i + 1] >= -10):
                    
                    continue
                
                elif (-10 <= z_a[i] - z_a[i-1] <= 10):
                    
                    n = n + 1   
                
            rospy.loginfo("Detected number of neighbors: "+str(n))

            R=[]
            r=np.array([])
            P = 0
		
            # Compares difference between angles in z_a to decide if it's a new robot, and if it is it creates and array r, which is then added to the list R
            for i in range(P,len(z_a)-1):
                
                if (z_a[i] - z_a[i + 1] >= -10) and i!=(len((z_a))-2):
                    
                    r=np.append(r,z_a[i])
                    #print(r)
                     
                elif (-10 <= z_a[i] - z_a[i - 1] <= 10):
                    r=np.append(r,z_a[i])
                    R.append(r)
                    P = 1+i
                    #print(P)
                    r=np.array([])

                elif (z_a[i] - z_a[i + 1] >= -10) and i==(len((z_a))-2):
                    r=np.append(r,z_a[i])
                    #print(r)
                    R.append(r)
                    #R[:][:]=np.int_(R[:][:])
                    #print R
                    
            for i in range(n):  #transform list R to array of integers
                R[i]=R[i].astype(int)
                #print R
            
            if z_a[0]==0 and z_a[-1]==359:  #This loop makes sure that a robot is not read twice if it is located at the scan starting point
                R[0]= np.append(R[0],R[-1])
                del R[-1]
                n=n-1
            
            
            k = 2 
                
            self.z_aX=np.zeros([len(R)])
            self.zn_X=np.zeros([len(R)])
            self.z_a_min=np.zeros([len(R)])
            self.z_a_max=np.zeros([len(R)])
            self.zn_min=np.zeros([len(R)])
            self.zn_max=np.zeros([len(R)])

            for j in range(0,n):
                
                if R[j] != np.array([]):
                    
                    self.z_aX[j] = int(np.round((R[j]).mean()))
        
                    self.zn_X[j] = np.float32(np.min(self.ranges[(R[j][0:(len(R[j]))])]))  # + 0.098;
                    
                    # Tracking variables

                    self.z_a_min[j] = np.min(np.int_(R[j][0:len(R[j])]))

                    self.z_a_max[j] = np.max(np.int_(R[j][0:len(R[j])]))

                    self.zn_min[j] = np.min(np.float32(self.ranges[np.int_(R[j][0:len(R[j])])]))

                    self.zn_max[j] = np.max(np.float32(self.ranges[np.int_(R[j][0:len(R[j])])]))

                    #fprintf('\nNexus: zn_1=%f', self.zn_(j))

                else:

                    self.z_aX[j] = 0

                    self.zn_X[j] = self.d

                    rospy.loginfo('Nexus = not found')
            
            self.z_aX=self.z_aX.astype(int)  #transform z_aX to an array of integers

            rospy.loginfo('Distance to agents = '+ str(self.zn_X))
            rospy.loginfo('Angle to agents    = '+ str(self.z_aX))

            Zval=[]
            self.zx=np.zeros([len(R)])
            self.zy=np.zeros([len(R)])
            self.z_values=[] #np.zeros([3*len(R)])

            for i in range(0,n): #FOR SOME REASON i thought this should go to n-1)

                self.zx[i] = np.float32(np.cos((self.z_aX[i]-np.int_(180))*2*np.pi/360)*self.zn_X[i])

                self.zy[i] = np.float32(np.sin((self.z_aX[i]-np.int_(180))*2*np.pi/360)*self.zn_X[i])

                Zval.append([self.zn_X[i], self.zx[i], self.zy[i]])
            
            '''for i in range(0,n):  #Loop to print the x and y distances of detected agents
                
                print('zx_[',i,'] = ', self.zx[i])
                print('zy_[',i,'] = ', self.zy[i])
                print('--')'''
            
            
        

            #print(Zval)
            #self.z_values=[]
            for i in range(0,n): #This loop puts all the values into the z_values matrix

                self.z_values= np.concatenate(Zval[:][:])

            self.z_values=np.asarray(self.z_values, dtype=np.float32)  #Ensure that all numbers are type np_float (without it the publisher doesn't publish the proper values)
            #rospy.loginfo("Z_values as Published: "+str(self.z_values))

            #Example of z_values shape:
            #self.z_values = np.array([self.zn_X[0], self.zx[0], self.zy[0], \
            #                          self.zn_X[1], self.zx[1], self.zy[1], \
            #                          self.zn_X[2], self.zx[2], self.zy[2]], dtype=np.float32)

            #Set z_values into Float32MultiArray so ROS may publish it
            self.z_values_multiarray = Float32MultiArray()
            self.z_values_multiarray.data = self.z_values

            #Publish data
            self.pub_n.publish(n)
            self.pub_z_val.publish(self.z_values_multiarray)

            #Logs and Time variables
            self.now = np.float64([rospy.get_time()])
            self.time = np.float64([self.now-self.begin])
            self.time_log = np.append(self.time_log, self.time)
            self.E1_log.append(self.z_values[0]-0.8)
            #self.E2_log.append(self.z_values[3]-0.8)
            rospy.loginfo("Time between runs: " + str(self.now-self.old) + "\n")
            self.old = self.now

        else:
            self.shutdown()
            

    
    def shutdown(self):
        """ Setting z = d in order to stop the robots from moving when shutting down the dataprocessingnode """

        rospy.loginfo("Stopping dataprocessingnode_"+str(int(sys.argv[1]))+"...")

        #self.z_values = np.array([self.d, self.d, 0, \
        #                          self.dd, self.dd, 0, \
        #                          self.d, self.d, 0], dtype=np.float32)

        self.z_values = np.array([self.d, self.d, 0, \
                                  self.d, self.d, 0], dtype=np.float32)

        self.z_values_multiarray = Float32MultiArray()
        self.z_values_multiarray.data = self.z_values

        self.pub_z_val.publish(self.z_values_multiarray)

        rospy.loginfo('Shutting Down')
        rospy.sleep(1)

if __name__ == '__main__':

    try:
        rospy.init_node('Dataprocessingnode_'+str(int(sys.argv[1])), anonymous=False)
        r = rospy.Rate(10)
        Determine_z_values()
        rospy.spin() 
    except rospy.ROSInterruptException:
        rospy.loginfo("Dataprocessingnode_"+str(int(sys.argv[1]))+" terminated.")
        pass


# Change for sim:
# rospy.Subscriber('/nexus1/scan', numpy_msg(Float32), self.calculate_z)
# def calculate_z(self, data):
#   self.ranges= data.data
