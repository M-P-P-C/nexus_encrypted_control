#!/usr/bin/env python  
import sys 
import rospy
import matplotlib.pyplot as pl
import numpy as np
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int32, Int64MultiArray

from std_msgs.msg import MultiArrayDimension


from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist

import rospkg
import csv #to read private key from csv file

from pymomorphic import pymorph


class controller:
    ''' The controller uses the interagent distances to determine the desired velocity of the Nexus '''

    ''' NOTE: this script requires the dataprocessing node to be running first, as well as an input
        argument, an example of how to properly run this code in the terminal is as following:
        
        rosrun lasmulticontrol3 dataprocessingnode_N.py "1" 
        
        where "1" is the value assigned to the robot
        '''
    
    def __init__(self):
        ''' Initiate self and subscribe to /z_values topic '''
        self.name='n_'+str(int(sys.argv[1]))
        # controller variables
        self.running = np.float32(1)
        self.d = int(0.8)
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        self.c = np.float32(0.5)
        self.U_old = np.array([0, 0])
        self.U_oldd = np.array([0, 0])

        # Motion parameters
        self.x_dot = np.float32(0)
        self.y_dot = np.float32(0)
        self.r_dot = np.float32(0)
        self.mu_x = self.x_dot*np.array([0, -1, 0, -1, 0])
        self.mut_x = self.x_dot*np.array([0, 1, 0, 1, 0])
        self.mu_y = self.y_dot*np.array([-1, 0, 0, 0, 1])
        self.mut_y = self.y_dot*np.array([1, 0, 0, 0, -1])
        self.mu_r = self.r_dot*np.array([-1, -1, 0, 1, -1])
        self.mut_r = self.r_dot*np.array([1, 1, 0, -1, 1])
        
        self.mu = self.mu_x+self.mu_y+self.mu_r
        self.mut = self.mut_x+self.mut_y+self.mut_r
                
        # prepare Log arrays
        self.E1_log = np.array([])
        self.E2_log = np.array([])
        self.E3_log = np.array([])
        self.E4_log = np.array([])
        self.E5_log = np.array([])
        self.E6_log = np.array([])
        self.Un = np.float32([])
        self.U_log = np.array([])
        self.time = np.float64([])
        self.time_log = np.array([])
        self.now = np.float64([rospy.get_time()])
        self.old = np.float64([rospy.get_time()])
        self.begin = np.float64([rospy.get_time()])
        self.k = 0

        self.mu_hat = [0, 0, 0]

        # prepare shutdown
        rospy.on_shutdown(self.shutdown)
        
        # prepare publisher
        self.pub = rospy.Publisher(self.name+'/cmd_vel_enc', Int64MultiArray, queue_size=1)
        self.velocity = Twist()
                
        # subscribe to z_values topic
        rospy.Subscriber(self.name+'/encrypted_data', Int64MultiArray, self.controller)
        rospy.Subscriber(self.name+'/agents', Int32, self.n)

        # subscribe to controller_variables
        rospy.Subscriber('/controller_variables', numpy_msg(Floats), self.update_controller_variables)

    def n(self, data):   #This is used to extract the value of n (i.e. the number of robots the agent detected, published from the from the dataprocessor node)
        if self.running < 10:
            self.n=data.data
       
        elif 10 < self.running < 1000:
            self.shutdown()

    def update_controller_variables(self, data):
        ''' Update controller variables '''
        if self.running < 10:
            # Assign data 
            self.controller_variables = data.data

            # Safe variables
            self.running = np.float32(self.controller_variables[0])
            self.d = np.float32(self.controller_variables[1])
            self.dd = np.float32(self.controller_variables[2])
            self.c = np.float32(self.controller_variables[3])
            self.x_dot = np.float32(self.controller_variables[4])
            self.y_dot = np.float32(self.controller_variables[5])
            self.r_dot = np.float32(self.controller_variables[6])
            
            # Calculate mu
            self.mu_x = self.x_dot*np.array([0, -1, 0, -1, 0])
            self.mut_x = self.x_dot*np.array([0, 1, 0, 1, 0])
            self.mu_y = self.y_dot*np.array([-1, 0, 0, 0, 1])
            self.mut_y = self.y_dot*np.array([1, 0, 0, 0, -1])
            self.mu_r = self.r_dot*np.array([-1, -1, 0, 1, -1])
            self.mut_r = self.r_dot*np.array([1, 1, 0, -1, 1])
            
            self.mu = self.mu_x+self.mu_y+self.mu_r
            self.mut = self.mut_x+self.mut_y+self.mut_r
        
    def controller(self, data):
        ''' Calculate U based on z_values and save error velocity in log arrays '''    
        if self.running < 10:
            # Input for controller
            z_values= data.data
            sizevec = data.layout.dim[1].size #size of each vector

            var = pymorph.variables_import() #variables used for encryption

            Bx=[]
            By=[]
            D = []
            E = []
            Ed=[]
            E_enc1=[] 

            cc=[]
            for i in range(0, len(z_values), sizevec):
                cc.append(list(z_values[i:sizevec+i]))
            
            sp = data.layout.dim[0].size - self.n*2
            datalen = sp/(self.n*2)
            for i in range(0, self.n): #This loop gathers the encrypted data from the subscribed 1-dimensional array and reconstructs it into the desired matrices
                Bx.append(cc[sp/2+i])#+3*i])

                By.append(cc[(sp/2)+self.n+i])#+3*i])

                D.append(cc[((datalen*self.n+datalen*i)+self.n*2):(datalen*self.n+datalen*(i+1))+self.n*2])

                E.append(cc[datalen*i:datalen*(i+1)])

            E_enc1.append(cc[-2])
            E_enc1.append(cc[-1])

            sk = pymorph.key_import(int(sys.argv[1])) #get secret key
            
            self.now = np.float64([rospy.get_time()])
            self.DT = (self.now-self.old) * 100

            #self.mu_hat = cc[data.layout.dim[0].size]
            Q = cc[data.layout.dim[0].size+1] #This is the gain constant used when multiplying in the control law 
            S1bDz_x1 = pymorph.hom_mul(var.q, [Bx[0]], D[0])
            S1bDz_y1 = pymorph.hom_mul(var.q, [By[0]], D[0]) #I cannot get the abolute value here (if that doesn't work I will have to calculate it before encryptng)

            print (float(pymorph.dec_hom(var.p, var.L, sk, [self.mu_hat])[0][0])/100000000000)/(2*12)
        
            #mu_hat_dot = 2*(Ed[0][0] - self.mu_hat) #I need to encrypt the Error in ENC1 form to substract

            FG_s = [[1, 41]] 

            FG_s_enc =  pymorph.enc_2_mat(var.p, var.L, var.q, var.r, var.N, sk, FG_s)
            
            import operator
            map_object = map(operator.sub, E_enc1[0],  self.mu_hat)
            de = list(map_object)
            de = [i*2 for i in de]
            #de = 2 * (E_enc1[0] - mu_hat)  #previously "mu_hat_dot"

            mu_hat_dot = [int(i*self.DT) for i in de]
            
            #self.mu_hat = map(operator.add, self.mu_hat, mu_hat_dot)

            self.mu_hat = pymorph.hom_mul_mat(var.q, var.N, FG_s_enc, [self.mu_hat, mu_hat_dot])[0]

            print(self.mu_hat)

            #self.mu_hat = pymorph.hom_mul_mat()
            
            #print float(pymorph.dec_hom(var.p, var.L, sk, self.mu_hat))/100000000000

            #self.mu_hat = self.mu_hat + mu_hat_dot  #This variable remains in memory and is always encrypted

            #self.DT IS USUALLY 0.1, Should I encrypt it beforehand and send it or t



            #S1bDz = S1bDz.reshape(2, 1)


            
            #print "Bx=", Bx
            #print "By=", By
            #print "D=", D
            #print "E=", E
            #print "Ed=", Ed

            # Formation shape control
            BbDz = [list(a) for a in zip(Bx, By)]
	    #print "BbDz= ", BbDz
            #Dzt=np.diag(D)
            Dzt=D
	    #print "Dzt= ", Dzt
	    #print "c= ", self.c
            #Ed=np.asarray(Ed)
            Ed=E
	    #print "Ed= ", Ed

            # print "error = ", Ed
            # print "z_values = ", z_values
            
            # Formation motion control
            #Ab = np.array([[self.mu[0], 0, self.mu[3], 0], \
                                #[0, self.mu[0], 0, self.mu[3]]])
            #z = np.array([z_values[7], z_values[8], z_values[1], z_values[2]])
            # z = [ edge 1 , edge 4]
            
            # Control law
            #U = self.c*BbDz.dot(Dzt).dot(Ed) #+ (Ab.dot(z)).reshape((2, 1))

            #print self.Hom_mul([self.Hom_mul([list(BbDz[0][0])], [list(Dzt.astype(int))])], [list((Ed.astype(int)))])

            #U =  self.Hom_mul([self.Hom_mul([BbDz[0][0]], Dzt[0])], Ed[0])

           

            U=[]
            Usum =[0]*sizevec
            Y=[]
            Ysum = [0]*sizevec
            from operator import add
            for i in range(self.n):
                U.append(pymorph.hom_mul(var.q, [pymorph.hom_mul(var.q, [BbDz[i][0]], Dzt[i])], Ed[i]))
                Y.append(pymorph.hom_mul(var.q, [pymorph.hom_mul(var.q, [BbDz[i][1]], Dzt[i])], Ed[i]))
                
                #pymorph.hom_mul(var.q, Q, [pymorph.hom_mul(var.q, S1bDz_x1, self.mu_hat)])
                #pymorph.hom_mul(var.q, Q, [pymorph.hom_mul(var.q, S1bDz_y1, self.mu_hat)])

                #pymorph.hom_mul(var.q, S1bDz_x1, self.mu_hat) #must turn mu_hat into Enc2
                #pymorph.hom_mul(var.q, S1bDz_y1, self.mu_hat)
                #+ 0.011*S1bDz.dot(self.mu_hat) 

                #Usum = map(add, Usum, U[i])
                #Ysum = map(add, Ysum, Y[i])

            #for i in range(self.n):
                #U[]from operator import add;map(add, , list2)
            
            if self.n == 3: #These loops add up all the inputs generated by the detected neighboring robots
                U = [sum(x) for x in zip(U[0], U[1], U[2])]
                Y = [sum(x) for x in zip(Y[0], Y[1], Y[2])]

            if self.n == 2:
                U = [sum(x) for x in zip(U[0], U[1])]
                Y = [sum(x) for x in zip(Y[0], Y[1])]

            if self.n == 1:
                U = [sum(x) for x in zip(U[0])]
                Y = [sum(x) for x in zip(Y[0])]
                




            #U =  pymorph.hom_mul(var.q, [pymorph.hom_mul(var.q, [BbDz[0][0]], Dzt[0])], Ed[0])

            #U = self.Hom_mul([BbDz[0][0]], Dzt[0])

            #Y = BbDz[0][0] #self.Hom_mul([self.Hom_mul([BbDz[0][1]], Dzt[0])], Ed[0])  

            

            Z = U+Y

            #print self.Hom_mul([self.Hom_mul([list(BbDz[0][0])], [list(Dzt.astype(int))])], [list((Ed.astype(int)))])

            #Z = [item for sublist in Z for item in sublist] #flattens list to be published by ros
            self.tosend = Int64MultiArray()
            #flat_list = [item for sublist in U for item in sublist] #ROS only works with 1D arrays
            self.tosend.data = Z
            self.tosend.layout.dim.append(MultiArrayDimension())
            self.tosend.layout.dim.append(MultiArrayDimension())
            self.tosend.layout.dim[0].label = "number of vectors"
            self.tosend.layout.dim[0].size = len(U)
            self.tosend.layout.dim[1].label = "length"
            self.tosend.layout.dim[1].size= len(U)

            self.pub.publish(self.tosend)

            self.now = np.float64([rospy.get_time()])
            self.old = self.now

                # publish
            #self.publish_control_inputs(self.tosend) #THIS IS NOT THE PUBLISHING FUNCTION
            
            A=self.n
        elif 10 < self.running < 1000:
            self.shutdown()


    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Encrypted_Controller_"+str(int(sys.argv[1]))+"...")
        self.running = np.float32(10000)
        self.velocity = Twist()
        self.tosend = Int64MultiArray()
        self.pub.publish(self.tosend)
        rospy.sleep(1)







if __name__ == '__main__':
    try:
        rospy.init_node('controller_enc_'+str(int(sys.argv[1])), anonymous=False)
        controller()
        print "Cloud Processing Working"
        rospy.spin()
    except:
        rospy.loginfo("Controller node_"+str(int(sys.argv[1]))+" terminated.")  


