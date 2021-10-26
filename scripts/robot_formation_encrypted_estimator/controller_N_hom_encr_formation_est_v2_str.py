#!/usr/bin/env python  
import sys 
import rospy
import matplotlib.pyplot as pl
import numpy as np
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int32, Int64MultiArray, String

from std_msgs.msg import MultiArrayDimension


from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist

import rospkg
import csv #to read private key from csv file

from pymomorphic import pymorph

import operator
from operator import add
import csv
import os


class controller:
    ''' The controller uses the interagent distances to determine the desired velocity of the Nexus '''

    ''' NOTE: this script requires the dataprocessing node to be running first, as well as an input
        argument, an example of how to properly run this code in the terminal is as following:
        
        rosrun lasmulticontrol3 dataprocessingnode_N.py "1" 
        
        where "1" is the value assigned to the robot
        '''
    
    def __init__(self):
        loop=0
        self.mu_hat_log =[]
        self.DT_log =[]


        var = pymorph.variables_import()

        self.p=var.p
        self.L=var.L
        self.q=var.q
        self.r=var.r
        self.N=var.N

        ''' Initiate self and subscribe to /z_values topic '''
        self.name='n_'+str(int(sys.argv[1]))
        # controller variables
        self.running = np.float32(1)
        self.d = int(0.8)
        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        self.c = np.float32(0.5)
        self.U_old = np.array([0, 0])
        self.U_oldd = np.array([0, 0])

        self.sk = pymorph.key_import(int(sys.argv[1])) #This is here for debugging purposes, the controller should not need to access decrypted data

        self.DT = 0.1 #(self.now-self.old)
        self.FG_s = [[1, int(self.DT*1000)]] #CHECK HERE 
        self.FG_s_enc =  pymorph.enc_2_mat(self.p, self.L, self.q, self.r, self.N, self.sk, self.FG_s) #THIS PROCESS SHOULD OCCUR IN THE ENCRYPTION SCRIPT

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

        self.mu_hat = [0]*(self.N+1) #initialize with N+1 size
        self.o = 1
        

        # prepare shutdown
        rospy.on_shutdown(self.shutdown)
        
        # prepare publisher
        self.pub = rospy.Publisher(self.name+'/cmd_vel_enc', String, queue_size=1)
        self.pub2 = rospy.Publisher(self.name+'/mu_hat_enc_tot', String, queue_size=1)
        self.pub3 = rospy.Publisher(self.name+'/mu_hat_enc', String, queue_size=1)

        self.velocity = Twist()
                
        # subscribe to z_values topic
        #rospy.Subscriber(self.name+'/encrypted_data', Int64MultiArray, self.controller)
        #rospy.Subscriber(self.name+'/agents', Int32, self.n)

        rospy.Subscriber(self.name+'/enc_error', String, self.recover_error)
        
        rospy.Subscriber(self.name+'/enc_rec_z', String, self.recover_z)

        rospy.Subscriber(self.name+'/enc_mu', String, self.recover_mu)

        rospy.Subscriber(self.name+'/enc_x_and_y', String, self.controller)




        # subscribe to controller_variables
        #rospy.Subscriber('/controller_variables', numpy_msg(Floats), self.update_controller_variables)


    def n(self, data):   #This is used to extract the value of n (i.e. the number of robots the agent detected, published from the from the dataprocessor node)

        self.n=data.data


    def recover_error(self, data): 
        e = data.data
        self.e2 = pymorph.recvr_pub_ros_str(e)

        
    def recover_z(self, data): 
        z =data.data
        self.z2 = pymorph.recvr_pub_ros_str(z)

    def recover_mu(self, data): 
        mu=data.data
        self.mu2 = pymorph.recvr_pub_ros_str(mu)

        #print "PRINTING MULTIPILICATION CONTROLLER: " + str([self.mu_hat, self.mu2[0]])
        



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
        if self.pub2.get_num_connections() > 0:#self.running < 10:
            # Input for controller

            print("-------------------------------------------")
            print("Controller of Nexus: "+str(int(sys.argv[1])))

            xy=data.data
            self.xy = pymorph.recvr_pub_ros_str(xy)
            #z_values= data.data
            #sizevec = data.layout.dim[1].size #size of each vector

            #var = pymorph.variables_import() #variables used for encryption

            Bx=[]
            By=[]
            D = []
            E = []
            Ed=[]
            E_enc1=[] 

            cc=[]
            '''for i in range(0, len(z_values), sizevec):
                cc.append(list(z_values[i:sizevec+i]))'''
            
            '''sp = data.layout.dim[0].size - self.n*2
            datalen = sp/(self.n*2)
            for i in range(0, self.n): #This loop gathers the encrypted data from the subscribed 1-dimensional array and reconstructs it into the desired matrices
                Bx.append(cc[sp/2+i])#+3*i])

                By.append(cc[(sp/2)+self.n+i])#+3*i])

                D.append(cc[((datalen*self.n+datalen*i)+self.n*2):(datalen*self.n+datalen*(i+1))+self.n*2])

                E.append(cc[datalen*i:datalen*(i+1)])

            E_enc1.append(cc[-2])
            E_enc1.append(cc[-1])'''

            #D = self.z2

            
            
            self.now = np.float64([rospy.get_time()])
            print "ACTUAL TIME BETWEEN RUNS: " + str(self.now-self.old) + " vs set DT: " + str(self.DT) 
            print "\n"

            
            self.mu_hat = pymorph.hom_mul_mat(self.q, self.N, self.FG_s_enc, [self.mu_hat, self.mu2[0]])[0]





            #print("self.mu2: " + str(pymorph.dec_hom(self.p, self.L, self.sk, [self.mu2])))

            #self.mu_hat = cc[data.layout.dim[0].size]
            #Q = cc[data.layout.dim[0].size+1] #This is the gain constant used when multiplying in the control law 

            #print (float(pymorph.dec_hom(var.p, var.L, self.sk, [self.mu_hat])[0][0])/1000000)
        
            #mu_hat_dot = 2*(Ed[0][0] - self.mu_hat) #I need to encrypt the Error in ENC1 form to substract

            


            #zeros = []

            #zeros = pymorph.enc_1(var.p, var.L, var.q, var.r, var.N, self.sk, zeros)
            


            '''errr = [int(i*1000) for i in self.e2[0]]
            
            if self.o ==2:
                map_object = map(operator.sub, errr,  self.mu_hat) #map(operator.sub, self.e2[0],  self.mu_hat)

            if self.o ==1:
                map_object = map(operator.sub, errr,  self.mu2[0]) #map(operator.sub, self.e2[0],  self.mu2[0])
                self.DT = 10
                self.o = 2

            
            de = list(map_object)
            de = [i*2 for i in de]
            #de = 2 * (E_enc1[0] - mu_hat)  #previously "mu_hat_dot"

            mu_hat_dot = [int(i*self.DT) for i in de]
            #self.mu_hat = [int(i*1000) for i in self.mu_hat]
            
            #self.mu_hat = map(operator.add, self.mu_hat, mu_hat_dot)'''

            #self.mu_hat = pymorph.hom_mul_mat(var.q, var.N, self.FG_s_enc, [self.mu_hat, mu_hat_dot])[0]






            #print(pymorph.dec_hom(var.p, var.L, self.sk, [self.mu_hat]))

            
            #print(pymorph.dec_hom(var.p, var.L, self.sk, [xy_by_mu[0]]))
            #print(pymorph.dec_hom(var.p, var.L, self.sk, [xy_by_mu[1]]))

            #self.mu_hat = pymorph.hom_mul_mat()
            
            #print float(pymorph.dec_hom(var.p, var.L, self.sk, self.mu_hat))/100000000000

            #self.mu_hat = self.mu_hat + mu_hat_dot  #This variable remains in memory and is always encrypted




            #S1bDz = S1bDz.reshape(2, 1)


            
            #print "Bx=", Bx
            #print "By=", By
            #print "D=", D
            #print "E=", E
            #print "Ed=", Ed

            # Formation shape control
            #BbDz = [list(a) for a in zip(Bx, By)]
	    #print "BbDz= ", BbDz
            #Dzt=np.diag(D)
            #Dzt=D
	    #print "Dzt= ", Dzt
	    #print "c= ", self.c
            #Ed=np.asarray(Ed)
            #Ed=E
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
            U2=[]
            #Usum =[0]*sizevec
            Y=[]
            Y2=[]
            #Ysum = [0]*sizevec

            #filler = [0]*len(self.mu_hat)
            
            
            #print("Mu within Controller: "+str(pymorph.dec_hom(self.p, self.L, self.sk, [self.mu_hat])))

            '''if int(sys.argv[1]) == 1:
                xy_by_mu = pymorph.hom_mul_mat(self.q, self.N, self.xy, [self.mu_hat, filler])

            if int(sys.argv[1]) == 2:
                xy_by_mu = pymorph.hom_mul_mat(self.q, self.N, self.xy, [self.mu_hat, filler])

            if int(sys.argv[1]) == 3:    
                xy_by_mu = pymorph.hom_mul_mat(self.q, self.N, self.xy, [filler, self.mu_hat]) #zeros[0], zeros[0]])#self.mu2[0], self.mu2[0]])'''

            #xy_by_mu = pymorph.hom_mul_mat(self.q, self.N, self.xy, [self.mu_hat, filler])
            #xy_by_mu_x = pymorph.hom_mul_mat(self.q, self.N, [[self.xy[0][0]]], [self.mu_hat])
            #xy_by_mu_y = pymorph.hom_mul_mat(self.q, self.N, [[self.xy[1][0]]], [self.mu_hat])
            xy_by_mu_x = pymorph.hom_mul(self.q, [self.mu_hat], [self.xy[0][0]])
            xy_by_mu_y = pymorph.hom_mul(self.q, [self.mu_hat], [self.xy[1][0]])
            xy_by_mu = [xy_by_mu_x[0], xy_by_mu_y[0]]

            #if int(sys.argv[1]) == 3: #used to make the thrid agent also estimate the rogue one instead of the cyclical pattern
            
            #    xy_by_mu_x = pymorph.hom_mul(self.q, [self.mu_hat], [self.xy[0][1]])
            #    xy_by_mu_y = pymorph.hom_mul(self.q, [self.mu_hat], [self.xy[1][1]])
            #    xy_by_mu = [xy_by_mu_x[0], xy_by_mu_y[0]]

            #print("XY within Controller: "+str(pymorph.dec_hom(self.p, self.L, self.sk, [pymorph.hom_mul_mat(self.q, self.N, self.xy, pymorph.enc_1(self.p, self.L, self.q, self.r, self.N, self.sk, [[1]*len(self.mu_hat), [1]*len(self.mu_hat)] ))))

            #print("XY_BY_MU within Controller: "+str(pymorph.dec_hom(self.p, self.L, self.sk, [xy_by_mu])))

            xy_err = pymorph.hom_mul_mat(self.q, self.N, self.xy, self.e2)

            #part1 = []
            #part2 = []

            U2 = (pymorph.hom_mul(self.q, [xy_by_mu[0]] , self.z2[0])) #append the values for mu_hat
            Y2 = (pymorph.hom_mul(self.q, [xy_by_mu[1]] , self.z2[0])) 

            #print("Estimator Velocities within Controller: "+str(pymorph.dec_hom(self.p, self.L, self.sk, [U2,Y2])))

            #if int(sys.argv[1]) == 3:
            #    U2 = (pymorph.hom_mul(self.q, [xy_by_mu[0]] , self.z2[1])) #append the values for mu_hat
            #    Y2 = (pymorph.hom_mul(self.q, [xy_by_mu[1]] , self.z2[1]))
            

            '''if int(sys.argv[1]) == 1:
                U2 = (pymorph.hom_mul(self.q, [xy_by_mu[0]] , self.z2[0])) #append the values for mu_hat
                Y2 = (pymorph.hom_mul(self.q, [xy_by_mu[1]] , self.z2[0])) 

            if int(sys.argv[1]) == 2:
                U2 = (pymorph.hom_mul(self.q, [xy_by_mu[0]] , self.z2[0])) #append the values for mu_hat
                Y2 = (pymorph.hom_mul(self.q, [xy_by_mu[1]] , self.z2[0])) 

            if int(sys.argv[1]) == 3:
                U2 = (pymorph.hom_mul(self.q, [xy_by_mu[0]] , self.z2[1])) #append the values for mu_hat
                Y2 = (pymorph.hom_mul(self.q, [xy_by_mu[1]] , self.z2[1])) '''

            #print(pymorph.dec_hom(var.p, var.L, self.sk, [U2]))

            #for i in range(self.n):
                
            U.append(pymorph.hom_mul(self.q, [xy_err[0]] , self.z2[0]))
            Y.append(pymorph.hom_mul(self.q, [xy_err[1]] , self.z2[1]))

                #U2.append(pymorph.hom_mul(var.q, [xy_by_mu[0]] , self.z2[0][i])) #append the values for mu_hat
                #Y2.append(pymorph.hom_mul(var.q, [xy_by_mu[1]] , self.z2[0][i]))

                #part2 += pymorph.hom_mul(var.q, xy_by_mu , self.z2[0][i])
                #U.append(pymorph.hom_mul(var.q, [pymorph.hom_mul(var.q, [BbDz[i][0]], Dzt[i])], Ed[i]))
                
                #Y.append(pymorph.hom_mul(var.q, [pymorph.hom_mul(var.q, [BbDz[i][1]], Dzt[i])], Ed[i]))
                
                #pymorph.hom_mul(var.q, Q, [pymorph.hom_mul(var.q, S1bDz_x1, self.mu_hat)])
                #pymorph.hom_mul(var.q, Q, [pymorph.hom_mul(var.q, S1bDz_y1, self.mu_hat)])

                #pymorph.hom_mul(var.q, S1bDz_x1, self.mu_hat) #must turn mu_hat into Enc2
                #pymorph.hom_mul(var.q, S1bDz_y1, self.mu_hat)
                #+ 0.011*S1bDz.dot(self.mu_hat) 

                #Usum = map(add, Usum, U[i])
                #Ysum = map(add, Ysum, Y[i])

            #for i in range(self.n):
                #U[]from operator import add;map(add, , list2)
            
            #for i in range(1, len(U)):
            #    U[0] = [sum(x) for x in zip(U[0], U[i])]
            #    Y[0] = [sum(x) for x in zip(Y[0], Y[1])]

            U = U[0]
            Y = Y[0]
            
            '''for i in range(1, len(U2)):
                U2[0] = [sum(x) for x in zip(U2[0], U2[i])]
                Y2[0] = [sum(x) for x in zip(Y2[0], Y2[1])]
            
            U2 = U2[0]
            Y2 = Y2[0]'''

            '''if self.n == 3: #These loops add up all the inputs generated by the detected neighboring robots
                U = [sum(x) for x in zip(U[0], U[1], U[2])]
                Y = [sum(x) for x in zip(Y[0], Y[1], Y[2])]

            if self.n == 2:
                U = [sum(x) for x in zip(U[0], U[1])]
                Y = [sum(x) for x in zip(Y[0], Y[1])]

            if self.n == 1:
                U = [sum(x) for x in zip(U[0])]
                Y = [sum(x) for x in zip(Y[0])]'''
                




            #U =  pymorph.hom_mul(var.q, [pymorph.hom_mul(var.q, [BbDz[0][0]], Dzt[0])], Ed[0])

            #U = self.Hom_mul([BbDz[0][0]], Dzt[0])

            #Y = BbDz[0][0] #self.Hom_mul([self.Hom_mul([BbDz[0][1]], Dzt[0])], Ed[0])  

            

            Z = [[U],[Y]]

            Z2 = [U2,Y2]
            #print self.Hom_mul([self.Hom_mul([list(BbDz[0][0])], [list(Dzt.astype(int))])], [list((Ed.astype(int)))])

                        
            mu_str = pymorph.prep_pub_ros_str(self.mu_hat)
            self.pub3.publish(String(mu_str))

            Z_str = pymorph.prep_pub_ros_str(Z) #Publishes the first element of the controller
            self.pub.publish(String(Z_str))

            Z2_str = pymorph.prep_pub_ros_str(Z2) #Publishes the estimating element of the controller
            self.pub2.publish(String(Z2_str))

            self.now = np.float64([rospy.get_time()])
            self.time = np.float64([self.now-self.begin])
            self.time_log = np.append(self.time_log, self.time)
            self.old = self.now

            #self.mu_hat_log.append(pymorph.dec_hom(self.p, self.L, self.sk, [self.mu_hat])[0][0]) #decrypt mu log to store value for monitoring
            
            #self.DT_log.append(self.DT)

            #rospack = rospkg.RosPack()
            ##PATH = os.path.dirname(os.path.abspath(__file__)) #This gets path from wherever the terminal used to launch the file was
            #PATH = rospack.get_path('nexhom') #This gets the path from the shown ROS package
            #FILEPATH = os.path.join(PATH+'/saved_variables', 'Controller_'+self.name+'.csv')
            #with open(FILEPATH, "w") as output:
                    
            #    np.savetxt("mu_hat_log"+self.name+".csv", (np.asarray(self.mu_hat_log)), delimiter=",")
                        
            #    self.writer = csv.writer(output, delimiter=',')
            #    self.writer.writerow(self.mu_hat_log)
            #    self.writer.writerow(self.DT_log)

            # publish
            #self.publish_control_inputs(self.tosend) #THIS IS NOT THE PUBLISHING FUNCTION
            
            #A=self.n
        elif 10 < self.running < 1000:
            self.shutdown()


    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Encrypted_Controller_"+str(int(sys.argv[1]))+"...")
        self.running = np.float32(10000)
        #self.velocity = Twist()
        self.tosend = String()
        self.pub.publish(self.tosend)
        rospy.sleep(1)







if __name__ == '__main__':
    try:
        rospy.init_node('controller_enc_'+str(int(sys.argv[1])), anonymous=False)
        r = rospy.Rate(10)
        controller()
        print "Cloud Processing Working"
        rospy.spin()
    except:
        rospy.loginfo("Controller node_"+str(int(sys.argv[1]))+" terminated.")  


