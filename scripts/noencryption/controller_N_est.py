#!/usr/bin/env python

import sys #This import is used to allow for inputs into the function (hence the use of "sys.argv[]") 
import rospy

from rospy_tutorials.msg import Floats
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist

import matplotlib.pyplot as pl
import numpy as np

class controller:
    ''' The controller uses the interagent distances to determine the desired velocity of the Nexus '''
    
    def __init__(self):
        ''' Initiate self and subscribe to /z_values topic '''

        self.name='n_'+str(int(sys.argv[1]))

        # controller variables
        self.running = np.float32(1)
        self.d = np.float32(0.8)        
        # if int(sys.argv[1])==1 :
        #     self.d = np.float32(0.8) #To check if estimator works when one robot has a distance mismatch assigns a reading error to robot 3
        if int(sys.argv[1])==4 :
            self.d = np.float32(0.9) 

        self.dd = np.float32(np.sqrt(np.square(self.d)+np.square(self.d)))
        self.c = np.float32(0.5)
        self.U_old = np.array([0, 0])
        self.U_oldd = np.array([0, 0])
	

        if int(sys.argv[1]) == 1:
            self.mu_hat = np.array([[0], [0]])

        if int(sys.argv[1]) == 2:
            self.mu_hat = 0.0

        if int(sys.argv[1]) == 3:
            self.mu_hat = np.array([[0], [0]])

        # Motion parameters
        self.x_dot = np.float32(0)
        self.y_dot = np.float32(0)
        self.r_dot = np.float32(0)
        self.mu_x =  self.x_dot*np.array([0, -1, 0, -1, 0])
        self.mut_x = self.x_dot*np.array([0, 1, 0, 1, 0])
        self.mu_y =  self.y_dot*np.array([-1, 0, 0, 0, 1])
        self.mut_y = self.y_dot*np.array([1, 0, 0, 0, -1])
        self.mu_r =  self.r_dot*np.array([-1, -1, 0, 1, -1])
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

        # prepare shutdown
        rospy.on_shutdown(self.shutdown)

        # prepare publisher
        self.pub = rospy.Publisher(self.name+'/cmd_vel', Twist, queue_size=1)
        self.velocity = Twist()
             
        # subscribe to z_values topic
        rospy.Subscriber(self.name+'/z_values', numpy_msg(Floats), self.controller)
        rospy.Subscriber(self.name+'/agents', Int32, self.n)

        # subscribe to controller_variables
        rospy.Subscriber('/controller_variables', numpy_msg(Floats), self.update_controller_variables)
        
    def n(self, data):  #This is used to extract the value of n (i.e. the number of robots the agent detected, published from the from the dataprocessor node)
        if self.running < 10:
            self.n=data.data
	    #if self.n==2:
        #        self.mu_hat = 0.0
	        #print ("mu_hat",self.mu_hat)
        #    elif self.n>2:
         #       self.mu_hat = np.array([[0], [0]]) 


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
            self.mu_x =  self.x_dot*np.array([ 0,-1, 0,-1, 0])
            self.mut_x = self.x_dot*np.array([ 0, 1, 0, 1, 0])
            self.mu_y =  self.y_dot*np.array([-1, 0, 0, 0, 1])
            self.mut_y = self.y_dot*np.array([ 1, 0, 0, 0,-1])
            self.mu_r =  self.r_dot*np.array([-1,-1, 0, 1,-1])
            self.mut_r = self.r_dot*np.array([ 1, 1, 0,-1, 1])
            
            self.mu = self.mu_x+self.mu_y+self.mu_r
            self.mut = self.mut_x+self.mut_y+self.mut_r
        
    def controller(self, data):
        ''' Calculate U based on z_values and save error velocity in log arrays '''    
        if self.running < 10:
            # Input for controller
            z_values= data.data

            Bx=np.array([])
            By=np.array([])
            D = np.array([])
            E = np.array([])
            Ed=[]

            for i in range(0,self.n):
                Bx=np.append(Bx, z_values[1+3*i])
                By=np.append(By, z_values[2+3*i])
                D=np.append(D, z_values[3*i]**(-1))
                E=np.append(E,z_values[3*i]-self.d)
                Ed.append([E[i]])

            # Formation shape control
            BbDz=np.append([Bx],[By], axis=0)
            Dzt=np.diag(D)
            Ed=np.asarray(Ed)
            

                                
            # Estimator
            self.now = np.float64([rospy.get_time()])
            self.DT = self.now-self.old

            #mu_hat_dot = 2*(self.Ed - self.mu_hat)

            #mu_hat_dot=2*np.subtract(self.Ed, self.mu_hat)

            #mu_hat_dot = 2*(np.array([self.Ed[0]- self.mu_hat[0], self.Ed[1]- self.mu_hat[1]]))

            if int(sys.argv[1]) == 1:
                #self.mu_hat = np.array([[0], [0]])
                print(self.mu_hat)
                mu_hat_dot = [[0, 0]]
                S1bDz = np.array([[0,0], [0,0]], dtype = np.float)
                tt=0
                for i in [0, 2]:
                    mu_hat_dot[0][tt] = 2*(Ed[i] - self.mu_hat[tt])
                    tt = tt+1

                tt=0 
                for i in [0, 2]:
                    for j in [0, 1]:
                        S1bDz[tt][j] = abs(z_values[(3*i)+(j+1)]/z_values[3*i])
                    
                    tt = tt+1
                
                self.mu_hat = self.mu_hat + mu_hat_dot[0] * self.DT
                
                #S1bDz = S1bDz.reshape(2, 2)


            if int(sys.argv[1]) == 2:
                #self.mu_hat = 0.0
                mu_hat_dot = 0
                S1bDz = np.array([[0,0]], dtype = float)
                tt=0
                for i in [0]:
                    mu_hat_dot = 2*(Ed[i][0] - self.mu_hat)
                
                for i in [0]:
                    for j in [0, 1]:
                        S1bDz[tt][j] = abs(z_values[(3*i)+(j+1)]/z_values[3*i])
                
                    tt=tt+1
                self.mu_hat = self.mu_hat + mu_hat_dot * self.DT[0]

                S1bDz = S1bDz.reshape(2, 1)

            if int(sys.argv[1]) == 3:
                #self.mu_hat = np.array([[0], [0]])
                mu_hat_dot = [[0, 0]]
                S1bDz = np.array([[0,0], [0,0]], dtype = np.float)
                tt=0
                print(self.n)
                for i in [0, 2]:
                    print(Ed[i])
                    mu_hat_dot[0][tt] = 2*(Ed[i] - self.mu_hat[tt])
                    tt = tt+1

                tt=0 
                for i in [0, 2]:
                    for j in [0, 1]:
                        S1bDz[tt][j] = abs(z_values[(3*i)+(j+1)]/z_values[3*i])
                    
                    tt = tt+1
                self.mu_hat = self.mu_hat + mu_hat_dot[0] * self.DT

            if int(sys.argv[1]) == 4:
                S1bDz = np.array([[0,0]], dtype = float)
                self.mu_hat = 0
                S1bDz = S1bDz.reshape(2, 1)


            #ESTIMATOR IS FLAWED DUE TO GAIN DEPENDENCY, AND IT IS NOT BEING AUTOMATIC
            

            '''if isinstance(self.mu_hat,float): #First part of this loop checks if it's two robots following or more
                if z_values[1]<0 and z_values[2]<0 and z_values[-2]<0 and z_values[-1]>0:
                    mu_hat_dot = 2*(Ed[-1] - self.mu_hat) #Estimate edge of last robot]
                    self.mu_hat = self.mu_hat + mu_hat_dot * self.DT
                    self.S1bDz = np.array([[z_values[-2]/z_values[-3]], [z_values[-1]/z_values[-3]]])
                    print 'Nexus'+self.name
                    print "estimating last agent (Triangle)"
                else:
                    mu_hat_dot = 2*(Ed[0] - self.mu_hat) #Estimate edge of 1st found robot
                    self.mu_hat = self.mu_hat + mu_hat_dot * self.DT
                    self.S1bDz = np.array([[z_values[1]/z_values[0]], [z_values[2]/z_values[0]]])
                    print 'Nexus'+self.name
                    print "estimating first agent (Triangle)"
            else:
                #if self.n>3 and z_values[-5]>0 and z_values[-4]>0 and z_values[-2]<0 and z_values[-1]>0:
                #    mu_hat_dot = 2*(np.array([Ed[-1]- self.mu_hat[0], self.mu_hat[1]]))
                #    self.mu_hat = self.mu_hat + mu_hat_dot * self.DT
            #    self.S1bDz = np.array([[z_values[-2]/z_values[-3], z_values[-1]/z_values[-3]], \
            #                           [0, 0]])
            #    print "estimating last agent (Multiple)"
                if z_values[1]<0 and z_values[2]<0 and z_values[-2]<0 and z_values[-1]>0 and z_values [-5]>0:                
                        mu_hat_dot = 2*(np.array([Ed[-1]- self.mu_hat[0], Ed[-2]- self.mu_hat[1]]))
                        self.mu_hat = self.mu_hat + mu_hat_dot * self.DT
                        self.S1bDz = np.array([[z_values[-2]/z_values[-3], z_values[-1]/z_values[-3]], \
                                        [z_values[-5]/z_values[-6], z_values[-4]/z_values[-6]]])
                        print 'Nexus'+self.name
                        print "estimating last and penultimate agents (Multiple)"

                else:
                    mu_hat_dot = 2*(np.array([Ed[0]- self.mu_hat[0], self.mu_hat[1]]))#Ed[-1]- self.mu_hat[1]]))
                    self.mu_hat = self.mu_hat + mu_hat_dot * self.DT
                    self.S1bDz = np.array([[z_values[1]/z_values[0], z_values[2]/z_values[0]], \
                                [np.float64(0) , np.float64(0)]])
                                        #[z_values[-2]/z_values[-3], z_values[-1]/z_values[-3]]])
                    print 'Nexus'+self.name
                    print "estimating first agent (Multiple)"'''

            #print mu_hat_dot
            #print self.S1bDz
            #print BbDz.dot(Dzt)


            #self.S1bDz = np.array([[z_values[1]/z_values[0]], [z_values[2]/z_values[0]]])

    #            print 'mu_hat4_1= \n', self.mu_hat
    #            print 's1bDz_1= \n', self.S1bDz
    #            print 'mu_hat4_1 * s1bDz= \n', self.S1bDz.dot(self.mu_hat)
                
                # Formation motion control
    #            self.Ab = np.array([[self.mu[0], 0, self.mu[3], 0], \
    #                                [0, self.mu[0], 0, self.mu[3]]])
    #            self.z = np.array([self.z_values[7], self.z_values[8], self.z_values[1], self.z_values[2]])

                # Control law
                #U2 = self.c*self.BbDz.dot(self.Dzt).dot(self.Ed) + (self.Ab.dot(self.z)).reshape((2, 1))+ 0.5*self.S1bDz.dot(self.mu_hat)

            
            '''print 'Nexus'+self.name
            if self.n==2:
                U = self.c*BbDz.dot(Dzt).dot(Ed) - 2.9*self.S1bDz.dot(self.mu_hat).reshape((2, 1))
                #print self.S1bDz.dot(self.mu_hat).reshape((2, 1))
            else:
                U = self.c*BbDz.dot(Dzt).dot(Ed) - 2.9*self.S1bDz.dot(self.mu_hat)
                print self.S1bDz.dot(self.mu_hat)
                print self.c*BbDz.dot(Dzt).dot(Ed)
                
            print "U = ", U '''

            print('-----------------')
            print('Nexus ' + str(int(sys.argv[1])))

            #print "Detected Robots: ", self.n


            U = self.c*BbDz.dot(Dzt).dot(Ed) + 0.011*S1bDz.dot(self.mu_hat)  #The constant here (gain) has to be calibrated by checking the values while the formation is functioning and sliding (calbrated for self.d=0.9)

            #to design (aka calibarate) the gain after changing offset use: (self.c*BbDz.dot(Dzt).dot(Ed)/S1bDz.dot(self.mu_hat)).mean()
            #for distance of 0.8 vs 1: -0.6172
            #I think this gain should be equal to self.DT maybe

            print("Estimator: ", S1bDz.dot(self.mu_hat))

            print("Control: ", self.c*BbDz.dot(Dzt).dot(Ed))

            print("E ", Ed)

            print("mu ", self.mu_hat)

            #print "U = ", U
                
            # Saturation
            v_max = 0.2
            v_min = 0.02
            for i in range(len(U)):          
                if U[i] > v_max:
                    U[i] = v_max
                elif U[i] < -v_max:
                    U[i] = -v_max
                elif -v_min < U[i] < v_min:
                    U[i] = 0
                
                # Set old U values in order to prevent shaking
            self.U_oldd = self.U_old
            self.U_old = U

            # Append 0 to error if no robot is detected to be able to plot later             
            if self.n > 3:
                self.E3_log = np.append(self.E3_log, 0)
            if self.n > 4:
                self.E4_log = np.append(self.E4_log, 0)
            if self.n > 5:
                self.E5_log = np.append(self.E5_log, 0)
            if self.n > 6:
                self.E6_log = np.append(self.E6_log, 0)

                #Append error and velocity in Log arrays

            self.E1_log = np.append(self.E1_log, Ed[0])
            self.E2_log = np.append(self.E2_log, Ed[1])
            if self.n > 2:
                self.E3_log = np.append(self.E3_log, Ed[2])
            if self.n > 3:
                self.E4_log = np.append(self.E4_log, Ed[3])
            if self.n > 4:
                self.E5_log = np.append(self.E5_log, Ed[4])
            if self.n > 5:
                self.E6_log = np.append(self.E6_log, Ed[5])

            self.Un = np.float32([np.sqrt(np.square(U[0])+np.square(U[1]))])
            self.U_log = np.append(self.U_log, self.Un)
                
            # Save current time in time log array
            if self.k < 1:
                    self.begin = np.float64([rospy.get_time()])
                    self.k = 10
            self.now = np.float64([rospy.get_time()])
            self.time = np.float64([self.now-self.begin])
            self.time_log = np.append(self.time_log, self.time)
            self.old = self.now
            # publish
            #self.publish_control_inputs(U[0], U[1])

        elif 10 < self.running < 1000:
            self.shutdown()

    def publish_control_inputs(self,x,y):
        ''' Publish the control inputs to command velocities '''

        self.velocity.linear.x = x
        self.velocity.linear.y = y
        
        # print 'cmd_vel NEXUS 1 (x,y)', self.velocity.linear.x, self.velocity.linear.y
        #        rospy.loginfo(self.velocity)

        self.pub.publish(self.velocity)

    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Nexus_"+str(int(sys.argv[1]))+"...")
        self.running = np.float32(10000)
        self.velocity = Twist()
        self.pub.publish(self.velocity)

        print(len(self.time_log))
        print(len(self.E1_log))

        rospy.sleep(1)
        pl.close("all")       
        pl.figure(0)
        pl.title("Inter-agent distance error measured by Nexus "+str(int(sys.argv[1])))
        pl.plot(self.time_log, self.E1_log, label="e1_nx"+str(int(sys.argv[1])), color='b')
        pl.plot(self.time_log, self.E2_log, label="e2_nx"+str(int(sys.argv[1])), color='y')
        if self.n > 2:
            pl.plot(self.time_log, self.E3_log, label="e3_nx"+str(int(sys.argv[1])), color='g')
        if self.n > 3:
            pl.plot(self.time_log, self.E4_log, label="e4_nx"+str(int(sys.argv[1])), color='r')
        if self.n > 4:
            pl.plot(self.time_log, self.E5_log, label="e5_nx"+str(int(sys.argv[1])), color='c')
        if self.n > 5:
            pl.plot(self.time_log, self.E6_log, label="e6_nx"+str(int(sys.argv[1])), color='m')
        pl.xlabel("Time [s]")
        pl.ylabel("Error [m]")
        pl.grid()

        pl.legend()
	#A="/home/mariano/Desktop/Plots/Nexus_Distance_"+str(int(sys.argv[1]))+".png"
        pl.savefig("/home/mariano/Desktop/Plots/Nexus_Distance_"+str(int(sys.argv[1]))+".png")
        
        pl.figure(1)
        pl.title("Input velocity Nexus "+str(int(sys.argv[1])))
        pl.plot(self.time_log, self.U_log, label="pdot_nx"+str(int(sys.argv[1])), color='b')
        pl.xlabel("Time [s]")
        pl.ylabel("Velocity [m/s]")
        pl.grid()
        pl.savefig("/home/mariano/Desktop/Plots/Nexus_Velocity_"+str(int(sys.argv[1]))+".png")
#        pl.legend()
        
        pl.pause(0)


if __name__ == '__main__':
    try:
        rospy.init_node('controller_'+str(int(sys.argv[1])), anonymous=False)
        controller()
        rospy.spin()
    except:
        rospy.loginfo("Controller node_"+str(int(sys.argv[1]))+" terminated.")  


