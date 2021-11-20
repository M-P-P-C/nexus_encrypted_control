#!/usr/bin/env python 
 
import sys 
import rospy

from rospy_tutorials.msg import Floats
from std_msgs.msg import Int32
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist

import numpy as np
import matplotlib.pyplot as pl

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
        self.d = np.float32(0.8)        
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

            Bx=np.array([])
            By=np.array([])
            D = np.array([])
            E = np.array([])
            Ed=[]


            for i in range(self.n):
                Bx=np.append(Bx, z_values[1+3*i])
                By=np.append(By, z_values[2+3*i])
                D=np.append(D, z_values[3*i]**(-1))
                E=np.append(E,z_values[3*i]-self.d)
                Ed.append([E[i]])
	    
            print("Bx=", Bx)
            print("By=", By)
            print("D=", D)
            print("E=", E)
            print("Ed=", Ed)

            # Formation shape control
            BbDz=np.append([Bx],[By], axis=0)
            print("BbDz= ", BbDz)
            Dzt=np.diag(D)
            print("Dzt= ", Dzt)
            print("c= ", self.c)
            Ed=np.asarray(Ed)
            print("Ed= ", Ed)

            # print "error = ", Ed
            # print "z_values = ", z_values
            
            # Formation motion control
            #Ab = np.array([[self.mu[0], 0, self.mu[3], 0], \
                                #[0, self.mu[0], 0, self.mu[3]]])
            #z = np.array([z_values[7], z_values[8], z_values[1], z_values[2]])
            # z = [ edge 1 , edge 4]
            
            # Control law
            U = self.c*BbDz.dot(Dzt).dot(Ed) #+ (Ab.dot(z)).reshape((2, 1))
            print("U = ", -U)
        
            # Saturation
            v_max = 0.2
            v_min = 0.02
            for i in range(len(U)):          
                if U[i] > v_max:
                    U[i] = v_max
                elif U[i] < -v_max:
                    U[i] = -v_max
                elif -v_min < U[i]+self.U_old[i]+self.U_oldd[i] < v_min : # preventing shaking 
                    U[i] = 0
            
            # Set old U values in order to prevent shaking
            self.U_oldd = self.U_old
            self.U_old = U
            print("Number of Robots Detected: ", self.n)
                
            '''# Append 0 to error if no robot is detected to be able to plot later 
            if self.n > 3:
                self.E3_log = np.append(self.E3_log, 0)
            if self.n > 4:
                self.E4_log = np.append(self.E4_log, 0)
            if self.n > 5:
                self.E5_log = np.append(self.E5_log, 0)
            if self.n > 6:
                self.E6_log = np.append(self.E6_log, 0)'''


                

                # Append error and velocity in Log array
            
            '''try:
                if self.n < 3 or self.n > 0: 
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
            except:
                print "Not enough robots detected to start plotting"'''

            self.Un = np.float32([np.sqrt(np.square(U[0])+np.square(U[1]))])
            self.U_log = np.append(self.U_log, self.Un)
                
            # Save current time in time log array
            if self.k < 1:
                self.begin = np.float64([rospy.get_time()])
                self.k = 10
            self.now = np.float64([rospy.get_time()])
            self.time = np.float64([self.now-self.begin])
            self.time_log = np.append(self.time_log, self.time)
            #if self.n > 2:
            #    print ("time",len(self.time_log))
            #print ("error",len(self.E1_log))
                # publish
            self.publish_control_inputs(U[0], U[1])
            A=self.n
        elif 10 < self.running < 1000:
            self.shutdown()

    def publish_control_inputs(self,x,y):
        ''' Publish the control inputs to command velocities'''

        self.velocity.linear.x = x
        self.velocity.linear.y = y
        
        # print 'cmd_vel NEXUS 1 (x,y)', self.velocity.linear.x, self.velocity.linear.y
        #        rospy.loginfo(self.velocity)
        self.pub.publish(self.velocity)
        print("Message Published by Controller: ")
        print(self.velocity)
        print("End of Message")
        #if self.n<3:
        #np.save('/home/mariano/Documents/Data/E1_log_nx'+str(int(sys.argv[1])), self.E1_log)
        #np.save('/home/mariano/Documents/Data/E2_log_nx'+str(int(sys.argv[1])), self.E2_log)
        #if self.n<4:
        #np.save('/home/mariano/Documents/Data/E3_log_nx'+str(int(sys.argv[1])), self.E3_log)
        #if self.n<5:
        #np.save('/home/mariano/Documents/Data/E4_log_nx'+str(int(sys.argv[1])), self.E4_log)
        #np.save('/home/mariano/Documents/Data/U_log_nx1', self.U_log)
        #np.save('/home/mariano/Documents/Data/time_log_nx1', self.time_log)
    def shutdown(self):
        ''' Stop the robot when shutting down the controller_1 node '''
        rospy.loginfo("Stopping Nexus_"+str(int(sys.argv[1]))+"...")
        self.running = np.float32(10000)
        self.velocity = Twist()
        self.pub.publish(self.velocity)



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
        #pl.legend()
        
        pl.pause(0)


if __name__ == '__main__':
    try:
        rospy.init_node('controller_'+str(int(sys.argv[1])), anonymous=False)
        controller()
        rospy.spin()
    except:
        rospy.loginfo("Controller node_"+str(int(sys.argv[1]))+" terminated.")  


