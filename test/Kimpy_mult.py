#!/usr/bin/env python

import numpy as np

import random

from pymomorphic import pymorph

#Number is usually 1 digit lower than the matlab code check rounding and instead make it a "ceiling function"

#CHECK THAT ALL VALUES BEING WORKED WITH ARE IN INTEGER FORM, OTHERWISE THE COMPUTER DOES NOT HAVE ENOUGH MEMORY TO GIVE THE RIGHT VALUES

class Kim:
    def __init__(self):    #The "self" structure replaces "env" from Kim's paper
        
        import rospkg
        import csv
        #rospack = rospkg.RosPack()
        #self.PATH = rospack.get_path('nexhom')

        '''with open(self.PATH + '/controlhom/agent_N/variables.csv', 'rb') as f: #This section reads the csv file to gather the private key to encrypt
            reader = csv.reader(f, delimiter='\n')
            lis = list(reader)
            self.p=eval(lis[0][0])
            self.L=eval(lis[1][0])
            self.q=eval(lis[2][0])
            self.r=eval(lis[3][0])
            self.N=eval(lis[4][0])'''

        #var= pymorph.variables_import() #get variables
        self.p=10**4#var.p
        self.L=10**4#var.L
        self.q=10**2#var.q
        self.r=10**1#var.r
        self.N=20#var.N
        #self.p=1*(10**15)
        #self.L=1*(10**4)
        #self.r=1*(10**1)
        #self.N=2

        self.q = self.L*self.p


    def Mod(self, x, p):
        #y = np.mod(x, p)

        try:
            length = (len(x))
        except:
            length = 1

        y=[0]*length

        for i in range(len(x)):
            y[i] = long(int(x[i] - int(x[i]/p)*p)) #remider from division

            if y[i]>=p/2:
                y[i] = int(y[i]) - p

        #y = y - (y>= p/2)*p
        
        return y

    def Mod2(self, x, p):
        #y = np.mod(x, p)

        try:
            length = (len(x))
        except:
            length = 1

        y=[0]*length

        for i in range(len(x)):
            y[i] = long(x[i] - int(x[i]/p)*p) #remider from division

        #y = y - (y>= p/2)*p
        
        return y

    def Keygen(self):

        #rand = np.random.randint(low = 1, high = self.q*self.L, size = [self.N,1], dtype=np.int64)

        #rand = rand.tolist()

        rand = [random.randrange(self.q*self.L) for iter in range(self.N)]

        #rand = [item for sublist in rand for item in sublist]

        self.sk = FUNC.Mod(rand, self.q*self.L) 
        
        #self.sk = np.array([[int(-185276313606)], [int(-94208062924)], [int(126986816294)], [int(-86624143860)]], dtype=np.int64)
        #self.sk = [-185276313606, -94208062924, 126986816294, -86624143860]

        #self.sk = [-23789654455134112992L, 271834883156680916307L, -279671991444874455279L, 98221942281401194356L, -406750167535451107636L]

        return self.sk

    def Enc(self, m):

        n = len(m)

        #A = np.random.randint(low = 1, high = self.q, size = [n,self.N])
        
        #A = A.tolist() #convert numpy array to list


        A=[]
        for i in range(n):
            A.append([random.randrange(self.q) for iter in range(self.N)])
        #A = [[random.randrange(self.q) for iter in range(self.N)]]

        #A = np.empty([1, 4], dtype=int)
        #A[0, 0] = 63235925
        #A[0, 1] = 9754041
        #A[0, 2] = 27849822
        #A[0, 3] = 54688152
        
        #np.array([int(63235925), int(9754041), int(27849822), int(54688152)], dtype=np.int64)#made this into integer otherwise calculation of large number in float type breaks

        #A = [[63235925,9754041, 27849822,54688152]]

        dum = np.random.randint(low = 1, high = self.r, size = [n,1])

        dum =[item for sublist in dum for item in sublist]# dum[:].tolist()

        e = FUNC.Mod(dum, self.r)

        #e = e.tolist()

        #e = [0]

        #e = [0]*40

        #b=np.zeros((m.shape[0], 1), dtype = int) #initialize variable
        b=[0]*len(m)
        for j in range(int(len(m))):
            for i in range(self.N):   #this loop replaces the dot multiplication of the numpy matrices as it doesn't handle large numbers well and outputs a wrong value
                #try:
                b[j] = b[j] + int(-A[j][i]) * int(sk[i]) 
                #except:
                #b[j] = b[j] + int(-A[j]) * int(sk[i]) 
                #print(b)
                #print(i)
        
        add=[0]*len(e)
        for i in range(len(e)):
            add[i] = [self.L*m[i]+ e[i]] #'''[:, np.newaxis]''' 

        for i in range(n):
            b[i] = [b[i] + add[i][0]]

        #b = 1.383578249590802e+19
        app = []

        b = [item for sublist in b for item in sublist]

        app.insert(0, b[0])

        for i in range(n): #append b to A [b, A]
            A[i].insert(0, b[i])
        
        #for i in range(len(b), len(A[0])+len(b)):
        #    try:
        #        app.insert(i, A[0][i-len(b)])
        #    except:
        #        app.insert(i, A[i-len(b)])
       
        ciphertext = [[0]*(self.N+1)]*n
        for i in range(n):
            ciphertext[i] = FUNC.Mod(A[i] , self.q)

        #ciphertext = np.array([8016128, -36764075, 9754041, 27849822, -45311848])

        #print(ciphertext)

        return ciphertext


    def Dec(self, c):

        #sk = self.sk.tolist()

        #s = [item for sublist in sk for item in sublist]

        #s=np.insert(self.sk, 0, 1) #insert a 1 into the matrix sk
        s=0
        s = self.sk[:]
        
        s.insert(0, 1)

        #s[0].insert(0, [1])

        #try: ################################
        #    dot = [[0]*(len(s))]*len(c)
        #    aa = len(s)

        #    for i in range(aa):
                #dot[i] = [sum(i[0] * i[1] for i in zip(c[0],s[i]))]
        #except:
        dot=[0]
        aa = 1
        dot[0] = [sum(long(i[0]) * long(i[1]) for i in zip(c,s))]
        
        
        #dot = [sum(i[0] * i[1] for i in zip(c, s))]  #dot multiplication of c and s

        plain = FUNC.Mod(dot[0], self.L*self.p)

        plaintext = float(plain[0])/self.L

        plaintext = int(round(plaintext))
        #print(plaintext)

        return plaintext

    def Decomp(self, c1):
        lq = int(np.log10(self.q))
        #c1 = np.mod(c1, self.q)
        c1 = self.Mod2(c1[0], self.q)
        #strdigits = np.empty([0], dtype = int)
        BBB=[]
        for i in range(lq):
            #Q = c1 - np.mod(c1, 10**(lq-1-i))
            Q = self.Mod2(c1, 10**(lq-1-i))
            Q = [j - k for j, k in zip(c1,Q)]
            #BBB = Q/10**(lq-1-i)
            BBB = ([j/(10**(lq-1-i)) for j in Q]) + BBB 
            #strdigits = np.insert(strdigits, 0, list(BBB[0])) 
            #c1 = c1 - Q
            c1 = [j - k for j, k in zip(c1,Q)]

        return BBB #strdigits.tolist()

    def Enc2(self, m):  #Encryption to carry out on the multiplier (2nd term) perform to multiplication

        lq = int(np.log10(self.q))
        R = np.kron( np.power(10, np.arange(lq)[:, np.newaxis]), np.eye(self.N+1)).astype(int) #THIS FUNCTION BREAKS AT HIGH NUMBERS

        Rl= R.tolist() #THIS IS CURRENTLY IN INTEGERS, SHOULD BE IN LONG

        dum = np.zeros(lq*(self.N+1), dtype = int).tolist()

        AAA = FUNC.Enc(dum)

        #AAA = [[1978L, -4496, -1101], [126L, 3635, 299], [-3385, 776, 968], [-2506, -1782, -1763], [-2023, -1679, 236], [-3792, -623, 325], [3386L, 3019, 780], [-485, -1483, -621], [3295L, 4278, -4689], [1920L, 336, 3985], [-3443, -2227, 509], [-1074, -393, -654]]


        from operator import add
        hold = [[ m[0] * i for i in inner ] for inner in Rl] #m*R + AAA
        for i in range(len(AAA)):    
            hold[i] = list( map(add, hold[i], AAA[i]) )

        ciphertext = [[0]*(len(AAA[0]))]*len(AAA)
        for i in range(len(AAA)):
            ciphertext[i] = FUNC.Mod(hold[i], self.q) ###########

        return ciphertext

    def Hom_mul(self, c1, c2): #Try this function with values of 10**15, see what calculation goes wrong
        
        #c1= [[-2685, -4064, -58]]
        c1 = FUNC.Decomp(c1)


        #dot = [0]*len(c2[0]) #initialize matrix in specific shape



        #for j in range(len(c2[0])): #dot multiplication of c1 and c2
            #c2_col = [int(item[j]) for item in c2]
            #dot[j] = int(sum(i[0] * i[1] for i in zip(c1, c2_col))) #TURN VALUES INTO INTEGERS OTHERWISE THE COMPUTER RUNS OUT OF MEMORY

        x=[0]*len(c2[0])
        for j in range(len(c2[0])):
            for i in range(len(c1)):
                x[j]=x[j]+(c1[i]*long(c2[i][j]))
                #print x

        print x
        #print dot
        dot =x

        return dot

        #c_mul= np.dot(FUNC.Decomp(c1), c2)

        #return c_mul.astype(int)

FUNC=Kim()

sk = FUNC.Keygen() #define the secret key

m1 = [3000]

m1_add = [5]

m2 = [1] 

m3 = [2]

m4 = [10]

#c1=[[-335544320, -3254246061, 2676085087, -2004427280, -1363783468, 2645833445]]
c1 = FUNC.Enc(m1)

#c1_ad = FUNC.Enc(m1_add)

#for i in range(len(c1_ad[0])):
#    c1[0][i] = c1_ad[0][i] +c1[0][i]

#for i in range(len(c1[0])): #to check effect of multiplication by integer
#    c1[0][i] = 500 * c1[0][i]

m1_dec = FUNC.Dec(c1[0])

print("m1 decoded = ", m1_dec)

#c1 = [-277056, 235925, -245959, -150178, -311848]

c2=FUNC.Enc2(m2)

c_mul=FUNC.Hom_mul(c1, c2)

print FUNC.Dec(c_mul)

c3=FUNC.Enc2(m3)

#print FUNC.Dec(c3[0])

c_mul=FUNC.Hom_mul([c_mul], c3)


c4=FUNC.Enc2(m4)

#print FUNC.Dec(c3[0])

c_mul=FUNC.Hom_mul([c_mul], c4)


#result = 

#print("c = ", result)

#m1_dec = FUNC.Dec(c1)

#m2_dec = FUNC.Dec(c2)

m_mul = FUNC.Dec(c_mul)


#print("m1 decoded = ", m1_dec)
#print("m2 decoded = ", m2_dec)
print("m2 = ", m2)
print("m_mul = ", m_mul)