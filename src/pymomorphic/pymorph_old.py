#!/usr/bin/env python
import csv
import random
import rospkg
import numpy as np
import os

#add check to ensure inputted numbers are (integers) and warning

def main():
    print "main function called, this function is used when trying to debug this script. It get's executed when running the file directly"
    
    var = variables_import()

    sk = key_import(1)

    m = [300]

    ciphertext = enc_1(var.p, var.L, var.q, var.r, var.N, sk, m) 

    #ciphertext = [[-49807360L, 77641302L, -495364491, -338818076, 128598971L, 405696980L]]

    plaintext = dec_hom(var.p, var.L, sk, ciphertext[0])

    print plaintext

    #example of z_values obtained with 3 neighboring robots
    z_values = np.array([[ 0.00934644, -0.05645715, -0.80737489,  1.23556484], [-0.00632714,  0.67307276, -0.42058253,  1.25996497],[0.01942838,  0.72351229,  0.38469833,  1.2203629 ]])

    z_values_scal, scal = smart_scaling(z_values, 100)
    
    print z_values_scal

    print scal


    

def mod_hom(x, p):
    '''modulus function that works with negative values'''

    try:
        length = (len(x))
    except:
        length = 1

    y=[0]*length

    for i in range(len(x)):
        y[i] = (int(x[i] - int(x[i]/p)*p)) #remider from division

        if y[i]>=p/2:
            y[i] = int(y[i]) - p
    return y
    
def mod_hom2(x, p):
        try:
            length = (len(x))
        except:
            length = 1

        y=[0]*length

        for i in range(len(x)):
            y[i] = long(x[i] - int(x[i]/p)*p) #remider from division
        
        return y

def variables_define(p = 10**4, L = 10**4, r=10**1, N = 5): 
    '''This function generates a csv file with the variables for homomorphic such that the different ROS nodes can access the same variables'''

    q = p * L

    PATH = os.path.dirname(os.path.abspath(__file__))
    FILEPATH = os.path.join(PATH, 'variables.csv')

    with open(FILEPATH, "w") as output:
        writer = csv.writer(output, delimiter=',')
        writer.writerow([p])
        writer.writerow([L])
        writer.writerow([q])
        writer.writerow([r])
        writer.writerow([N])


def variables_import(ros_package = 'nexhom'):
    '''This function imports the variables from the csv file'''

    class variables: #This creates an object to store the variables in
        def __init__(self): 

            #rospack = rospkg.RosPack()
            PATH = os.path.dirname(os.path.abspath(__file__)) #rospack.get_path(ros_package)
            FILEPATH = os.path.join(PATH, 'variables.csv')

            with open(FILEPATH, 'rb') as f: #This section reads the csv file to gather the private key to encrypt
                reader = csv.reader(f, delimiter='\n')
                lis = list(reader)
                self.p=eval(lis[0][0])
                self.L=eval(lis[1][0])
                self.q=eval(lis[2][0])
                self.r=eval(lis[3][0])
                self.N=eval(lis[4][0])

    var = variables()

    return var


def key_generate(q, L, N, robot):
    '''This function generates a secret key to encrypt and saves it into a csv file'''
    
    rand = [random.randrange(q*L) for iter in range(N)]

    sk = mod_hom(rand, q*L)

    PATH = os.path.dirname(os.path.abspath(__file__)) #rospack.get_path(ros_package)
    FILEPATH = os.path.join(PATH, 'private_key_'+str(robot)+'.csv')

    with open(FILEPATH, "w") as output:
        writer = csv.writer(output, delimiter=',')
        for val in sk:
            writer.writerow([val])



def key_import(robot, ros_package = 'nexhom'):
    '''This function imports the key created into a csv file by the "key_generate" function'''
    #rospack = rospkg.RosPack()
    #PATH = rospack.get_path(ros_package)

    sk = []

    PATH = os.path.dirname(os.path.abspath(__file__)) #rospack.get_path(ros_package)
    FILEPATH = os.path.join(PATH, 'private_key_'+str(robot)+'.csv')

    with open(FILEPATH, 'rb') as f: #This section reads the csv file to gather the private key to encrypt
        reader = csv.reader(f, delimiter='\n')
        lis = list(reader)
        for i in lis:
            sk.append(eval(i[0]))

    return sk



def enc_1(p, L, q, r, N, secret_key, m): #This function encrypts the values into a vector (use this to sum homomorphically)
        
        n = len(m)
        
        A=[]
        for i in range(n):
            A.append([random.randrange(q) for iter in range(N)])


        dum = np.random.randint(low = 1, high = r, size = [n,1])

        dum =[item for sublist in dum for item in sublist]

        e = mod_hom(dum, r)

        b=[0]*len(m)
        for j in range(int(len(m))):
            for i in range(N):   #this loop replaces the dot multiplication of the numpy matrices as it doesn't handle large numbers well and outputs a wrong value

                b[j] = b[j] + int(-A[j][i]) * int(secret_key[i]) 

        
        add=[0]*len(e)
        for i in range(len(e)):
            add[i] = [L*m[i]+ e[i]] 

        for i in range(n):
            b[i] = [b[i] + add[i][0]]

        app = []

        b = [item for sublist in b for item in sublist]

        app.insert(0, b[0])

        for i in range(n): #append b to A [b, A]
            A[i].insert(0, b[i])
        
       
        ciphertext = [[0]*(N+1)]*n
        for i in range(n):
            ciphertext[i] = mod_hom(A[i] , q)

        #ciphertext = np.array([8016128, -36764075, 9754041, 27849822, -45311848])

        #print(ciphertext)

        return ciphertext

def enc_2(p, L, q, r, N, secret_key, m): #This function encrypts the values to be homomorphically multiplied
    lq = int(np.log10(q))
    R = np.kron( np.power(10, np.arange(lq)[:, np.newaxis]), np.eye(N+1)).astype(int) #THIS FUNCTION BREAKS AT HIGH NUMBERS

    Rl= R.tolist() #THIS IS CURRENTLY IN INTEGERS, SHOULD BE IN LONG

    dum = np.zeros(lq*(N+1), dtype = int).tolist()

    AAA = enc_1(p, L, q, r, N, secret_key, dum)

    #AAA = [[1978L, -4496, -1101], [126L, 3635, 299], [-3385, 776, 968], [-2506, -1782, -1763], [-2023, -1679, 236], [-3792, -623, 325], [3386L, 3019, 780], [-485, -1483, -621], [3295L, 4278, -4689], [1920L, 336, 3985], [-3443, -2227, 509], [-1074, -393, -654]]


    from operator import add
    hold = [[ m[0] * i for i in inner ] for inner in Rl] 
    for i in range(len(AAA)):    
        hold[i] = list( map(add, hold[i], AAA[i]) )

    ciphertext = [[0]*(len(AAA[0]))]*len(AAA)
    for i in range(len(AAA)):
        ciphertext[i] = mod_hom(hold[i], q)

    return ciphertext

def dec_hom(p, L, secret_key, c):
    '''This function decrypts a homomorphically encrypted value'''

    s=0
    s = secret_key[:]
    
    s.insert(0, 1)

    dot=[0]
    aa = 1
    dot[0] = [sum(long(i[0]) * long(i[1]) for i in zip(c,s))]
    
    
    #dot = [sum(i[0] * i[1] for i in zip(c, s))]  #dot multiplication of c and s

    plain = mod_hom(dot[0], L*p)

    plaintext = float(plain[0])/L

    plaintext = int(round(plaintext))
    #print(plaintext)

    return plaintext

def decomp(q, c1): #function to carry out before multiplying used by the function "hom_mul"
    lq = int(np.log10(q))

    c1 = mod_hom2(c1[0], q)

    BBB=[]
    for i in range(lq):
        Q = mod_hom2(c1, 10**(lq-1-i))
        Q = [j - k for j, k in zip(c1,Q)]
        BBB = ([j/(10**(lq-1-i)) for j in Q]) + BBB 

        c1 = [j - k for j, k in zip(c1,Q)]

    return BBB 

def hom_mul(q, c1, c2):
    ''' This function performs the multiplication of two homomorphically encrypted values, c2 must be encrypted using the function "enc2" '''
        
    c1 = decomp(q, c1)

    x=[0]*len(c2[0])
    for j in range(len(c2[0])):
        for i in range(len(c1)):
            x[j]=x[j]+(c1[i]*long(c2[i][j]))


    return x

def smart_scaling(input, scal = 100):
    '''
    This function scales it's input (z_values) until all values reach a minimum amount set by the input "scal"
    @param input: matrix to be scaled an array of arrays: np.array([[1,2],[1,2]])
    @param scal: lowest value desired (usually a power of 10)
    @return: the input scaled up
    '''

    low = abs(input) < 0.0001 #This is to take away values that are too small that would otherwise break this process
            
    input[low] = scal #replace the values that are to small with the scal value to avoid the algorithm trying to scale them up

    values = [] #initialize list to save the scaling values

    for i in range(len(input[0])):
        store = 1 #initialize variable to record the amount scaled up

        while np.any(abs(input[:, i])<=scal):
            
            input[:, i] *= 10

            store *= 10 

        values.append(store)

    input[low] = 1 #replace the undesired small values previously identified by 1

    if min(values[1:3]) == max(values[1:3]):
        pass

    else:
        
        #index_max = values[1:3].index(max(values[1:3]))+1 #middle values must be scaled to the same amount

        high = max(values[1:3])

        low = min(values[1:3])
        
        index_min = values[1:3].index(low)+1

        difference = high/low

        input[:, index_min] *= difference

        values[index_min] = high

    scaling = values[0]*values[1]*values[2]

    return input, scaling







if __name__ == '__main__':
    main()
