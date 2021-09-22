"""Final Project controller."""
# -*- coding: utf-8 -*-



#
#Controller might take a couple a seconds
#







from controller import Robot, DistanceSensor, Motor, Camera, Keyboard
import math
from scipy.signal import convolve2d
import numpy as np
import matplotlib.pyplot as plt
import random
import heapq

# time in [ms] of a simulation step
TIME_STEP = 32

MAX_SPEED = 6.28
# #MAX_SPEED = 3.14
AXLE_LENGTH = 0.052
MAX_SPEED_MS = 0.6


# create the Robot instance.
robot = Robot()
timestep = int(32)
#Camera
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

# Using keyboard setup from lab 5 for player driven game
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)


# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
state='forward1'
time1=0

gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Odometry
pose_y = 0
pose_x = 0
# n = compass.getValues()
# rad = -((math.atan2(n[0], n[2]))-1.5708)
pose_theta = 0


vL = 0
vR = 0

goal_points=[]
state=0

#p1=.5*MAX_SPEED
#p2=.5*MAX_SPEED
p3=MAX_SPEED

# mode=''
#Uncomment to run Epuck
mode = 'planning'


#DO NOT UNCOMMENT UNLESS ON DEV
# mode = 'game'
#mode = 'tracking'

#For player game
redBool = False
blueBool = False
greenBool = False
pinkBool = False
cyanBool = False
yellowBool = False
time = 0
inPlay = True
if mode == 'game':
    print("Get all the colors to win the game!")
    print("Colors available: Red, Blue, Green, Pink, Yellow, Cyan")

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    if mode == 'game' and inPlay == True:
        time = time + .032
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT:
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        else:
            vL *= 0.75
            vR *= 0.75
        cameraImage = camera.getImage()
        #print("width ", camera.getWidth())
        #print("height ", camera.getHeight())
        redTop = camera.imageGetRed(cameraImage, camera.getWidth(), 53, 9)
        redBot = camera.imageGetRed(cameraImage, camera.getWidth(), 459, 266)
        greenTop = camera.imageGetGreen(cameraImage, camera.getWidth(), 53, 9)
        greenBot = camera.imageGetGreen(cameraImage, camera.getWidth(), 459, 266)
        blueTop = camera.imageGetBlue(cameraImage, camera.getWidth(), 53, 9)
        blueBot = camera.imageGetBlue(cameraImage, camera.getWidth(), 459, 266)
        grey = camera.imageGetGrey(cameraImage, camera.getWidth(), 5, 10)
        #notGreen = False
        #notRed = False
        #notBlue = False
        if((greenTop > 180) and (greenBot > 180)):
            if((blueTop > 180) and (blueBot > 180)):
                #notGreen = True
                if cyanBool == False:
                    cyanBool = True
                    print("CYAN HIT!")
            elif((redTop > 180) and (redBot > 180)):
                #notGreen = True
                if yellowBool == False:
                    yellowBool = True
                    print("YELLOW HIT!")
            else:
                if((greenBool == False) and (((blueTop < 180) and (blueBot < 180)) and (redTop < 180 and redBot < 180))):
                    greenBool = True
                    print("GREEN HIT!")
        if((redTop > 180) and (redBot > 180)):
            if((blueTop > 180) and (blueBot > 180)):
                #notRed = True
                if pinkBool == False:
                    pinkBool = True
                    print("PINK HIT!")
            elif((greenTop > 180) and (greenBot > 180)):
                #notGreen = True
                if yellowBool == False:
                    yellowBool = True
                    print("YELLOW HIT!")
            else:
                if((redBool == False) and (((blueTop < 180) and (blueBot < 180)) and (greenTop < 180 and greenBot < 180))):
                    redBool = True
                    #print(camera.imageGetRed(cameraImage, camera.getWidth(), 53, 9))
                    #print(camera.imageGetRed(cameraImage, camera.getWidth(), 459, 266))
                    #print(blueTop, blueBot)
                    print("RED HIT!")
        if((blueTop > 180) and (blueBot > 180)):
            if((redTop > 180) and (redBot > 180)):
                if pinkBool == False:
                    pinkBool = True
                    print("PINK HIT!")
            elif((greenTop > 180) and (greenBot > 180)):
                #notBlue = True
                if cyanBool == False:
                    cyanBool = True
                    print("CYAN HIT!")
            else:
                if((blueBool == False) and (((redTop < 180) and (redBot < 180)) and (greenTop < 180 and greenBot < 180))):
                    blueBool = True
                    print("BLUE HIT!")
        if blueBool == True:
            if redBool == True:
                if greenBool == True:
                    if pinkBool == True:
                        if yellowBool == True:
                            if cyanBool == True:
                                print("Congrats, game has been completed in time: ", time*31.25/60)
                                inPlay = False
        #print(redTop,greenTop,blueTop,grey)

        # initialize motor speeds at 50% of MAX_SPEED.
        #leftSpeed  = 0.5 * MAX_SPEED
        #rightSpeed = 0.5 * MAX_SPEED
        # modify speeds according to obstacles
        #if forward_obstacle:
             #turn right
            #leftSpeed  = 0#0.5 * MAX_SPEED
            #rightSpeed = -0.5 * MAX_SPEED
        #elif backward_obstacle:
             #turn left
            #leftSpeed  = -0.5 * MAX_SPEED
            #rightSpeed = 0.5 * MAX_SPEED
        # write actuators inputs
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)
    
    
    if mode == 'tracking':
        
        p=math.sqrt(math.pow((pose_x-goal_points[state][0]),2)+math.pow((pose_y-goal_points[state][1]),2))
        a=math.atan2((goal_points[state][1]-pose_y),(goal_points[state][0]-pose_x))+pose_theta
        #n=goal_points[state][2]-pose_theta
        
        print("P", p)
        print("A", a)

        if(p < .4):
            state = state+1
        print(state)
        
        if(a > .1):
            xr = 1*p
            theta_r = -8*a
            vL = xr + (theta_r/2*AXLE_LENGTH)
            vR = xr - (theta_r/2*AXLE_LENGTH)
        elif(a < -.1):
            xr = 1*p
            theta_r = -8*a
            vL = xr + (theta_r/2*AXLE_LENGTH)
            vR = xr - (theta_r/2*AXLE_LENGTH)
        else:
            xr = 2*p
            theta_r = -4*a
            vL = xr + (theta_r/2*AXLE_LENGTH)
            vR = xr - (theta_r/2*AXLE_LENGTH)
        
        
        
        
        # #STEP 2: Controller (with gains)
        # if(a > .2 or a < -.2):
            # xr = 1*p
            # theta_r = 8*a# + p3*n
        # else:
            # xr = 2*p
            # theta_r = 4*a# + p3*n

        #STEP 3: Compute wheelspeeds
        # vL = xr + (theta_r/2*AXLE_LENGTH)
        # vR = xr - (theta_r/2*AXLE_LENGTH)


        #STEP 4: Normalize wheelspeed
        if(vL > MAX_SPEED):
            vL = MAX_SPEED
        if(vR > MAX_SPEED):
            vR = MAX_SPEED
        if(vL < -MAX_SPEED):
            vL = -MAX_SPEED
        if(vR < -MAX_SPEED):
            vR = -MAX_SPEED
        if(state == len(goal_points)-1):
            vR = 0
            vL = 0

        ################# Do not modify this block of the code ########################
        # Odometry code. Don't change speeds after this
        pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*TIME_STEP/1000.0*math.cos(pose_theta)
        pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*TIME_STEP/1000.0*math.sin(pose_theta)
        pose_theta += (vL-vR)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*TIME_STEP/1000.0
        print("X: %f Y: %f Theta: %f" % (pose_x, pose_y, pose_theta)) # Comment out this if you want
        ##############################################################################

        # Enter here functions to send actuator commands
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)


    if mode == 'planning':
    
        map = np.load("../final_proj/map.npy")
        map = np.flipud(map)
        
        K = np.ones((30,30))
    
        cspace = convolve2d(map,K,mode="same")
        cspace = np.multiply(cspace>0.8, 1)
        
        # plt.imshow(cspace)
        # plt.show()
        
        start_point = (50,300)
        
        goal_point = (288, 55)
        
        
        def adjacent_states(state, map):
            adj_states = []
        
            #Right
            if (state[0]+1) <= 359:
                if map[state[1]][state[0]+1] == 0:
                    adj_states.append((state[0]+1, state[1]))
            #Left
            if state[0]-1 >= 0:
                if map[state[1]][state[0]-1] == 0:
                    adj_states.append((state[0]-1, state[1]))
        
            #Up
            if state[1]+1<=359:
                if map[state[1]+1][state[0]] == 0:
                    adj_states.append((state[0], state[1]+1))
            #Down
            if state[1]-1>=0:
                if map[state[1]-1][state[0]] == 0:
                    adj_states.append((state[0], state[1]-1))
        
            #Upper Right
            if state[0]+1<= 359 and state[1]+1 <= 359:
                if map[state[1]+1][state[0]+1] == 0:
                    adj_states.append((state[0]+1, state[1]+1))
        
            #Upper Left
            if state[0]-1>= 0 and state[1]+1 <= 359:
                if map[state[1]+1][state[0]-1] == 0:
                    adj_states.append((state[0]-1, state[1]+1))
        
            #Bottom Right
            if state[0]+1<= 359 and state[1]-1 >= 0:
                if map[state[1]-1][state[0]+1] == 0:
                    adj_states.append((state[0]+1, state[1]-1))
        
            #Bottom Left
            if state[0]-1>= 0 and state[1]-1 >= 0:
                if map[state[1]-1][state[0]-1] == 0:
                    adj_states.append((state[0]-1, state[1]-1))
            return adj_states
        
        def heuristic_eucl(state, goal):
            x_num = pow(goal[0] - state[0], 2)
            y_num = pow(goal[1] - state[1], 2)
            dis = math.sqrt(x_num+y_num)
            return dis
            
        def path(previous, s):
            if s is None:
                return []
            else:
                return path(previous, previous[s])+[s]
    
        def pathcost(path):
            cost = 0
            for s in range(len(path)-1):
                cost += heuristic_eucl(path[s], path[s+1])
            return cost
    
        class Frontier_PQ:
    
            def __init__(self, start, cost):
                #Minimum pathcost to a node
                self.states = {start:cost}
                #Frontier
                self.q = [(cost, start)]
        
            def add(self, state, cost):
                heapq.heappush(self.q, (cost,state))
                self.states[state] = cost
                return
        
            def pop(self):
                x = heapq.heappop(self.q)
                self.states.pop(x[1])
                return x
        
            def replace(self, state, cost):
                self.states[state] = cost
                for i in range(len(self.q)-1):
                    if self.q[i][1] == state and self.q[i][0] > cost:
                        self.q[i] = (cost, state)
                return
    
        def thiago_search(map, start_w, end_w):
        
            #start_w = THIAGO INITIAL POSE
            #end_w = THIAGO GOAL POSE
        
            #Already explored nodes
            explored = {}
            #Nodes on deck
            frontier = Frontier_PQ(start_w, 0)
            #The Dictionary of parent nodes
            _previous = {start_w:None}
            #num of explored
            n_exp=0
            #While there are still nodes to explore
            while len(frontier.q) != 0:
                #Get the node object
                curr = frontier.pop()
                n_exp+=1
                #Retrieve the corrdinates
                currN = curr[1]
                # print("Current Node:", currN)
                # #Check whether or not we are explore the goal
                if currN == end_w:
                    solP = path(_previous, currN)
                    # print("Number of nodes explored:", n_exp)
                    return solP
        
                explored[currN] = pathcost(path(_previous, currN))
        
                pos_moves = adjacent_states(currN, map)
        
                for i in pos_moves:
                    newcost = explored[currN] + heuristic_eucl(currN, i) + heuristic_eucl(i, end_w)
                    if (i not in explored) and (i not in frontier.states):
                        frontier.add(i, newcost)
                        _previous[i] = currN
                    elif (i in frontier.states) and (frontier.states[i] > newcost):
                        frontier.replace(i, newcost)
                        _previous[i] = currN
        
            print("Path not found!")
            return []
            
        path = thiago_search(cspace, start_point, goal_point)
        # goal_points = []
        for i in range(len(path)):
            goal_points.append(((path[i][0]/36)-5, (path[i][1]/36)-5))
            # print(goal_points[i])
        
        for i in range(len(cspace)):
            for j in range(len(cspace[i])):
                if (j,i) in path:
                    cspace[i][j] = 2
                    
        # plt.imshow(cspace)
        # plt.show()
        
        
            
        mode='tracking'
        # print(path)
        
    
        
        ##################
        #
        #ATTEMPTED RRT
        
        
        # state_bounds =  np.array([[0,360],[0,360]])
        
        # class Node:

            # def __init__(self, pt, parent=None):
                # self.point = pt # n-Dimensional point
                # self.parent = parent # Parent node
                # self.path_from_parent = [] # List of points along the way from the parent node (for edge's collision checking)
        
                
        # def state_is_valid(state):
        
            # for dim in range(state_bounds.shape[0]):
                # if state[dim] < state_bounds[dim][0]: 
                    # return False
                # if state[dim] >= state_bounds[dim][1]: 
                    # return False
            # if (map[state[1]][state[0]] != 0):
                # return False
                
            # return True
            
        # def get_random_valid_vertex(state_is_valid, bounds):
            
            # vertex = None
            # while vertex is None: # Get starting vertex
                # pt = np.random.rand(bounds.shape[0]) * (bounds[:,1]-bounds[:,0]) + bounds[:,0]
                # if state_is_valid(pt):
                    # vertex = pt
            # return vertex       

        
        # def get_nearest_vertex(node_list, q_point):
            
            # #Holding Node Variable
            # near_n = None
            # #Distance comparison/Holding variable
            # min_dist = 1e+20
            # #Transverse Node List
            # for i in range(len(node_list)):
                # p_sum = np.linalg.norm((q_point - node_list[i].point))
                # if p_sum < min_dist:
                    # near_n = node_list[i]
                    # min_dist = p_sum
        
            # return near_n
        
        # def steer(from_point, to_point, delta_q):
            
            # len_v  = to_point - from_point
            # len_p = np.linalg.norm(len_v)
            # if len_p > delta_q:
                # norm_v = (len_v/len_p)*delta_q
                # new_to = from_point + norm_v
                # new_to = (int(new_to[0]), int(new_to[1]))
                # print("Path Start:", from_point)
                # print("Path End:", new_to)
                # path=[]
                # path.append(from_point)
                # for i in range(1,12):
                    # path.append(from_point+(i*((from_point[0]-to_point[0])/12)),to_point+(i*((from_point[1]-to_point[1])/12)))
                # print(path)
                
                    
            # else:
                # print("Path Start:", from_point)
                # print("Path End:", to_point)
                # path = np.linspace(from_point, to_point, 12)
        
            # print("Path:", path)
            # return path
        
        # def check_path_valid(path, state_is_valid):
            
            # for i in range(len(path)):
                # if not state_is_valid(path[i]):
                    # return False
            # return True
        
        # def rrt(world_bounds, state_is_valid, starting_point, goal_point, k, delta_q):
            # print("Hi")
            
            
            # node_list=[]
            # node_list.append(Node(starting_point, parent=None))
            
            
            # count_k = 0

            # while count_k != k:
                # rand_n = get_random_valid_vertex(state_is_valid, state_bounds)
                # if goal_point is not None:
                    # if random.random() < 0.1:
                        # rand_n = goal_point
                # near_n = get_nearest_vertex(node_list, rand_n)
                # path_n = steer(near_n.point, rand_n, delta_q)
                # valid = check_path_valid(path_n, state_is_valid)
                # if valid:
                    # new_n = Node(path_n[11], parent=near_n)
                    # new_n.path_from_parent = path_n
                    # node_list.append(new_n)
                    # if goal_point is not None:
                        # if (np.linalg.norm(new_n.point - goal_point) < 0.00001):
                            # break
                    # count_k+=1
        
            # return node_list
        
        
        # nodes = rrt(state_bounds, state_is_valid, start_point, goal_point, 50, np.linalg.norm(state_bounds/10))
        # print(nodes)
    # initialize motor speeds at 50% of MAX_SPEED.
    #leftSpeed  = 0.5 * MAX_SPEED
    #rightSpeed = 0.5 * MAX_SPEED
    # modify speeds according to obstacles
    #if forward_obstacle:
         #turn right
        #leftSpeed  = 0#0.5 * MAX_SPEED
        #rightSpeed = -0.5 * MAX_SPEED
    #elif backward_obstacle:
         #turn left
        #leftSpeed  = -0.5 * MAX_SPEED
        #rightSpeed = 0.5 * MAX_SPEED
    # write actuators inputs
    #leftMotor.setVelocity(leftSpeed)
    #rightMotor.setVelocity(rightSpeed)
