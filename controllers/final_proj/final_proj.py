"""Final Project controller."""
# -*- coding: utf-8 -*-


#
#Controller might take a couple a seconds
#




from controller import Robot, Motor, Camera, RangeFinder, Lidar, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import convolve2d # Uncomment if you want to use something else for finding the configuration space
import heapq

MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12

prev_aa=0

LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# The Tiago robot has a couple more sensors than the e-Puck
# Some of them are mentioned below. We will use its LiDAR for Lab 5

# range = robot.getDevice('range-finder')
# range.enable(timestep)
# camera = robot.getDevice('camera')
# camera.enable(timestep)
# camera.recognitionEnable(timestep)
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# We are using a keyboard to remote control the robot
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The display is used to display the map. We are using 360x360 pixels to
# map the 10x10m2 maze
display = robot.getDevice("display")

# Odometry
pose_x     = 2.96
pose_y     = -3.07
pose_theta = 0

vL = 0
vR = 0

goal_points=[]

# mode=''

#Uncomment to move the thiago around and map the maze
# mode = 'manual'

#Uncomment to have the robot load the map and contruct a path
#to a goal node and travel there
mode = 'planner'


lidar_sensor_readings = []
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # remove blocked sensor rays

# We are using a GPS and compass to disentangle mapping and localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

###################
#
# Planner
#
###################
if mode == 'planner':
    # start_w = (4.46793,8.05674) # (Pose_X, Pose_Z) in meters
    # end_w = (9.56152,6.76048) # (Pose_X, Pose_Z) in meters

    # Convert the start_w and end_W from webot's coordinate frame to map's
    # start = (int(start_w[0]*30),int(start_w[1]*30)) # (x, y) in 360x360 map
    
       # end = (int(end_w[0]*30),int(end_w[1]*30)) # (x, y) in 360x360 map
    
    # start = (100,100)
    # end = (200, 200)
    # print("Start and End" , start, end)
    
    start=(288, 55)
    
    end = (50,300)
    
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
            #Minimum pathcpst to a node
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
                print("Number of nodes explored:", n_exp)
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
        
    



    map = np.load("map.npy")
    map = np.flipud(map)
    #plt.imshow(np.fliplr(map))
    #plt.imshow(map2)
    #plt.show()


    K = np.ones((30,30))
    
    cspace = convolve2d(map,K,mode="same")
    cspace = np.multiply(cspace>0.8, 1)
    #print(cspace)
    # plt.imshow(cspace)
    # plt.show()
    # print(adjacent_states((190,190),cspace))
    # print(heuristic_eucl((190,190),(283,151)))


    path = thiago_search(cspace, start, end)
    for i in range(len(path)):
        goal_points.append(((path[i][0]/36), (path[i][1]/36)))
        # print(goal_points[i])
    # print(path)
    # print("Length of path", len(path))
    for i in range(len(cspace)):
        for j in range(len(cspace[i])):
            if (j,i) in path:
                cspace[i][j] = 2
                
    plt.imshow(cspace)
    plt.show()
    
    mode='autonmous'




if mode == 'manual':
    map = np.zeros((360,360)) 
    
    # print(map)
    # display.drawPixel(32, 32)
    


state = 0 # use this to iterate through your path

while robot.step(timestep) != -1 and mode != 'planner':

###################
#
# Sensing
#
###################
    # Ground truth pose
    pose_y = gps.getValues()[2] + 5 
    pose_x = gps.getValues()[0] + 5 

    n = compass.getValues()
    rad = -((math.atan2(n[0], n[2]))-1.5708)
    pose_theta = rad

    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            rho = LIDAR_SENSOR_MAX_RANGE

        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        wx =  math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x
        wy =  -(math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y
        
        # print("wx", wx)
        # print("wy", wy)
        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))

        if rho < 0.75*LIDAR_SENSOR_MAX_RANGE:

            try:
                # print("hi")
                if (map[360-int(wy*36),[int(wx*36)]] < .90):
                    map[360-int(wy*36),[int(wx*36)]] = map[360-int(wy*36),[int(wx*36)]] +0.003
                g = int(map[360-int(wy*36),[int(wx*36)]]*255)
                f = (g*256**2+g*256+g)
                display.setColor(int(f))
                display.drawPixel(360-int(wy*36),int(wx*36))
                # print(360-int(wy*36),int(wx*36))
            except:
                # print("hi")
                pass

    display.setColor(int(0xFF0000))
    display.drawPixel(360-int(pose_y*36),int(pose_x*36))



###################
#
# Control the robot to map the maze
# Must click on robot to have it register your keyboard movements
#
#
#
#
###################
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
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
        elif key == ord('S'):

            map = np.multiply(map>0.8, 1)
            np.save("map", map)
            print("Map file saved")
        elif key == ord('L'):
            
            map = np.load("map.npy")
            print("Map loaded")
            print(map)
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    else:
        p=math.sqrt(math.pow((pose_x-goal_points[state][0]),2)+math.pow((pose_y-goal_points[state][1]),2))
        
        
        a=-math.atan2((goal_points[state][1]-pose_y),(goal_points[state][0]-pose_x))
        
        if((prev_aa - a) > 2.1415):
            a = -a
            
        prev_aa = a
        a+=pose_theta
        print("Distnace", p)
        print("Bearing", a)

        if(p < .8):
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
        
        
        if(vL > MAX_SPEED):
            vL = MAX_SPEED/10
        if(vR > MAX_SPEED):
            vR = MAX_SPEED/10
        if(vL < -MAX_SPEED):
            vL = -MAX_SPEED/10
        if(vR < -MAX_SPEED):
            vR = -MAX_SPEED/10
        if(state == len(goal_points)-1):
            vR = 0
            vL = 0


    # Odometry code. Don't change speeds after this
    
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta)) #/3.1415*180))

    # Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)