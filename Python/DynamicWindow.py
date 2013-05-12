import math, random, pygame, time
from threading import Thread
from pygame.locals import *

from Geometry import GLib
from ClearanceCalculator import CLib
from Plotter import Plotter

CLOSE_ENOUGH_TO_GOAL = .5

ANGULAR_ACCEL = 1.0
ANGULAR_INC = .1
ANGULAR_MIN = -.4
ANGULAR_MAX = .4

LINEAR_ACCEL = 1.0
LINEAR_INC = .1
LINEAR_MIN = 0
LINEAR_MAX = .75

DT = .1

CLEARANCE_MIN = .2
CLEARANCE_MAX = 4

LINEAR_WEIGHT = .25
CLEARANCE_WEIGHT = .2
GOAL_DIR_WEIGHT = .4

SLOW_DOWN_RADIUS = 1.0
SIZE_RADIUS = .6
BUFFER_SPACE = .1

NUM_OBSTACLES = 200
    
def DynamicWindow(
    action_out,
    cur_x,
    cur_y,
    cur_dir,
    cur_linear,
    cur_angular,
    mycloud,
    goal_x, 
    goal_y
    ):
    
    distToGoal = GLib.euclid(goal_x, goal_y, cur_x, cur_y)
    
    if distToGoal < CLOSE_ENOUGH_TO_GOAL:
        action_out.linear = 0
        action_out.angular = 0
        return
    
    best_weight = -1
    best_linear = None
    best_angular = None
    best_found = False
    
    for angular in range(ANGULAR_MIN, ANGULAR_MAX+ANGULAR_INC, ANGULAR_INC):
        if angular < cur_angular - ANGULAR_ACCEL or \
           angular > cur_angular + ANGULAR_ACCEL:
            continue
        
        for linear in range(LINEAR_MIN, LINEAR_MAX+ANGULAR_INC, LINEAR_INC):
            if linear < cur_linear - LINEAR_ACCEL or \
               linear > cur_linear + LINEAR_ACCEL:
                continue
            
            #
            # get clearance and normalize
            #
            res = CLib.calcIntersection(
                cur_x, 
                cur_y, 
                cur_dir, 
                linear, 
                angular, 
                mycloud
                )
                
            clearance = CLEARANCE_MAX
            color = "gray"
                
            if res.point and res.delta < CLEARANCE_MAX:
                clearance = res.delta
                color = "red"
            
                if clearance < CLEARANCE_MIN:
                    continue
            
            clearanceNorm = clearance/CLEARANCE_MAX
            
            #
            # get normalized goal direction weight
            #
            newDir = cur_dir + angular*DT
            goalDir = math.atan2(goal_y - cur_y, goal_x - cur_x)
            goalDirDif = math.pi - math.abs(GLib.angleDif(goalDir, newDir))
            goalDirDifNorm = goalDirDif/math.pi
            
            #
            # get normalized linear velocity weight
            #
            linearNorm = linear/LINEAR_MAX
            
            #
            # get the final weight, compare it with what we've seen so far
            #
            weight = clearanceNorm*CLEARANCE_WEIGHT + \
                         goalDirDifNorm*GOAL_DIR_WEIGHT + \
                         linearNorm*LINEAR_WEIGHT
            
            if weight > best_weight:
                best_weight = weight
                best_linear = linear
                best_angular = angular
                best_found = True
            
            #
            # plot trajectory & intersection point, if there was one
            #
            if res.point:
                plotter.plotPoint(res.point.x, res.point.y, .1, color)
            
            if res.traj.type == "circle":
                plotter.plotCircle(res.traj.x, res.traj.y, res.traj.r, color)
            elif res.traj.type == "vector":
                plotter.plotVector(res.traj.x, res.traj.y, res.traj.dir, color)
    
    if not best_found or (math.abs(best_linear) <= 1e-6 and math.abs(best_angular) <= 1e-6):
        action_out.linear = 0
        action_out.angular = 0
        return
    
    if distToGoal < SLOW_DOWN_RADIUS:
        best_linear *= (distToGoal/SLOW_DOWN_RADIUS)
    
    action_out.linear = best_linear
    action_out.angular = best_angular

cur_x = 0
cur_y = 0
cur_dir = 0
goal_x = 5
goal_y = 5
cur_lin = 0
cur_ang = 0
cloud = []

class CloudPoint:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

for i in range(NUM_OBSTACLES):
    cloud.append(CloudPoint(
        random.random()*26-13, 
        random.random()*14-7,
        SIZE_RADIUS + BUFFER_SPACE
        ))

def plotEverything():
    plotter.clear()
    
    plotter.plotPoint(cur_x, cur_y, CLEARANCE_MAX, "lightBlue")
    plotter.plotPoint(cur_x, cur_y, SIZE_RADIUS, "lightGreen")
    plotter.plotCircle(cur_x, cur_y, SIZE_RADIUS, "darkGreen")
    plotter.drawAxises(1, 1)
    
    for cloudPoint in cloud:
        plotter.plotPoint(cloudPoint.x, cloudPoint.y, 0.05, "black")
        plotter.plotCircle(cloudPoint.x, cloudPoint.y, cloudPoint.r, "gray")
    
    plotter.plotPoint(goal_x, goal_y, 0.1, "blue")
    plotter.display()

class Action:
    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular
    
def step():
    global cur_x, cur_y, cur_dir, cur_lin, cur_ang, goal_x, goal_y

    plotEverything()
    
    action_out = Action(0, 0)
    
    mycloud = []
    for cloudPoint in cloud:
        if GLib.euclid(cloudPoint.x, cloudPoint.y, cur_x, cur_y) < CLEARANCE_MAX:
            mycloud.append(cloudPoint);

    DynamicWindow(
        action_out,
        cur_x,
        cur_y,
        cur_dir,
        cur_lin,
        cur_ang,
        mycloud,
        goal_x,
        goal_y
        )
    
    cur_lin = action_out.linear
    cur_ang = action_out.angular

    kine = GLib.calcTrajectoryStepFromTime(
        cur_x, 
        cur_y, 
        cur_dir, 
        cur_lin, 
        cur_ang,
        DT
        )
    
    cur_x = kine[0]
    cur_y = kine[1]
    cur_dir = kine[2]
    
    plotter.plotArrow(cur_x, cur_y, cur_dir, "red")

def onmousedown(x, y):
    global goal_x, goal_y

    coords = plotter.getPlotCoords(x, y)
    goal_x = coords[0]
    goal_y = coords[1]
    
WINDOW_WIDTH = 1500
WINDOW_HEIGHT = 700

if __name__ == '__main__':
    print "everything compiles!"
    
    pygame.init()
    window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    plotter = Plotter(window, WINDOW_WIDTH, WINDOW_HEIGHT, .5, .5, 50, 50)
    plotEverything()

    Thread(target=step, args=[]).start()
    
    while True:
        for event in pygame.event.get():
            if event.type == MOUSEBUTTONDOWN:  
                if event.button == 1:
                    onmousedown(event.pos[0], event.pos[1])
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
                    
