import random
import pybullet as pb
import time
import cv2 as cv
import math
import numpy as  np
from pybullet import getLinkState, getNumJoints



# parameters of simulation
L = 0.14    # length of the corpus
d = 0.064     # diameter of wheels
dx = 0.1    # distance from wheel axle to center
alpha = 0   # the angle of the robot's course in radians (relative to the X axis)
dt = 1/240  # pybullet simulation step
g = -9.8    # Gravity force
cameraHeight = 3.0 # Z coordinate of camera
IMG_SIDE = 200
halfFieldSize = 4.0/2
accuracy = 0.0005
kd = 20 # koeff for control
v = 1 # speed of the robot
targetPosition = [[-2,1],0.0]    # [[positon x,y], oriantation angel] the aim of the robot
xd = targetPosition[0][0]
yd = targetPosition[0][1]

physicsClient = pb.connect(pb.GUI)  # pb.GUI for graphical version

# camera settings
pb.resetDebugVisualizerCamera(
    cameraDistance=1,
    cameraYaw=90,
    cameraPitch=89.999,    # No image if set on 90
    cameraTargetPosition=[0.0, 0.0, cameraHeight]
)   # Set сamera directly above the field
camera_proj_matrix = pb.computeProjectionMatrixFOV(
                    fov=2 * math.atan(halfFieldSize / cameraHeight) * 180 / math.pi,
                    aspect=1,
                    nearVal=1,
                    farVal=3.5)
viewMatrix = pb.computeViewMatrix(cameraEyePosition=pb.getDebugVisualizerCamera()[11],
                                      cameraTargetPosition=[0, 0, 0.01],
                                      cameraUpVector=[1.0, 0, 0])
pb.setGravity(0,0, g)

# load robots and field
field = pb.loadURDF("field.urdf",[0,0,0])
robot1 = pb.loadURDF("robot.urdf",[0,0,0.25])

# add aruco cube and aruco texture
c1 = pb.loadURDF('aruco.urdf', (halfFieldSize-0.15, halfFieldSize-0.15, 0.0), useFixedBase=True)
c2 = pb.loadURDF('aruco.urdf', (-1, 0, 0.0), useFixedBase=True)
x = pb.loadTexture('aruco_cube.png')
pb.changeVisualShape(c1, -1, textureUniqueId=x)
pb.changeVisualShape(c2, -1, textureUniqueId=x)
# get rid of all the default damping forces
pb.changeDynamics(robot1, 1, linearDamping=0, angularDamping=0)
pb.changeDynamics(robot1, 2, linearDamping=0, angularDamping=0)
pb.changeDynamics(robot1, 3, linearDamping=0, angularDamping=0)

#   settings for robot
pb.setJointMotorControl2(bodyIndex=robot1, jointIndex=1, targetVelocity= 10, controlMode=pb.VELOCITY_CONTROL,force=100)
pb.setJointMotorControl2(bodyIndex=robot1, jointIndex=0, targetVelocity= 10, controlMode=pb.VELOCITY_CONTROL,force=100)
#pb.getAxisAngleFromQuaternion(pb.getBasePositionAndOrientation(robot1)[1])
#pb.resetBasePositionAndOrientation(robot1, posObj = [0,0,0], ornObj = pb.getQuaternionFromAxisAngle([0.0, 0.0, 1.0], 0))
while True:
    pb.getCameraImage(
                    width=IMG_SIDE,
                    height=IMG_SIDE,
                    viewMatrix=viewMatrix,
                    projectionMatrix=camera_proj_matrix,
                    renderer=pb.ER_TINY_RENDERER
                ) # get frame from camera
    '''
    while True:
        alpha = round(pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(c2)[1])[2], 3)
        x=pb.getBasePositionAndOrientation(c2)[0][0]
        y=pb.getBasePositionAndOrientation(c2)[0][1]
        for j in range(100):
            while pow(x - xd,2) + pow(y - yd,2) > accuracy:
                print(pb.getEulerFromQuaternion(getLinkState(bodyUniqueId=robot1, linkIndex=0)[1]))
                frame = pb.getCameraImage(
                    width=IMG_SIDE,
                    height=IMG_SIDE,
                    viewMatrix=pb.computeViewMatrix(cameraEyePosition=pb.getDebugVisualizerCamera()[11],
                                                    cameraTargetPosition=[0, 0, 0.01],
                                                    cameraUpVector=[1.0, 0, 0]),
                    projectionMatrix=camera_proj_matrix,
                    renderer=pb.ER_TINY_RENDERER
                ) # get frame from camera
                new_frame = help.resize_frame(frame)
                cv.imshow('frame', new_frame)
                if cv.waitKey(1) == ord('q'):
                    break
                (corners, ids, rejected) = detector.detectMarkers(new_frame)
                #print(corners, ids, rejected)
                omega = kd*(math.atan2((yd - y) , (xd - x)) - alpha)
                movement = [math.cos(alpha)*v*dt,v*dt*math.sin(alpha),0]
                newPosition = [pb.getBasePositionAndOrientation(c2)[0][i]+movement[i] for i in range(3)]
                pb.resetBasePositionAndOrientation(c2, newPosition, pb.getQuaternionFromEuler([0.0, 0.0, alpha+omega*dt]))
                alpha = round(pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(c2)[1])[2], 3)
                x = pb.getBasePositionAndOrientation(c2)[0][0]
                y = pb.getBasePositionAndOrientation(c2)[0][1]
                pb.stepSimulation()
                time.sleep(dt)
            #print("for", targetPosition, "is fine", j)
            targetPosition = [[random.uniform(-2.0, 2.0),random.uniform(-2.0, 2.0)], 0.0]
            #print(targetPosition)
            xd = targetPosition[0][0]
            yd = targetPosition[0][1]
    
        pb.stepSimulation()
        #print(pb.getBasePositionAndOrientation(c2))
        #print(pb.getEulerFromQuaternion(pb.getBasePositionAndOrientation(3)[1]), "dfgdfg")
        #print(pb.getAxisAngleFromQuaternion(pb.getBasePositionAndOrientation(3)[1]), "dfgdfg11111")
        # pb.ER_BULLET_HARDWARE_OPENGL pb.ER_TINY_RENDERER
        # print(pb.getDebugVisualizerCamera()[11]) #position of camera
        time.sleep(dt)
    '''
    time.sleep(dt)
    pb.disconnect()