import random
import os
import cv2 as cv

import CameraSetUp
from help import positionControl, simPositionControl,posControler, fromvVectorToAngel,posAndOrientControl,fromAngelToVector,angle180,orientControlSetedLinSpeed
import numpy as np
import matplotlib.pyplot as plt
control ="orientControlSetedLinSpeed"
flag = "Simulation" # Real or Simulation
#flag = "Real"
dt = 1/240
maxTime = 10
logTime = np.arange(0.0, maxTime, dt)
sz = len(logTime)
logPosX = np.zeros(sz)
logPosY = np.zeros(sz)
logAngel = np.zeros(sz)
reflogPosX = np.zeros(sz)
reflogPosY = np.zeros(sz)
reflogAngel = np.zeros(sz)
if flag == "Simulation":
    from classtest import Stand
    import pybullet as pb
    import time
    from CameraSetUp import imgSide,halfFieldSize
    targetPosition = [imgSide, imgSide]
    print(targetPosition)
    for i in range(30):
        kx= random.uniform(0.2, 0.8)
        ky= random.uniform(0.2, 0.8)
        ka = random.uniform(-1.0, 1.0)
        print(kx,ky,ka)
        targetPosition = [int(kx*imgSide), int(ky*imgSide)]
        refAngel = ka*180
        itter = 0
        logPosX = np.zeros(sz)
        logPosY = np.zeros(sz)
        logAngel = np.zeros(sz)
        reflogPosX = np.zeros(sz)
        reflogPosY = np.zeros(sz)
        reflogAngel = np.zeros(sz)
        # print(stand.detectAruco.__doc__) #use for info
        stand = Stand(coordinates_of_robots=[[halfFieldSize, halfFieldSize, 0.08]])
        stand.setCamera()
        #pb.addUserDebugLine([0, 0, 0], [0 ,0, 0.5], [0, 0, 1],1,parentObjectUniqueId= pb.getBodyUniqueId(stand.robots[0]))
        #pb.addUserDebugLine([0, 0, 0], [0 + 0.5, 0, 0], [1, 0, 0],1, parentObjectUniqueId= pb.getBodyUniqueId(stand.robots[0]))
        #pb.addUserDebugLine([0,0, 0], [0, 0 + 0.5, 0], [0, 1, 0],1,parentObjectUniqueId= pb.getBodyUniqueId(stand.robots[0]))
        # Это надо доделать
        aruco = pb.loadURDF(fileName='aruco.urdf', basePosition=[2, 2, 1], useFixedBase=True)
        pb.changeVisualShape(aruco, -1, textureUniqueId=pb.loadTexture('aruco_cube.png'))
        #
        stand.setControl([[100, 100]])
        while itter<sz:
            stand.setArucoOnRobot(stand.robots[0],aruco)
            image = stand.getCvImage()
            det = stand.detectAruco(image)
            #
            cv.circle(det[0], (targetPosition[0], imgSide - targetPosition[1]), 4, (0, 0, 255), -1)
            if control == "orientControlSetedLinSpeed":
                vec = fromAngelToVector(refAngel)
                cv.line(det[0], [targetPosition[0], imgSide - targetPosition[1]],
                        [targetPosition[0] + int(vec[0] * 10), imgSide - targetPosition[1] - int(vec[1] * 10)],
                        (0, 0, 255), 2)
            cv.imshow('frame', det[0])
            if cv.waitKey(1) == ord('q'):
                break
            try:
                flag = orientControlSetedLinSpeed(angle180(det[2][0]),refAngel)
                print("control lw- ", flag[0][0], "rw-",flag[0][1])
                stand.setControl([flag[0]])
                logPosX[itter] = det[1][0][0]
                logPosY[itter] = det[1][0][1]
                logAngel[itter] = angle180(det[2][0])
                reflogAngel[itter] = refAngel
            except:
                print("no aruco " )
            try:
                print(det[1][0], "robot angel ", float(det[2][0]))
                print(fromvVectorToAngel([targetPosition[0] - det[1][0][0], targetPosition[1] - det[1][0][1]]))
            except:
                print("no aruco ")
            reflogPosX[itter] = targetPosition[0]
            reflogPosY[itter] = targetPosition[1]
            itter += 1
            if(flag[1] == True):
                break
            pb.stepSimulation()
            time.sleep(dt)

        plt.subplot(3, 1, 1)
        plt.grid(True)
        plt.plot(logTime, logPosX, label="simPosX")
        plt.plot(logTime, reflogPosX, label="refPosX")
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.grid(True)
        plt.plot(logTime, logPosY, label="ssimPosY")
        plt.plot(logTime, reflogPosY, label="refPosY")
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.grid(True)
        plt.plot(logTime, logAngel, label="simAngel")
        plt.plot(logTime, reflogAngel, label="refAngel")
        plt.legend()
        name = "\control_" +str(control) + "_x_" + str(targetPosition[0])+'_y_'+str(targetPosition[1])+'maxTime_'+str(maxTime)+'.png'
        save ="D:\pythonProjects\RoboFoot-master\plots"+name
        print(os.getcwd()+"plots"+name)
        print(save)
        plt.savefig(save)
        plt.close('all')
        pb.disconnect()


elif (flag == "Real"):
    from classtest import RealCamera
    from UDP1 import UDP
    Cam = RealCamera()
    udp = UDP()
    #param = Cam.calibrateCamera()
    while True:
        if not Cam.cap.isOpened():
            print("Cannot open camera")
            exit()
        inf = Cam.detectAruco()
        cv.circle(inf[0], (targetPosition[0], targetPosition[1]), 4, (0, 0, 255), -1)
        cv.imshow('realFrame',inf[0])
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        try:
            udp.send(positionControl(targetPosition,inf[1][0],inf[2][0]))
        except:
            udp.send(";100,100/:")


