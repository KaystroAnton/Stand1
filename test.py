import random
import os
import cv2 as cv

from help import positionControl, simPositionControl,posControler, fromvVectorToAngel,posAndOrientControl,fromAngelToVector,angle180,orientControlSetedLinSpeedTrue
import numpy as np
import matplotlib.pyplot as plt
control ="orientControlSetedLinSpeedNew"
flag = "Simulation" # Real or Simulation
flag = "Real"
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
    for i in range(1):
        kx= random.uniform(0.2, 0.8)
        ky= random.uniform(0.2, 0.8)
        ka = random.uniform(-1.0, 1.0)
        print(kx,ky,ka)
        targetPosition = [int(kx*imgSide), int(ky*imgSide)]
        refAngle = 30
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
            cv.circle(det[0], (targetPosition[0], targetPosition[1]), 4, (0, 0, 255), -1)
            if control == "orientControlSetedLinSpeed":
                vec = fromAngelToVector(refAngle)
                cv.line(det[0], [targetPosition[0], targetPosition[1]],
                        [targetPosition[0] + int(vec[0] * 10), targetPosition[1] - int(vec[1] * 10)],
                        (0, 0, 255), 2)
            cv.imshow('frame', det[0])
            if cv.waitKey(1) == ord('q'):
                break
            try:
                flag = posControler([targetPosition[0], imgSide - targetPosition[1]],det[1][0],det[2][0])
                print("control lw- ", flag[0][0], "rw-",flag[0][1])
                stand.setControl([flag[0]])
                logPosX[itter] = det[1][0][0]
                logPosY[itter] = det[1][0][1]
                logAngel[itter] = angle180(det[2][0])
                reflogAngel[itter] = refAngle
            except:
                print("no aruco " )
                break
            try:
                print(det[1][0], "robot angle ", float(det[2][0]))
                print(fromvVectorToAngel([targetPosition[0] - det[1][0][0],imgSide - targetPosition[1] - det[1][0][1]]))
            except:
                print("no aruco ")
            reflogPosX[itter] = targetPosition[0]
            reflogPosY[itter] = imgSide - targetPosition[1]
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
        plt.plot(logTime, logPosY, label="simPosY")
        plt.plot(logTime, reflogPosY, label="refPosY")
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.grid(True)
        plt.plot(logTime, logAngel, label="simAngle")
        plt.plot(logTime, reflogAngel, label="refAngle")
        plt.legend()

        name = "\control_" +str(control) + "_x_" + str(targetPosition[0])+'_y_'+str(imgSide - targetPosition[1])+'maxTime_'+str(maxTime)+'refAngle_'+str(refAngle)+'.png'
        save ="D:\pythonProjects\RoboFoot-master\plots"+name
        print(os.getcwd()+"plots"+name)
        print(save)
        plt.savefig(save)
        plt.close('all')
        pb.disconnect()


elif (flag == "Real"):
    from classtest import RealCamera
    from UDP1 import UDP
    import time
    Cam = RealCamera()
    if not Cam.cap.isOpened():
        print("Cannot open camera")
        exit()
    udp = UDP()
    iterr = 0
    kx = random.uniform(0.2, 0.8)
    ky = random.uniform(0.2, 0.8)
    ka = random.uniform(-1.0, 1.0)
    yImage =Cam.shape[0]
    xImage = Cam.shape[1]
    targetPosition = (int(kx*xImage),yImage-int(ky*yImage))
    print("x_lenfth - ",xImage,"y_lenfth - ",yImage)
    print("target -",targetPosition)
    #param = Cam.calibrateCamera()
    print("time from the beginning of the loops -", time.time())
    while True:
        t0 = time.time()
        inf = Cam.detectAruco()
        t1 = time.time()
        cv.circle(inf[0], (targetPosition[0],yImage-targetPosition[1]), 4, (0, 0, 255), -1)
        cv.imshow('realFrame',inf[0])
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        try:
            udp.send(positionControl(targetPosition,inf[1][0],inf[2][0]))
            t2 = time.time()
        except:
            udp.send(";"+str(iterr)+".100,100/:")
            t2 = time.time()
        iterr = iterr+1
        print("time from starting getting image to getting ccordinates -", t1-t0, "beginning -", t0, "ending -", t1)


