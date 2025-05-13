import random
import os

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from classtest import Stand
import pybullet as pb
import time
from CameraSetUp import imgSide


from help import fromvVectorToAngel,fromAngelToVector,angle180
import help
class Test:
    'Creat a stand'
    def __init__(self,flag,targetPos = None, refAng = None,standParam = None,
                 dt = 1/240,maxTime =10, numberOfTests = 3,
                 controlFuncAndParam = [help.orientControlSetedLinSpeed,["refOrientation","robotOrientation"]]):
        flagAngle = False
        self.flag = flag
        self.func = controlFuncAndParam[0]
        if self.flag == "Simulation":
            for k in range(numberOfTests):
                if standParam == None:
                    self.stand = Stand()
                else:
                    self.stand = Stand(standParam)
                self.stand.setCamera()
                aruco = pb.loadURDF(fileName='aruco.urdf', basePosition=[2, 2, 1], useFixedBase=True)
                pb.changeVisualShape(aruco, -1, textureUniqueId=pb.loadTexture('aruco_cube.png'))
                itter = 0
                if targetPos == None:
                    kx = random.uniform(0.2, 0.8)
                    ky = random.uniform(0.2, 0.8)
                    targetPosition = [int(kx * imgSide), imgSide - int(ky * imgSide)]
                    print("targetPosition - ", targetPosition)
                else:
                    targetPosition= targetPos
                if refAng == None:
                    ka = random.uniform(-1.0, 1.0)
                    refAngle = 180 * ka
                else:
                    refAngle= refAng
                for controlParam in controlFuncAndParam[1]:
                    if controlParam == "refPosision":
                        print("targetPosition - ",targetPosition)
                    elif controlParam == "refOrientation":
                        print("refAngle - ",refAngle)
                        flagAngle =True
                logTime = np.arange(0.0, maxTime, dt)
                sz = len(logTime)
                logPosX = np.zeros(sz)
                logPosY = np.zeros(sz)
                logAngel = np.zeros(sz)
                reflogPosX = np.zeros(sz)
                reflogPosY = np.zeros(sz)
                reflogAngel = np.zeros(sz)
                while itter < sz:
                    self.controlParam = []
                    self.stand.setArucoOnRobot(self.stand.robots[0], aruco)
                    image = self.stand.getCvImage()
                    det = self.stand.detectAruco(image)
                    cv.circle(det[0], (targetPosition[0], imgSide - targetPosition[1]), 4, (0, 0, 255), -1)
                    if(flagAngle):
                        vec = fromAngelToVector(refAngle)
                        cv.line(det[0], [targetPosition[0], imgSide - targetPosition[1]],
                                [targetPosition[0] + int(vec[0] * 10), imgSide - targetPosition[1] - int(vec[1] * 10)],
                                (0, 0, 255), 2)
                    cv.imshow('frame', det[0])
                    if cv.waitKey(1) == ord('q'):
                        break
                    try:
                        for controlParam in controlFuncAndParam[1]:
                            if controlParam == "refPosision":
                                self.controlParam.append(targetPosition)
                            elif controlParam == "refOrientation":
                                self.controlParam.append(refAngle)
                            elif controlParam == "robotPosision":
                                self.controlParam.append(det[1][0])
                            elif controlParam == "robotOrientation":
                                self.controlParam.append(angle180(det[2][0]))
                            elif controlParam == "time":
                                self.controlParam.append(dt)
                            else:
                                print("ParamIsNotRecognized - ", controlParam)
                                exit()
                        control = self.func(*tuple(self.controlParam))
                        print("control lw- ", control[0][0], "rw-", control[0][1])
                        self.stand.setControl([control[0]])
                        logPosX[itter] = det[1][0][0]
                        logPosY[itter] = det[1][0][1]
                        logAngel[itter] = angle180(det[2][0])
                        print(det[1][0], "robot angle ", angle180(det[2][0]))
                        print(angle180(fromvVectorToAngel(
                            [targetPosition[0] - det[1][0][0], targetPosition[1] - det[1][0][1]])))
                    except:
                        print("no aruco,end test ")
                        break
                    reflogAngel[itter] = refAngle
                    reflogPosX[itter] = targetPosition[0]
                    reflogPosY[itter] = targetPosition[1]
                    itter += 1
                    if (control[1] == True):
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

                name = "\control_" + str("a") + "_x_" + str(targetPosition[0]) + '_y_' + str(
                    targetPosition[1]) + 'maxTime_' + str(maxTime) + 'refAngle_' + str(refAngle) + '.png'
                path = os.getcwd() + "\\plots" + name
                plt.savefig(path)
                plt.close('all')
                pb.disconnect()
        elif self.flag == "Real":
            from classtest import RealCamera
            from UDP1 import UDP
            logTime = [0]
            logPosX = []
            logPosY = []
            logAngle = []
            reflogPosX = []
            reflogPosY = []
            reflogAngle = []
            cam = RealCamera()
            if not cam.cap.isOpened():
                print("Cannot open camera")
                exit()
            udp = UDP()
            iterr = 0
            yImage = cam.shape[0]
            xImage = cam.shape[1]
            if targetPos == None:
                kx = random.uniform(0.2, 0.8)
                ky = random.uniform(0.2, 0.8)
                targetPosition = (int(kx * xImage), yImage - int(ky * yImage))
            else:
                targetPosition = targetPos
            print("x_lenfth - ", xImage, "y_lenfth - ", yImage)
            print("target -", targetPosition)
            if refAng == None:
                ka = random.uniform(-1.0, 1.0)
                refAngle = 180 * ka
            else:
                refAngle = refAng
            for controlParam in controlFuncAndParam[1]:
                if controlParam == "refOrientation":
                    print("refAngle - ", refAngle)
                    flagAngle = True
            param = cam.calibrateCamera()
            tStart = time.time()
            # print("time from the beginning of the loops -", tStart)
            while True:
                t0 = time.time()
                det = cam.detectAruco(calibrateParam=param)
                t1 = time.time()
                cv.circle(det[0], (targetPosition[0], yImage - targetPosition[1]), 4, (0, 0, 255), -1)
                if (flagAngle):
                    vec = fromAngelToVector(refAngle)
                    cv.line(det[0], [targetPosition[0], yImage - targetPosition[1]],
                            [targetPosition[0] + int(vec[0] * 10), yImage - targetPosition[1] - int(vec[1] * 10)],
                            (0, 0, 255), 2)
                if iterr == 0:
                    startImage = det[0]
                    try:
                        logPosX.append(det[1][0][0])
                        logPosY.append(det[1][0][1])
                        logAngle.append(det[2][0])
                    except:
                        logPosX.append(0)
                        logPosY.append(0)
                        logAngle.append(0)

                    reflogPosX.append([targetPosition[0]])
                    reflogPosY.append([targetPosition[1]])
                    if flagAngle:
                        reflogAngle.append(refAngle)
                    else:
                        reflogAngle.append([0])
                # Cam.out.write(inf[0])
                cv.imshow('realFrame', det[0])
                if cv.waitKey(1) & 0xFF == ord('q'):
                    # Cam.out.release()
                    udp.send(";" + str(iterr) + ".0,0/:")
                    time.sleep(0.2)
                    udp.send(";" + str(iterr) + ".0,0/:")
                    time.sleep(0.2)
                    udp.send(";" + str(iterr) + ".0,0/:")
                    time.sleep(0.2)
                    udp.send(";" + str(iterr) + ".0,0/:")
                    break
                t2 = time.time()
                try:
                    for controlParam in controlFuncAndParam[1]:
                        if controlParam == "refPosision":
                            self.controlParam.append(targetPosition)
                        elif controlParam == "refOrientation":
                            self.controlParam.append(refAngle)
                        elif controlParam == "robotPosision":
                            self.controlParam.append(det[1][0])
                        elif controlParam == "robotOrientation":
                            self.controlParam.append(angle180(det[2][0]))
                        elif controlParam ==  "time":
                            self.controlParam.append(t2 - t0)
                        else:
                            print("ParamIsNotRecognized - ", controlParam)
                            exit()
                    control = self.func(*tuple(self.controlParam))
                    if not control[1]:
                        udp.send(";" + str(iterr) + control[0])
                        logPosX.append(det[1][0][0])
                        logPosY.append(det[1][0][1])
                        logAngle.append(det[2][0])
                    else:
                        break
                    print(det[1][0], det[2][0])
                except:
                    udp.send(";" + str(iterr) + ".10,10/:")
                    logPosX.append(logPosX[iterr])
                    logPosY.append(logPosY[iterr])
                    logAngle.append(logAngle[iterr])
                reflogPosX.append([targetPosition[0]])
                reflogPosY.append([targetPosition[1]])
                if flagAngle:
                    reflogAngle.append(refAngle)
                else:
                    reflogAngle.append([0])
                logTime.append(logTime[iterr] + (t2 - t0))
                iterr = iterr + 1
                # print("time from starting getting image to getting coordinates -", t1-t0, "beginning -", t0, "ending -", t1)

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
            plt.plot(logTime, logAngle, label="simAngle")
            plt.plot(logTime, reflogAngle, label="refAngle")
            plt.legend()

            name = "\controlReal_" + str("a") + "_x_" + str(targetPosition[0]) + '_y_' + str(
                targetPosition[1]) + 'maxTime_' + str(round(logTime[len(logTime) - 1], 2)) + 'refAngle_' + str(
                reflogAngle[0]) + '.png'
            path = os.getcwd() + "\\plots" + name
            plt.savefig(path)
            plt.close('all')
            path = os.getcwd() + "\\trajectories" + name
            help.saveTraectory(startImage, path, logPosX, logPosY, yImage, targetPosition)
        else:
            print("unknown flag- ",flag, " expected Simulation or Real")
            exit()


'''
control ="posAndOrientControl"
flag = "Simulation" # Real or Simulation
#flag = "Real"
if flag == "Simulation":
    for i in range(25):
        kx= random.uniform(0.2, 0.8)
        ky= random.uniform(0.2, 0.8)
        ka = random.uniform(-1.0, 1.0)
        print(kx,ky,ka)
        targetPosition = [int(kx*imgSide), int(ky*imgSide)]
        refAngle = 180*ka
        itter = 0
        dt = 1 / 240
        maxTime = 10
        logTime = np.arange(0.0, maxTime, dt)
        sz = len(logTime)
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
            if control == "posAndOrientControl":
                vec = fromAngelToVector(refAngle)
                cv.line(det[0], [targetPosition[0], targetPosition[1]],
                        [targetPosition[0] + int(vec[0] * 10), targetPosition[1] - int(vec[1] * 10)],
                        (0, 0, 255), 2)
            cv.imshow('frame', det[0])
            if cv.waitKey(1) == ord('q'):
                break
            try:
                #flag = posControler([targetPosition[0], imgSide - targetPosition[1]],det[1][0],det[2][0])
                #flag = help.realposControlerPI([targetPosition[0], imgSide - targetPosition[1]], det[1][0], det[2][0],dt)
                #flag = help.orientControlSetedLinSpeed(det[2][0],refAngle)
                flag = help.posAndOrientControl([targetPosition[0], imgSide - targetPosition[1]],refAngle,det[1][0],angle180(det[2][0]))
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
        path = os.getcwd() + "\\plots" + name
        plt.savefig(path)
        plt.close('all')
        pb.disconnect()


elif (flag == "Real"):
    from classtest import RealCamera
    from UDP1 import UDP
    import time

    logTime = [0]
    logPosX = []
    logPosY = []
    logAngle = []
    reflogPosX = []
    reflogPosY = []
    reflogAngle = []
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
    param = Cam.calibrateCamera()
    tStart = time.time()
    #print("time from the beginning of the loops -", tStart)
    while True:
        t0 = time.time()
        inf = Cam.detectAruco(calibrateParam = param)
        t1 = time.time()
        cv.circle(inf[0], (targetPosition[0],yImage-targetPosition[1]), 4, (0, 0, 255), -1)
        if iterr ==0:
            startImage = inf[0]
            logPosX.append(inf[1][0][0])
            logPosY.append(inf[1][0][1])
            logAngle.append(inf[2][0])
            reflogPosX.append([targetPosition[0]])
            reflogPosY.append([targetPosition[1]])
            reflogAngle.append([0])
        #Cam.out.write(inf[0])
        cv.imshow('realFrame',inf[0])
        if cv.waitKey(1) & 0xFF == ord('q'):
            #Cam.out.release()
            udp.send(";" + str(iterr) + ".0,0/:")
            time.sleep(0.2)
            udp.send(";" + str(iterr) + ".0,0/:")
            time.sleep(0.2)
            udp.send(";" + str(iterr) + ".0,0/:")
            time.sleep(0.2)
            udp.send(";" + str(iterr) + ".0,0/:")
            break
        t2 = time.time()
        try:
            result = help.realposControlerPI(targetPosition,inf[1][0],inf[2][0],t2-t0)
            if not result[1]:
                udp.send(";" + str(iterr)+result[0])
                logPosX.append(inf[1][0][0])
                logPosY.append(inf[1][0][1])
                logAngle.append(inf[2][0])
            else:
                break
            print(inf[1][0],inf[2][0])
        except:
            udp.send(";"+str(iterr)+".10,10/:")
            logPosX.append(logPosX[iterr])
            logPosY.append(logPosY[iterr])
            logAngle.append(logAngle[iterr])
            t2 = time.time()
        reflogPosX.append([targetPosition[0]])
        reflogPosY.append([targetPosition[1]])
        reflogAngle.append([0])
        logTime.append(logTime[iterr] + (t2 - t0))
        iterr = iterr + 1
        #print("time from starting getting image to getting coordinates -", t1-t0, "beginning -", t0, "ending -", t1)

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
    plt.plot(logTime, logAngle, label="simAngle")
    plt.plot(logTime, reflogAngle, label="refAngle")
    plt.legend()

    name = "\controlReal_" + str(control) + "_x_" + str(targetPosition[0]) + '_y_' + str(
        targetPosition[1]) + 'maxTime_' + str(round(logTime[len(logTime)-1],2)) + 'refAngle_' + str(reflogAngle[0]) + '.png'
    path = os.getcwd() +  "\\plots" + name
    plt.savefig(path)
    plt.close('all')
    path = os.getcwd() + "\\trajectories" + name
    help.saveTraectory(startImage,path,logPosX,logPosY,yImage,targetPosition)
'''

