import os
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import pybullet as pb
import time

import Constants
from classtest import Stand
from help import fromvVectorToAngel,angle180
import help


class Test:
    'init test'
    def __init__(self,flag,targetPos = None, refAng = None,standParam = None,
                 dt = 1/240,maxTime =10, controlFuncAndParam = None, numberOfTests = 1):
        self.flag = flag
        self.targetPos = targetPos
        self.refAng = refAng
        self.standParam = standParam
        self.dt = dt
        self.maxTime = maxTime
        if controlFuncAndParam == None:
            self.controlFuncAndParam = [help.posControlerPI,["refPosition","robotPosition","robotOrientation","time"]]
        else:
            self.controlFuncAndParam = controlFuncAndParam
        self.function_name = help.fromFuncToName(self.controlFuncAndParam[0])
        print("control function -", self.function_name)
        print("control parameters -", self.controlFuncAndParam[1])
        self.flagAngle = False
        self.numberOfTests = numberOfTests
    def run(self):
        'run test with set parameters'
        if self.flag == "Simulation":
            for i in range(self.numberOfTests):
                targetP,refA = help.initCoordAndAngle(self.flag,self.targetPos,self.refAng)
                self.simTest(targetP,refA)
        elif self.flag == "Real":
            self.realTest(*help.initCoordAndAngle(self.flag, self.targetPos, self.refAng))
        else:
            print("unknown flag- ", self.flag, " expected Simulation or Real")
            exit()

    def simTest(self, targetPosition, refAngle):
            if self.standParam == None:
                stand = Stand()
            else:
                stand = Stand(*self.standParam)
            stand.setCamera()
            aruco = pb.loadURDF(fileName='aruco.urdf', basePosition=[2, 2, 1], useFixedBase=True) # can be updated for multi robots
            pb.changeVisualShape(aruco, -1, textureUniqueId=pb.loadTexture('aruco_cube.png')) # can be updated for multi robots
            iterr = 0
            for controlParam in self.controlFuncAndParam[1]:
                if controlParam == "refPosition":
                    print("targetPosition - ",targetPosition)
                elif controlParam == "refOrientation":
                    print("refAngle - ",refAngle)
                    self.flagAngle =True
            logTime = np.arange(0.0, self.maxTime, self.dt)
            sz = len(logTime)
            logTime,logPosX,logPosY,logAngle,reflogPosX,reflogPosY,reflogAngle = [],[],[],[],[],[],[]
            while iterr < sz:
                self.controlParam = []
                stand.setArucoOnRobot(stand.robots[0], aruco) # can be updated for multi robots
                image = stand.getCvImage()
                det = stand.detectAruco(image)
                help.drawAim("Simulation", det[0], self.flagAngle, targetPosition, refAngle)
                cv.imshow('frame', det[0])
                if cv.waitKey(1) == ord('q'):
                    break
                try:
                    self.createParamListOfControl(targetPosition,refAngle,det,self.dt)
                    control = self.controlFuncAndParam[0](*self.controlParam)
                    print("control lw- ", control[0][0], "rw-", control[0][1])
                    stand.setControl([control[0]])
                    logTime.append(iterr * self.dt)
                    logPosX.append(det[1][0][0])
                    logPosY.append(det[1][0][1])
                    logAngle.append(angle180(det[2][0]))
                    print(det[1][0], "robot angle ", angle180(det[2][0]))
                    print(angle180(fromvVectorToAngel(
                        [targetPosition[0] - det[1][0][0], targetPosition[1] - det[1][0][1]])))
                except:
                    print("no aruco,end test ")
                    break
                reflogAngle.append(refAngle)
                reflogPosX.append(targetPosition[0])
                reflogPosY.append(targetPosition[1])
                iterr += 1
                if (control[1] == True):
                    break
                pb.stepSimulation()
                time.sleep(self.dt)

            for controlParam in self.controlFuncAndParam[1]:
                if ((controlParam == "refPosition") and self.flagAngle):
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
                    break
                elif controlParam == "refPosition":
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
                    plt.legend()
                    break
                elif controlParam == "refOrientation":
                    plt.subplot(3, 1, 1)
                    plt.grid(True)
                    plt.plot(logTime, logPosX, label="simPosX")
                    plt.legend()

                    plt.subplot(3, 1, 2)
                    plt.grid(True)
                    plt.plot(logTime, logPosY, label="simPosY")
                    plt.legend()

                    plt.subplot(3, 1, 3)
                    plt.grid(True)
                    plt.plot(logTime, logAngle, label="simAngle")
                    plt.plot(logTime, reflogAngle, label="refAngle")
                    plt.legend()
                    break

            name = ("\controlSim_" + self.function_name + "_x_" + str(targetPosition[0]) + '_y_' + str(targetPosition[1]) +
                    'maxTime_' + str(self.maxTime) + 'refAngle_' + str(refAngle) + '.png')
            path = os.getcwd() + "\\plots" + name
            plt.savefig(path)
            plt.close('all')
            pb.disconnect()

    def realTest(self,targetPosition,refAngle):
        from classtest import RealCamera
        from udp import UDP
        udp = UDP()
        cam = RealCamera()
        logTime = [0]
        logPosX,logPosY,logAngle,reflogPosX,reflogPosY,reflogAngle = [],[],[],[],[],[]
        if not cam.cap.isOpened():
            print("Cannot open camera")
            exit()
        iterr = 0
        yImage = cam.shape[0]
        xImage = cam.shape[1]
        for controlParam in self.controlFuncAndParam[1]:
            if controlParam == "refPosition":
                print("targetPosition - ", targetPosition)
            elif controlParam == "refOrientation":
                print("refAngle - ", refAngle)
                self.flagAngle = True
        param = cam.calibrateCamera()
        while True:
            self.controlParam = []
            t0 = time.time()
            det = cam.detectAruco(calibrateParam=param)
            help.drawAim("Real",det[0],self.flagAngle,targetPosition,refAngle)
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
                reflogAngle.append(refAngle)
            cv.imshow('realFrame', det[0])
            if cv.waitKey(1) & 0xFF == ord('q'):
                for i in range(4):
                    udp.send(";" + str(iterr) + ".0,0/:")
                    time.sleep(0.2)
                break
            t2 = time.time()
            try:
                self.createParamListOfControl(targetPosition,refAngle,det,t2-t0)
                control = self.controlFuncAndParam[0](*tuple(self.controlParam))
                if not control[1]:
                    udp.send(";" + str(iterr) + help.fromSimtoReal(control[0]))
                    logPosX.append(det[1][0][0])
                    logPosY.append(det[1][0][1])
                    logAngle.append(det[2][0])
                else:
                    break
                print("robotPosition - ",det[1][0], "robotOrientation - ", angle180(det[2][0]))
            except:
                udp.send(";" + str(iterr) + ".10,10/:")
                logPosX.append(logPosX[iterr])
                logPosY.append(logPosY[iterr])
                logAngle.append(logAngle[iterr])
            reflogPosX.append([targetPosition[0]])
            reflogPosY.append([targetPosition[1]])
            reflogAngle.append(refAngle)
            logTime.append(logTime[iterr] + (t2 - t0))
            iterr = iterr + 1

        for controlParam in self.controlFuncAndParam[1]: # can be modified to present only the necessary parameters
            if controlParam == "refPosition" and self.flagAngle:
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
                break
            elif controlParam == "refPosition":
                plt.subplot(2, 1, 1)
                plt.grid(True)
                plt.plot(logTime, logPosX, label="simPosX")
                plt.plot(logTime, reflogPosX, label="refPosX")
                plt.legend()

                plt.subplot(2, 1, 2)
                plt.grid(True)
                plt.plot(logTime, logPosY, label="simPosY")
                plt.plot(logTime, reflogPosY, label="refPosY")
                plt.legend()
                break
            elif controlParam == "refOrientation":
                plt.subplot(1, 1, 3)
                plt.grid(True)
                plt.plot(logTime, logAngle, label="simAngle")
                plt.plot(logTime, reflogAngle, label="refAngle")
                plt.legend()
                break

        name = "\controlReal_" + self.function_name + "_x_" + str(targetPosition[0]) + '_y_' + str(
            targetPosition[1]) + 'maxTime_' + str(round(logTime[len(logTime) - 1], 2)) + 'refAngle_' + str(
            reflogAngle[0]) + '.png'
        path = os.getcwd() + "\\plots" + name
        plt.savefig(path)
        plt.close('all')
        path = os.getcwd() + "\\trajectories" + name
        help.saveTraectory(startImage, path, logPosX, logPosY, yImage, targetPosition)

    def createParamListOfControl(self, targetPosition, refAngle, det, time,listForExpand = Constants.listForExpand):
        ''' Create parameter list fo control function. Standard list of parameters
        can be expanded by changing listForExpand in Constants.py and can be changed by formating paramList  '''
        paramList = [["refPosition", targetPosition], ["refOrientation", refAngle],
                     ["robotPosition", det[1][0]], ["robotOrientation", angle180(det[2][0])], ["time", time]]
        for element in listForExpand:
            try:
                help.expandList(paramList,element[0],element[1],element[2])
            except:
                help.expandList(paramList, element[0], element[1])
        print(paramList)
        for controlParam in self.controlFuncAndParam[1]:
            recognized = False
            for Param in paramList:
                if controlParam == Param[0]:
                    self.controlParam.append(Param[1])
                    recognized = True
                    break
            if not recognized:
                print("ParamIsNotRecognized - ", controlParam)
                exit()
