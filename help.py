import random
import numpy as  np
import cv2 as cv
import os
from CameraSetUp import imgSide,RealCameraWidth,RealCameraHeight
accuracyAnge = 10
accuracyCoor = 20
simStoppingAngel = 20
integrateRegulatorSumL = 0
integrateRegulatorSumR = 0
Kp = 0.6
ki = 0.25

def posControler(refPosition,robotPosision,roborOriantation):
    e0 = angle180(fromvVectorToAngel([refPosition[0] - robotPosision[0],refPosition[1] - robotPosision[1]])- roborOriantation)
    print(e0,"e0")
    es = np.sqrt(pow(refPosition[0]-robotPosision[0],2) + pow(refPosition[1]-robotPosision[1],2))*np.cos(e0*np.pi/180.0)
    print(es, "es")
    reg = regulator(e0,es)
    if np.sqrt(pow(refPosition[0] - robotPosision[0], 2) + pow(refPosition[1] - robotPosision[1], 2) > accuracyCoor):
        return [scale(motrorScale(reg[0]), motrorScale(reg[1])), False]
    else:
        return [scale(motrorScale(reg[0]), motrorScale(reg[1])), True]

def orientControlSetedLinSpeed(refOrientation, robotOrientation,linspeed = 60,L=0.23,kProp = 1.5):
    turn = (angle180(refOrientation - robotOrientation) / L)
    if turn >= 100 - linspeed:
        turn = 100 - linspeed
    elif turn <= -100 + linspeed:
        turn = -100 + linspeed
    motorL =  kProp * (linspeed - turn)
    motorR =  kProp * (linspeed + turn)
    if motorL<=60 and  motorL>=0:
        motorL = 60
    elif motorL>=-60 and  motorL<0:
        motorL = -60
    if motorR <= 60 and motorR >= 0:
        motorR = 60
    elif motorR >= -60 and motorR < 0:
        motorR = -60
    return [[motrorScale(motorL),motrorScale(motorR)],False]

def posAndOrientControl(refPosition,refOrientation,robotPosision,robotOriantation):
    betta = angle180(fromvVectorToAngel([refPosition[0] - robotPosision[0],refPosition[1] - robotPosision[1]]))
    gamma = angle180(betta - refOrientation)
    if gamma <=-90:
        gamma = -90
    elif gamma >=90:
        gamma = 90
    print("betta - gamma ", betta - gamma)
    print(gamma)
    xRef = robotPosision[0] +np.sqrt(pow(refPosition[0]-robotPosision[0],2) + pow(refPosition[1]-robotPosision[1],2))* np.cos((betta +gamma)*np.pi/180)
    yRef = robotPosision[1] +np.sqrt(pow(refPosition[0]-robotPosision[0],2) + pow(refPosition[1]-robotPosision[1],2))* np.sin((betta +gamma)*np.pi/180)
    res = posControler([xRef,yRef],robotPosision,robotOriantation)
    if (abs(refOrientation - robotOriantation)<accuracyAnge and res[1]):
        return [[0,0],True]
    else:
        return [res[0],False]


def posControlerPI(refPosition,robotPosision,roborOriantation,t):
    e0 = angle180(fromvVectorToAngel([refPosition[0] - robotPosision[0],refPosition[1] - robotPosision[1]])- roborOriantation)
    print(e0,"e0")
    es = np.sqrt(pow(refPosition[0]-robotPosision[0],2) + pow(refPosition[1]-robotPosision[1],2))*np.cos(e0*np.pi/180.0)
    print(es, "es")
    reg = regulatorPI(t, e0, es)
    if np.sqrt(pow(refPosition[0] - robotPosision[0],2) + pow(refPosition[1] - robotPosision[1],2) > accuracyCoor):
        return [[motrorScale(reg[0]),motrorScale(reg[1])],False]
    else:
        return [[motrorScale(reg[0]),motrorScale(reg[1])], True]

def regulator(angle, dist, L = 0.23, kProp = 0.9):
    motorL =  kProp * (dist - angle /L)
    motorR =  kProp * (dist + angle / L)
    return motorL,motorR

def regulatorPI(time,angle, dist, L = 0.23, kProp = 0.7,kIntegr = 0.25,MaxSpeed = 65):
    global integrateRegulatorSumL
    global integrateRegulatorSumR
    if abs(integrateRegulatorSumL) < 100/kIntegr:
        integrateRegulatorSumL = integrateRegulatorSumL + ((dist - angle / L) * time)/5.0
    motorL = kProp * (dist - angle / L) + kIntegr * integrateRegulatorSumL
    if abs(integrateRegulatorSumR) < 100 / kIntegr:
        integrateRegulatorSumR = integrateRegulatorSumR + ((dist + angle / L) * time)/5.0
    motorR = kProp * (dist + angle / L) + kIntegr * integrateRegulatorSumR
    print(round(dist,2)  ,"- dist", round(angle/ L,2), "- angleRotation", round(integrateRegulatorSumL,2) , "- Lsum",round(integrateRegulatorSumR,2), "-Rsum",round(time,2), " -sum")
    return (motorL,motorR)

def motrorScale(motor,MaxSpeed = 100):
    if motor>=MaxSpeed:
        return MaxSpeed
    elif motor <=-MaxSpeed:
        return -MaxSpeed
    else:
        return  motor

def scale(motorL,motorR):
    scale1 = (abs(motorL) + abs(motorR))/100.0
    return [motorL/scale1,motorR/scale1]

def saveTraectory(frame,path,logX,logY,shapeY,target):
    for i in range(len(logX)):
        cv.circle(frame,[logX[i],shapeY - logY[i]],4,(255, 0 ,0),-1)
    cv.circle(frame, [logX[0], shapeY - logY[0]], 4, (0, 0, 255), -1)
    cv.circle(frame, [target[0],shapeY - target[1]], 4, (0, 0, 255), -1)
    cv.imwrite(path, frame)

def expandList(list,expandElement1,expandElement2,element2Type = float):
    if type(expandElement1) is not str:
        print("can't expend list, first element tipe is ",type(expandElement1) ,"isn't string.")
        exit()
    if type(expandElement2) is not element2Type:
        print("can't expend list, first element tipe is ",type(expandElement2) ,"isn't",element2Type,".")
        exit()
    list.append([expandElement1,expandElement2])

def initCoordAndAngle(flag,targetPos,refAng):
    if flag == "Simulation":
        X, Y = imgSide, imgSide
    else:
        X, Y = RealCameraWidth, RealCameraHeight
    if refAng == None:
        ka = random.uniform(-1.0, 1.0)
        refAngle = 180 * ka
    else:
        refAngle = refAng
    kx = random.uniform(0.2, 0.8)
    ky = random.uniform(0.2, 0.8)
    if targetPos == None:
        targetPosition = [int(kx * X), Y - int(ky * Y)]
    else:
        targetPosition = targetPos
    return targetPosition,refAngle

def drawAim(flag,image,flagAngle,targetPosition,refAngle):
    if flag == "Simulation":
        X,Y = imgSide, imgSide
    else:
        X,Y = RealCameraWidth,RealCameraHeight
    cv.circle(image, (targetPosition[0], Y - targetPosition[1]), 4, (0, 0, 255), -1)
    if flagAngle:
        vec = fromAngelToVector(refAngle)
        cv.line(image, [targetPosition[0], Y - targetPosition[1]],
                [targetPosition[0] + int(vec[0] * 10), Y - targetPosition[1] - int(vec[1] * 10)],
                (0, 0, 255), 2)

def resize_frame(frame):
    pbImg = np.resize(np.asarray(frame[2], dtype=np.uint8), (frame[0], frame[1], 4))
    cvImg = pbImg[:, :, [2, 1, 0]]
    return cvImg

def loadImagesFromFolder(path_folder= 'C:\PythonProjects\RoboFoot\RoboFoot\calibrateimages', max_images=10):
    images = []
    for filename in os.listdir(path_folder):
        if len(images) >= max_images:
            break
        img_path = os.path.join(path_folder, filename)
        if os.path.isfile(img_path):  # Проверяем, является ли это файлом
            img = cv.imread(img_path)
            if img is not None:  # Проверяем, успешно ли загружено изображение
                images.append(img)
    return images

def fromvVectorToAngel(vector, vector1 = None):
    if (vector1 == None):
        vector1 = [1,0]
    """ Returns the angle in radians between vectors 'v1' and 'v2'"""
    vector = vector / np.linalg.norm(vector)
    if (vector[1]>=vector1[1]):
        return np.arccos(np.clip(np.dot(vector,vector1), -1.0, 1.0))/np.pi*180
    elif(vector[1]<vector1[1]):
        return 360 - np.arccos(np.clip(np.dot(vector,vector1), -1.0, 1.0))/np.pi*180


def fromAngelToVector(angel):
    return[np.cos(angel*np.pi/180),np.sin(angel*np.pi/180)]

def  angle180(angle):
    if 0 < angle <= 180:
        return  angle
    elif 180 < angle <= 360:
        return  angle-360
    elif -180 < angle <= 0:
        return  angle
    elif -360 <= angle < 180:
        return  angle+360
    else:
        print("angle more then 360")
        return angle180(angle % 360)

def fromSimtoReal(motors):
    return "."+ str(int(motors[0]))+ ","+str(int(motors[1]))+ "/:"

def fromFuncToName(func):
    return str(func)[10:str(func).index(" at ",0, len(str(func)))]





























