import numpy as  np
import cv2 as cv
import os

accuracyAnge = 10
accuracyCoor = 5
simStoppingAngel = 20
integrateRegulatorSumL = 0
integrateRegulatorSumR = 0


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

def positionControl(targetPosition,initPosition,initaAngel):
    angelOnTarget = fromvVectorToAngel([targetPosition[0] - initPosition[0],targetPosition[1] - initPosition[1]])
    if(abs(angelOnTarget - initaAngel) > accuracyAnge):
        if(abs(angelOnTarget - initaAngel)<=180):
            wheelSpeed =((angelOnTarget - initaAngel) / 180 * 100)
        elif(abs(angelOnTarget - initaAngel)>180):
            wheelSpeed = ((360 - angelOnTarget + initaAngel) / 180 * 100)
        return(";"+str(int(wheelSpeed))+","+str(int(-wheelSpeed))+"/:")
    else:
        print("angel locked")
        return(";0,0/:")

def simPositionControl(targetPosition,initPosition,initaAngel,i,j):
    accuracyCoord = accuracyCoor/(i+1)
    accuracyAngel = accuracyAnge/(j+1)
    angelOnTarget = fromvVectorToAngel([targetPosition[0] - initPosition[0],800 - targetPosition[1] - initPosition[1]])
    if np.sqrt(pow(targetPosition[0] - initPosition[0], 2) + pow(targetPosition[1] - initPosition[1], 2) < accuracyCoord):
        return [[0, 0], True]
    if (turn(angelOnTarget - initaAngel,accuracyAngel) != [0,0]):
        return [turn(angelOnTarget - initaAngel,accuracyAngel),False]
    elif np.sqrt(pow(targetPosition[0] - initPosition[0],2) + pow(targetPosition[1] - initPosition[1],2) > accuracyCoord):
        return [[100,100],False]
    else:
        return [[0,0],True]



def turn(difAngel,accuracyAngel):
    wheelSpeed = 0
    if(abs(difAngel) > accuracyAngel):
        if(abs(difAngel)<=180):
            if abs(difAngel)>simStoppingAngel:
                wheelSpeed =100
            else:
                wheelSpeed =(abs(difAngel) / simStoppingAngel * 100)
        elif(abs(difAngel)>180):
            if (360 - abs(difAngel))>simStoppingAngel:
                wheelSpeed = 100
            else:
                wheelSpeed = ((360 - abs(difAngel)) / simStoppingAngel * 100)
        if (0<difAngel<180):
            wheelSpeed = -wheelSpeed
        elif(-180<difAngel<0):
            pass
        elif(difAngel>=180):
            pass
        else:
            wheelSpeed = -wheelSpeed
        return[wheelSpeed,-wheelSpeed]
    else:
        print("angel locked")
        return([0,0])

def posControler(refPosition,robotPosision,roborOriantation):
    e0 = angle180(fromvVectorToAngel([refPosition[0] - robotPosision[0],refPosition[1] - robotPosision[1]])- roborOriantation)
    print(e0,"e0")
    es = np.sqrt(pow(refPosition[0]-robotPosision[0],2) + pow(refPosition[1]-robotPosision[1],2))*np.cos(e0*np.pi/180.0)
    print(es, "es")
    if np.sqrt(pow(refPosition[0] - robotPosision[0],2) + pow(refPosition[1] - robotPosision[1],2) > accuracyCoor):
        reg = regulator(e0, es)
        return [[motrorScale(reg[0]),motrorScale(reg[1])],False]
    else:
        return [[0,0], True]

def realposControler(refPosition,robotPosision,roborOriantation):
    e0 = angle180(fromvVectorToAngel([refPosition[0] - robotPosision[0], refPosition[1] - robotPosision[1]]) - roborOriantation)
    print(e0, "e0")
    es = np.sqrt(pow(refPosition[0] - robotPosision[0], 2) + pow(refPosition[1] - robotPosision[1], 2)) * np.cos(e0 * np.pi / 180.0)
    print(es, "es")
    if np.sqrt(pow(refPosition[0] - robotPosision[0], 2) + pow(refPosition[1] - robotPosision[1], 2) > accuracyCoor):
        return [regulator(e0, es), False]
    else:
        return [";0,0/:", True]

def orientControlSetedLinSpeedTrue(robotOrientation, refOrientation,linspeed = 60,L=0.23,kProp = 0.8):
    turn = (angle180(refOrientation - robotOrientation) / L)
    if turn>=100-linspeed:
        turn = 100-linspeed
    elif turn<=-100+linspeed:
        turn = -100+linspeed
    motorL =  kProp * (linspeed - turn)
    motorR =  kProp * (linspeed + turn)
    return [[motrorScale(motorL),motrorScale(motorR)],False]

def orientControlSetedLinSpeed(robotOrientation, refOrientation,linspeed = 60,L=0.23,kProp = 0.8):
    turn = (angle180(refOrientation - robotOrientation) / L)
    if turn >= 100 - linspeed:
        turn = 100 - linspeed
    elif turn <= -100 + linspeed:
        turn = -100 + linspeed
    motorL =  kProp * (linspeed - turn)
    motorR =  kProp * (linspeed + turn)
    return [[motorL,motorR],False]

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
    return posControler([xRef,yRef],robotPosision,robotOriantation)

def regulator(angle, dist, L = 0.23, kProp = 0.5):
    motorL =  kProp * (dist - angle /L)
    motorR =  kProp * (dist + angle / L)
    return scale(motorL,motorR)

def regulatorPI(time,angle, dist, L = 0.23, kProp = 0.5,kIntegr = 0.1):
    global integrateRegulatorSumL
    integrateRegulatorSumL = integrateRegulatorSumL + (dist - angle / L) * time
    global integrateRegulatorSumR
    integrateRegulatorSumR = integrateRegulatorSumR + (dist + angle / L) * time

    motorL = kProp * (dist - angle / L) + kIntegr*integrateRegulatorSumL
    motorR = kProp * (dist + angle / L) + kIntegr*integrateRegulatorSumR
    return scale(motorL, motorR)

def motrorScale(motor):
    if motor>=100:
        return 100
    elif motor <=-100:
        return -100
    elif abs(motor)<60:
        if motor>0:
            return 60
        else:
            return -60
    else:
        return  motor

def scale(motorL,motorR):
    scale1 = (abs(motorL) + abs(motorR))/100.0
    return [motorL/scale1,motorR/scale1]




























