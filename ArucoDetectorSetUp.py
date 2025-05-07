import cv2 as cv
DetectorParam = [cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50),cv.aruco.DetectorParameters(),cv.aruco.CORNER_REFINE_SUBPIX]
# init aruco detector
dictionary = DetectorParam[0]
parameters = DetectorParam[1]
parameters.cornerRefinementMethod = DetectorParam[2]
detector = cv.aruco.ArucoDetector(dictionary, parameters)