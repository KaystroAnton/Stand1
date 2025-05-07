import pybullet as pb
import math
imgSide = 600
halfFieldSize = 4.0/2.0
cameraHeight = 3.0 # Z coordinate of camera
cameraSetUp = [1, 0 ,89.999, [halfFieldSize, halfFieldSize, cameraHeight]] # [cameraDistance, cameraYaw, cameraPitch,cameraTargetPosition]
viewMatrix = pb.computeViewMatrix(cameraEyePosition=[halfFieldSize,halfFieldSize,cameraHeight],
                                  cameraTargetPosition=[halfFieldSize, halfFieldSize, 0.0],
                                  cameraUpVector=[0.0, 1.0, 0.0])
# computeViewMatrix: cameraEyePosition - coordinates of the camera, cameraTargetPosition - focus point, cameraUpVector - vector perpendicular to the camera direction
camera_proj_matrix = pb.computeProjectionMatrixFOV(
                    fov= 2* math.atan(halfFieldSize / cameraHeight) * 180.0 / math.pi,
                    aspect=1,
                    nearVal=0,
                    farVal=3.5)

# fov - field of thew in degrees, aspect = 1 - the frame will be square, nearVal - near plane dist, farVal - far plane dist
imgSetUp = [imgSide,imgSide,camera_proj_matrix,pb.ER_TINY_RENDERER]
# [image width,image height,projectionMatrix,renderer]