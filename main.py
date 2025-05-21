import test
import Constants
flag = "Simulation"
#flag = "Real"
for control in range(3):
    test1 = test.Test(flag,targetPos=None,refAng= None,standParam=None,dt = 1/240,maxTime =10,
                      controlFuncAndParam= Constants.controlFuncAndParamList[2],numberOfTests = 1)
    test1.run()
# for info use __docs__
#print(test1.createParamListOfControl.__doc__)