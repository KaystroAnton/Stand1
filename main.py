import test
import Constants
flag = "Simulation"
#flag = "Real"
for control in range(3):
    test1 = test.Test(flag,controlFuncAndParam= Constants.controlFuncAndParamList[2])
    test1.run()
# for info use __docs__
#print(test.createParamListOfControl.__doc__)