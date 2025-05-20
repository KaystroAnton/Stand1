import test
import Constants
flag = "Simulation"
#flag = "Real"
for control in (Constants.controlFuncAndParamList):
    test1 = test.Test(flag,controlFuncAndParam= control)
    test1.run()
# for info use __docs__
#print(test.createParamListOfControl.__doc__)