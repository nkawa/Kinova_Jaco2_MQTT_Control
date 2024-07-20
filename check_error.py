import jacomodule
import time
import json
from paho.mqtt import client as mqtt

robot = jacomodule.Jaco2()

time.sleep(1)

robot.startControlAPI()

ret, count = robot.getSystemErrorCount()

print("Ret:",ret,"Count:",count)


def e2s(se):
    print("Header:",se.header,  "se.firm",se.firmware, "keos",se.keosVersion, "tm",se.systemTime)

for i in range(0,count):
    ret, ss = robot.getSystemError(i)
    print("err",i,ret,e2s(ss))

robot.clearErrorLog()

