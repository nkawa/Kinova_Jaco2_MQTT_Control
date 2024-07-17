import jacomodule
import time

robot = jacomodule.Jaco2()
print("hoge")

time.sleep(1)

robot.start()

ret = robot.getCartesianPoint()

print(ret)

#robot.sendTrajectory(ret)


angle = robot.getAngularPosition()

print("Angle:",angle)
