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

ret = robot.setAngularControl()
#ret = robot.setCartesianControl()
print("Set AngularControl",ret)
angle[5]=180
angle[4]=180

print("SendAngle:",angle)

ret = robot.sendAngleTrajectory(angle)

print("Set Angle",ret)
