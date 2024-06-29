##
##  MQTT Control for Kinova

import json
import re
import random
from paho.mqtt import client as mqtt

import jacomodule

class MQTTKinova(object):
    def __init__(self):
        #initialize robot
        self.robot = jacomodule.Jaco2()
        ret = self.robot.setCartesianControl()
        print("Set Cartesian mode:", ret)

        self.lx = 0
        self.ly= 0
        self.lz  =0
        self.clt_id = f'kinova-mqtt-{random.randint(0, 1000)}'
        pass

    def on_message(self,client, userdata, msg):
        js = json.loads(msg.payload)
#        print("Message!",js)

        if 'pos' in js:
            x = js['pos']['x']
            y = js['pos']['y']
            z = js['pos']['z']
            xd = js['ori']['x']
            yd = js['ori']['y']
            zd = js['ori']['z']
        else:
            print("JSON",js)
            return
        if self.lx ==0 and self.ly == 0 and self.lz ==0:
            self.lx = x
            self.ly = y
            self.lz = z
            self.lxd = xd
            self.lyd = yd
            self.lzd = zd
        if self.lx != x or self.ly !=y or self.lz != z:
            sc = 0.1  #
            sd = 0.01
            dx = (x-self.lx)*sc
            dy = (y-self.ly)*sc
            dz = (z-self.lz)*sc
            dxd = (xd-self.lxd )*sd
            dyd = (yd-self.lyd)*sd
            dzd = (zd-self.lzd)*sd
            print("D:",dx,dy,dz,dxd,dyd,dzd)
            
            if 'pad' in js:
                pd = js['pad']
                if pd['bA']:
#                    print("reset")
#                    self.resetRobot()
                    pass
                if pd['b0']!=1:
                    self.relativeMove(dx,dy,dz,dxd,dyd,-dzd)

            self.lx = x
            self.ly = y
            self.lz = z
            self.lxd = xd
            self.lyd = yd
            self.lzd = zd


            
    def connect_mqtt(self):
        self.client = mqtt.Client()  
# MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect("192.168.207.22", 1883, 60)
#  client.loop_start()   # 通信処理開始
        self.client.loop_start()   # 通信処理開始

    def log_txt(self, txt):
        print(txt)

# ブローカーに接続できたときの処理
    def on_connect(self,client, userdata, flag, rc,properties):
        print("Connected with result code " + str(rc))  # 接続できた旨表示
        self.client.subscribe("webxr/pose") #　connected -> subscribe
        self.log_txt("Connected MQTT"+"\n")


# ブローカーが切断したときの処理
    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")


    def connect_mqtt(self):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)  
# MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect("192.168.207.22", 1883, 60)
#  client.loop_start()   # 通信処理開始
#        self.client.loop_start()   # 通信処理開始

    def getPose(self):
        ret = self.robot.getCartesianPoint()
        print("GetPose",ret)
        return ret

    
    def relativeMove(self,y,z,x,xd,zd,yd):

#        print("getMove")
        pose = self.getPose()
        if pose[0] == 25555:
            return
        
#        if pose:
#        if False:
        if True:
            pose[0]=(float(pose[0])-x)
#            pose[1]=(float(pose[1])-y)
#            pose[2]=(float(pose[2])+z)
#            pose[3]=(float(pose[3])-xd)
#            pose[4]=(float(pose[4])-yd)
#            pose[5]=(float(pose[5])+zd)

#            ret = self.robot.sendTrajectory(pose)
        ret = self.robot.sendTrajectory(pose)

        print(ret, pose)


kv = MQTTKinova()

print("Connect MQTT")
kv.connect_mqtt()

kv.client.loop_forever()

