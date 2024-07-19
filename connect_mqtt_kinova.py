
import json
from paho.mqtt import client as mqtt
import random

import jacomodule


class Kinova_MQTT:
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

    def on_connect(self,client, userdata, flag, rc):
        print("Connected MQTT!")

        print("Connected with result code " + str(rc))  # 接続できた旨表示
        self.client.subscribe("kinova/state") #　connected -> subscribe

# ブローカーが切断したときの処理
    def on_disconnect(self,client, userdata, rc):
        if  rc != 0:
            print("Unexpected disconnection.")

    def on_message(self,client, userdata, msg):
#        print("Message",msg.payload)
        js = json.loads(msg.payload)
#        print("rot:",js)
        pose = self.robot.getCartesianPoint()
        print("GetPose",pose)
        
        if 'rotate' in js:
            rot = js['rotate']
            print("rot  :",rot)

        angle = self.robot.getAngularPosition()
        print("angle:",angle)

        j1 = 180-float(rot['j1'])  # for diff
        j2 = float(rot['j2'])+180 
        j3 = 180-float(rot['j3'])
        j4 = 180-float(rot['j4'])
        j5 = 180-float(rot['j5'])
        j6 = -float(rot['j6'])

        sangle = [j1,j2,j3,j4,j5,j6]
#        ret =  self.robot.eraseAllTrajectories()
#        print("Erase All",ret)
        ret= self.robot.sendAngleTrajectory(sangle)
        print("Send :",ret, sangle)

        

    def connect_mqtt(self):
        self.client = mqtt.Client()  
# MQTTの接続設定
        self.client.on_connect = self.on_connect         # 接続時のコールバック関数を登録
        self.client.on_disconnect = self.on_disconnect   # 切断時のコールバックを登録
        self.client.on_message = self.on_message         # メッセージ到着時のコールバック
        self.client.connect("192.168.207.22", 1883, 60)
#  client.loop_start()   # 通信処理開始
        self.client.loop_forever()   # 通信処理開始




mq = Kinova_MQTT()

print("Connecting MQTT!")
mq.connect_mqtt()