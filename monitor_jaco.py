import jacomodule
import time
import json
from paho.mqtt import client as mqtt

robot = jacomodule.Jaco2()

time.sleep(1)

robot.startControlAPI()

#robot.start()

#ret = robot.getCartesianPoint()

#print(ret)

#robot.sendTrajectory(ret)

def on_connect(client, userdata, flag, rc):
    print("Connected MQTT!")

def on_disconnect():
    print("Disconnected MQTT!")

def on_message(client, userdata, msg):
    print("get message")


client = mqtt.Client()  
# MQTTの接続設定
client.on_connect = on_connect         # 接続時のコールバック関数を登録
client.on_disconnect = on_disconnect   # 切断時のコールバックを登録
client.on_message = on_message         # メッセージ到着時のコールバック
client.connect("192.168.207.22", 1883, 60)
client.loop_start()   # 通信処理開始
#        client.loop_forever()   # 通信処理開始

while True:
    angle = robot.getAngularPosition()
    print("Angle:",angle)
    if angle[0]==-25555.0:
        print("Check!! We need to restart!")
        time.sleep(1)
        robot = jacomodule.Jaco2()
        robot.startControlAPI()
#        robot.start()

    client.publish("kinova/real",json.dumps(angle))
    time.sleep(0.15)



