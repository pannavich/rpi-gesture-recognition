import paho.mqtt.publish as publish 
import paho.mqtt.client as mqtt
import time
import json
from gpiozero import Motor, Buzzer, TonalBuzzer
from gpiozero.tones import Tone
import RPi.GPIO as GPIO
import adafruit_dht
import board
import broadlink

# Set MQTT variable
port = 1883 
Server_ip = "broker.netpie.io" 
Subscribe_Topic = "@msg/#"
Publish_Topic = "@shadow/data/update"
Client_ID = "YOUR CLIENT ID"
Token = "YOUR TOKEN"
Secret = "YOUR SECRET"
MqttUser_Pass = {"username":Token,"password":Secret}

# Initialize Fan
motor = Motor(23,24)
fan_status = "off"

# Initialize IR
broadlink.setup('YOUR WIFI NAME', 'YOUR WIFI PASSWORD', 3)
device = broadlink.discover()[0]
air_on = b'&\x00l\x01\xa3F\x0c;\x0c\x17\x0c\x18\x0b\x18\x0c;\x0b\x18\x0c\x17\x0c\x18\x0b\x18\x0c;\x0b\x18\x0c;\x0b;\x0c\x18\x0b;\x0c;\x0c;\x0b;\x0c;\x0c\x17\x0c;\x0c\x17\x0c\x18\x0b\x18\x0c\x17\x0c\x18\x0b\x18\x0c;\x0b;\x0c\x18\x0b\x18\x0c\x17\x0c\x18\x0b\x18\x0c;\x0b\x18\x0c\x17\x0c\x18\x0b\x18\x0c\x18\x0b\x18\x0b\x18\x0c\x17\x0c\x18\x0b\x18\x0c\x18\x0b\x18\x0b\x18\x0c\x18\x0b;\x0c;\x0b;\x0c;\x0c\x18\x0b\x18\x0b\x18\x0c\x00\x03\xc3\xa6E\x0e9\x0e\x15\x0e\x16\r\x16\x0e9\r\x16\x0e\x15\x0e\x16\r\x16\x0e9\r\x16\x0e9\r9\x0f\x15\r9\x0e9\x0f8\r9\x0f8\x0e\x15\x0e9\x0e\x16\r\x16\r\x16\x0e\x16\r\x16\r\x16\x0e9\r:\r\x16\x0e\x15\x0e\x16\r\x16\r\x16\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e9\x0e9\x0e\x15\x0e\x15\x0e9\x0e\x15\x0e9\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e9\r\x16\x0e\x15\x0e\x16\r\x16\x0e9\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e9\x0e\x15\x0e\x16\r\x16\x0e9\x0e8\x0e\x16\r9\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r\x16\x0e\x15\x0e\x16\r9\x0e\x16\r\x16\x0e\x15\x0e\x16\r9\x0e\x16\r\x16\x0e9\r9\x0e9\x0e\x00\r\x05'
air_off = b'&\x00l\x01\xa4E\r:\r\x17\x0c\x17\x0c\x17\r:\x0c\x17\r\x17\x0c\x17\x0c\x17\r:\x0c\x17\r:\x0c:\r\x17\x0c;\x0c:\r:\x0c;\x0c:\r\x17\x0c:\r\x17\x0c\x17\x0c\x17\r\x17\x0c\x17\x0c\x17\r:\x0c;\x0c\x17\r\x17\x0c\x17\x0c\x17\x0c\x17\r:\x0c\x17\r\x17\x0c\x17\x0c\x17\r\x17\x0c\x17\r\x16\r\x17\x0c\x17\r\x16\r\x17\x0c\x17\r\x16\r\x17\x0c:\r:\r:\x0c:\r\x17\x0c\x17\r\x16\r\x00\x03\xc2\xa6F\x0c:\r\x17\x0c\x17\r\x16\r:\x0c\x17\r\x17\x0c\x17\r\x16\r:\r\x16\r:\r:\x0c\x17\r:\x0c:\r:\r:\x0c:\r\x17\x0c:\r\x17\x0c\x17\r\x16\r\x17\x0c\x17\r\x16\r:\r:\x0c\x17\r\x16\r\x17\x0c\x17\r\x16\r\x17\x0c\x17\r\x16\r\x17\x0c\x17\r\x16\r:\r:\x0c\x17\r\x16\r:\r\x16\r:\r\x16\r\x17\x0c\x17\r\x16\r\x17\x0c\x17\r\x16\r\x17\x0c\x17\r\x16\r\x17\x0c\x17\r\x17\x0c\x17\x0c:\r\x17\x0c\x17\r\x17\x0c\x17\x0c\x17\r\x17\x0c\x17\x0c\x17\r\x17\x0c\x17\x0c\x17\r\x17\x0c\x17\x0c\x17\r\x17\x0c\x17\x0c\x17\r\x17\x0c\x17\x0c\x17\r\x17\x0c\x17\x0c\x17\r:\x0c\x17\r\x17\x0c\x17\x0c;\x0c:\r\x17\x0c:\r\x17\x0c\x17\x0c\x17\r\x17\x0c\x17\r\x16\r\x17\x0c\x17\r\x16\r\x17\x0c\x17\x0c\x17\r\x17\x0c\x17\x0c\x17\r\x17\x0c:\r\x17\x0c\x17\r:\x0c:\r\x17\x0c\x17\x0c\x17\r:\r:\x0c:\r\x00\r\x05'
air_status = 'off'
device.auth()

# Initialize Temp Humi
dhtDevice = adafruit_dht.DHT11(board.D17)
humidity = dhtDevice.humidity
temperature = dhtDevice.temperature


# Initialize Buzzer
ky006 = TonalBuzzer(14)

v1 = ["E4", "E4", "E4"]
v2 = ["E4", "E4", "E4"]
v3 = ["E4", "G4", "C4", "D4", "E4"]
v4 = ["F4", "F4", "F4"]

song = [v1,v2,v3,v4]

# alarm function for buzzer
def alarm():
    for verse in song:
        for note in verse:
            ky006.play(note)
            time.sleep(0.4)
            ky006.stop()
            time.sleep(0.1)
        time.sleep(0.2)

# data to publish to NETPIE
sensor_data = {"temp": int(temperature),
               "humi": int(humidity),
               "fan": fan_status,
               "air" : air_status
               }

# on MQTT connect
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(Subscribe_Topic)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global fan_status, air_status
    topic = msg.topic
    message = msg.payload.decode("UTF-8")
    # fan control
    if(topic=='@msg/fan'):
        if(message=='on'):
            fan_status = "on"
            motor.forward()
        elif(message=='off'):
            fan_status = 'off'
            motor.stop()
    # air control
    elif(topic=='@msg/air'):
        if(message=='on'):
            air_status = 'on'
            device.send_data(air_on)  
        elif(message=='off'):
            air_status='off'
            device.send_data(air_off)
    # alarm control
    elif(msg.topic=='@msg/buzzer'):
        if(message=='on'):
            alarm()

    print(topic+" : "+message)

# Initialize MQTT
client = mqtt.Client(protocol=mqtt.MQTTv311,client_id=Client_ID, clean_session=True)
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set(Token,Secret)
client.connect(Server_ip, port)
client.loop_start()

# read data and publish to MQTT
while True:
    try:
        humidity = dhtDevice.humidity
        temperature = dhtDevice.temperature
        sensor_data["temp"] = int(temperature)
        sensor_data["humi"] = int(humidity)
        sensor_data["fan"] = fan_status
        sensor_data["air"] = air_status
        data_out=json.dumps({"data": sensor_data}) # encode object to JSON
        print(data_out)
        client.publish(Publish_Topic, data_out, retain= True)
        print ("Publish.....")
        time.sleep(1)
    except (KeyboardInterrupt, SystemExit):
        GPIO.cleanup()
    except RuntimeError:
        time.sleep(1)