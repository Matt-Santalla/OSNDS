#A script to receive the data from the sensors in a JSON format
import paho.mqtt.client as mqtt
import time
import json

#Method to format the incoming JSON data
def on_message(client, userdata, message):
    print"Message received: ",str(message.payload.decode("utf-8"))
    print"Message topic: ",message.topic
    print("\n")
########################################

broker_address="iot.eclipse.org"
client = mqtt.Client("Andrew's Receiver") #create new instance
client.on_message=on_message #attach function to callback
client.connect(broker_address) #connect to broker
print("Subscribing to topic","osnds/#")
client.subscribe("osnds/#", 0)

while True:
        client.loop(15)
