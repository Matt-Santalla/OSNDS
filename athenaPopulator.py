import time
import board
import busio
from uuid import getnode as get_mac
import uuid
from collections import OrderedDict

import boto3
import json
import decimal

#Imports the Pressure/Altitude Sensor (mpl3115a2)
import adafruit_mpl3115a2

#Create an instance to connect to the database
dynamodb = boto3.client('athena')


# Initialize the I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

#Initializes global variables
global altitudePressureSensor

#Initializes sensor
def initializeSensor():
    # Initialize the Altitude/Pressure Sensor (MPL3115A2)
    # Alternatively you can specify a different I2C address for the device:
    #sensor = adafruit_mpl3115a2.MPL3115A2(i2c, address=0x10)
    global altitudePressureSensor
    altitudePressureSensor = adafruit_mpl3115a2.MPL3115A2(i2c, address=0x60)

    altitudePressureSensor.sealevel_pressure = 101760

#Method that generates JSON formatting (The OrderedDict() method is used to ensure the json variable ordering)
def getJSON(value, data_type):
    sampleUUID = str(uuid.uuid1())
    jsonFormat = {
        "UNIT_ID": get_mac(),
        "SAMPLE_ID": sampleUUID,
        "DATA": {
                    "DATA_TYPE": data_type,
                    "TIME_STAMP": time.strftime("%d-%m-%Y %H:%M:%S", time.localtime()),
                    "SENSOR_DATA": value
                }
    }
    print("[" + json.dumps((jsonFormat), sort_keys=True, indent = 4) + "]")
    return("[" + json.dumps((jsonFormat), sort_keys=True, indent = 4) + "]")

#Method to get Altitude (MPL3115A2)
def getAltitude():
    return altitudePressureSensor.altitude

initializeSensor()
while True:
    msg = getJSON(getAltitude(), "altitude")

    print("\nITERATION COMPLETE\n")


#
