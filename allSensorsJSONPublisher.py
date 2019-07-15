##WRITE BASH SCRIPT TO APPEND TO JSON FILE
#First test for all sensors using JSON
import time
import board
import busio
import json
from uuid import getnode as get_mac
import uuid
from collections import OrderedDict
import os

#Imports the Pressure/Altitude Sensor (mpl3115a2)
import adafruit_mpl3115a2
#Imports the Accelerometer Sensor (lsm9ds1)
import adafruit_lsm9ds1
#Imports the Color/Light Sensor (APDS9960)
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_apds9960 import colorutility
#Import the Radiation Sensor (Geiger Counter)
from PiPocketGeiger import RadiationWatch

#Import MQTT
import paho.mqtt.client as mqtt

# Initialize the I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

#Initializes global variables
global altitudePressureSensor
global accelerationSensor
global rgbSensor
global radiationSensor
global run_threads
run_threads = True

#MQTT variables
client = mqtt.Client("sensor-sender")
client.connect("iot.eclipse.org", 1883, 60)

#Method to initialize all sensors using the global variables
def initializeSensors():
    # Initialize the Altitude/Pressure Sensor (MPL3115A2)
    # Alternatively you can specify a different I2C address for the device:
    #sensor = adafruit_mpl3115a2.MPL3115A2(i2c, address=0x10)
    global altitudePressureSensor
    try:
        altitudePressureSensor = adafruit_mpl3115a2.MPL3115A2(i2c, address=0x60)
        # You can configure the pressure at sealevel to get better altitude estimates.
        # This value has to be looked up from your local weather forecast or meteorlogical
        # reports.  It will change day by day and even hour by hour with weather
        # changes.  Remember altitude estimation from barometric pressure is not exact!
        # Set this to a value in pascals:
        altitudePressureSensor.sealevel_pressure = 101760
    except(OSError, ValueError):
        print("Altitude sensor not detected")

    #Initialize the Acceleration Sensor (lsm9ds1)
    global accelerationSensor
    try:
        accelerationSensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
    except(OSError, ValueError):
        print("Acceleration sensor not detected")

    #Initialize the RGB Sensor (APDS9960)
    global rgbSensor
    try:
        rgbSensor = APDS9960(i2c)
        rgbSensor.enable_color = True
        rgbSensor.enable_proximity = True
        rgbSensor.enable_gesture = True
    except(OSError, ValueError):
        print("RGB sensor not detected")

    global radiationSensor
    try:
        radiationSensor = RadiationWatch(24, 23)
        radiationSensor.setup()
    except(OSError, ValueError):
        print("Radiation sensor not detected")

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

#Method to send the data via MQTT in the JSON format
def sendDataMQTT(msg, sensor_name):
    client.publish("osnds/" + sensor_name, msg)

#Method to get Altitude (MPL3115A2)
def getAltitude():
    return altitudePressureSensor.altitude

#Method to get Temp (MPL3115A2)
def getTemp():
    return altitudePressureSensor.temperature

#Method to get Pressure (MPL3115A2)
def getPressure():
    return altitudePressureSensor.pressure

#MetgetAltitudehod to get Acceleration (LSM9DS1)
def getAcceleration():
    accelerationArray = []
    accel_x, accel_y, accel_z = accelerationSensor.acceleration
    accelerationArray.append(accel_x)
    accelerationArray.append(accel_y)
    accelerationArray.append(accel_z)
    return accelerationArray


#Method to get Magnetometer (LSM9DS1)
def getMagnetometer():
    magnetometerArray = []
    mag_x, mag_y, mag_z = accelerationSensor.magnetic
    magnetometerArray.append(mag_x)
    magnetometerArray.append(mag_y)
    magnetometerArray.append(mag_z)
    return magnetometerArray

#Method to get Gyroscope (LSM9DS1)
def getGyro():
    gyroscopeArray = []
    gyro_x, gyro_y, gyro_z = accelerationSensor.gyro
    gyroscopeArray.append(gyro_x)
    gyroscopeArray.append(gyro_y)
    gyroscopeArray.append(gyro_z)
    return gyroscopeArray

#Method to get RGB
def getRGB():
    rgbArray = []
    r, g, b, c = rgbSensor.color_data
    rgbArray.append(r)
    rgbArray.append(g)
    rgbArray.append(b)
    rgbArray.append(c)
    rgbArray.append(colorutility.calculate_color_temperature(r, g, b))
    try:
        rgbArray.append(colorutility.calculate_lux(r, g, b))
    except(ZeroDivisionError):
        print("Light is off")
    return rgbArray

#Method to get radiation counts
def getRadiation():
    return radiationSensor.status()

#Starts threads
def runThreads():
    global run_threads
    run_threads = True

#Stops threads from running
def dontRunThreads():
    global run_threads
    run_threads = False

def saveToFile(msg):
    fileName = time.strftime("%d-%m-%Y", time.localtime()) + "_data"
    f = open(fileName, "a+")
    f.write(msg)
    f.close()

def runAllSensors():
    #poll rates in seconds
    altitudePollRate = 1
    temperaturePollRate = 1
    pressurePollRate = 1
    accelerationPollRate = 1
    magnetometerPollRate = 1
    gyroscopePollRate = 1
    rgbPollRate = 1
    radiationPollRate = 1


    while True:
        dtime = int(time.strftime("%S", time.localtime()))
        print(dtime)
        if(dtime % altitudePollRate == 0):
            try:
                msg = getJSON(getAltitude(), "altitude")
                sendDataMQTT(msg, "altitude")
                saveToFile(msg)
            except(OSError, ValueError):
                print("Altitude sensor not detected")

        if(dtime % temperaturePollRate == 0):
            try:
                msg = getJSON(getTemp(), "temperature")
                sendDataMQTT(msg, "temperature")
                saveToFile(msg)
            except(OSError, ValueError):
                print("Temperature sensor not detected")

        if(dtime % pressurePollRate == 0):
            try:
                msg = getJSON(getPressure(), "pressure")
                sendDataMQTT(msg, "pressure")
                saveToFile(msg)
            except(OSError, ValueError):
                print("Pressure sensor not detected")

        if(dtime % accelerationPollRate == 0):
            try:
                msg = getJSON(getAcceleration(), "acceleration")
                sendDataMQTT(msg, "acceleration")
                saveToFile(msg)
            except(OSError, ValueError):
                print("Acceleration sensor not detected")

        if(dtime % magnetometerPollRate == 0):
            try:
                msg = getJSON(getMagnetometer(), "magnetometer")
                sendDataMQTT(msg, "magnetometer")
                saveToFile(msg)
            except(OSError, ValueError):
                print("Magnetometer sensor not detected")

        if(dtime % gyroscopePollRate == 0):
            try:
                msg = getJSON(getGyro(), "gyroscope")
                sendDataMQTT(msg, "gyroscope")
                saveToFile(msg)
            except(OSError, ValueError):
                print("Gyroscope sensor not detected")

        if(dtime % rgbPollRate == 0):
            try:
                msg = getJSON(getRGB(), "rgb")
                sendDataMQTT(msg, "RGB")
                saveToFile(msg)
            except(OSError, ValueError):
                print("RGB sensor not detected")

        if(dtime % radiationPollRate == 0):
            try:
                msg = getJSON(getRadiation(), "radiation")
                sendDataMQTT(msg, "radiation")
                saveToFile(msg)
            except(OSError, ValueError):
                print("Radiation sensor not detected")

        print("\nITERATION COMPLETE\n")

initializeSensors()
runAllSensors()
