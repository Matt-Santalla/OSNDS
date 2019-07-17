#SENSOR DIAGNOSTIC
import time
import board
import busio
import json
import os
import subprocess
import re

#Import MQTT
import paho.mqtt.client as mqtt

#Imports the Pressure/Altitude Sensor (mpl3115a2)
import adafruit_mpl3115a2
#Imports the Accelerometer Sensor (lsm9ds1)
import adafruit_lsm9ds1

#Import the Radiation Sensor (Geiger Counter)
from PiPocketGeiger import RadiationWatch

#Initializes global variables
global altitudePressureSensor
global accelerationSensor
global radiationSensor

#Method to initialize all sensors using the global variables
def initializeSensors():
    # Test initializing the I2C
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
    except:
        print("I2C bus could not be initialized")

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
    except(OSError, ValueError, NameError):
        print("Altitude sensor not detected. Please check the connection to the sensor.\n")

    #Initialize the Acceleration Sensor (LSM9DS1)
    global accelerationSensor
    try:
        accelerationSensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
    except(OSError, ValueError, NameError):
        print("Acceleration sensor not detected. Please check the connection to the sensor.\n")

    global radiationSensor
    try:
        radiationSensor = RadiationWatch(24, 23)
        radiationSensor.setup()
    except(OSError, ValueError, NameError):
        print("Radiation sensor not detected. Please check the connection to the sensor.\n")

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

#Method to get radiation counts
def getRadiation():
    return radiationSensor.status()

def testAllSensors():
    try:
        getAltitude()
    except(OSError, ValueError, NameError):
        print("Altitude sensor not detected. Please check the connection to the sensor.\n")

    try:
        getTemp()
    except(OSError, ValueError, NameError):
        print("Temperature sensor not detected. Please check the connection to the sensor.\n")

    try:
        getPressure()
    except(OSError, ValueError, NameError):
        print("Pressure sensor not detected. Please check the connection to the sensor.\n")

    try:
        getAcceleration()
    except(OSError, ValueError, NameError):
        print("Acceleration sensor not detected. Please check the connection to the sensor.\n")

    try:
        getMagnetometer()
    except(OSError, ValueError, NameError):
        print("Magnetometer sensor not detected. Please check the connection to the sensor.\n")

    try:
        getGyro()
    except(OSError, ValueError, NameError):
        print("Gyroscope sensor not detected. Please check the connection to the sensor.\n")

    try:
        getRadiation()
    except(OSError, ValueError, NameError):
        print("Radiation sensor not detected. Please check the connection to the sensor.\n")

def testFileWriting():
    f = "fileWritingTest.txt"
    try:
        file = open(f, "w+")
        file.close()
    except(FileNotFoundError):
        #cant open file error
        print("File " + f + " was not found. Please make sure the file exists and that you have permission to write to the folder.")

def testMQTTConnection():
    try:
        mqtt.Client("sensor-sender").connect("iot.eclipse.org", 1883, 60)
    except(ConnectionError):
        print("The program can't connect to the server. \nIf the system is connected to the internet, make sure the server namecan be resolved. \nServer address is: " + serverName)

def testWIFIConnection():
    hostname = "google.com"
    response = os.system("ping -c 1 " + hostname)

    if (response == 0):
        print(hostname, 'is up!\n')
    else:
        print(hostname, 'is down\n')

def testI2C():
    # Test initializing the I2C
    print("I2C addresses: ")
    bashCommand = "i2cdetect -y 1"
    os.system(bashCommand)
    print('\n')

def testInternalTemperature():
    #Test the internal temperature of the Raspberry Pi
    bashCommand = "vcgencmd measure_temp"
    os.system(bashCommand)
    print("\n")

def testClockSpeed():
    #Returns the current clock rate of the Raspberry Pi
    print("CPU clock speed (MHz): ")
    bashCommand = "cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq"
    os.system(bashCommand)
    print("\n")

def testExternalStorage():
    #Returns the amount of free space on the external the external storage
    print("Available storage space: ")
    os.system("df -h")

def runAllTests():
    testI2C()
    initializeSensors()
    testAllSensors()
    testFileWriting()
    testMQTTConnection()
    testWIFIConnection()
    testInternalTemperature()
    testClockSpeed()
    testExternalStorage()

runAllTests()
