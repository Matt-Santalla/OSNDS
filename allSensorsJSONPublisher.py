#First test for all sensors using JSON
import time
import board
import busio
import json
from uuid import getnode as get_mac
import uuid
from collections import OrderedDict

#Imports the Pressure/Altitude Sensor (mpl3115a2)
import adafruit_mpl3115a2
#Imports the Accelerometer Sensor (lsm9ds1)
import adafruit_lsm9ds1
#Imports the Color/Light Sensor (APDS9960)
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_apds9960 import colorutility
#Import the Radiation Sensore (Geiger Counter)
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

#MQTT variables 
broker_address = "iot.eclipse.org"
client = mqtt.Client("sensor-sender")
client.connect("iot.eclipse.org", 1883, 60)

#Method to initialize all sensors using the global variables
def initializeSensors():
    # Initialize the Altitude/Pressure Sensor (MPL3115A2)
    # Alternatively you can specify a different I2C address for the device:
    #sensor = adafruit_mpl3115a2.MPL3115A2(i2c, address=0x10)
    global altitudePressureSensor
    altitudePressureSensor = adafruit_mpl3115a2.MPL3115A2(i2c, address=0x60)
    
    #Initialize the Acceleration Sensor (lsm9ds1)
    global accelerationSensor
    accelerationSensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
    
    #Initialize the RGB Sensor (APDS9960)
    global rgbSensor
    rgbSensor = APDS9960(i2c)
    rgbSensor.enable_color = True
    rgbSensor.enable_proximity = True
    rgbSensor.enable_gesture = True

    global radiationSensor
    radiationSensor = RadiationWatch(24, 23)
    radiationSensor.setup()

    # You can configure the pressure at sealevel to get better altitude estimates.
    # This value has to be looked up from your local weather forecast or meteorlogical
    # reports.  It will change day by day and even hour by hour with weather
    # changes.  Remember altitude estimation from barometric pressure is not exact!
    # Set this to a value in pascals:
    altitudePressureSensor.sealevel_pressure = 101760

#Method that generates JSON formatting (The OrderedDict() method is used to ensure the json variable ordering)
def getJSON(value):
    sampleUUID = str(uuid.uuid1())
    jsonFormat = [
        ("UNIT_ID", get_mac()),
        ("SAMPLE_ID", sampleUUID),
        ("TIME_STAMP", time.strftime("%d-%m-%Y %H:%M:%S", time.localtime())),
        ("SENSOR_DATA", value)
    ]
    print(json.dumps(OrderedDict(jsonFormat)))
    return json.dumps(OrderedDict(jsonFormat))

#Method to send the data via MQTT in the JSON format
def sendDataMQTT(msg, topic):
    client.publish("osnds/" + topic, msg)

#Method to get Altitude (MPL3115A2)
def getAltitude():
    return altitudePressureSensor.altitude

#Method to get Temp (MPL3115A2)
def getTemp():
    return altitudePressureSensor.temperature

#Method to get Pressure (MPL3115A2)
def getPressure():
    return altitudePressureSensor.pressure

#Method to get Acceleration (LSM9DS1)
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
    rgbArray.append(colorutility.calculate_lux(r, g, b))
    return rgbArray

#Method to get radiation counts
def getRadiation():
    return radiationWatch.status()

initializeSensors()
while True:
    msg = getJSON(getAltitude())
    sendDataMQTT(msg, "altitude")
    msg = getJSON(getTemp())
    sendDataMQTT(msg, "temperature")
    msg = getJSON(getPressure())
    sendDataMQTT(msg, "pressure")
    msg = getJSON(getAcceleration())
    sendDataMQTT(msg, "acceleration")
    msg = getJSON(getMagnetometer())
    sendDataMQTT(msg, "magnetometer")
    msg = getJSON(getGyro())
    sendDataMQTT(msg, "gyroscope")
    msg = getJSON(getRGB())
    sendDataMQTT(msg, "RGB")
    print("\nITERATION COMPLETE\n")
