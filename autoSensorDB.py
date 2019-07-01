#A script to receive the data from the sensors in a JSON format
import paho.mqtt.client as mqtt
import time
import json
import boto3
import decimal


#Create an instance to connect to the database
dynamodb = boto3.resource('dynamodb')

#Instance of the table we will insert the data into
table = dynamodb.Table('SENSOR')

jsonFilename = "sensorData.json"

def writeJSON(sensorText):
	fname = "sensorData.json"
	f= open(fname,"w+")
	f.write(sensorText)
	f.close
	
def dynamoInsert(sensorText):
	print(sensorText)
	
	writeJSON(sensorText)

	with open(jsonFilename) as json_file:
		sensors = json.load(json_file, parse_float = decimal.Decimal)
	    
		#Loop to read each JSON key and bind it to a variable
		for sensor in sensors:
			SAMPLE_ID = sensor['SAMPLE_ID']
			UNIT_ID = int(sensor['UNIT_ID'])
			DATA = sensor['DATA']

			print("Adding data:", SAMPLE_ID, UNIT_ID)

			#Inserting the PK SAMPLE_ID and the FK UNIT_ID into the
			#database, and DATA is written as a JSON object
			table.put_item(
				Item={
					'SAMPLE_ID': SAMPLE_ID,
					'UNIT_ID': UNIT_ID,
					'DATA': DATA,
				}
			)

#Method to format the incoming JSON data
def on_message(client, userdata, message):
	#print(message.payload.decode("utf-8"))
	dynamoInsert(message.payload.decode("utf-8"))
########################################

broker_address="iot.eclipse.org"
client = mqtt.Client("Bryan's Receiver") #create new instance
client.on_message=on_message #attach function to callback
client.connect(broker_address) #connect to broker
print("Subscribing to topic","osnds/#")
client.subscribe("osnds/#", 0)

while True:
        client.loop(15)


