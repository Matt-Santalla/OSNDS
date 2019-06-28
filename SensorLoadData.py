import boto3
import json
import decimal

#Create an instance to connect to the database
dynamodb = boto3.resource('dynamodb')

#Instance of the table we will insert the data into
table = dynamodb.Table('SENSOR')

jsonFilename = "sensorData2.json"

#open specified JSON file
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
