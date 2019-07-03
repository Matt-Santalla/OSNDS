import boto3

dynamodb = boto3.resource('dynamodb')


table = dynamodb.create_table(
    TableName='JULY3',
    KeySchema=[
        {
            'AttributeName': 'SAMPLE_ID',
            'KeyType': 'HASH'  #Partition key
        },
        {
            'AttributeName': 'UNIT_ID',
            'KeyType': 'RANGE'  #Sort key
        }
    ],
    AttributeDefinitions=[
        {
            'AttributeName': 'SAMPLE_ID',
            'AttributeType': 'S'
        },
        {
            'AttributeName': 'UNIT_ID',
            'AttributeType': 'N'
        },

    ],
    ProvisionedThroughput={
        'ReadCapacityUnits': 10,
        'WriteCapacityUnits': 10
    }
)

print("Table status:", table.table_status)
