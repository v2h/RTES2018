import time
import ttn
import json

app_id = "rtos2018"
access_key = "ttn-account-v2.A6gUwifb-nioOPlWRlgUC1RF7x8yWP-V7ne_C1xHD-A"

def uplink_callback(msg, client):
  print("Received uplink from ", msg.dev_id)
  print(msg)
  output = msg._asdict()
  output['payload_fields'] = (output['payload_fields'])._asdict()
  output['metadata'] = (output['metadata'])._asdict()
  gateways_dict = []
  for gateway in output['metadata']['gateways']:
    gateway = gateway._asdict()
    gateways_dict.append(gateway)
  output['metadata']['gateways'] = gateways_dict
  output_json = json.dumps(output)
  print('\n')
  print(str(output_json))

handler = ttn.HandlerClient(app_id, access_key)

# using mqtt client
mqtt_client = handler.data()
mqtt_client.set_uplink_callback(uplink_callback)
mqtt_client.connect()
time.sleep(60)
mqtt_client.close()

# using application manager client
#app_client =  handler.application()
#my_app = app_client.get()
#print(my_app)
#my_devices = app_client.devices()
#print(my_devices)