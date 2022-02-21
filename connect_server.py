import serial
import serial.tools.list_ports as list_ports
import time
import sys
import requests
import json
import pandas as pd
import keyboard

dic = {"subject1": {"posture1": []}}
# Database info
URL = "https://msti-514-project-default-rtdb.firebaseio.com/"
# For anonymous sign in, **TODO** Change the key below to be the API key of your Firebase project (Project Settings > Web API Key).
AUTH_URL = "https://identitytoolkit.googleapis.com/v1/accounts:signUp?key=AIzaSyAXdzFKGFmkEj-rAarM0UEVS6P4gxebj3s"
headers = {"Content-type": "application/json"}
auth_req_params = {"returnSecureToken": "true"}

# Start connection to Firebase and get anonymous authentication
connection = requests.Session()
connection.headers.update(headers)
auth_request = connection.post(url=AUTH_URL, params=auth_req_params)
auth_info = auth_request.json()
auth_params = {"auth": auth_info["idToken"]}

# get data from arduino
record_time = 3
ary = []
usingPorts = list(list_ports.comports())
for port in usingPorts:
    # debug to detect Serial name
    print(port.description)
    if sys.platform.startswith("win"):
        if "Serial" in port.description:
            serialPort = port.device
            break
        # end
    elif sys.platform.startswith("darwin"):
        if "IOUSBHostDevice" in port.description:
            serialPort = port.device
            break
# make sure the 'COM#' is set according the Windows Device Manager
# port="/dev/cu.usbserial-AK061JZ0"
ser = serial.Serial("/dev/cu.usbserial-AK061JZ0", baudrate=115200, timeout=1)
time.sleep(1)
start_record = False
data = ""
while True:
    if keyboard.is_pressed("s"):
        print("start recording...")
        start_record = True
        break
while start_record:
    if keyboard.is_pressed("q"):
        print("quit record")
        start_record = False
        break

    # start_time = time.time()
    # while time.time() - start_time < record_time:
    # for i in range(50):
    line = ser.readline()  # read a byte

    if line:
        data = str(line, "ascii").split(",")
        data[-1] = data[-1].replace("\r\n", "")
        ary.append(data)
ser.close()
time.sleep(2)
print("data in local")
dic["subject1"]["posture1"] = ary
print(dic)

"""
# Sending get request and obtaining the image
get_request = connection.get(url=URL + "test.json")
# Extracting data in json format, this is a string representing the image
image_str = get_request.json()
"""

print("start to send the data to database...")
results = dic
# Jasonify the results before sending
data_json = json.dumps(results)
print(data_json)
# The URL for the part of the database we will put the detection results
detection_url = URL + "test.json"
# Post the data to the database
post_request = connection.put(url=detection_url, data=data_json, params=auth_params)
# Make sure data is successfully sent
print("Detection data sent: " + str(post_request.ok))

