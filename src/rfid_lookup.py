#!/usr/bin/env python

import RPi.GPIO as GPIO
from mfrc522 import MFRC522
from mfrc522 import SimpleMFRC522
import time
from datetime import datetime

dictionary = {
    '584197060066': 1,
    '584184960952': 1,
    '584194568843': 1,
    '584183660994': 1,
    '584190170475': 1,
    '584183554850': 1,
    '584185939387': 1,
    '584193790966': 2,
    '584191241442': 2,
    '584184595218': 2,
    '584188166360': 2,
    '584188758488': 2,
    '584190784005': 2,
    '584184683080': 2,
    '584190171238': 2,
    '584199419079': 2,
    '584184942975': 3,
    '584182834475': 3,
    '584184999967': 3,
    '584195867076': 3,
    '584196839436': 3,
    '584186687936': 3,
    '584193404354': 3,
    '584197592251': 3 
    }

# reader = SimpleMFRC522()

# id, text = reader.read()
# id = str(id)
# dest = dictionary[id]
# print("DESTINATION BIN:", dest)


# print("LED on")
# GPIO.output(22,GPIO.HIGH)

# time.sleep(3)

# print("LED off")
# GPIO.output(22,GPIO.LOW)


