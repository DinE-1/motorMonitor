# sms.py

import serial
import time

gsm = serial.Serial('/dev/serial0', 9600, timeout=1)

PHONE = "+91xxxxxxxxxx"

def send_sms(state):

    if state:
        message = "Pump turned ON"
    else:
        message = "Pump turned OFF"

    gsm.write(b'AT+CMGF=1\r')
    time.sleep(1)

    cmd = f'AT+CMGS="{PHONE}"\r'
    gsm.write(cmd.encode())
    time.sleep(1)

    gsm.write(message.encode())
    gsm.write(bytes([26]))    # Ctrl+Z

    time.sleep(5)
