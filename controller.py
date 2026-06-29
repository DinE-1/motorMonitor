from gpiozero import DistanceSensor, OutputDevice
from time import sleep
from sms import send_sms
import threading

sensor = DistanceSensor(
    echo=5,
    trigger=6
)

relay = OutputDevice(
    26,
    active_high=False
)

def send_sms_background(state):
    threading.Thread(
        target=send_sms,
        args=(state,),
        daemon=True
    ).start()

previous_state = False

while True:
    distance = sensor.distance * 100

    print(f"Distance: {distance:.1f} cm")

    # Hysteresis
    if distance > 12:
        current_state = True
    elif distance < 8:
        current_state = False
    else:
        current_state = previous_state

    # Control relay
    if current_state:
        relay.on()
    else:
        relay.off()

    # Send SMS only if state changed
    if current_state != previous_state:
        send_sms_background(current_state)
        previous_state = current_state

    sleep(0.1)
