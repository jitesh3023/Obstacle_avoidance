
# importing 2 required libraries
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Trigger pin of ultrasonic sensor is connected to GPIO pin 16 of rpi 3B
# Echo pin of ultrasonic sensor is connected to GPIO pin 15 of rpi 3B
# Led is connected to GPIO pin 22 of rpi 3B`
Trig = 16
Echo = 15
led = 22

# This 4 are the 4 GPIO pins on raspberry pi which will be connected to the 4 input terminals of the
# l293d module. Like :-
# GPIO 7 (pin26 on rpi) will be connected to pin 2 of l293d this is the input pin of l293d
# GPIO 8 (pin24 on rpi) will be connected to pin 7 of l293d this is the input pin of l293d
# GPIO 11 (pin23 on rpi) will be connected to pin 10 of l293d this is the input pin of l293d
# GPIO 12 (pin32 on rpi) will be connected to pin 15 of l293d this is the input pin of l293d
motor12 = 7
motor11 = 8
motor21 = 11
motor22 = 12

# Initializing the trig ,that is GPIO pin 16 as output mode
# Initializing the Echo ,that is GPIO pin 15 as input mode
# Initializing the Led ,that is GPIO pin 22 as input mode
GPIO.setup(Trig, GPIO.OUT)
GPIO.setup(Echo, GPIO.IN)
GPIO.setup(led, GPIO.OUT)

# Initializing the m11 or the GPIO pin 16 to output mode
# Initializing the m12 or the GPIO pin 12 to output mode
# Initializing the m21 or the GPIO pin 21 to output mode
# Initializing the m22 or the GPIO pin 20 to output mode
GPIO.setup(motor11, GPIO.OUT)
GPIO.setup(motor12, GPIO.OUT)
GPIO.setup(motor21, GPIO.OUT)
GPIO.setup(motor22, GPIO.OUT)

GPIO.output(led, 1)
time.sleep(5)    # delay of 5 ms

# Now we will be defining separate functions for robots forward, backward, left, right and halt

def forward():
    start = 0
    start = time.time()
    elapsed = 0
    while (elapsed <= 2.0):
        elapsed = time.time() - start
        GPIO.output(motor11, 1)
        GPIO.output(motor12, 0)
        GPIO.output(motor21, 1)
        GPIO.output(motor22, 0)
        print("Forward")

def back():
    start = 0
    start = time.time()
    elapsed = 0
    while (elapsed <= 2.0):
        elapsed = time.time() - start
        GPIO.output(motor11, 0)
        GPIO.output(motor12, 1)
        GPIO.output(motor21, 0)
        GPIO.output(motor22, 1)
        print("Backward")


def left():
    start = 0
    start = time.time()
    elapsed = 0
    while (elapsed <= 2.0):
        elapsed = time.time() - start
        GPIO.output(motor11, 0)
        GPIO.output(motor12, 0)
        GPIO.output(motor21, 1)
        GPIO.output(motor22, 0)
        print("Left")


def right():
    start = 0
    start = time.time()
    elapsed = 0
    while (elapsed <= 2.0):
        elapsed = time.time() - start
        GPIO.output(motor11, 1)
        GPIO.output(motor12, 0)
        GPIO.output(motor21, 0)
        GPIO.output(motor22, 0)
        print("Right")


def stop():
    GPIO.output(motor11, 0)
    GPIO.output(motor12, 0)
    GPIO.output(motor21, 0)
    GPIO.output(motor22, 0)
    print("Stop")



while True:
    i = 0
    avgDistance = 0
    for i in range(5):
        # As we know that for the ultrasonic sensor to send ultrasonic waves, we need to trigger it by give a pulse of 10micro sec
        GPIO.output(Trig, False)  # Making the trigger low
        time.sleep(0.1)

        GPIO.output(Trig, True)   # Making the trigger high
        time.sleep(0.00001)       # giving a delay of 10micro sec

        GPIO.output(Trig, False)

        while GPIO.input(Echo) == 0:  # Checking whether the echo is low
            GPIO.output(led, False)
        pulse_start = time.time()     # Starting the timer i.e. the time when the ultrasonic wave is emmited by the transmitter

        while GPIO.input(Echo) == 1:  # Checking whether the echo is high, which is when the ultrasonic wave has returned back to the receiver after hitting the obstacle
            GPIO.output(led, False)
        pulse_end = time.time()       # Stopping the timer

        pulse_duration = pulse_end - pulse_start # Time take by the wave to conplete the whole trajectory

        distance = pulse_duration * (34300/2) # Calculating the distance between the sensor and the obstacle
                                              # speed = (2 * distance)/time
                                              # speed of sound is taken as 343m/sec or 34300cm/sec

        distance = round(distance, 2)         # rounding the distance to 2 decimal places
        avgDistance = avgDistance + distance  # summing up the 5 readings

    avgDistance = avgDistance/5           # taking the avg distance

    print(avgDistance)


    if avgDistance < 20:  # So we are checking if the obstacle is in the range of 20 cm from the sensor

        stop()            # Robot stops when the obstacle is detected
        print("Robot has detected an obstacle")
        time.sleep(1)     # delay for 1sec

    # running
    stop()
    forward()     # Robot will move forward for 2 sec
    left()        # Robot will move left for 2 sec
    right()       # Robot will move right for 2 sec











