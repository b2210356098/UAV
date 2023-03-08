import serial
import RPi.GPIO as GPIO

def servo_control(repeat, sleep, pvm, freq, x1, x2):

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pvm, GPIO.OUT)
    p = GPIO.PWM(pvm, freq)
    p.start(50)
    i = 0
    try:

        while i < repeat:
            print(str(pvm) + " Working Correctly")
          
            p.ChangeDutyCycle(x1)
            time.sleep(sleep)
            p.ChangeDutyCycle(x2)
            time.sleep(sleep)
            i += 1
            return True
    except:
        p.stop()
        GPIO.cleanup()
servo_control(1, 1, 33, 80, 5, 12.5)
time.sleep(1)
servo_control(1, 1, 32, 35, 12.5, 5)
