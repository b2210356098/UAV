import GPIO
def servo_control(repeat, sleep):
  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(33, GPIO.OUT)
  p = GPIO.PWM(33, 50)
  p.start(2.5)
  i=0
  try:

      while i<repeat :
        p.ChangeDutyCycle(5)
        time.sleep(sleep)
        p.ChangeDutyCycle(12.5)
        time.sleep(sleep)
        i+=1
  except :
      p.stop()
      GPIO.cleanup()
      
servo_control(3, 0.5)
