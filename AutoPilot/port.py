import serial

ser = serial.Serial(port="COM2", baudrate=9600,bytesize=8,timeout=2,stopbits=serial.STOPBITS_ONE)

for r in range(100):
    ser.write("asdfasdf")
    receieve = ser.read()
    print(receieve.decode("Ascii"))
    time.sleep(1)
