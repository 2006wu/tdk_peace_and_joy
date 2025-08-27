import serial, time

ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)

while True:
    ser.write(b"MODE 1")
    print("Sent MODE 1")
    time.sleep(1)
