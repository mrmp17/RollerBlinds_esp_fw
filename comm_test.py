import serial

ser = serial.Serial("/dev/tty.usbserial-0001", 9600, timeout=None)

comm1 = bytearray([1, 123, 69, 0, 0, 0, 0, 0, 1])
comm1.append((sum(comm1) + 1) % 256)
print(list(comm1), len(comm1))

ser.write(comm1)
print("Waiting comm2...")

try:
    while True:
        comm2 = ser.read(size=13)
        print(list(comm2))
finally:
    print("exiting")
    ser.close()
