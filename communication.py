import random
import serial
import serial.tools.list_ports

# from asyncio.timeouts import timeout

class Communication:
    baudrate = 9600
    portName = "COM6"
    dummyPlug = False
    ports = serial.tools.list_ports.comports()
    ser = serial.Serial()

    def __init__(self):
        self.baudrate = 9600
        for port in sorted(self.ports):
            print(("{}".format(port)))
        try:
            self.ser = serial.Serial("COM6", 9600) 
            print("Serial connected")
        except serial.serialutil.SerialException:
            print("Can't open : ", self.portName)

    def close(self):
        if(self.ser.isOpen()):
            self.ser.close()
        else:
            print(self.portName, " it's already closed")

    # reads data from the XBee
    def getData(self):
        value = self.ser.readline()  # read line (single value) from the serial port
        decoded_bytes = str(value[0:len(value) - 2].decode("utf-8"))
        print(decoded_bytes)
        value_chain = decoded_bytes.split(",")
        return value_chain

    def isOpen(self):
        return self.ser.isOpen()
    
    def write(self, sendCommand):
        self.ser.write(sendCommand.encode())

    def dummyMode(self):
        return self.dummyPlug
