import serial.tools.list_ports

if __name__ == '__main__':
    while(1):
        comList = serial.tools.list_ports.comports()
        connected = []
        for element in comList:
            connected.append(element.device)
        print("COM ports:",connected)
        #print("Available COM Ports:",serial_ports())
        comPort = input("Type COM port to open: ")
        print("Opening " + comPort)
        try:
            serialPort = serial.Serial(port=comPort, baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        except serial.SerialException:
            print("Invalid COM port, try again")
            pass
        else:
            print("Good COM port")
            while(1):
                if(serialPort.in_waiting>0):
                        #Read data in serial buffer
                        serialString=serialPort.readline().rstrip().decode("utf-8")
                        try:
                            serialDict=eval(serialString)
                        except (NameError,SyntaxError,ValueError) as e:
                            pass
                        else:
                            print(serialDict)

