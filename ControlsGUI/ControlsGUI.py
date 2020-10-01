import serial.tools.list_ports

if __name__ == '__main__':
    while(1):
        comList = serial.tools.list_ports.comports()        #Get list of available COM ports
        connected = []
        for element in comList:
            connected.append(element.device)
        print("COM ports:",connected)
        comPort = input("Type COM port to open: ")          #Get COM port from user
        print("Opening " + comPort)
        try:
            serialPort = serial.Serial(port=comPort, baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)        #Open the chosen COM port
        except serial.SerialException:
            print("Invalid COM port, try again")    #Catch any invalid COM port picks
            pass
        else:
            print("Good COM port")
            while(1):
                if(serialPort.in_waiting>0):
                        serialString=serialPort.readline().rstrip().decode("utf-8")     #Read data in serial buffer
                        try:
                            serialDict=eval(serialString)                               #Try to convert the raw string into a dictionary
                        except (NameError,SyntaxError,ValueError) as e:                 #catch any errors when trying to convert to dictionary
                            pass
                        else:
                            print(serialDict)                                           #print out dictionary

