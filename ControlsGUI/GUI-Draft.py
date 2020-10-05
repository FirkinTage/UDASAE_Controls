import tkinter as tk
from tkinter import ttk
from PIL import Image
from PIL import ImageTk

######################################################## COM PORT INPUT GUI ####################################################
# function to return port # input
def format_port(port_number):
    com_port_str = 'COM'+str(port_number)
    return(com_port_str)

def get_port(entry):
    print('Port selected: ', entry)
    global comport_root
    comport_root.destroy()
    
    global com_port
    com_port = format_port(entry)
    return com_port

# comport GUI     
comport_root = tk.Tk()            
comport_root.title('Please enter the comport number')
comport_root.geometry('300x30')

# entry box
entry = tk.Entry(comport_root)
entry.grid()

# enter button
button = tk.Button(comport_root, \
    text = 'Enter', 
    bg = 'white',
    command = lambda: get_port(entry.get()))  
button.grid(row=0, column=1)
comport_root.mainloop()
print('Global variable test: ',com_port)
################################################### SELECTING A COM PORT ######################################################
# import serial.tools.list_ports
# if __name__ == '__main__':
#     while(1):
#         comList = serial.tools.list_ports.comports()        #Get list of available COM ports
#         connected = []
#         for element in comList:
#             connected.append(element.device)
#         print("COM ports:",connected)
#         #comPort = input("Type COM port to open: ")          #Get COM port from user
#         comPort = com_port
#         print("Opening " + comPort)
#         try:
#             serialPort = serial.Serial(port=comPort, baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)        #Open the chosen COM port
#         except serial.SerialException:
#             print("Invalid COM port, try again")    #Catch any invalid COM port picks
#             pass
#         else:
#             print("Good COM port")
#             while(1):
#                 if(serialPort.in_waiting>0):
#                         serialString=serialPort.readline().rstrip().decode("utf-8")     #Read data in serial buffer
#                         try:
#                             serialDict=eval(serialString)                               #Try to convert the raw string into a dictionary
#                         except (NameError,SyntaxError,ValueError) as e:                 #catch any errors when trying to convert to dictionary
#                             pass
#                         else:
#                             print(serialDict)                                           #print out dictionary
##################################################### BUILDING THE GUI ########################################################
# creating the master window (root)
root = tk.Tk()
root.title('Ground Station GUI')

# creating tab control
tabcontrol = ttk.Notebook(root,\
     height = 1000, 
     width = 1800)

# adding tabs
dashboard = ttk.Frame(tabcontrol) 
live_data = ttk.Frame(tabcontrol)
tabcontrol.add(dashboard, text = 'Dashboard')
tabcontrol.add(live_data, text = 'Live DAS Data')
tabcontrol.pack(expand = 1, fill = 'both')

############################################# CURRENT/PAYLOAD ALTITUDES AND SPEED ############################################
alt_frame = tk.Frame(dashboard)
alt_frame.place(relheigh = 0.165, relwidth = 0.70) 

# live altitude
altitude = 0.00
str_alt = tk.StringVar()
str_alt.set(altitude)

live_label = tk.Label(alt_frame, text="Current Altitude (ft):", font=("Fixedsys", 24), fg = 'black')
live_label.grid(sticky="nesw")

live_num = tk.Label(alt_frame, textvariable=str_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
live_num.grid(row = 1, sticky="ns")

# CDA altitude
cda_alt = 0.00
str_cda_alt = tk.StringVar()
str_cda_alt.set(cda_alt)

cda_label = tk.Label(alt_frame, text="Colonist drop (ft):", font=("Fixedsys", 24), fg = 'black')
cda_label.grid(row = 0, column = 2, sticky="nesw", padx = 15)

cda_num = tk.Label(alt_frame, textvariable=str_cda_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
cda_num.grid(row=1, column = 2, sticky="ns")

# Water/habitat altitude
wh_alt = 0.00
str_wh_alt = tk.StringVar()
str_wh_alt.set(wh_alt)

wh_label = tk.Label(alt_frame, text="Water/habitat drop (ft):", font=("Fixedsys", 24), fg = 'black')
wh_label.grid(row=0, column = 4, sticky="nesw", padx = 12)

wh_num = tk.Label(alt_frame, textvariable=str_cda_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
wh_num.grid(row=1, column = 4, sticky="ns")

# Speed
speed = 0.00
str_speed = tk.StringVar()
str_speed.set(speed)

speed_label = tk.Label(alt_frame, text="Speed (mph):", font=("Fixedsys", 24), fg = 'black')
speed_label.grid(row=0, column = 6, sticky="nesw", padx = 15)

speed_num = tk.Label(alt_frame, textvariable=str_cda_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
speed_num.grid(row=1, column = 6, sticky="ns")

########################################################## GPS ###############################################################
# gps_display = tk.Canvas(dashboard, width=300, height=500, background="white")
# gps_display.place(x=1100, y=330)
img = Image.open("formatted_airfield_test.jpeg")
img = img.resize((400,480),Image.ANTIALIAS)
gps_img = ImageTk.PhotoImage(img)
#gps_display.create_image(10,10,image=gps_map, anchor="nw")
gps_display = tk.Label(dashboard, image = gps_img)
gps_display.place(x=1000, y=250)

########################################################## FPV ###############################################################
# # Create the camera object
# # First, create a camera object by importing the USBCamera class from the library by executing the following Python code cell. 
# # Please note, you can only create one USBCamera instance. 
# # Set the capture_device= to the correct number found when you listed the system video devices. 
# # If you have /dev/video0, then set capture_device=0. 
# # If you have /dev/video1, set capture_device=1 in the code line below.

# from jetcam.usb_camera import USBCamera
# #TODO change capture_device if incorrect for your system
# camera = USBCamera(width=224, height=224, capture_width=640, capture_height=480, capture_device=0)


# # Create a widget to view the image stream
# # We can create a "widget" to display this image in the notebook. 
# # In order to see the image, convert it from its blue-green-red format (brg8) to a format the browser can display (jpeg).

# import ipywidgets
# from IPython.display import display
# from jetcam.utils import bgr8_to_jpeg
# ​
# image_widget = ipywidgets.Image(format='jpeg')
# ​
# image_widget.value = bgr8_to_jpeg(image)
# ​
# display(image_widget)

# # You should see an image from the camera if all is working correctly. 
# # If there seems to be an image but it's fuzzy or a funny color, check to make sure there is no protective film or cap on the lens.
# # Now let's watch a live stream from the camera. 
# # Set the running value of the camera to continuously update the value in background. 
# # This allows you to attach "callbacks" to the camera value changes.

# # The "callback" here is the function, update_image, which is attached by calling the observe method below. 
# # update_image is executed whenever there is a new image available to process, which is then displayed in the widget.

# camera.running = True
# ​
# def update_image(change):
#     image = change['new']
#     image_widget.value = bgr8_to_jpeg(image)
    
# camera.observe(update_image, names='value')


################################################# ATTITUDE INDICATOR #########################################################


root.mainloop()
