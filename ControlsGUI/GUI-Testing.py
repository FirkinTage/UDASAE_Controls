import tkinter as tk
from tkinter import ttk
from PIL import Image
from PIL import ImageTk
import serial as sr
import numpy as np
import pandas as pd
from skimage.transform import resize
# import csv
# import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
#from matplotlib.figure import Figure
import matplotlib.pyplot as plt
################################################### SERIAL DATA FUNCTIONS ########################################################
# a = {\
#     "pkt":[15],"drp":[101],"alt":[65.42],"temp":[84.59],"lat":[49.854826],
#     "lon":[72.958413],"hdng":[20.54],"spd":[10.96],"aclx":[2.12],"acly":[6.54],
#     "aclz":[1.25],"gyrox":[1.26],"gyroy":[1.48],"gyroz":[2.08],"magx":[1.85],
#     "magy":[8.95],"magz":[4.85],"habDrp":[68.95],"cdaDrp":[23.39],"watDrp":[102.65],"rssi":[-40]
#     }  # Dummy Data
# df = pd.DataFrame()
# i = 0

# cond = True
# def get_data():
#     global cond, df, i, a, data, BLX, BLY, dx, dy, ax, canvas, plt #, s
#     if (cond == True):
#         # serialString=s.readline().rstrip().decode("utf-8")     #Read data in serial buffer
#         try:
#             # serialDict=eval(serialString)                               #Try to convert the raw string into a dictionary
#             serialDict = a
#         except (NameError,SyntaxError,ValueError) as e:                 #catch any errors when trying to convert to dictionary
#             pass
#         else:
#             # ax.remove()
#             lon = pd.Series([dx+BLX, 2*dx+BLX, 3*dx+BLX])
#             lat = pd.Series([dy+BLY, 2*dy+BLY, 3*dy+BLY])
#             theta = [0, 90, 180]
#             if i < 3:
#                 paths = plt.scatter(lat[i], lon[i], marker = (3, 0, theta[i]))
#                 # ax.scatter(lat[i], lon[i], marker = (3, 0, theta[i]))
#                 i += 1
#                 canvas.draw()
#                 # plt.show()
#     root.after(1000, get_data)
#     # paths.remove()



######################################################## COM PORT INPUT GUI ####################################################
# function to return port # input
# def format_port(port_number):
#     com_port_str = 'COM'+str(port_number)
#     return(com_port_str)

# def get_port(entry):
    
#     print('Port selected: ', entry)
#     global comport_root
#     comport_root.destroy()
    
#     global cond
#     cond = True
#     global com_port
#     com_port = format_port(entry)
#     return com_port

# # comport GUI     
# comport_root = tk.Tk()            
# comport_root.title('Please enter your COM port')
# comport_root.geometry('380x80')


# # entry box
# entry = tk.Entry(comport_root)
# entry.grid(row = 1, column = 0)

# # label of available COM ports
# comList = serial.tools.list_ports.comports()
# #Get list of available COM ports
# connected = ''
# for element in comList:
#     connected += (str(element.device) + '  ')
# label = tk.Label(comport_root,\
#     text = 'Available COM Ports:\n' + connected,
#     bg = 'white')
# label.grid(row = 0, column = 0)

# # enter button
# button = tk.Button(comport_root, \
#     text = 'Enter', 
#     bg = 'white',
#     command = lambda: get_port(entry.get()))  
# button.grid(row=1, column=1)

# comport_root.mainloop()

#################################################### BUILDING THE MAIN GUI ########################################################
# creating the master window (root)
root = tk.Tk()
root.title('Ground Station GUI')
# creating tab control
tabcontrol = ttk.Notebook(root,\
     height = 1000, 
     width = 1800)

# adding tabs
dashboard = ttk.Frame(tabcontrol) 
all_data = ttk.Frame(tabcontrol)
tabcontrol.add(dashboard, text = 'Dashboard')
tabcontrol.add(all_data, text = 'All DAS Data')
tabcontrol.pack(expand = 1, fill = 'both')
cond = True

# ------------------------------------------------------- DASHBOARD TAB ---------------------------------------------------------------
# GPS
# GPS
# you must declare the variables as 'global' in the fxn before using#

# creating plot object in GUI
# im = Image.open("/Users/jamespolemeni/Desktop/GUI/GUI_DRAFTS/formatted_airfield_test.jpeg")
# im = im.resize((400,480),Image.ANTIALIAS)
# im = ImageTk.PhotoImage(im)
img = Image.open("/Users/jamespolemeni/Desktop/GUI/GUI_DRAFTS/new-gardens-airport.png")
img = img.resize((370,615),Image.ANTIALIAS)
gps_img = ImageTk.PhotoImage(img)
gps_display = tk.Label(dashboard, image = gps_img)
gps_display.place(x=1010, y=125)


#SPENNYYYYYYYYYYYY
# TRX = -75.750364          #top right longitude
# TRY = 39.681350            #top right latitude
# BLX = -75.751419        #bottom left longitude
# BLY = 39.680995             #bottom left latitude
# im = plt.imread('/Users/jamespolemeni/Desktop/GUI/GUI_DRAFTS/spenny.png')

# CLAREMONT AIRPORT
# TRX = -75.866880       # TOP RIGHT LONGITUDE
# TRY = 39.575000        # TOP RIGHT LATTITUDE
# BLX = -75.865333    # BOTTOM LEFT LONGITUDE
# BLY = 39.571872    # BOTTOM LEFT LATTITUDE

# im = plt.imread('/Users/jamespolemeni/Desktop/GUI/GUI_DRAFTS/claremont-airport.png')
# # im = np.array(im)
# fig = plt.Figure()
# ax = fig.add_subplot(111)
# ax.imshow(im, extent=[BLX, TRX, BLY, TRY])
# ax.set_xticks([]) 
# ax.set_yticks([]) 

# # ax.set_title('GPS')
# # ax.set_xlabel('longitude')
# # ax.set_ylabel('lattitude')
# ax.set_xlim(BLX, TRX)
# ax.set_ylim(BLY, TRY)

# # lines = ax.plot([],[])[0]

# canvas = FigureCanvasTkAgg(fig, master = dashboard)
# canvas.get_tk_widget().place(x = 980, y = 260, width = 400)
# canvas.draw()

# CURRENT/PAYLOAD ALTITUDES AND SPEED 
alt_frame = tk.Frame(dashboard)
alt_frame.place(relheigh = 0.165, relwidth = 1) 

# live altitude
altitude = 0.00
str_alt = tk.StringVar()
str_alt.set(altitude)

alt_label = tk.Label(alt_frame, text="Current Altitude (ft):", font=("Fixedsys", 24), fg = 'black')
alt_label.grid(sticky="nesw")

alt_num = tk.Label(alt_frame, textvariable=str_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
alt_num.grid(row = 1, sticky="ns")

# CDA altitude
cda_alt = 0.00
str_cda_alt = tk.StringVar()
str_cda_alt.set(cda_alt)

cda_label = tk.Label(alt_frame, text="Colonist drop (ft):", font=("Fixedsys", 24), fg = 'black')
cda_label.grid(row = 0, column = 2, sticky="nesw", padx = 12)

cda_num = tk.Label(alt_frame, textvariable=str_cda_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
cda_num.grid(row=1, column = 2, sticky="ns")

# Water/habitat altitude
wh_alt = 0.00
str_wh_alt = tk.StringVar()
str_wh_alt.set(wh_alt)

wh_label = tk.Label(alt_frame, text="Water/habitat drop (ft):", font=("Fixedsys", 24), fg = 'black')
wh_label.grid(row=0, column = 4, sticky="nesw", padx = 12)

wh_num = tk.Label(alt_frame, textvariable=str_wh_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
wh_num.grid(row=1, column = 4, sticky="ns")

# Speed
speed = 0.00
str_speed = tk.StringVar()
str_speed.set(speed)

speed_label = tk.Label(alt_frame, text="Speed (mph):", font=("Fixedsys", 24), fg = 'black')
speed_label.grid(row=0, column = 6, sticky="nesw", padx = 11)

speed_num = tk.Label(alt_frame, textvariable=str_speed, font=("Fixedsys", 48), bd=5, relief="sunken")
speed_num.grid(row=1, column = 6, sticky="ns")

# Roll
roll = 0.00
str_roll = tk.StringVar()
str_roll.set(speed)

roll_label = tk.Label(alt_frame, text="Roll (deg):", font=("Fixedsys", 24), fg = 'black')
roll_label.grid(row=0, column = 7, sticky="nesw", padx = 12)

roll_num = tk.Label(alt_frame, textvariable=str_roll, font=("Fixedsys", 48), bd=5, relief="sunken")
roll_num.grid(row=1, column = 7, sticky="ns")

# Pitch
pitch = 0.00
str_pitch = tk.StringVar()
str_pitch.set(speed)

pitch_label = tk.Label(alt_frame, text="Pitch (deg):", font=("Fixedsys", 24), fg = 'black')
pitch_label.grid(row=0, column = 8, sticky="nesw", padx = 12)

pitch_num = tk.Label(alt_frame, textvariable=str_pitch, font=("Fixedsys", 48), bd=5, relief="sunken")
pitch_num.grid(row=1, column = 8, sticky="ns")

# Yaw
yaw = 0.00
str_yaw = tk.StringVar()
str_yaw.set(speed)

yaw_label = tk.Label(alt_frame, text="Yaw (deg):", font=("Fixedsys", 24), fg = 'black')
yaw_label.grid(row=0, column = 9, sticky="nesw", padx = 12)

yaw_num = tk.Label(alt_frame, textvariable=str_yaw, font=("Fixedsys", 48), bd=5, relief="sunken")
yaw_num.grid(row=1, column = 9, sticky="ns")
# ----------------------------------------------------- DATA PLAYBACK TAB ---------------------------------------------------------------
BOTH = 'both'
LEFT = 'left'
RIGHT = 'right'
VERTICAL = 'vertical'
Y = 'y'
# making a main frame
main_frame = tk.Frame(all_data)
main_frame.pack(fill=BOTH, expand=1)
# making a canvas
data_canvas = tk.Canvas(main_frame)
data_canvas.pack(side = LEFT, fill = BOTH, expand = 1)
# making a scrollbar
scrollbar = ttk.Scrollbar(main_frame, orient = VERTICAL, command = data_canvas.yview)
scrollbar.pack(side = RIGHT, fill = Y)
# configure the canvas 
data_canvas.configure(yscrollcommand = scrollbar.set)
data_canvas.bind('<Configure>', lambda e: data_canvas.configure(scrollregion = data_canvas.bbox('all')))
# making a second frame for the data
second_frame = tk.Frame(data_canvas)
# adding that^ frame to a window to be placed in the canvas
data_canvas.create_window((0,300), window = second_frame, anchor = 'nw')
# # test of grid layout manager with dummy csv
# with open("Dynamic-DF.csv", newline="") as file:
#     reader = csv.reader(file)
#     # r and c tell us where to grid the labels
#     r = 1
#     for col in reader:
#         c = 0
#         for row in col:
#             # i've added some styling
#             label = tk.Label(second_frame, width=6, height=1,
#                                 text=row, relief=tk.RIDGE)
#             label.grid(row=r, column=c)
#             c += 1
#         r += 1
# start button that initiates serial data reading
root.update()
show_data_button = tk.Button(second_frame, \
                  text='Click for data', command=lambda: data_stop())
show_data_button.grid(row = 0, column = 0)

# # stop button that displays collected data in all_data tab
# root.update()
# stop = tk.Button(data_canvas, \
#                  text='Stop', command=lambda: data_stop())
# stop.grid(row=0, column=1)



# s = sr.Serial('COM21', 9600)  # change COM Port name
# s.reset_input_buffer()
# root.after(1, get_data)
root.mainloop()