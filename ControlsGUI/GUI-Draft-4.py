import tkinter as tk
from tkinter import ttk
from PIL import Image
from PIL import ImageTk
import serial as sr
import numpy as np
import pandas as pd
import csv
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
#from matplotlib.figure import Figure
import matplotlib.pyplot as plt

################################################### SERIAL DATA FUNCTIONS ########################################################
cond = False
df = pd.DataFrame()

def get_data():
    global cond, s, df, lat, lon, lat_input, long_input, pos_x, pos_y, BLX, BLY, TRX, TRY  # , a, df
    cda_check_drop = 0
    wh_check_drop = 0
    if (cond == True):
        serialString = s.readline().rstrip().decode("utf-8")  # Read data in serial buffer
        try:
            serialDict = eval(serialString)  # Try to convert the raw string into a dictionary
        except (NameError, SyntaxError, ValueError) as e:  # catch any errors when trying to convert to dictionary
            pass
        else:
            #print(serialDict)

            # ALTITUDES AND SPEED
            df = df.append(serialDict, ignore_index=True)

            altitude = format(serialDict['alt'],".2f")
            str_alt.set(altitude)

            speed = format(serialDict['spd'], ".2f")
            #str_spd = tk.StringVar()
            str_spd.set(speed)
            # speed_num = tk.Label(alt_frame, textvariable=str_spd, font=("Fixedsys", 48), bd=5, relief="sunken")
            # speed_num.grid(row=1, column=6, sticky="ns")

            if float(serialDict['habHeight']) != 0 and wh_check_drop == 0:
                wh_alt = serialDict['habHeight']
                wh_cda_alt = tk.StringVar()
                wh_cda_alt.set(wh_alt)

                wh_num = tk.Label(alt_frame, textvariable=wh_cda_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
                wh_num.grid(row=1, column=4, sticky="ns")
                wh_check_drop = 1

            if float(serialDict['cdaHeight']) != 0 and cda_check_drop == 0:
                cda_alt = serialDict['cdaHeight']
                str_cda_alt = tk.StringVar()
                str_cda_alt.set(cda_alt)

                cda_num = tk.Label(alt_frame, textvariable=str_cda_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
                cda_num.grid(row=1, column=2, sticky="ns")
                cda_check_drop = 1
            
            root.update_idletasks()
            
            # GPS
            lon =  float(serialDict['lon'])
            lat = float(serialDict['lat'])
            ax.scatter(lon, lat, color = 'c')
            # lines.set_xdata(lon)
            # lines.set_ydata(lat)
            canvas.draw()

    root.after(100, get_data)


def data_stop():
    global cond, df, second_frame
    #cond = False
    for col_num, col_name in enumerate(df.columns):
        label = tk.Label(second_frame, width=6, height=1,
                         text=col_name, relief=tk.RIDGE)
        label.grid(row=1, column=col_num)
    for i in range(len(df)):
        for j in range(len(df.iloc[0, :])):
            element = round(df.iloc[i, j], 2)
            label = tk.Label(second_frame, width=6, height=1,
                             text=element, relief=tk.RIDGE)
            label.grid(row=i + 2, column=j)


######################################################## COM PORT INPUT GUI ####################################################
# function to return port # input
def format_port(port_number):
    com_port_str = 'COM' + str(port_number)
    return (com_port_str)


def get_port(entry):
    print('Port selected: ', entry)
    global comport_root
    comport_root.destroy()

    global cond
    cond = True
    # s.reset_input_buffer()

    global com_port
    if len(entry)<3: # formats entry properly if only a number was entered
        com_port = format_port(entry)
    else: 
        com_port = entry
        return com_port


# comport GUI
comport_root = tk.Tk()
comport_root.title('Please enter your COM port')
comport_root.geometry('380x80')

# entry box
entry = tk.Entry(comport_root)
entry.grid(row=1, column=0)

# label of available COM ports
comList = serial.tools.list_ports.comports()
# Get list of available COM ports
connected = ''
for element in comList:
    connected += (str(element.device) + '  ')
label = tk.Label(comport_root, \
                 text='Available COM Ports:\n' + connected,
                 bg='white')
label.grid(row=0, column=0)

# enter button
button = tk.Button(comport_root, \
                   text='Enter',
                   bg='white',
                   command=lambda: get_port(entry.get()))
button.grid(row=1, column=1)

comport_root.mainloop()

#################################################### BUILDING THE MAIN GUI ########################################################
# creating the master window (root)
root = tk.Tk()
root.title('Ground Station GUI')
# creating tab control
tabcontrol = ttk.Notebook(root, \
                          height=1000,
                          width=1800)

# adding tabs
dashboard = ttk.Frame(tabcontrol)
all_data = ttk.Frame(tabcontrol)
tabcontrol.add(dashboard, text='Dashboard')
tabcontrol.add(all_data, text='All DAS Data')
tabcontrol.pack(expand=1, fill='both')

# ------------------------------------------------------- DASHBOARD TAB ---------------------------------------------------------------
# GPS
# you must declare the variables as 'global' in the fxn before using#
lat = 0
lon = 0
pos_x = 0
pos_y = 0

#adjust these values based on your location and map, lat and long are in decimal degrees
TRX = -74.591483          #top right longitude
TRY = 40.164253            #top right latitude
BLX = -74.591002         #bottom left longitude
BLY = 40.164636             #bottom left latitude
lat_input = 0            #latitude of home marker
long_input = 0           #longitude of home marker

# creating plot object in GUI
# img = Image.open("formatted_airfield_test.jpeg")
# img = img.resize((400,480),Image.ANTIALIAS)
im = plt.imread('House.png')
fig = plt.Figure()
ax = fig.add_subplot(111)
ax.imshow(im, extent=[BLX, TRX, BLY, TRY])

ax.set_title('GPS')
ax.set_xlabel('longitude')
ax.set_ylabel('lattitude')
ax.set_xlim(BLX, TRX)
ax.set_ylim(BLY, TRY)
# lines = ax.plot([],[])[0]

canvas = FigureCanvasTkAgg(fig, master = dashboard)
canvas.get_tk_widget().place(x = 890, y = 240, width = 500, height = 500)
canvas.draw()

# CURRENT/PAYLOAD ALTITUDES AND SPEED
# Current altitude
alt_frame = tk.Frame(dashboard) # building a frame
alt_frame.place(relheigh=0.165, relwidth=0.70)

alt_label = tk.Label(alt_frame, text="Current Altitude (ft):", font=("Fixedsys", 24), fg='black')
alt_label.grid(sticky="nesw")

str_alt = tk.StringVar() # will be updated live in get_data() function
str_alt.set("")
alt_num = tk.Label(alt_frame, textvariable=str_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
alt_num.grid(row=1, column=0, sticky="ns")

# CDA altitude
cda_label = tk.Label(alt_frame, text="Colonist drop (ft):", font=("Fixedsys", 24), fg='black')
cda_label.grid(row=0, column=2, sticky="nesw", padx=15)

# Water/habitat altitude
wh_label = tk.Label(alt_frame, text="Water/habitat drop (ft):", font=("Fixedsys", 24), fg='black')
wh_label.grid(row=0, column=4, sticky="nesw", padx=12)

# Speed
speed_label = tk.Label(alt_frame, text="Speed (mph):", font=("Fixedsys", 24), fg='black')
speed_label.grid(row=0, column=6, sticky="nesw", padx=15)

str_spd = tk.StringVar()
str_spd.set("")
speed_num = tk.Label(alt_frame, textvariable=str_spd, font=("Fixedsys", 48), bd=5, relief="sunken")
speed_num.grid(row=1, column = 6, sticky="ns")

# ----------------------------------------------------- DATA PLAYBACK TAB ---------------------------------------------------------------
BOTH = 'both'
LEFT = 'left'
RIGHT = 'right'
VERTICAL = 'vertical'
Y = 'y'
style = ttk.Style()
style.configure("Vertical.TScrollbar", gripcount=0,
                background="Green", darkcolor="DarkGreen", lightcolor="LightGreen",
                troughcolor="gray", bordercolor="blue", arrowcolor="white")
# making a main frame
main_frame = tk.Frame(all_data)
main_frame.pack(fill=BOTH, expand=1)
# making a canvas
data_canvas = tk.Canvas(main_frame)
data_canvas.pack(side=LEFT, fill=BOTH, expand=1)
# making a scrollbar
scrollbar = ttk.Scrollbar(main_frame, orient=VERTICAL, command=data_canvas.yview, style="Vertical.TScrollbar")
scrollbar.pack(side=RIGHT, fill=Y)
# configure the canvas
data_canvas.configure(yscrollcommand=scrollbar.set)
data_canvas.bind('<Configure>', lambda e: data_canvas.configure(scrollregion=data_canvas.bbox('all')))
# making a second frame for the data
second_frame = tk.Frame(data_canvas)
# adding that^ frame to a window to be placed in the canvas
data_canvas.create_window((0, 300), window=second_frame, anchor='nw')
# adding a display data button
show_data_button = tk.Button(all_data, \
                             text='Click for data', command=lambda: data_stop())
show_data_button.place(anchor='nw')

s = sr.Serial(com_port, 9600)  # change COM Port name
s.reset_input_buffer()

root.after(5, get_data)
root.mainloop()
