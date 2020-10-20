import tkinter as tk
from tkinter import ttk
from PIL import Image
from PIL import ImageTk
import serial as sr
import numpy as np
import pandas as pd
import csv
import serial.tools.list_ports

################################################### SERIAL DATA FUNCTIONS ########################################################
# a = {\
#     "pkt":15,"drp":101,"alt":65,"temp":84.59,"lat":49.854826,
#     "lon":72.958413,"hdng":20.54,"spd":10.96,"aclx":2.12,"acly":6.54,
#     "aclz":1.25,"gyrox":1.26,"gyroy":1.48,"gyroz":2.08,"magx":1.85,
#     "magy":8.95,"magz":4.85,"habDrp":68.95,"cdaDrp":23.39,"watDrp":102.65,"rssi":-40
#     }  # Dummy Data
# df = pd.DataFrame.from_dict(data = a, orient = 'index').T
cond = False
df = pd.DataFrame()


def get_data():
    global cond, s, df  # , a, df
    cda_check_drop = 0
    wh_check_drop = 0
    if (cond == True):
        serialString = s.readline().rstrip().decode("utf-8")  # Read data in serial buffer
        try:
            serialDict = eval(serialString)  # Try to convert the raw string into a dictionary
            # serialDict = a
        except (NameError, SyntaxError, ValueError) as e:  # catch any errors when trying to convert to dictionary
            pass
        else:
            print(serialDict)
            df = df.append(serialDict, ignore_index=True)

            try:
                altitude = format(serialDict['alt'],".2f")
            except(NameError, SyntaxError, ValueError) as e:
                pass
            else:
                str_alt.set(altitude)

            speed = serialDict['spd']
            str_spd = tk.StringVar()
            str_spd.set(speed)
            speed_num = tk.Label(alt_frame, textvariable=str_spd, font=("Fixedsys", 48), bd=5, relief="sunken")
            speed_num.grid(row=1, column=6, sticky="ns")
            try:
                habCheck = float(serialDict['habHeight'])
            except(NameError, SyntaxError, ValueError) as e:
                pass
            else:
                if habCheck != 0 and wh_check_drop == 0:
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
    root.after(100, get_data)


def data_stop():
    global cond, df, second_frame
    #cond = False
    for col_num, col_name in enumerate(df.columns):
        label = tk.Label(second_frame, width=6, height=1,
                         text=col_name, relief=tk.RIDGE)
        label.grid(row=2, column=col_num+1)
    for i in range(len(df)):
        for j in range(len(df.iloc[0, :])):
            element = round(df.iloc[i, j], 2)
            label = tk.Label(second_frame, width=6, height=1,
                             text=element, relief=tk.RIDGE)
            label.grid(row=i + 3, column=j+1)


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
    com_port = format_port(entry)
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
# CURRENT/PAYLOAD ALTITUDES AND SPEED
alt_frame = tk.Frame(dashboard)
alt_frame.place(relheigh=0.165, relwidth=0.70)

# live altitude
# altitude = 0.00
# str_alt = tk.StringVar()
# str_alt.set(altitude)

alt_label = tk.Label(alt_frame, text="Current Altitude (ft):", font=("Fixedsys", 24), fg='black')
alt_label.grid(sticky="nesw")

str_alt = tk.StringVar()
str_alt.set("")
alt_num = tk.Label(alt_frame, textvariable=str_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
alt_num.grid(row=1, column=0, sticky="ns")
# alt_num = tk.Label(alt_frame, textvariable=str_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
# alt_num.grid(row = 1, sticky="ns")

# CDA altitude
# cda_alt = 0.00
# str_cda_alt = tk.StringVar()
# str_cda_alt.set(cda_alt)

cda_label = tk.Label(alt_frame, text="Colonist drop (ft):", font=("Fixedsys", 24), fg='black')
cda_label.grid(row=0, column=2, sticky="nesw", padx=15)

# cda_num = tk.Label(alt_frame, textvariable=str_cda_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
# cda_num.grid(row=1, column = 2, sticky="ns")

# Water/habitat altitude
# wh_alt = 0.00
# str_wh_alt = tk.StringVar()
# str_wh_alt.set(wh_alt)

wh_label = tk.Label(alt_frame, text="Water/habitat drop (ft):", font=("Fixedsys", 24), fg='black')
wh_label.grid(row=0, column=4, sticky="nesw", padx=12)

# wh_num = tk.Label(alt_frame, textvariable=str_cda_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
# wh_num.grid(row=1, column = 4, sticky="ns")

# Speed
# speed = 0.00
# str_speed = tk.StringVar()
# str_speed.set(speed)

speed_label = tk.Label(alt_frame, text="Speed (mph):", font=("Fixedsys", 24), fg='black')
speed_label.grid(row=0, column=6, sticky="nesw", padx=15)

# speed_num = tk.Label(alt_frame, textvariable=str_cda_alt, font=("Fixedsys", 48), bd=5, relief="sunken")
# speed_num.grid(row=1, column = 6, sticky="ns")

# # GPS
# img = Image.open("formatted_airfield_test.jpeg")
# img = img.resize((400,480),Image.ANTIALIAS)
# gps_img = ImageTk.PhotoImage(img)
# gps_display = tk.Label(dashboard, image = gps_img)
# gps_display.place(x=1000, y=250)

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
