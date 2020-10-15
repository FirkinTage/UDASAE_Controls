import tkinter as tk
import serial as sr
import numpy as np
import pandas as pd
import csv 

cond = False
# a = {\
#     "pkt":[15],"drp":[101],"alt":[65.42],"temp":[84.59],"lat":[49.854826],
#     "lon":[72.958413],"hdng":[20.54],"spd":[10.96],"aclx":[2.12],"acly":[6.54],
#     "aclz":[1.25],"gyrox":[1.26],"gyroy":[1.48],"gyroz":[2.08],"magx":[1.85],
#     "magy":[8.95],"magz":[4.85],"habDrp":[68.95],"cdaDrp":[23.39],"watDrp":[102.65],"rssi":[-40]
#     }
df = pd.DataFrame()
def get_data():
    global cond, df
    if (cond == True):
        a = s.readline()
        a.decode()
        df_new = pd.DataFrame.from_dict(data = a)
        df_new.iloc[0,2] += 10
        if (len(df) < 20):
            df = pd.concat([df, df_new], ignore_index=True) 
        else:
            df[0:19] = df[1:20]
            df = pd.concat([df, df_new], ignore_index=True)
            df.to_csv('Dynamic-DF.csv')
            
        # open file
            with open("Dynamic-DF.csv", newline = "") as file:
                reader = csv.reader(file)
                # r and c tell us where to grid the labels
                r = 1
                for col in reader:
                    c = 0
                    for row in col:
                        # i've added some styling
                        label = tk.Label(root, width = 10, height = 2, \
                            text = row, relief = tk.RIDGE)
                        label.grid(row = r, column = c)
                        c += 1
                    r += 1

    root.after(1, get_data)
    
def data_start():
    global cond
    cond = True
    #s.reset_input_buffer()

def data_stop():
    global cond
    cond = False
    print(df)


root = tk.Tk()
root.geometry('1000x1800')
root.update()
start = tk.Button(root,\
     text = 'Start', command = lambda: data_start())
start.grid()

root.update()
stop = tk.Button(root,\
    text = 'Stop', command = lambda: data_stop())
stop.grid(row = 0, column = 1)

root.after(1, get_data)
root.mainloop()

s = sr.Serial('/dev/cu.URT0', 115200) # change COM Port name
s.reset_input_buffer()
