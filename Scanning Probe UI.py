import PySimpleGUI as sg
import pyscreenshot as ImageGrab
import math
import tkinter as tk
import random
import time
import RPi.GPIO as GPIO
import spidev
import serial as sr
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def GraphData(DATAX,DATAY,DATAZ):
    #print(DATAX,DATAY,DATAZ)
    fig = plt.figure()
    ax = plt.axes(projection = '3d')
    ax.scatter3D(DATAX,DATAY,DATAZ, c = DATAZ, cmap = 'Greens')
    plt.show()

def senddatatinyg(data): #sending anything to the motor controller
    if port != None: #checks if there is an open port 
        port.write((data + "\n").encode('utf-8')) #sends data message that the tinyg can read
        time.sleep(0.2)
def getdatatinyg(): #recieveing data from tinyg
    response = port.readline()
    time.sleep(0.2)
    return response
def DAC_COMM(val,Port):
#Controls the voltage of the output pins of the DAC
    bus = 0
    device = 0 #CE0 pin of the raspberry pi is connected to the DAC

    spi = spidev.SpiDev(0,0)
    spi.close()
    spi.open(bus, device)
    #Open SPI connection w/ DAC
    spi.max_speed_hz = 500000
    #Speed at which the Master(Pi) communicates

    spi.mode = 0 #Clock Polarity and Phase


    GPIO.setmode(GPIO.BCM) #Setting CE0 as Output (May not be neccesary but oh well)
    GPIO.setup(8,GPIO.OUT)
    #see DAC data sheet for specific bit commands (LTC2668)
    if Port == 0 and (3276 < val < 62258): #making sure the value we want to send is within the tolerances we want (5% - 95% of our range)
        commandwrite = 0b00000000   #Write command
        commandupdate = 0b00010000  #Update command
    elif Port == 1 and (3276 < val < 62258):
        commandwrite = 0b00000001
        commandupdate = 0b00010001
    elif Port == 2 and (3276 < val < 32765):
        commandwrite = 0b00000010
        commandupdate = 0b00010010
    elif Port == 3 and (3276 < val < 62258):
        commandwrite = 0b00000011
        commandupdate = 0b00010011
    elif Port == 4 and (3276 < val < 62258):
        commandwrite = 0b00000100
        commandupdate = 0b00010100
    elif Port == 5 and (3276 < val < 62258):
        commandwrite = 0b00000101
        commandupdate = 0b00010101
    elif Port == 6 and (3276 < val < 62258):
        commandwrite = 0b00000110
        commandupdate = 0b00010110
    elif Port == 7 and (3276 < val < 62258):
        commandwrite = 0b00000111
        commandupdate = 0b00010111
    elif Port == 8 and (3276 < val < 62258):
        commandwrite = 0b00001000
        commandupdate = 0b00011000
    elif Port == 9 and (3276 < val < 62258):
        commandwrite = 0b00001001
        commandupdate = 0b00011001
    elif Port == 10 and (3276 < val < 62258):
        commandwrite = 0b00001010
        commandupdate = 0b00011010
    elif Port == 11 and (3276 < val < 62258):
        commandwrite = 0b00001011
        commandupdate = 0b00011011
    elif Port == 12 and (3276 < val < 62258):
        commandwrite = 0b00001100
        commandupdate = 0b00011100
    elif Port == 13 and (3276 < val < 62258):
        commandwrite = 0b00001101
        commandupdate = 0b00011101
    elif Port == 14 and (3276 < val < 62258):
        commandwrite = 0b00001110
        commandupdate = 0b00011110
    elif Port == 15 and (3276 < val < 62258):
        commandwrite = 0b00001111
        commandupdate = 0b00011111
    else:
        return None
    val = int(val)
    bin_val = format(val,'016b')    #Taking input and converting it to binary
    lowByte = bin_val[8:16]         #Chopping up 16 bit binary into HighByte and LowByte
    highByte = bin_val[0:8]
    #print (bin_val)
    #print(lowByte)
    #print(highByte)
    lowByte1 = int(lowByte, base = 2)       #Converting Highbyte and lowbyte back to int for spi.xfer2 function
    highByte1 = int(highByte, base = 2)


    '''
    SpanCommand = 0b11100000
    spanhigh = 0b10101010
    spanlow = 0b10101010
    Commands for changing the output range of the DAC board, it is currently manually wired to output +- 10V
    '''
    spi.xfer2([commandwrite,highByte1,lowByte1]) #Sending command and data to write to DAC register
    spi.xfer2([commandupdate,highByte1,lowByte1])  #Sending command to update DAC register
    spi.close()
def ADC_COMM_CANT(Channel): #Commuication with 0V - 5V inputs from ADC returns input Voltage
    bus = 0
    device = 1 #CE1 is hard wired from the pi to the ADC

    spi = spidev.SpiDev(0,1)
    spi.close()
    spi.open(bus, device)
    #Open SPI connection
    spi.max_speed_hz = 500000
    #Speed at which the Master(Pi) communicates

    spi.mode = 0 #Clock Polarity and Phase
    GPIO.setmode(GPIO.BCM) #Setting CE0 as Output (May not be neccesary but oh well)
    GPIO.setup(7,GPIO.OUT)
    #choosing which MUX input to read (from 0-5V) to change edit bits 5 and 6 (look at data sheet for more info DC682A)
    if Channel == 0:
        MUX = 0b10001000
    if Channel == 1:
        MUX = 0b11001000
    if Channel == 2:
        MUX = 0b10011000
    if Channel == 3:
        MUX = 0b11011000
    if Channel == 4:
        MUX = 0b10101000
    if Channel == 5:
        MUX = 0b11101000
    if Channel == 6:
        MUX = 0b10111000
    if Channel == 7:
        MUX = 0b11111000
    #print(MUX)
    #GPIO.output(7,0)
    #time.sleep(2)
    #Recieving 1011 values from ADC and finding the mean to help mitigate error
    VCant = np.zeros(1011)
    i = 1010
    while i > 0:
        ADC_Output = spi.xfer([MUX,MUX])
        LowByte = ADC_Output[1]
        HighByte = ADC_Output[0]
    #print (ADC_Output)
        Result = (HighByte << 8) | (LowByte)
    #print (Result)
        ResultBin = format(Result,'016b')
    #print (ResultBin)
        VCant[i] = (Result / 65535) * 5
        i = i-1
    #print (Voltage)
    #GPIO.output(7,0)
    #time.sleep(0.01)
    Voltage = np.trim_zeros(VCant)
    Voltage = np.mean(Voltage)
    spi.close()
    #print (Voltage)
    return Voltage
#Function for saving the graph
def Save_Element(element,filename):
    widget = element.Widget
    #Getting the bounding box for the graph
    box = (widget.winfo_rootx(), widget.winfo_rooty(), widget.winfo_rooty() + widget.winfo_width(),widget.winfo_rooty()+widget.winfo_height())
    grab = ImageGrab.grab(bbox=box)
    grab.save(filename)
#Function to move a point to a specific location on the graph
def Move_PointAbs(figure,x,y):
    graph.RelocateFigure(figure,x,y)
    graph.update()
#Function to Move a point relative to its last location
def Move_Point(figure,x,y):
    graph.MoveFigure(figure,x,y)
    graph.update()
#Function that turns the Voltage int that we sent to the DAC into the corresponding distance from center the piezotube moves
def CalculateDistances(VoltBitX,VoltBitY,VoltBitZ):
    base0 = 32763
    nmperBitt = 0.4917
    DistX = 0
    DistY = 0
    DistZ = 0
    DistX = (VoltBitX - base0) * nmperBitt
    DistY = (VoltBitY - base0) * nmperBitt
    DistZ = (VoltBitZ - base0) * nmperBitt
    return DistX,DistY,DistZ
#DAC port 14 => +X, port 15 => -X, port 0 => +Y, port 1 => -Y, port 2 => Z
def GetNegativeVoltage(PosVoltage):
    mask = 0b1111111111111111 #takes a positive voltage and flips the bits so that the negative voltage is obtained
    return (PosVoltage^mask)
def storeData(dateuh,dataname):
    np.savetxt("{}.dat".format(dataname),dateuh) #saving our data as a .dat file to be graphed later
def PID(Reading, DiffOld,PastI,setpoint):
    try:
        #time.sleep(0.1)
        delta_t = 0.1 #Time inbetween steps in the pid
        Pconst = 10 #constants that need to be developed
        Iconst = 0.1
        Dconst = 10
        Diff = setpoint - Reading
        print("Cant Voltage")
        print(Reading)
        P = Pconst * Diff #proportional component
        print("P")
        print (P)
        I = PastI + Iconst*Diff*delta_t #Integral component
        print("I")
        print(I)
        D = -(Dconst * (Diff - DiffOld))/(delta_t) #Differential component
        print("D")
        print(D)

        Output = P + I + D
        #print(Output)
        #print(Diff)
        return Output,Diff,I
    except KeyboardInterrupt:
        print("Press Ctrl-C to terminate loop")
#Constants are defined here
retractedZ = 300
Base0 = 32700
Base0Z = 32650
XINC = 1 #how many pixels the dots on the UI graph move
YINC = 1

setpoint = 2.5 #set voltage which corresponds to the desired height we want the cantilever above the sample when scanning
Tol = 0.1
CoarseSteps = 0 #Number of steps taken by the coarse Z positioning (keeps track so we retract to the same height as we started)

#setting up the variables for the PID
CANTVOLTAGE = ADC_COMM_CANT(7)
Difference = 0
Change = 0
PastI = 0

VoltageZ = Base0 #setting all the values corresponding to the output voltages to zero
VoltageX = Base0
VoltageY = Base0
NegVoltageX = Base0
NegVoltageY = Base0

zincriment = 0.05 #corresponds to 1.5 degree turn of stepper motor
xincriment = 1
yincriment = 1

DAC_COMM(Base0,14) #setting all outputs to zero
DAC_COMM(Base0,15)
DAC_COMM(Base0,0)
DAC_COMM(Base0,1)
DAC_COMM(Base0,2)

PIDcontrol = False #Setting the default for the PID control to be off
sg.theme('Dark Blue 3')

#Defining what we can do with graph, I.E events, saving, communication
#These are the buttons on the right of the graph
guy =[[sg.B('Scan',key = '-SCAN-',enable_events = True)],\
    [sg.Button('Exit')],\
    [sg.B('Retract Z Coarse', key = '-RETRACT-', enable_events = True)],\
    [sg.B('Approach Z Coarse', key = '-APPROACH-', enable_events = True)],\
    #[sg.Text('Current Piezo Position',key = '-CURRENTLABEL-')],\
    #[sg.Text('Current Coarse Position',key = '-CURRENTLABELCOARSE-')],\
    [sg.Text(key='info',size=(60,1))],\
    [sg.Text(key = '-CURRENTPOSITION-',size = (60,1))],\
    [sg.B('Return Piezo to Zero',key = '-RETURNZERO-',enable_events = True)],\
    [sg.B('3D Graph',key = '-3DGRAPH-',enable_events = True),],]
Coarsepositioning=[[sg.B('+X Coarse',key = '-XUPONE-',enable_events = True)],\
    [sg.B('-X Coarse',key = '-XDOWNONE-',enable_events = True)],\
    [sg.B('+Y Coarse',key = '-YUPONE-',enable_events = True)],\
    [sg.B('-Y Coarse',key = '-YDOWNONE-',enable_events = True)],\
    [sg.B('+Z Coarse',key = '-ZUPONE-',enable_events = True)],\
    [sg.B('-Z Coarse',key = '-ZDOWNONE-',enable_events = True)],]\
#Defining the Menu
menu_def = [['File',['Open','Save','Exit']],\
    ['Edit',['Paste',['Special','Normal',],'Undo'],],\
    ['Help',['See Documentation lol'],],]
#Creating the bottom left and top right corners of the graph coordinate system
#Will be useful later when changing coordinates
graph_bottom_leftX = 0
graph_bottom_leftY = 0
graph_top_rightX = 900
graph_top_rightY= 900
Cgraph_bottom_leftX = 700
Cgraph_bottom_leftY = 1100
Cgraph_top_rightX = 1000
Cgraph_top_rightY= 1400

port = sr.Serial("/dev/ttyUSB0", 115200, timeout = 0.5) #Opening serial port to tinyg and setting it to relative movement
senddatatinyg("g91")

#Defining the main window layout, buttons and inputs that are here are under our graph
layout = [[sg.Menu(menu_def)],\
    #Defining how many pixels the Graph takes up
    [sg.Graph(canvas_size = (900,900),\
    #Defining the bottom left xy values and the top right xy values
    graph_bottom_left = (graph_bottom_leftX,graph_bottom_leftY),\
    graph_top_right = (graph_top_rightX,graph_top_rightY),\
    key = "-GRAPH-",\
    change_submits = True,\
    background_color = 'white',\
    drag_submits = True),\
    sg.Graph(canvas_size = (400,400),\
    #Defining the bottom left xy values and the top right xy values
    graph_bottom_left = (Cgraph_bottom_leftX,Cgraph_bottom_leftY),\
    graph_top_right = (Cgraph_top_rightX,Cgraph_top_rightY),\
    key = "-CGRAPH-",\
    change_submits = True,\
    background_color = 'white',\
    drag_submits = True),\
    sg.Col(Coarsepositioning),\
    #Calling buttons defined earlier
    sg.Col(guy)],]
    #Defining the Inputs for absolute movement and the outputs for current position and scale

#Creating our main window
window = sg.Window("Graph Test",layout,size = (1920,1080),finalize=True)

#Graph we defined above (sg.graph)
graph = window['-GRAPH-']
#Creating the  Current point as well as storing its id to manipulate them later
idcurrent = graph.draw_point((450,450),1,'blue')

cgraph = window['-CGRAPH-']
idcoarse = cgraph.draw_point((850,1250),1,'red')
#dragging is by default false, making it so that you need an event
dragging = False
start_point = end_point = prior_fig = None
graph.bind('<Button-3>','+Right+')

while True:
    event, values = window.read(timeout = 10)
    if event == None or event == 'Exit':
        break
    #draw_axis()
    elif event == '-RETURNZERO-':
        layout14 = [[sg.Checkbox('Uncheck to Stop',default = True, key = '-STOPBUTTONBACK-')],\
        [sg.B('Go',key = '-StartZero-',enable_events = True)],[sg.Exit()]]
        window14 = sg.Window('Return to Zero')
        window14.layout(layout14)
        window14.finalize()
        PIDcontrol = False
        while True:
            event,values = window14.read(timeout = 10)
            if event == None or event == 'Exit':
                break
            if event == '-StartZero-':
                #If any of the voltages are not at 0, loop through and change the ones not at zero until they reach zero
                while (int(VoltageZ) != Base0Z) or (int(VoltageX) != Base0) or (int(VoltageY) != Base0) or (int(NegVoltageX) != Base0) or (int(NegVoltageY) != Base0):
                    print(VoltageZ)
                    time.sleep(0.01)
                    if (VoltageZ > Base0Z):
                        VoltageZ = VoltageZ - 1
                        DAC_COMM(VoltageZ,2)
                    if (VoltageZ < Base0Z):
                        VoltageZ = VoltageZ + 1
                        DAC_COMM(VoltageZ,2)
                    if (VoltageX > Base0):
                        VoltageX = VoltageX - 1
                        DAC_COMM(VoltageX,14)
                    if (VoltageX < Base0):
                        VolatgeX = VoltageX + 1
                        DAC_COMM(VoltageX,14)
                    if(NegVoltageX > Base0):
                        NegVoltageX = NegVoltageX - 1
                        DAC_COMM(NegVoltageX,15)
                    if(NegVoltageX < Base0):
                        NegVoltageX = NegVoltageX + 1
                        DAC_COMM(NegVoltageX,15)
                    if(VoltageY > Base0):
                        VoltageY = VoltageY - 1
                        DAC_COMM(VoltageY,0)
                    if(VoltageY < Base0):
                        VoltageY = VoltageY + 1
                        DAC_COMM(VoltageY,0)
                    if(NegVoltageY > Base0):
                        NegVoltageY = NegVoltageY - 1
                        DAC_COMM(NegVoltageY,1)
                    if(NegVoltageY < Base0):
                        NegVoltageY = NegVoltageY + 1
                        DAC_COMM(NegVoltageY,1)
        window14.close()
    elif event == '-3DGRAPH-':
        #Creating a pop up window that lets the user input the filename they want to store the image as
        layout5 = [[sg.T('Open data file')],\
        [sg.Input('',key = '-IN-')],\
        [sg.FileBrowse(target = '-IN-')],\
        [sg.Button('Ok'),sg.Button('Exit')]]
        window5 = sg.Window('Open Graph'). Layout(layout5)
        while True:
            event,values = window5.read(timeout = 10)
            if event == None or event == 'Exit':
                break
            if event == 'Ok':
                filename = values['-IN-']   #getting input from the browse buttom or user 
                fulldata = np.loadtxt(filename) #opening the data file 
                datax = fulldata[0] #getting x y and z data
                #print (datax)
                datay = fulldata[1]
                #print (datay)
                dataz = fulldata[2]
                #print (dataz)
                GraphData(datax,datay,dataz)
        window5.close()

    #Below is events in the x y coarse positioning system
    elif event == '-ZUPONE-':
        #tinyg gets movement as g0x0.01
        senddatatinyg("g0z{}".format(xincriment))
    elif event == '-ZDOWNONE-':
        senddatatinyg("g0z{}".format(-xincriment))
    elif event == '-XUPONE-':
        senddatatinyg("g0x{}".format(xincriment))
        cgraph.MoveFigure(idcoarse,1,0)
        cgraph.update()
    elif event == '-XDOWNONE-':
        senddatatinyg("g0x{}".format(-xincriment))
        cgraph.MoveFigure(idcoarse,-1,0)
        cgraph.update()
    elif event == '-YUPONE-':
        senddatatinyg("g0y{}".format(yincriment))
        cgraph.MoveFigure(idcoarse,0,1)
        cgraph.update()
    elif event == '-YDOWNONE-':
        senddatatinyg("g0y{}".format(-yincriment))
        cgraph.MoveFigure(idcoarse,0,-1)
        cgraph.update()
    elif event == '-RETRACT-':
        #making the window
        layout4 = [[sg.T('Enter timeout between Z movements in milliseconds below')],\
        [sg.Input('50', key = '-TIMEOUTRET-')],\
        [sg.Checkbox('Uncheck to Stop',default = True, key = '-STOPBUTTONRET-')],\
        [sg.B('Go',key = '-StartRet-',enable_events = True)],[sg.Exit()]]
        window4 = sg.Window('Retract Z')
        window4.layout(layout4)
        window4.finalize()
        PIDcontrol = True
        while True:
            event,values = window4.read(timeout = 10)
            if event == None or event == 'Exit':
                break
            if event == '-StartRet-':
                #getting the value for the timout
                TimeoutRet = values['-TIMEOUTRET-']
                StopCheckRet = values['-STOPBUTTONRET-']
                for element in TimeoutRet: #error checking
                    if (element != '0' and element != '1' and element != '2' and element != '3' and element != '4'\
                    and element != '5' and element != '6' and element != '7' and element != '8' and element != '9'):
                        TimeoutRet = -1
                if (TimeoutRet == -1): #popup if error
                    layout12 = [[sg.T('Please enter a single integer for Timeout (in milliseconds)')],\
                    [sg.Button('Ok'),sg.Button('Exit')]]
                    window12 = sg.Window('Whoops')
                    window12.layout(layout12)
                    window12.finalize()
                    while True:
                        event,values = window12.read(timeout = 10)
                        if event == 'Exit' or event == 'Ok':
                            break
                    window12.close()
                    break
                else:
                    TimeoutRet = int(TimeoutRet)
                while(CoarseSteps > 0 and StopCheckRet):
                    senddatatinyg("g0z{}".format(-zincriment))
                    time.sleep(3)
                    CoarseSteps = CoarseSteps - 1
                    print(CoarseSteps)
                    event,values = window4.read(timeout = 10)
                    StopCheckRet = values['-STOPBUTTONRET-']
            StopCheckRet = values['-STOPBUTTONRET-']
            CANTVOLTAGE = ADC_COMM_CANT(7)
            while (not((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)) and PIDcontrol): #if not within the tolerance we want
                Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI,setpoint) #feed into PID system
                VoltageZ = VoltageZ + Change
                DAC_COMM(VoltageZ, 2)
                CANTVOLTAGE = ADC_COMM_CANT(7)
                if ((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)):
                    break
                event,values = window4.read(timeout = 10)
                StopCheckRet = values['-STOPBUTTONRET-']
        PIDcontrol = False
        window4.close()
    elif event == '-APPROACH-':
        layout2 = [[sg.T('Enter timeout between Z movements in milliseconds below')],\
        [sg.Input('50', key = '-TIMEOUTAPP-')],\
        [sg.Checkbox('Uncheck to Stop',default = True, key = '-STOPBUTTONAPP-')],\
        [sg.B('Go',key = '-StartApp-',enable_events = True)],[sg.Exit()]]
        window2 = sg.Window('Approach Z')
        window2.layout(layout2)
        window2.finalize()
        while True:
            event,values = window2.read(timeout = 10)

            if event == None or event == 'Exit':
                break
            if event == '-StartApp-':
                TimeoutApp = values['-TIMEOUTAPP-']
                StopCheckApp = values['-STOPBUTTONAPP-']
                PIDcontrol = False
                for element in TimeoutApp:
                    if (element != '0' and element != '1' and element != '2' and element != '3' and element != '4'\
                    and element != '5' and element != '6' and element != '7' and element != '8' and element != '9'):
                        TimeoutApp = -1
                if (TimeoutApp == -1):
                    layout11 = [[sg.T('Please enter a single integer for Timeout (in milliseconds)')],\
                    [sg.Button('Ok'),sg.Button('Exit')]]
                    window11 = sg.Window('Whoops')
                    window11.layout(layout11)
                    window11.finalize()
                    while True:
                        event,values = window11.read(timeout = 10)
                        if event == 'Exit' or event == 'Ok':
                            break
                    window11.close()
                    break
                else:
                    TimeoutApp = int(TimeoutApp)
                DAC_COMM(VoltageZ,2) #setting piezotube to normal
                CANTVOLTAGE = ADC_COMM_CANT(7)
                if (math.isnan(CANTVOLTAGE)):
                    CANTVOLTAGE = 0
                #while loop that does the approach, we step in the z direction until we hit the threshold value
                while (CANTVOLTAGE <= 3 and StopCheckApp): #EXP need to check what noise value we are getting and make sure that it stops at an actual reading not noise
                    senddatatinyg("g0z{}".format(zincriment))
                    time.sleep(3)
                    CANTVOLTAGE = ADC_COMM_CANT(7)
                    print("Cant Voltage")
                    print(CANTVOLTAGE)
                    if (math.isnan(CANTVOLTAGE)):
                        CANTVOLTAGE = 0
                    print(CoarseSteps)
                    CoarseSteps = CoarseSteps + 1
                    event,values = window2.read(timeout = 10)
                    StopCheckApp = values['-STOPBUTTONAPP-']
                #turning the PID control on
                PIDcontrol = True
            #PID control takes over and backs the piezotube up to where we want it
            StopCheckApp = values['-STOPBUTTONAPP-']
            CANTVOLTAGE = ADC_COMM_CANT(7)
            while (not((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)) and PIDcontrol): #if not within the tolerance we want
                Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI,setpoint) #feed into PID system
                VoltageZ = VoltageZ + Change
                DAC_COMM(VoltageZ,2)
                CANTVOLTAGE = ADC_COMM_CANT(7)
                if ((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)):
                    break
                event,values = window2.read(timeout = 10)
                StopCheckApp = values['-STOPBUTTONAPP-']
        window2.close()
    elif event == '-SCAN-':
        menu_def1 = [['Help',['Insert Timeout between movements in milliseconds(first box),then Scanning range in nm for x(second box), the same for y (third box), then filename w/o extension'],],]
        layout3 = [[sg.Menu(menu_def1)],\
        [sg.T('Enter timeout in milliseconds below')],\
        [sg.Input('50', key = '-TIMEOUT-')],\
        [sg.T('Enter X scanning range in nanometers below')],\
        [sg.Input('1000', key = '-SCANRANGEX-')],\
        [sg.T('Enter Y scanning range in nanometers below')],\
        [sg.Input('1000', key = '-SCANRANGEY-')],\
        [sg.T('Enter Filename without extention below')],\
        [sg.Input('Dataname', key = '-DATANAME-')],\
        [sg.Checkbox('Uncheck to Stop',default = True, key = '-STOPBUTTON-')],\
        [sg.Button('Ok'),sg.Exit()]]
        window3 = sg.Window('Scan')
        window3.layout(layout3)
        window3.finalize()
        PIDcontrol = True
        while True:
            event,values = window3.read(timeout = 10)
            if event == None or event == 'Exit':
                break
        #Retrieving information user inputs for incriments
            if event == 'Ok':
                Timeout = (values['-TIMEOUT-'])
                Yrangenm = (values['-SCANRANGEY-'])
                Xrangenm = (values['-SCANRANGEX-'])
                Dataname = values['-DATANAME-']
                StopCheck = values['-STOPBUTTON-']
                #----------------------input checking-----------------------------
                for element in Timeout:
                    if (element != '0' and element != '1' and element != '2' and element != '3' and element != '4'\
                    and element != '5' and element != '6' and element != '7' and element != '8' and element != '9'):
                        Timeout = -1
                if (Timeout == -1):
                    layout8 = [[sg.T('Please enter a single integer for Timeout (in milliseconds)')],\
                    [sg.Button('Ok'),sg.Button('Exit')]]
                    window8 = sg.Window('Whoops')
                    window8.layout(layout8)
                    window8.finalize()
                    while True:
                        event,values = window8.read(timeout = 10)
                        if event == 'Exit' or event == 'Ok':
                            break
                    window8.close()
                    break
                else:
                    Timeout = int(Timeout)
                for element in Xrangenm:
                    if (element != '0' and element != '1' and element != '2' and element != '3' and element != '4'\
                    and element != '5' and element != '6' and element != '7' and element != '8' and element != '9'):
                        Xrangenm = 0
                if Xrangenm == 0:
                    layout9 = [[sg.T('Please enter a single integer for X scanning range (in nanometers)')],\
                    [sg.Button('Ok'),sg.Button('Exit')]]
                    window9 = sg.Window('Whoops')
                    window9.layout(layout9)
                    window9.finalize()
                    while True:
                        event,values = window9.read(timeout = 10)
                        if event == 'Exit' or event == 'Ok':
                            break
                    window9.close()
                    break
                else:
                    Xrangenm = int(Xrangenm)
                for element in Yrangenm:
                    if (element != '0' and element != '1' and element != '2' and element != '3' and element != '4'\
                    and element != '5' and element != '6' and element != '7' and element != '8' and element != '9'):
                        Yrangenm = 0
                if (Yrangenm == 0):
                    layout10 = [[sg.T('Please enter a single integer for Y scanning range (in nanometers)')],\
                    [sg.Button('Ok'),sg.Button('Exit')]]
                    window10 = sg.Window('Whoops')
                    window10.layout(layout10)
                    window10.finalize()
                    while True:
                        event,values = window10.read(timeout = 10)
                        if event == 'Exit' or event == 'Ok':
                            break
                    window10.close()
                    break
                else:
                    Yrangenm = int(Yrangenm)
                if (Yrangenm > 30000 or Yrangenm < 9):
                    layout6 = [[sg.T('Yrange is 10 to 30000 Nano Meters')],\
                    [sg.Button('Ok'),sg.Button('Exit')]]
                    window6 = sg.Window('Whoops')
                    window6.layout(layout6)
                    window6.finalize()
                    while True:
                        event,values = window6.read(timeout = 10)
                        if event == 'Exit' or event == 'Ok':
                            break
                    window6.close()
                    break
                if (Xrangenm > 30000 or Xrangenm < 9):
                    layout7 = [[sg.T('Xrange is 10 to 30000 Nano Meters')],\
                    [sg.Button('Ok'),sg.Button('Exit')]]
                    window7 = sg.Window('Whoops')
                    window7.layout(layout7)
                    window7.finalize()
                    while True:
                        event,values = window7.read(timeout = 10)
                        if event == 'Exit' or event == 'Ok':
                            break
                    window7.close()
                    break
            #----------------------end input checking-----------------------------
            #starts with setting up graph stuff this is not important to the actual scan just gives a visual of it
            #will mark where the actual scanning stuff starts
            #----------------------Graph Setup------------------------------------
            #Getting the current locations of each point
                CBOX = graph.GetBoundingBox(idcurrent)
                COARSEBOX = cgraph.GetBoundingBox(idcoarse)
            #GetBoundingBox gives two pairs of xy coordinates in the form of ((x,y),(x,y)) for each point
                CBBOX = CBOX[0]
                CBBOY = CBOX[1]

                CurrentX = CBBOX[0] - 2
                CurrentY = CBBOY[1] + 2

                CoarseX = COARSEBOX[0]
                CoarseY = COARSEBOX[1]

                CurrentCoarse = CoarseX[0] - 2
                CurrentCoarse = CoarseY[1] + 2
            #print(CurrentX)
            #print(CurrentY)
            #----------------------End Graph Setup------------------------------------
            #Starting of the actual scanning code
            #TODO look into getting the piezotube to start in the bottom left of its range to maximize scanning range
                VoltageX = 32767 #reading 0 Volts on DAC, this number in binary will be set to DAC to output a corresponding voltage
                VoltageY = 32767
            #We set the Z voltage very low because at the end of the coarse positioning approach the piezotube will be at this same value

                Piezomin = 0 #piezomin and max control the min and max values for the Z output since it must stay negative
                Piezomax = 32767
                Yrange = 25
                Xrange = 25 #Determines how many data point we collect (50x50 for example) HIGHLY VARIABLE
                #arrays that will store position for X,Y, and Z
                DATAZ = []
                DATAX = []
                DATAY = []
                nmperbit = 0.4917
                Xrangebit =(Xrangenm / nmperbit)  #Converting the userinput for scanning range into bits to feed to DAC
                Yrangebit =(Xrangenm / nmperbit)  #Takes the desired length we want to scan and sees how many bits it will take
                XSTEP = int(Xrangebit/Xrange)      #Takes the number of bits the scan will take and divides it by the number of desired steps
                YSTEP = int(Yrangebit/Yrange)
                CANTVOLTAGE = ADC_COMM_CANT(7)
                if(math.isnan(CANTVOLTAGE)):
                    CANTVOLTAGE = 0
                if (not((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol))): #if not within the tolerance we want
                    while StopCheck: #Piezomin < VoltageZ < Piezomax: #loop unless the Voltage in the Z gets out of range
                        print("cant Reading")
                        print(CANTVOLTAGE)
                        Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI,setpoint) #feed into PID system
                        VoltageZ = VoltageZ + Change
                        DAC_COMM(VoltageZ, 2)
                        CANTVOLTAGE = ADC_COMM_CANT(7)
                        if ((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)):
                            break
                        event,values = window3.read(timeout = 10)
                        StopCheck = values['-STOPBUTTON-']
                while Yrange > 0 and StopCheck:
                    Xrange = 25
                    XrangeBack = 25
                    while Xrange > 0 and StopCheck:
                        DistanceX,DistanceY,DistanceZ = CalculateDistances(VoltageX,VoltageY,VoltageZ)
                        DATAX.append(DistanceX)             #Storing Data into arrays
                        DATAY.append(DistanceY)
                        DATAZ.append(DistanceZ)
                        CurrentX = CurrentX + XINC #purely for the UI graph
                        VoltageX = VoltageX + XSTEP #actual change to the scan
                        CANTVOLTAGE = ADC_COMM_CANT(7) #getting our input from cantilever
                        print("cant Reading")
                        print(CANTVOLTAGE)
                        #Z FEEDBACK
                        if (not((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol))): #if not within the tolerance we want
                            while StopCheck: #Piezomin < VoltageZ < Piezomax: #loop unless the Voltage in the Z gets out of range
                                Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI,setpoint) #feed into PID system
                                VoltageZ = VoltageZ + Change
                                DAC_COMM(VoltageZ, 2)
                                CANTVOLTAGE = ADC_COMM_CANT(7)
                                if ((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)):
                                    break
                                event,values = window3.read(timeout = 10)
                                StopCheck = values['-STOPBUTTON-']
                        DAC_COMM(VoltageX,14) #Sending Data to DAC
                        NegVoltageX = GetNegativeVoltage(VoltageX)
                        DAC_COMM(NegVoltageX,15)
                        Xrange = Xrange - 1
                        event,values = window3.read(timeout = 10)
                        StopCheck = values['-STOPBUTTON-']
                        #updating the graph
                        graph.MoveFigure(idcurrent,XINC,0)
                        graph.update()
                        info = window["-CURRENTPOSITION-"]
                        info.update(value = f"Current position is ({CurrentX},{CurrentY})")
                        window.Read(timeout = Timeout)
                        window.Refresh()

                    while XrangeBack > 0 and StopCheck:
                        CurrentX = CurrentX - XINC
                        VoltageX = VoltageX - XSTEP
                        CANTVOLTAGE = ADC_COMM_CANT(7)
                        print("cant Reading")
                        print(CANTVOLTAGE)
                        if (not((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol))): #if not within the tolerance we want
                            while StopCheck: #Piezomin < VoltageZ < Piezomax: #loop unless the Voltage in the Z gets out of range
                                Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI,setpoint) #feed into PID system
                                VoltageZ = VoltageZ + Change
                                DAC_COMM(VoltageZ, 2)
                                CANTVOLTAGE = ADC_COMM_CANT(7)
                                if ((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)):
                                    break
                                event,values = window3.read(timeout = 10)
                                StopCheck = values['-STOPBUTTON-']
                        DAC_COMM(VoltageX,14)
                        NegVoltageX = GetNegativeVoltage(VoltageX)
                        DAC_COMM(NegVoltageX,15)
                        XrangeBack = XrangeBack - 1
                        event,values = window3.read(timeout = 10)
                        StopCheck = values['-STOPBUTTON-']
                        graph.MoveFigure(idcurrent,-XINC,0)
                        graph.update()
                        info = window["-CURRENTPOSITION-"]
                        info.update(value = f"Current position is ({CurrentX},{CurrentY})")
                        window.Read(timeout = Timeout/10)
                        window.Refresh()
                    CurrentY = CurrentY + YINC
                    VoltageY = VoltageY + YSTEP
                    DistanceX,DistanceY,DistanceZ = CalculateDistances(VoltageX,VoltageY,VoltageZ)
                    DATAX.append(DistanceX)             #Storing Data into arrays
                    DATAY.append(DistanceY)
                    DATAZ.append(DistanceZ)
                    event,values = window3.read(timeout = 10)
                    StopCheck = values['-STOPBUTTON-']
                    CANTVOLTAGE = ADC_COMM_CANT(7)
                    print("cant Reading")
                    print(CANTVOLTAGE)
                    if (not((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol))): #if not within the tolerance we want
                            while StopCheck: #Piezomin < VoltageZ < Piezomax: #loop unless the Voltage in the Z gets out of range
                                Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI,setpoint) #feed into PID system
                                VoltageZ = VoltageZ + Change
                                DAC_COMM(VoltageZ, 2)
                                CANTVOLTAGE = ADC_COMM_CANT(7)
                                if ((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)):
                                    break
                                event,values = window3.read(timeout = 10)
                                StopCheck = values['-STOPBUTTON-']
                    DAC_COMM(VoltageY,0)
                    NegVoltageY = GetNegativeVoltage(VoltageY)
                    DAC_COMM(NegVoltageY,1)
                    Yrange = Yrange - 1
                    graph.MoveFigure(idcurrent,1,YINC)
                    graph.update()
                    info = window["-CURRENTPOSITION-"]
                    info.update(value = f"Current position is ({CurrentX},{CurrentY})")
                    window.Read(timeout = Timeout)
                    window.Refresh()
                Data = [DATAX,DATAY,DATAZ]
                storeData(Data,Dataname)
            StopCheck = values['-STOPBUTTON-']
            CANTVOLTAGE = ADC_COMM_CANT(7)
            while (not((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)) and PIDcontrol and StopCheck): #if not within the tolerance we want
                    Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI,setpoint) #feed into PID system
                    VoltageZ = VoltageZ + Change
                    DAC_COMM(VoltageZ, 2)
                    CANTVOLTAGE = ADC_COMM_CANT(7)
                    event,values = window3.read(timeout = 10)
                    StopCheck = values['-STOPBUTTON-']
        window3.close()
    while (not((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)) and PIDcontrol): #if not within the tolerance we want
        Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI,setpoint) #feed into PID system
        VoltageZ = VoltageZ + Change
        DAC_COMM(VoltageZ, 2)
        CANTVOLTAGE = ADC_COMM_CANT(7)
        if ((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol)):
            break
        event,values = window.read(timeout = 10)
window.close()
'''
    cgraph.MoveFigure(idcurrent,XINC,0)
    cgraph.update()
    info = window["-CURRENTPOSITIONCOARSE-"]
    info.update(value = f"Current position is ({CoarseX},{CoarseY})")
    window.Read(timeout = Timeout)
    window.Refresh()
'''
