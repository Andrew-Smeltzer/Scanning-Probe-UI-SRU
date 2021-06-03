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

def senddatatinyg(data):
    if port != None:
        port.write((data + "\n").encode('utf-8'))
        time.sleep(0.2)
def getdatatinyg():
    response = port.readline()
    time.sleep(0.2)
    return response
def DAC_COMM(val,Port):
#Controls the voltage of the output pins of the DAC
    try:
        bus = 0
        device = 0

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
        if Port == 0:
            commandwrite = 0b00000000   #Write command
            commandupdate = 0b00010000  #Update command
        if Port == 1:
            commandwrite = 0b00000001
            commandupdate = 0b00010001
        if Port == 2:
            commandwrite = 0b00000010
            commandupdate = 0b00010010
        if Port == 3:
            commandwrite = 0b00000011
            commandupdate = 0b00010011
        if Port == 4:
            commandwrite = 0b00000100
            commandupdate = 0b00010100
        if Port == 5:
            commandwrite = 0b00000101
            commandupdate = 0b00010101
        if Port == 6:
            commandwrite = 0b00000110
            commandupdate = 0b00010110
        if Port == 7:
            commandwrite = 0b00000111
            commandupdate = 0b00010111
        if Port == 8:
            commandwrite = 0b00001000
            commandupdate = 0b00011000
        if Port == 9:
            commandwrite = 0b00001001
            commandupdate = 0b00011001
        if Port == 10:
            commandwrite = 0b00001010
            commandupdate = 0b00011010
        if Port == 11:
            commandwrite = 0b00001011
            commandupdate = 0b00011011
        if Port == 12:
            commandwrite = 0b00001100
            commandupdate = 0b00011100
        if Port == 13:
            commandwrite = 0b00001101
            commandupdate = 0b00011101
        if Port == 14:
            commandwrite = 0b00001110
            commandupdate = 0b00011110
        if Port == 15:
            commandwrite = 0b00001111
            commandupdate = 0b00011111
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
    except KeyboardInterrupt:
        print("Press Ctrl-C to terminate loop")
def ADC_COMM_CANT(Channel): #Commuication with 0V - 5V inputs from ADC returns input Voltage
    try:
        bus = 0
        device = 1

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
        #Recieving 10 values from ADC and finding the mean to help mitigate error
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
    except KeyboardInterrupt:
        print("Press Ctrl-C to terminate loop")
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
#Function that turns the int that we sent to the DAC into the corresponding distance from center the piezotube moves
def CalculateDistances(VoltBitX,VoltBitY,VoltBitZ):
    try:
        base0 = 32767
        nmperBitt = 0.4917
        DistX = 0
        DistY = 0
        DistZ = 0
        DistX = (VoltBitX - base0) * nmperBitt
        DistY = (VoltBitY - base0) * nmperBitt
        DistZ = (VoltBitZ - base0) * nmperBitt
        return DistX,DistY,DistZ
    except KeyboardInterrupt:
        print("Press Ctrl-C to terminate loop")
#DAC port 14 => +X, port 15 => -X, port 0 => +Y, port 1 => -Y, port 2 => Z
def GetNegativeVoltage(PosVoltage):
    mask = 0b1111111111111111
    return (PosVoltage^mask)
def storeData(dateuh,dataname):
    np.savetxt("{}.dat".format(dataname),dateuh)
def PID(Reading, DiffOld,PastI):
    try:
        time.sleep(1)
        setpoint = 1.5
        delta_t = 1 #TODO figure out what delta_t is supposed to be
        Pconst = 1 #constants that need to be developed
        Iconst = 1
        Dconst = 1
        Diff = setpoint - Reading
        print(Diff)
        P = Pconst * Diff #proportional component
        print("P")
        print (P)
        I = PastI + Iconst*Diff*delta_t #Integral component
        print("I")
        print(I)
        D = (Dconst * (Diff - DiffOld))/(delta_t) #Differential component
        print("D")
        print(D)

        Output = P + I + D
        #print(Output)
        #print(Diff)
        return Output,Diff,I
    except KeyboardInterrupt:
        print("Press Ctrl-C to terminate loop")

retractedZ = 2
extentedZ = 32760
sg.theme('Dark Blue 3')

#Defining what we can do with graph, I.E events, saving, communication
#These are the buttons on the right of the graph
guy =[[sg.B('Scan',key = '-SCAN-',enable_events = True)],\
    [sg.Button('Exit')],\
    #[sg.Text('Current Piezo Position',key = '-CURRENTLABEL-')],\
    #[sg.Text('Current Coarse Position',key = '-CURRENTLABELCOARSE-')],\
    [sg.Text(key='info',size=(60,1))],\
    [sg.Text(key = '-CURRENTPOSITION-',size = (60,1))],\
    #[sg.B('Save Graph',key = '-SAVE-',enable_events = True)],\
    [sg.B('3D Graph',key = '-3DGRAPH-',enable_events = True),],]
Coarsepositioning=[[sg.B('+X Coarse',key = '-XUPONE-',enable_events = True)],\
    [sg.B('-X Coarse',key = '-XDOWNONE-',enable_events = True)],\
    [sg.B('+Y Coarse',key = '-YUPONE-',enable_events = True)],\
    [sg.B('-Y Coarse',key = '-YDOWNONE-',enable_events = True)],]\
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

#Making the steps of each of the x y and z coarse positioning motors
#EXP check to see how far each incriment moves when the room temp model is done
zincriment = 0.005
xincriment = 0.01
yincriment = 0.01

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
    event, values = window.read()
    if event == None or event == 'Exit':
        break
    #draw_axis()
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
                filename = values['-IN-']
                fulldata = np.loadtxt(filename)
                datax = fulldata[0]
                #print (datax)
                datay = fulldata[1]
                #print (datay)
                dataz = fulldata[2]
                #print (dataz)
                GraphData(datax,datay,dataz)
        window5.close()

    #Below is events in the x y coarse positioning system
    elif event == '-XUPONE-':
        #tinyg gets movement as g0x0.01
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

        while True:
            event,values = window3.read()
            if event == None or event == 'Exit':
                break
        #Retrieving information user inputs for incriments
            if event == 'Ok':
                Timeout = (values['-TIMEOUT-'])
                Yrangenm = (values['-SCANRANGEY-'])
                Xrangenm = (values['-SCANRANGEX-'])
                Dataname = values['-DATANAME-']
                StopCheck = values['-STOPBUTTON-']
                #input checking
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
                        event,values = window8.read(timeout = 1000)
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
                        event,values = window9.read(timeout = 1000)
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
                        event,values = window10.read(timeout = 1000)
                        if event == 'Exit' or event == 'Ok':
                            break
                    window10.close()
                    break
                else:
                    Yrangenm = int(Yrangenm)
                if (Yrangenm > 38001 or Yrangenm < 9):
                    layout6 = [[sg.T('Yrange is 10 to 38000 Nano Meters')],\
                    [sg.Button('Ok'),sg.Button('Exit')]]
                    window6 = sg.Window('Whoops')
                    window6.layout(layout6)
                    window6.finalize()
                    while True:
                        event,values = window6.read(timeout = 1000)
                        if event == 'Exit' or event == 'Ok':
                            break
                    window6.close()
                    break
                if (Xrangenm > 38001 or Xrangenm < 9):
                    layout7 = [[sg.T('Xrange is 10 to 38000 Nano Meters')],\
                    [sg.Button('Ok'),sg.Button('Exit')]]
                    window7 = sg.Window('Whoops')
                    window7.layout(layout7)
                    window7.finalize()
                    while True:
                        event,values = window7.read(timeout = 1000)
                        if event == 'Exit' or event == 'Ok':
                            break
                    window7.close()
                    break
                #CoarsePositioningScanZ(Xrangenm,Yrangenm,Dataname)
                #================================================================================================
                #trying to put this stop button in
                zstep = 0.005 #EXP get the proper distance when stepping d own in the z direction
                DAC_COMM(32670,2) #setting piezotube to normal
                O = 0;
                V = ADC_COMM_CANT(7)
                if (math.isnan(V)):
                    V = 0
                while (V <= 2 and StopCheck): #EXP need to check what noise value we are getting and make sure that it stops at an actual reading not noise
                    senddatatinyg("g0z{}".format(zstep))
                    time.sleep(2)
                    V = ADC_COMM_CANT(7)
                    if (math.isnan(V)):
                        V = 0
                    O = O + 1
                    event,values = window3.read(timeout = 10)
                    StopCheck = values['-STOPBUTTON-']
            #starts with setting up graph stuff this is not important to the actual scan just gives a visual of it
            #will mark where the actual scanning stuff starts
                XINC = 1
                YINC = 1
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
            #########################################################################################################################
            #Starting of the actual scanning code
            #TODO look into getting the piezotube to start in the bottom left of its range to maximize scanning range
                VoltageX = 32767 #reading 0 Volts on DAC, this number in binary will be set to DAC to output a corresponding voltage
                VoltageY = 32767
            #We set the Z voltage very low because at the end of the coarse positioning approach the piezotube will be at this same value
                VoltageZ = 32670
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
                #The target height above the sample we want the tip to be
                setpoint = 1.0
                Tol = 0.1   #TODO figure out the tolerance we want and the setpoint

                Difference = 0 #setting up the variables for the PID
                Change = 0
                PastI = 0
                CANTVOLTAGE = ADC_COMM_CANT(7)
                if(math.isnan(CANTVOLTAGE)):
                    CANTVOLTAGE = 0
                if (not((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol))): #if not within the tolerance we want
                    while StopCheck: #Piezomin < VoltageZ < Piezomax: #loop unless the Voltage in the Z gets out of range
                        Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI) #feed into PID system
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
                                Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI) #feed into PID system
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

                    while XrangeBack > 50 and StopCheck:
                        CurrentX = CurrentX - XINC
                        VoltageX = VoltageX - XSTEP
                        CANTVOLTAGE = ADC_COMM_CANT(7)
                        print("cant Reading")
                        print(CANTVOLTAGE)
                        if (not((setpoint + Tol) > CANTVOLTAGE > (setpoint - Tol))): #if not within the tolerance we want
                            while StopCheck: #Piezomin < VoltageZ < Piezomax: #loop unless the Voltage in the Z gets out of range
                                Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI) #feed into PID system
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
                                Change,Difference,PastI = PID(CANTVOLTAGE,Difference,PastI) #feed into PID system
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
                zmovement = O * -zstep
                senddatatinyg("g0z{}".format(zmovement))
        window3.close()
window.close()
