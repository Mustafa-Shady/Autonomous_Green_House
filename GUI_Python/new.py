from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.uic import loadUi
import Resource_file_images_rc
import sys
import serial
import time

G_H_Value = 10
G_T_Value = 20
G_Z_Value = 10
GF_Cooling = 1
GF_Heating = 1
GF_WaterTank = 1
GF_Humidifier = 1
GF_DeHumidifier = 1
GF_Irrigation = 1

#Initialization
app = QtWidgets.QApplication(sys.argv)
ser = serial.Serial('/dev/ttyUSB0',9600,timeout=5)
ser.flush()

class UI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        loadUi("GP_GUI.ui", self)
        self.update_StyleSheet()
        self.showFullScreen()
        
        # Start a timer to periodically read the value from UART
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.read_uart)
        self.timer.start(1000)  # Read every 1000 milliseconds = 1 Second
        
    def update_StyleSheet(self):
        global G_H_Value
        global G_T_Value
        global G_Z_Value
        global GF_Cooling 
        global GF_Heating 
        global GF_WaterTank 
        global GF_Humidifier 
        global GF_DeHumidifier 
        global GF_Irrigation 
        
        #Strings that have the QFrame StyleSheet
        self.stylesheet_HRing = """
        QFrame
        {
        border-radius: 85px;
        background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:{H_STOP_1} rgba(0, 0, 0, 0), stop:{H_STOP_2} rgba(27, 130, 0, 255));	
        }
        """
        self.stylesheet_TRing = """
        QFrame
        {
        border-radius: 85px;
        background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:{T_STOP_1} rgba(0, 0, 0, 0), stop:{T_STOP_2} rgba(255, 94, 0, 255));
        }
        """ 
        self.stylesheet_ZRing = """
        QFrame
        {
        border-radius: 85px;
        background-color: qconicalgradient(cx:0.5, cy:0.5, angle:90, stop:{Z_STOP_1} rgba(0, 0, 0, 0), stop:{Z_STOP_2} rgba(13, 0, 255, 255));
        }
        """ 
        self.stylesheet_Activated="""
        QFrame{
        border-radius:15px;
        background-color: rgb(0, 140, 0);
        }
        """ 
        self.stylesheet_Not_Activated="""
        QFrame{
        border-radius:15px;
        background-color: rgb(170, 0, 0);
        }
        """        

        #Changing Values to (0~1)string Form
        H_Value_pr1 = (100 - G_H_Value) / 100.00
        H_Value_pr2 = str(H_Value_pr1 - 0.001)
        T_Value_pr1 = (100 - G_T_Value) / 100.00
        T_Value_pr2 = str(T_Value_pr1 - 0.001)
        Z_Value_pr1 = (100 - G_Z_Value) / 100.00
        Z_Value_pr2 = str(Z_Value_pr1 - 0.001)
        
        #Replace the Original QFrame StyleSheet stop Values with the new Values
        self.New_stylesheet_HRing = self.stylesheet_HRing.replace("{H_STOP_1}", H_Value_pr2).replace("{H_STOP_2}", str(H_Value_pr1))
        self.New_stylesheet_TRing = self.stylesheet_TRing.replace("{T_STOP_1}", T_Value_pr2).replace("{T_STOP_2}", str(T_Value_pr1))
        self.New_stylesheet_ZRing = self.stylesheet_ZRing.replace("{Z_STOP_1}", Z_Value_pr2).replace("{Z_STOP_2}", str(Z_Value_pr1))

        #Update the New StyleSheet
        self.H_Ring.setStyleSheet(self.New_stylesheet_HRing)
        self.T_Ring.setStyleSheet(self.New_stylesheet_TRing)
        self.Z_Ring.setStyleSheet(self.New_stylesheet_ZRing)
        
        #Updating Indications
        if GF_Cooling:
            self.Cooling_Indicator.setStyleSheet(self.stylesheet_Activated)
        else:
            self.Cooling_Indicator.setStyleSheet(self.stylesheet_Not_Activated)
        if GF_Heating:
            self.Heating_Indicator.setStyleSheet(self.stylesheet_Activated)
        else:
            self.Heating_Indicator.setStyleSheet(self.stylesheet_Not_Activated)
        if GF_WaterTank:
            self.TankPump_Indicator.setStyleSheet(self.stylesheet_Activated)
        else:
            self.TankPump_Indicator.setStyleSheet(self.stylesheet_Not_Activated)
        if GF_Humidifier:
            self.Humidifier_Indicator.setStyleSheet(self.stylesheet_Activated)
        else:
            self.Humidifier_Indicator.setStyleSheet(self.stylesheet_Not_Activated)
        if GF_DeHumidifier:
            self.DeHumidifier_Indicator.setStyleSheet(self.stylesheet_Activated)
        else:
            self.DeHumidifier_Indicator.setStyleSheet(self.stylesheet_Not_Activated)
        if GF_Irrigation:
            self.Irrigation_Inidicator.setStyleSheet(self.stylesheet_Activated)
        else:
            self.Irrigation_Inidicator.setStyleSheet(self.stylesheet_Not_Activated)            
        
        #Update the Labels with the New Values
        self.H_Labe_2.setText(f"{int(G_H_Value)}{'%'}")
        self.T_Labe_2.setText(f"{int(G_T_Value)}{'°'}")
        self.Z_Labe_2.setText(f"{int(G_Z_Value)}{'%'}")

    def read_uart(self):
        global G_H_Value
        global G_T_Value
        global G_Z_Value
        global GF_Cooling 
        global GF_Heating 
        global GF_WaterTank 
        global GF_Humidifier 
        global GF_DeHumidifier 
        global GF_Irrigation 
        
        if ser.in_waiting:
            # Read the value from UART and update Global Values
            G_H_Value = int(ser.readline().decode('utf-8').strip())
            GF_Humidifier = int(ser.readline().decode('utf-8').strip())
            GF_DeHumidifier = int(ser.readline().decode('utf-8').strip())
            G_T_Value = int(ser.readline().decode('utf-8').strip())
            GF_Heating = int(ser.readline().decode('utf-8').strip())
            GF_Cooling = int(ser.readline().decode('utf-8').strip())
            G_Z_Value = int(ser.readline().decode('utf-8').strip())
            GF_Irrigation = int(ser.readline().decode('utf-8').strip())
            GF_WaterTank = int(ser.readline().decode('utf-8').strip())
            self.update_StyleSheet()        
        
ui=UI()

app.exec_()