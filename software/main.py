# -*- coding: utf-8 -*-
from PyQt5 import QtCore,QtGui,QtWidgets
from PyQt5.QtWidgets import  QMainWindow, QApplication, QFileDialog
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from gui import Ui_MainWindow

from time import sleep
import paho.mqtt.client as mqtt
import sys, getopt
import os
import serial
import serial.tools.list_ports
import time
import random
from datetime import timezone
from datetime import datetime,timedelta
import json
import threading

com_port_list = []
port_list   =  serial.tools.list_ports.comports()


COM_PORT    = '/dev/ttyUSB0'
DATA_PATH   = "data/"


#MQTT ports
mqtt_host 	= "35.223.234.244"
mqtt_usser	= "iowlabs"
mqtt_password = "!iow_woi!"
mqtt_port = 1883

hv_cltr_topic_send = "hvc/rx"
hv_cltr_topic_recive = "hvc/tx"
debug = True

list_id = ["HVPG-01","HVPG-01","HVPG-01"]


# Función de conexión
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Conexión exitosa al broker MQTT")
    else:
        print("Error al conectar, código de error:", rc)

class SerialReader(QThread):
	data_received = pyqtSignal(str)  # Señal para enviar datos a la GUI
	finished = pyqtSignal()

	def __init__(self, port, baudrate=9600,sampling_rate = 1000):
		super().__init__()
		self.port = port
		self.baudrate = baudrate
		self.sampling_rate = sampling_rate #datos por segundo
		self.running = True  # Para detener el hilo correctamente
		self.ser = None

	def run(self):
		try:
			self.ser = serial.Serial(self.port, self.baudrate, timeout= 1)
			while self.running:
				if self.ser.in_waiting > 0:
					rcvd_msg = self.ser.readline().decode('utf-8').rstrip()
					if rcvd_msg:
						try:
							print("msg from serial: ")
							print(rcvd_msg)
							self.data_received.emit(str(rcvd_msg))  # Enviar a la GUI
						except ValueError:
							print(f"Error al parsear: {rcvd_msg}")  # En caso de datos corruptos
		except serial.SerialException as e:
			print(f"Error con el puerto serial: {e}")
		self.finished.emit()

	def stop(self):
		self.running = False
		self.ser.close()
		self.quit()
		self.wait()

class MainWindow(QtWidgets.QMainWindow):
	def __init__(self):
		super(MainWindow,self).__init__()
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		self.setWindowTitle("HV pulse controller")

		self.file_name      = ""
		self.path           = DATA_PATH
		self.data_path 		= "./data/"
		self.data_file      = None
		self.data_writer    = None
		self.label_time     = ""
		self.connection_usb = True
		self.connected 		= False

		self.mqtt_listener 	= mqtt.Client()
		self.mqtt_sender 	= mqtt.Client()
		self.mqtt_listener.on_message = self.on_message

		self.target_id = list_id[0]

		self.parameters = [	{"channel":1, "en":False , "settings":{"freq":80,"amp":100,"duty":50,"pulse":100}},
							{"channel":2, "en":False , "settings":{"freq":80,"amp":100,"duty":50,"pulse":100}},
							{"channel":3, "en":False , "settings":{"freq":80,"amp":100,"duty":50,"pulse":100}},
							{"channel":4, "en":False , "settings":{"freq":80,"amp":100,"duty":50,"pulse":100}},
							{"channel":5, "en":False , "settings":{"freq":80,"amp":100,"duty":50,"pulse":100}},
							{"channel":6, "en":False , "settings":{"freq":80,"amp":100,"duty":50,"pulse":100}}]

		for port, desc, hwid in sorted(port_list):
			self.ui.comboBox.addItem("{}".format(port))

		for _id in list_id:
			self.ui.comboBox_2.addItem("{}".format(_id))

		self.freq_spinbox  	= [self.ui.spinBox_22,self.ui.spinBox_23,self.ui.spinBox_24,self.ui.spinBox_25,self.ui.spinBox_26,self.ui.spinBox_27]
		self.duty_spinbox 	= [self.ui.spinBox_2,self.ui.spinBox_3,self.ui.spinBox_4,self.ui.spinBox_5,self.ui.spinBox_6,self.ui.spinBox_7]
		self.pulse_spinbox 	= [self.ui.spinBox_15,self.ui.spinBox_16,self.ui.spinBox_17,self.ui.spinBox_18,self.ui.spinBox_19,self.ui.spinBox_20]
		self.amp_spinbox   	= [self.ui.spinBox,self.ui.spinBox_9,self.ui.spinBox_10,self.ui.spinBox_11,self.ui.spinBox_12,self.ui.spinBox_13]
		self.check_channel 	= [self.ui.checkBox,self.ui.checkBox_2,self.ui.checkBox_3,self.ui.checkBox_4,self.ui.checkBox_5,self.ui.checkBox_6]
		self.setButtons    	= [self.ui.pushButton_2,self.ui.pushButton_7,self.ui.pushButton_8,self.ui.pushButton_9,self.ui.pushButton_10,self.ui.pushButton_11]
		self.triggerButtons	= [self.ui.pushButton,self.ui.pushButton_13,self.ui.pushButton_14,self.ui.pushButton_15,self.ui.pushButton_16,self.ui.pushButton_17]

		"""-----------------------
   			INITIAL STATE
		----------------------"""
		self.ui.radioButton.setDisabled(False)
		self.ui.radioButton.setChecked(True)
		self.ui.radioButton_2.setDisabled(False)
		self.ui.pushButton_3.setDisabled(False)
		self.ui.pushButton_4.setDisabled(True)
		self.ui.pushButton_6.setDisabled(True)
		self.ui.pushButton_5.setDisabled(True)
		self.ui.label_2.setDisabled(True)
		self.ui.comboBox_2.setDisabled(True)

		for i in range(6):
			self.freq_spinbox[i].setSingleStep(1)
			self.freq_spinbox[i].setValue(self.parameters[i]["settings"]["freq"])
			self.freq_spinbox[i].setMinimum(50)
			self.freq_spinbox[i].setMaximum(100)
			self.freq_spinbox[i].setDisabled(True)

			self.pulse_spinbox[i].setSingleStep(1)
			self.pulse_spinbox[i].setValue(self.parameters[i]["settings"]["pulse"])
			self.pulse_spinbox[i].setMinimum(1)
			self.pulse_spinbox[i].setMaximum(1000)
			self.pulse_spinbox[i].setDisabled(True)

			self.duty_spinbox[i].setSingleStep(10)
			self.duty_spinbox[i].setValue(self.parameters[i]["settings"]["duty"])
			self.duty_spinbox[i].setMinimum(1)
			self.duty_spinbox[i].setMaximum(100)
			self.duty_spinbox[i].setDisabled(True)

			self.amp_spinbox[i].setValue(self.parameters[i]["settings"]["amp"])
			self.amp_spinbox[i].setMinimum(0)
			self.amp_spinbox[i].setMaximum(255)
			self.amp_spinbox[i].setDisabled(True)

			self.check_channel[i].setDisabled(True)


		self.disableChannels()

		self.ui.plainTextEdit.setReadOnly(True)

		"""--------------------------
    		General behaivour
		-----------------------------"""
		self.ui.radioButton.toggled.connect(self.connectionMode)	#RUN TEST
		self.ui.pushButton_3.clicked.connect(self.connectUSB)
		self.ui.pushButton_4.clicked.connect(self.disconnectUSB)
		self.ui.pushButton_6.clicked.connect(self.connectMQTT)
		self.ui.pushButton_5.clicked.connect(self.disconnectMQTT)

		self.check_channel[0].stateChanged.connect(lambda:self.enChannel(0))
		self.check_channel[1].stateChanged.connect(lambda:self.enChannel(1))
		self.check_channel[2].stateChanged.connect(lambda:self.enChannel(2))
		self.check_channel[3].stateChanged.connect(lambda:self.enChannel(3))
		self.check_channel[4].stateChanged.connect(lambda:self.enChannel(4))
		self.check_channel[5].stateChanged.connect(lambda:self.enChannel(5))

		self.ui.checkBox_7.stateChanged.connect(self.enAllChannels)

		self.setButtons[0].clicked.connect(lambda:self.sendParameters(0))
		self.setButtons[1].clicked.connect(lambda:self.sendParameters(1))
		self.setButtons[2].clicked.connect(lambda:self.sendParameters(2))
		self.setButtons[3].clicked.connect(lambda:self.sendParameters(3))
		self.setButtons[4].clicked.connect(lambda:self.sendParameters(4))
		self.setButtons[5].clicked.connect(lambda:self.sendParameters(5))

		self.triggerButtons[0].clicked.connect(lambda:self.sendTrigger(0))
		self.triggerButtons[1].clicked.connect(lambda:self.sendTrigger(1))
		self.triggerButtons[2].clicked.connect(lambda:self.sendTrigger(2))
		self.triggerButtons[3].clicked.connect(lambda:self.sendTrigger(3))
		self.triggerButtons[4].clicked.connect(lambda:self.sendTrigger(4))
		self.triggerButtons[5].clicked.connect(lambda:self.sendTrigger(5))


	def enableChannels(self):
		print("eneabeling channels")
		for i in range(6):
			self.check_channel[i].setDisabled(False)
			if self.parameters[i]["en"]:
				self.enChannel(i)
			#self.pulse_spinbox[i].setDisabled(False)
			#self.freq_spinbox[i].setDisabled(False)
			#self.amp_spinbox[i].setDisabled(False)
			#self.setButtons[i].setDisabled(False)
			#self.triggerButtons[i].setDisabled(False)

		self.ui.label_3.setDisabled(False)
		self.ui.label_4.setDisabled(False)
		self.ui.label_5.setDisabled(False)
		self.ui.label_6.setDisabled(False)
		self.ui.label_7.setDisabled(False)
		self.ui.label_8.setDisabled(False)
		self.ui.checkBox_7.setDisabled(False)
		if self.ui.checkBox_7.isChecked():
			self.ui.spinBox_28.setDisabled(False)
			self.ui.spinBox_8.setDisabled(False)
			self.ui.spinBox_14.setDisabled(False)
			self.ui.spinBox_21.setDisabled(False)
			self.ui.pushButton_12.setDisabled(False)
			self.ui.pushButton_18.setDisabled(False)

	def disableChannels(self):
		print("diseabeling channels")
		for i in range(6):
			self.check_channel[i].setDisabled(True)
			self.pulse_spinbox[i].setDisabled(True)
			self.duty_spinbox[i].setDisabled(True)
			self.freq_spinbox[i].setDisabled(True)
			self.amp_spinbox[i].setDisabled(True)
			self.setButtons[i].setDisabled(True)
			self.triggerButtons[i].setDisabled(True)

		self.ui.label_3.setDisabled(True)
		self.ui.label_4.setDisabled(True)
		self.ui.label_5.setDisabled(True)
		self.ui.label_6.setDisabled(True)
		self.ui.label_7.setDisabled(True)
		self.ui.label_8.setDisabled(True)
		self.ui.checkBox_7.setDisabled(True)
		self.ui.spinBox_28.setDisabled(True)
		self.ui.spinBox_8.setDisabled(True)
		self.ui.spinBox_14.setDisabled(True)
		self.ui.spinBox_21.setDisabled(True)
		self.ui.pushButton_12.setDisabled(True)
		self.ui.pushButton_18.setDisabled(True)

	def connectionMode(self):
		print("changing mode of connection")
		self.connection_usb = not self.connection_usb
		if self.connection_usb:
			#USB connection
			self.disconnectMQTT()
			self.ui.pushButton_3.setDisabled(False)
			self.ui.pushButton_4.setDisabled(True)
			self.ui.label.setDisabled(False)
			self.ui.comboBox.setDisabled(False)

			#self.ui.radioButton_2.setChecked(False)
			self.ui.pushButton_6.setDisabled(True)
			self.ui.pushButton_5.setDisabled(True)
			self.ui.label_2.setDisabled(True)
			self.ui.comboBox_2.setDisabled(True)

			self.ui.plainTextEdit.appendPlainText("Connection mode :  USB")
		else:
			#MQTT connection
			self.disconnectUSB()
			self.ui.pushButton_3.setDisabled(True)
			self.ui.pushButton_4.setDisabled(True)
			self.ui.label.setDisabled(True)
			self.ui.comboBox.setDisabled(True)

			#self.ui.radioButton_2.setChecked(True)
			self.ui.pushButton_6.setDisabled(False)
			self.ui.pushButton_5.setDisabled(True)
			self.ui.label_2.setDisabled(False)
			self.ui.comboBox_2.setDisabled(False)

			self.ui.plainTextEdit.appendPlainText("Connection mode :  MQTT")

	def connectUSB(self):
		self.ui.pushButton_3.setDisabled(True)
		self.ui.pushButton_4.setDisabled(False)
		#add connection
		self.connected = True
		_port = self.ui.comboBox.currentText()
		self.serial_thread = SerialReader(_port)
		self.serial_thread.data_received.connect(self.listen_usb)
		self.serial_thread.finished.connect(self.disconnectUSB)
		self.serial_thread.start()

		print("try to enable channels:")
		self.enableChannels()
		self.ui.plainTextEdit.appendPlainText("Connected to USB")

	def disconnectUSB(self):
		self.ui.pushButton_3.setDisabled(False)
		self.ui.pushButton_4.setDisabled(True)
		#add disconnect to ubs
		self.connected = False
		try:
			self.serial_thread.running = False
			self.serial_thread.stop()

		except Exception as e:
			print(e)
		print("try to disable channels:")
		self.disableChannels()
		self.ui.plainTextEdit.appendPlainText("Disonnected from USB")

	def connectMQTT(self):
		self.ui.pushButton_6.setDisabled(True)
		self.ui.pushButton_5.setDisabled(False)
		#add connection
		self.connected = True
		#connect sender
		self.mqtt_sender.username_pw_set(mqtt_usser, mqtt_password)
		self.mqtt_sender.connect(mqtt_host, mqtt_port, 60)

		self.thread_listen = threading.Thread(target=self.listen_mqtt, daemon=True)
		self.thread_listen.start()

		print("try to enable channels:")
		self.enableChannels()
		self.ui.plainTextEdit.appendPlainText("Connected to MQTT")

	def disconnectMQTT(self):
		self.ui.pushButton_6.setDisabled(False)
		self.ui.pushButton_5.setDisabled(True)
		#add disconnect from MQTT
		self.connected = False
		try:
			self.mqtt_sender.disconnect()
			self.mqtt_listener.disconnect()
			print("stopping MQTT")
		except Exception as e:
			print("MQTT not connected. Closing")

		print("try to disable channels:")
		self.disableChannels()
		self.ui.plainTextEdit.appendPlainText("Disconnected from MQTT")

	def listen_mqtt(self):
		# Configurar usuario y contraseña
		self.mqtt_listener.username_pw_set(mqtt_usser, mqtt_password)
		self.mqtt_listener.connect(mqtt_host, mqtt_port, 60)
		self.mqtt_listener.subscribe(hv_cltr_topic_recive)
		while self.connected:
			self.mqtt_listener.loop_forever()

	def listen_usb(self,rcvd_msg):
		self.ui.plainTextEdit.appendPlainText(rcvd_msg)

	def enChannel(self, _ch):
		if self.check_channel[_ch].isChecked():
			self.parameters[_ch]["en"] = True
			self.ui.plainTextEdit.appendPlainText(f"channel {_ch +1} enabled")
			self.pulse_spinbox[_ch].setDisabled(False)
			self.duty_spinbox[_ch].setDisabled(False)
			self.freq_spinbox[_ch].setDisabled(False)
			self.amp_spinbox[_ch].setDisabled(False)
			self.setButtons[_ch].setDisabled(False)
			self.triggerButtons[_ch].setDisabled(False)


		else:
			self.parameters[_ch]["en"] = False
			self.ui.plainTextEdit.appendPlainText(f"channel {_ch +1} disabled")
			self.pulse_spinbox[_ch].setDisabled(True)
			self.duty_spinbox[_ch].setDisabled(True)
			self.freq_spinbox[_ch].setDisabled(True)
			self.amp_spinbox[_ch].setDisabled(True)
			self.setButtons[_ch].setDisabled(True)
			self.triggerButtons[_ch].setDisabled(True)

	def enAllChannels(self):
		if self.ui.checkBox_7.isChecked():
			for i in range(6):
				self.parameters[i]["en"] = True
				self.check_channel[i].setChecked(True)
			self.ui.spinBox_28.setDisabled(False)
			self.ui.spinBox_8.setDisabled(False)
			self.ui.spinBox_14.setDisabled(False)
			self.ui.spinBox_21.setDisabled(False)
			self.ui.pushButton_12.setDisabled(False)
			self.ui.pushButton_18.setDisabled(False)
		else:
			for i in range(6):
				self.parameters[i]["en"] = False
				self.check_channel[i].setChecked(False)
			self.ui.spinBox_28.setDisabled(True)
			self.ui.spinBox_8.setDisabled(True)
			self.ui.spinBox_14.setDisabled(True)
			self.ui.spinBox_21.setDisabled(True)
			self.ui.pushButton_12.setDisabled(True)
			self.ui.pushButton_18.setDisabled(True)

	def sendParameters(self,_ch):
		self.parameters[_ch]["settings"]["freq"]	= self.freq_spinbox[_ch].value()
		self.parameters[_ch]["settings"]["duty"] 	= self.duty_spinbox[_ch].value()
		self.parameters[_ch]["settings"]["pulse"] 	= self.pulse_spinbox[_ch].value()
		self.parameters[_ch]["settings"]["amp"] 	= self.amp_spinbox[_ch].value()

		cmd = "set"
		id 	= self.parameters[_ch]["channel"]
		arg = self.parameters[_ch]["settings"]
		print(f"sending cmd {cmd} to channel {_ch+1}: ")
		self.sendCMD(id, cmd,arg )

	def sendTrigger(self,_ch):
		cmd = "trigger"
		ch  = self.parameters[_ch]["channel"]
		arg = {}
		print(f"sending cmd {cmd} to channel {_ch+1}: ")
		self.sendCMD(ch,cmd,arg)

	def sendCMD(self, _ch,  _cmd, _arg):
		_id = self.ui.comboBox_2.currentText()
		data = {"ch":_ch,"id":_id,"cmd":_cmd,"arg":_arg}
		json_string = json.dumps(data)
		print(json_string)
		if self.connected:
			if self.connection_usb:
				try:
					self.serial_thread.ser.write(json_string.encode())
				except Exception as e:
					print(e)
			else:
				try:
					self.mqtt_sender.publish(hv_cltr_topic_send, json_string)
				except Exception as e:
					print(e)
		self.ui.plainTextEdit.appendPlainText(f"sending cmd : {_cmd} to ch : {_ch}  \n{json_string}")

	"""
		Function triggered when there is a new msj on the topic.
	"""
	def on_message(self, client, userdata, msg):
		try:
			payload = json.loads(msg.payload.decode("utf-8"))  # Decodificar JSON
			print(payload)

		except json.JSONDecodeError:
			print("Error: Mensaje recibido no es un JSON válido")



	def Close(self):
		try:
			if self.connection_usb:
				self.disconnectUSB()
			else:
				self.disconnectMQTT()
		except Exception as e:
			print("Controller not connected. Closing")


if __name__ == "__main__":



	# Iniciar el loop del cliente
	#client.loop_start()

	app = QtWidgets.QApplication([])
	main = MainWindow()
	main.show()
	#main.showFullScreen()

	ret = app.exec_()
	main.Close()
	print("stoped")
	sys.exit(ret)
