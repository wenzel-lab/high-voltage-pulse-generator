# -*- coding: utf-8 -*-
from PyQt5 import QtCore,QtGui,QtWidgets
from PyQt5.QtWidgets import  QMainWindow, QApplication, QFileDialog

from OvalSimGui import Ui_MainWindow

from time import sleep
import paho.mqtt.client as mqtt
import sys, getopt
import os
import time
import random
from datetime import timezone
from datetime import datetime,timedelta
import json
import threading



#MQTT ports
COM_PORT    = '/dev/ttyUSB0'
DATA_PATH   = "data/"

mqtt_host 	= "mqtt.q4t.cl"
mqtt_usser	= "tds"
mqtt_password = "m0squ1t0tds"
mqtt_port = 1884

mqtt_topic_send = "id.up"
mqtt_topic_recv = "id.down"

VERSION  = "OVALrp.v1.0"

debug = True

Oval_register_db = [	{"name_id":"Oval01", "mac":"94:E6:86:2E:04:0C"},
						{"name_id":"Oval02", "mac":"94:E6:86:2E:03:E0"},
						{"name_id":"Oval03", "mac":"94:E6:86:2E:03:F0"},
						{"name_id":"Oval04", "mac":"94:E6:86:2E:04:08"},
						{"name_id":"Oval13", "mac":"70:B8:F6:62:5E:A0"},
						{"name_id":"Oval14", "mac":"70:B8:F6:62:99:18"},
						{"name_id":"Oval15", "mac":"70:B8:F6:62:8F:78"},
						{"name_id":"Oval16", "mac":"70:B8:F6:62:C5:94"},
						{"name_id":"Oval17", "mac":"70:B8:F6:62:70:C0"},
						{"name_id":"Oval18", "mac":"70:B8:F6:62:8F:1C"},
						{"name_id":"Oval19", "mac":"70:B8:F6:62:3F:28"},
						{"name_id":"Oval20", "mac":"70:B8:F6:62:7F:A0"},
						{"name_id":"Oval21", "mac":"70:B8:F6:63:44:0C"},
						{"name_id":"Oval22", "mac":"70:B8:F6:62:C9:04"},
						{"name_id":"Oval23", "mac":"70:B8:F6:63:A7:8C"},
						{"name_id":"Oval24", "mac":"70:B8:F6:62:4D:E4"},
						{"name_id":"Oval25", "mac":"70:B8:F6:63:33:C8"}
]


# Función de conexión
def on_connect(client, userdata, flags, rc):
	if rc == 0:
		print("Conexión exitosa al broker MQTT")
	else:
		print("Error al conectar, código de error:", rc)


def on_message(client, userdata, msg):
	try:
		payload = json.loads(msg.payload.decode("utf-8"))  # Decodificar JSON
		#if "ID" in payload:
		#    print(f"Mensaje recibido con ID: {payload}")
		print(f"Mensaje recibido con ID: {payload}")
	except json.JSONDecodeError:
		print("Error: Mensaje recibido no es un JSON válido")


class oval:
	def __init__(self, _broker, _port, _topic_up = mqtt_topic_send, _topic_down = mqtt_topic_recv, _client_id ="OVAL00" ):

		self.id = "OVAL00"
		self.mac = "43:8F:A6:54:4C:24"
		self.time = 0
		self.type = "datos"
		self.lux = 0
		self.p = 0.0
		self.t = 0.0
		self.h = 0.0
		self.p01 = 0.0
		self.p10 = 0.0
		self.p25 = 0.0
		self.co2 = 0.0
		self.tvoc = 0.0
		self.noise_max = 0
		self.noise_mean = 0
		self.lat = 0.0
		self.lon = 0.0
		self.status = 0
		self.err = "OK"

		self.running = False
		self.automatic_time = True
		self.elapsed_time = 0

		self.file_name      = ""
		self.path           = DATA_PATH
		self.data_path 		= "./data/"
		self.data_file      = None
		self.data_writer    = None
		self.label_time     = ""

		self.client_id 		= _client_id
		self.topic_up 		= _topic_up
		self.topic_down 	= _topic_down
		self.broker 		= _broker
		self.port 			= _port

		self.fpub = 0
		self.ssid = ""
		self.password = ""
		self.ver = ""
		self.resp = ""

		self.mqtt_listener =mqtt.Client()
		self.mqtt_sender = mqtt.Client()

		self.mqtt_listener.on_message = self.on_message


		self.sample_time = 60

		"""-----------------------
   			INITIAL STATE
		----------------------"""


		"""--------------------------
    		Functions
		-----------------------------"""
	def reset(self):
		print("restarting device")

	def setWifi(self, _arg):
		print("setting Wifi")

	def setID(self, _id):
		self.id =_id
		self.updateJson()

	def setSampleTime(self, _arg):
		print("new sample time setted")

	def setPublishTime(self, _arg):
		print("new publish time setted")

	def updateTime(self):
		print("update time!")

	def update(self):
		print("updating code")


	def setMAC(self):
		self.mac = self.ui.lineEdit_6.text()
		self.updateJson()


	def proccessCMD(self,payload ):
		try:
			cmd = payload['cmd']
			arg = payload['arg']

			if cmd == 'rst':
				self.reset()
			elif cmd == 'setID':
				self.setID(arg["new_id"])
			elif cmd == 'setWifi':
				self.setWifi(arg)
			elif cmd == 'updateTime':
				self.updateTime()
			elif cmd == 'update':
				self.update()
		except Error as e:
			print(e)

	"""
		Function triggered when there is a new msj on the topic.
	"""
	def on_message(self, client, userdata, msg):
		try:
			payload = json.loads(msg.payload.decode("utf-8"))  # Decodificar JSON
			print(f"Mensaje recibido con ID [{self.client_id}]: {payload}")
			print(payload['mac'])
			if payload['mac'] == self.mac:
				print("msg for me")
				self.proccessCMD(payload)
		except json.JSONDecodeError:
			print("Error: Mensaje recibido no es un JSON válido")



	def sendData(self):
		_msg = json.dumps(self.json_msg)
		try:
			client.publish(mqtt_topic, _msg)
			print(f"Mensaje enviado: {_msg}")
		except Exception as e:
			print(e)


	def randomizeMAC(self):
		# Genera seis valores hexadecimales de 2 dígitos cada uno
		_mac = [random.randint(0x00, 0xFF) for _ in range(6)]
    	# Formatea en estilo de dirección MAC, separada por ":"
		self.mac =  ":".join(f"{octeto:02X}" for octeto in _mac)
		self.updateJson()



	def randomizeAll(self):
		self.lux = random.randint(0,10000)/100.0
		self.p = random.randint(0,10000)/100.0
		self.t = float(random.randint(0,4000)/100.0)
		self.h = random.randint(0,10000)/100.0
		self.p01 = random.randint(0,5000)/100.0
		self.p10 = random.randint(0,5000)/100.0
		self.p25 = random.randint(0,5000)/100.0
		self.co2 = random.randint(40000,80000)/100.0
		self.tvoc = random.randint(0,10000)/100.0
		self.noise_max = random.randint(6000,8000)/100.0
		self.noise_mean = random.randint(6000,8000)/100.0
		self.lat = -33.0 - random.randint(0,100)/100.0
		self.lon = - 77 -   random.randint(0,100)/100.0
		self.updateJson()

	def readSensors(self):
		self.randomizeAll()
		#read

	def setAll():
		pass

	def updateJson(self):
		if self.automatic_time :
			dt = datetime.now(timezone.utc)
			utc_time = dt.replace(tzinfo=timezone.utc)
			self.time = int(utc_time.timestamp())
		self.json_msg = {"id":self.id,
			"mac":self.mac,
			"time":self.time,
			"type":self.type,
			"lux":self.lux,
			"p":self.p,
			"t":self.t,
			"h":self.h,
			"p01":self.p01,
			"p10":self.p10,
			"p25":self.p25,
			"co2":self.co2,
			"tvoc":self.tvoc,
			"noise_max":self.noise_max,
			"noise_mean":self.noise_mean,
			"lat":self.lat,
			"lon":self.lon,
			"status":self.status,
			"err":self.err}
		#print(json.dumps(self.jsonMSG, indent = 4))

	def publish_random_data(self):
		self.mqtt_sender.username_pw_set(mqtt_usser, mqtt_password)
		self.mqtt_sender.connect(mqtt_host, mqtt_port, 60)
		while True:
			self.readSensors()
			self.updateJson()
			self.mqtt_sender.publish(mqtt_topic_send, json.dumps(self.json_msg))
			print(f"Publicado en {mqtt_topic_send}: {self.json_msg}")
			time.sleep(self.sample_time)

	def listen_mqtt(self):
		# Configurar usuario y contraseña
		self.mqtt_listener.username_pw_set(mqtt_usser, mqtt_password)
		self.mqtt_listener.connect(mqtt_host, mqtt_port, 60)
		self.mqtt_listener.subscribe(mqtt_topic_recv+"."+self.mac)
		self.mqtt_listener.loop_forever()

	def run(self):
		self.running = True
		self.thread_listen = threading.Thread(target=self.listen_mqtt, daemon=True)
		self.thread_publish = threading.Thread(target=self.publish_random_data, daemon=True)

		self.thread_listen.start()
		self.thread_publish.start()


	def stop(self):
		try:
			self.running = False
			self.mqtt_sender.disconnect()
			self.mqtt_listener.disconnect()
			print("stopping Oval")
		except Exception as e:
			print("Controller not connected. Closing")


if __name__ == "__main__":

	oval = oval(_broker=mqtt_host, _port=mqtt_port)
	#oval.randomizeMAC()
	print(" MAC generada :")
	print(oval.mac)

	# Crear e iniciar los hilos
	oval.run()

	# Mantener activo por 2 minutos
	time.sleep(120)
	oval.stop()
