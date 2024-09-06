#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import tkinter as tk
import customtkinter as ctk
from tkinter import messagebox
import serial
import serial.tools.list_ports
import json
import time

class SerialMonitor(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Settings of Multi Channel High Voltage Pulse Generator Board")

        # Crear menú desplegable para seleccionar el puerto COM
        self.selected_port = tk.StringVar(self, value="")
        self.ports = self.list_ports()
        self.port_menu = ctk.CTkOptionMenu(self, variable=self.selected_port, values=self.ports)
        ctk.CTkLabel(self, text="Select COM Port:").grid(row=0, column=0, padx=10, pady=10)
        self.port_menu.grid(row=0, column=1, padx=10, pady=10)

        # Crear el menú desplegable para seleccionar el canal
        self.channel_var = tk.StringVar(self, value="CH1")
        self.channel_menu = ctk.CTkOptionMenu(self, variable=self.channel_var, values=[f"CH{i+1}" for i in range(6)] + ["ALL"])  # CH1 a CH6 primero, luego ALL
        ctk.CTkLabel(self, text="Select Channel:").grid(row=1, column=0, padx=10, pady=10)
        self.channel_menu.grid(row=1, column=1, padx=10, pady=10)

        # Etiqueta y campo de entrada para el valor del potenciómetro
        ctk.CTkLabel(self, text="Amplitud Value (step):").grid(row=0, column=2, padx=10, pady=10)
        self.potentiometer_entry = ctk.CTkEntry(self, width=120)
        self.potentiometer_entry.insert(0, "255")
        self.potentiometer_entry.grid(row=0, column=3, padx=10, pady=10)

        # Etiqueta y campo de entrada para la frecuencia
        ctk.CTkLabel(self, text="Frequency (KHz):").grid(row=1, column=2, padx=10, pady=10)
        self.frequency_entry = ctk.CTkEntry(self, width=120)
        self.frequency_entry.insert(0, "80")
        self.frequency_entry.grid(row=1, column=3, padx=10, pady=10)

        # Etiqueta y campo de entrada para el ancho de pulso
        ctk.CTkLabel(self, text="Pulse Width (us):").grid(row=2, column=2, padx=10, pady=10)
        self.pulse_width_entry = ctk.CTkEntry(self, width=120)
        self.pulse_width_entry.insert(0, "200")
        self.pulse_width_entry.grid(row=2, column=3, padx=10, pady=10)

        # Botón para enviar datos por puerto serial
        ctk.CTkLabel(self, text="Channel settings:").grid(row=2, column=0, padx=10, pady=10)
        serial_button = ctk.CTkButton(self, text="Send to ESP32", command=self.send_by_serial, fg_color="#4CAF50")
        serial_button.grid(row=2, column=1, padx=10, pady=10)

        # Crear un área de texto para el monitor serial
        self.monitor_area = ctk.CTkTextbox(self, width=600, height=200, wrap=tk.WORD)
        self.monitor_area.grid(row=6, column=0, columnspan=6, padx=10, pady=10)
        
        # Botón para limpiar el monitor serial dentro del contenedor
        button_frame = ctk.CTkFrame(self)
        button_frame.grid(row=7, column=0, columnspan=6, padx=10, pady=10, sticky='n')
        clear_button = ctk.CTkButton(button_frame, text="Clear Monitor", command=self.clear_monitor, width=120)
        clear_button.pack()
        
        # Inicializar puerto serial
        self.serial_port = None
        
        # Iniciar lectura del puerto serial
        self.after(100, self.read_serial)
        
    def list_ports(self):
        """Lista los puertos seriales disponibles."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def update_monitor(self, message):
        """Actualiza el monitor serial con un nuevo mensaje."""
        self.monitor_area.insert(tk.END, message + "\n")
        self.monitor_area.yview(tk.END)

    def clear_monitor(self):
        """Limpia el monitor serial."""
        self.monitor_area.delete("1.0", tk.END)
    
    def read_serial(self):
        """Lee datos del puerto serial y actualiza el monitor serial."""
        if self.serial_port and self.serial_port.in_waiting:
            try:
                line = self.serial_port.readline()
                #print(line) #agregado
                data = line.decode('utf-8').strip() # latino-1
                if data:
                    self.update_monitor(f"ESP32 response: {data}")
            except UnicodeDecodeError as e:
                self.update_monitor(f"Error decoding data: {e}")
        self.after(100, self.read_serial)

    def send_by_serial(self):
        """Envía datos por puerto serial."""
        port = self.selected_port.get()
        if not port:
            messagebox.showerror("Error", "Please select a COM port.")
            return

        try:
            frequency = int(self.frequency_entry.get())
            if not (50 <= frequency <= 100):
                messagebox.showerror("Error", "Frequency must be between 50 and 100 KHz and with a step of 1 kHz.")
                return
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid numerical value for frequency.")
            return

        try:
            potentiometer_value = int(self.potentiometer_entry.get())
            if not (0 <= potentiometer_value <= 255):
                messagebox.showerror("Error", "Potentiometer value must be between 0 and 255.")
                return
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid numerical value for potentiometer.")
            return

        try:
            pulse_width = int(self.pulse_width_entry.get())
            if not (1 <= pulse_width <= 255):
                messagebox.showerror("Error", "Pulse width must be between 1 and 255 us.")
                return
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid numerical value for pulse width.")
            return

        selected_channel = self.channel_var.get()

        # Crear el paquete de datos
        data_packet = {
            'frequency': frequency,
            'pot_value': potentiometer_value,
            'pulse_width': pulse_width,
            'channel': selected_channel
        }

        # Convertir a JSON y enviar
        data_packet_json = json.dumps(data_packet) + "\n"  # Añadir salto de línea para separación
        self.update_monitor(f"Sending data by serial port: {data_packet_json}")

        try:
            if self.serial_port is None or not self.serial_port.is_open:
                self.serial_port = serial.Serial(port, 9600, timeout=1)
                time.sleep(1)  # Espera 1 segundo para que el puerto se estabilice
                self.serial_port.flushInput()  # Limpiar el buffer de entrada
                self.serial_port.flushOutput()  # Limpiar el buffer de salida
                #self.serial_port.write(b'\n')  # Enviar un byte vacío para sincronización
            # self.serial_port.write('\n'.encode('UTF-8')) #agregado
            self.serial_port.write(data_packet_json.encode('UTF-8'))  # Enviar datos como bytes
            print("Sending data by serial port:", data_packet_json)
        except serial.SerialException as e:
            messagebox.showerror("Error", f"Failed to connect to the selected COM port: {e}")

if __name__ == "__main__":
    app = SerialMonitor()
    app.mainloop()



# In[ ]:




