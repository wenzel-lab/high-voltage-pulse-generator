#!/usr/bin/env python
# coding: utf-8

# In[2]:


import tkinter as tk
import customtkinter as ctk
from tkinter import messagebox
import serial
import serial.tools.list_ports

# Configuración de la ventana principal con tema oscuro
ctk.set_appearance_mode("dark")
root = ctk.CTk()
root.title("Settings of Multi Channel High Voltage Pulse Generator Board")

# Crear menú desplegable para seleccionar el puerto COM
def list_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

# Variables para manejar el puerto serial seleccionado
selected_port = tk.StringVar(root, value="")

# Crear un área de texto para el monitor serial
monitor_area = ctk.CTkTextbox(root, width=600, height=200, wrap=tk.WORD)
monitor_area.grid(row=6, column=0, columnspan=6, padx=10, pady=10)

def update_monitor(message):
    """Actualiza el monitor serial con un nuevo mensaje."""
    current_text = monitor_area.get("1.0", tk.END)
    monitor_area.delete("1.0", tk.END)
    monitor_area.insert(tk.END, current_text + message + "\n")
    monitor_area.yview(tk.END)

# Función para manejar el envío de datos de frecuencia
def set_frequency():
    global frequency_set
    try:
        frequency = int(frequency_entry.get())
        if 50 <= frequency <= 100:
            frequency_set = frequency
            update_monitor(f"Frequency set to: {frequency}")
        else:
            messagebox.showerror("Error", "Frequency must be between 50 and 100 kHz and with a step of 1 kHz.")
            frequency_set = None
    except ValueError:
        messagebox.showerror("Error", "Please enter a valid numerical value for frequency.")
        frequency_set = None

# Función para manejar el envío de datos de cada potenciómetro
def set_pot_value():
    global potentiometer_set
    try:
        value = int(potentiometer_entry.get())
        if 0 <= value <= 255:
            potentiometer_set = value
            update_monitor(f"Potentiometer value set to: {value}")
        else:
            messagebox.showerror("Error", "The potentiometer value must be between 0 and 255.")
            potentiometer_set = None
    except ValueError:
        messagebox.showerror("Error", "Please enter a valid numerical value for the potentiometer.")
        potentiometer_set = None

# Función para manejar el envío del ancho de pulso
def set_pulse_width():
    global pulse_width_set
    try:
        pulse_width = int(pulse_width_entry.get())
        if 1 <= pulse_width <= 255:
            pulse_width_set = pulse_width
            update_monitor(f"Pulse width set to: {pulse_width} us")
        else:
            messagebox.showerror("Error", "Pulse width must be between 1 and 255 us.")
            pulse_width_set = None
    except ValueError:
        messagebox.showerror("Error", "Please enter a valid numerical value for pulse width.")
        pulse_width_set = None

# Función para enviar datos por serial
def send_by_serial():
    port = selected_port.get()
    if not port:
        messagebox.showerror("Error", "Please select a COM port.")
        return

    try:
        frequency = int(frequency_entry.get())
        if not (50 <= frequency <= 100):
            messagebox.showerror("Error", "Frequency must be between 50 and 100 KHz and with a step of 1 kHz.")
            return
    except ValueError:
        messagebox.showerror("Error", "Please enter a valid numerical value for frequency.")
        return

    try:
        potentiometer_value = int(potentiometer_entry.get())
        if not (0 <= potentiometer_value <= 255):
            messagebox.showerror("Error", "Potentiometer value must be between 0 and 255.")
            return
    except ValueError:
        messagebox.showerror("Error", "Please enter a valid numerical value for potentiometer.")
        return

    try:
        pulse_width = int(pulse_width_entry.get())
        if not (1 <= pulse_width <= 255):
            messagebox.showerror("Error", "Pulse width must be between 1 and 255 us.")
            return
    except ValueError:
        messagebox.showerror("Error", "Please enter a valid numerical value for pulse width.")
        return

    selected_channel = channel_var.get()

    # Estructura de datos para todos los canales o un solo canal
    data_packet = {
        'frequency': frequency,
        'pot_value': potentiometer_value,
        'pulse_width': pulse_width,
        'channel': selected_channel
    }

    update_monitor(f"Sending data by serial port: {data_packet}")

    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            ser.write(str(data_packet).encode())
            print("Sending data by serial port:", data_packet)
    except serial.SerialException:
        messagebox.showerror("Error", "Failed to connect to the selected COM port.")

# Crear el menú desplegable para seleccionar el puerto COM
ports = list_ports()
port_menu = ctk.CTkOptionMenu(root, variable=selected_port, values=ports)
ctk.CTkLabel(root, text="Select COM Port:").grid(row=0, column=0, padx=10, pady=10)
port_menu.grid(row=0, column=1, padx=10, pady=10)

# Crear el menú desplegable para seleccionar el canal
channel_var = tk.StringVar(root, value="ALL")
channel_menu = ctk.CTkOptionMenu(root, variable=channel_var, values=["ALL"] + [f"CH{i+1}" for i in range(6)])
ctk.CTkLabel(root, text="Select Channel:").grid(row=1, column=0, padx=10, pady=10)
channel_menu.grid(row=1, column=1, padx=10, pady=10)

# Etiqueta y campo de entrada para el valor del potenciómetro
ctk.CTkLabel(root, text="Amplitud Value (step):").grid(row=0, column=2, padx=10, pady=10)
potentiometer_entry = ctk.CTkEntry(root, width=120)
potentiometer_entry.insert(0, "0")
potentiometer_entry.grid(row=0, column=3, padx=10, pady=10)

# Etiqueta y campo de entrada para la frecuencia
ctk.CTkLabel(root, text="Frequency (KHz):").grid(row=1, column=2, padx=10, pady=10)
frequency_entry = ctk.CTkEntry(root, width=120)
frequency_entry.insert(0, "80")
frequency_entry.grid(row=1, column=3, padx=10, pady=10)

# Etiqueta y campo de entrada para el ancho de pulso
ctk.CTkLabel(root, text="Pulse Width (us):").grid(row=2, column=2, padx=10, pady=10)
pulse_width_entry = ctk.CTkEntry(root, width=120)
pulse_width_entry.insert(0, "100")
pulse_width_entry.grid(row=2, column=3, padx=10, pady=10)

# Botón para enviar datos por puerto serial
ctk.CTkLabel(root, text="Channel settings:").grid(row=2, column=0, padx=10, pady=10)
serial_button = ctk.CTkButton(root, text="Send to ESP32", command=send_by_serial, fg_color="#4CAF50")
serial_button.grid(row=2, column=1, padx=10, pady=10)

# Crear un contenedor para centrar el botón
button_frame = ctk.CTkFrame(root)
button_frame.grid(row=7, column=0, columnspan=6, padx=10, pady=10, sticky='n')

# Botón para limpiar el monitor serial dentro del contenedor
clear_button = ctk.CTkButton(button_frame, text="Clear Monitor", command=lambda: monitor_area.delete("1.0", tk.END), width=120)
clear_button.pack()

# Iniciar el bucle de la interfaz gráfica
root.mainloop()


# In[ ]:




