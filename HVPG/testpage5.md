# Control Circuit Board

This circuit allows generating high-voltage pulses of 5KV peak-to-peak at 100KHz on up to 6 channels. The ESP32 controls the position of the digital potentiometers MCP4451 (U1, U2, U3), allowing the high-voltage output of each channel to be remotely adjusted via the I2C protocol. Additionally, the ESP32 generates the PWM signals that control each inverter circuit (Full bridge single-phase inverter), enabling switching at up to 100KHz without overheating the circuit. Two power supplies are required to power the circuit, 5V/1A and 24V/3A.

[![](images/blue_pcb2.png)](images/blue_pcb2.png)

## Design Specifications

These instructions will guide you on how to design and assemble the control PCB. You can access the project by clicking here **[PCB_Project_Control](https://365.altium.com/files/9CAFADF4-6241-4E7E-99EF-D0BC9A7F499A)**


* [.](schematic2.md){step}
* [.](assembly2.md){step}



