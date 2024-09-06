# General Schematic

The circuit is divided into 7 sections detailed below:

## 1. Power Supply

Different protection circuits (overvoltage/overcurrent and reverse voltage protection) are included for both power supplies, and fixed voltage regulators of 3.3V and 12V are also added.
[![](images/POWER_SUPPLY.png)](images/POWER_SUPPLY.png)

## 2. Main Control Circuit

Pinout of ESP32-DEVKITC-32E. Includes *PWM signals*: UxPWMXA, UxPWMXB (different units and operators), *SPI and SERIAL* connectivity, *External Trigger*, *I2C protocol* (SDA, SCL), *Switch SW6* which allows the microcontroller to be powered from an external source or from the ESP32 serial port.

[![](images/MAIN_CONTROL_CIRCUIT.png)](images/MAIN_CONTROL_CIRCUIT.png)


>!! **Warning** 
>!!
>!! While you have the ESP32 serial port communication connected, you cannot enable the power supply from the external source.

## 3. Digital Potentiometer

Pinout of ESP32-DEVKITC-32E. Includes *PWM signals*: UxPWMXA, UxPWMXB (different units and operators), *SPI and SERIAL* connectivity, *External Trigger*, *I2C protocol* (SDA, SCL), *Switch SW6* which allows the microcontroller to be powered from an external source or from the ESP32 serial port.


<div style="display: flex; overflow-x: auto; align-items: center;">
  <img src="/live/editor1/images/DIGITAL_POTENTIOMETER.png" alt="Digital Potentiometer" style="max-height: 400px; margin-right: 20px;">
  <div>
    <table border="1">
      <thead>
        <tr>
          <th style="min-width: 200px;">CHIP</th>
          <th style="min-width: 100px;">PORT</th>
          <th style="min-width: 150px;">CHANNEL HVPG</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <td style="min-width: 200px;">CHIP1_ADDR (U1)</td>
          <td>P3W, P3B</td>
          <td>CH1</td>
        </tr>
        <tr>
          <td style="min-width: 200px;">CHIP3_ADDRU0 (U3)</td>
          <td>P1W, P1B</td>
          <td>CH2</td>
        </tr>
        <tr>
          <td style="min-width: 200px;">CHIP3_ADDRU0 (U3)</td>
          <td>P2W, P2B</td>
          <td>CH3</td>
        </tr>
        <tr>
          <td style="min-width: 200px;">CHIP1_ADDRU1 (U1)</td>
          <td>P1W, P1B</td>
          <td>CH4</td>
        </tr>
        <tr>
          <td style="min-width: 200px;">CHIP2_ADDR (U2)</td>
          <td>P3W, P3B</td>
          <td>CH5</td>
        </tr>
        <tr>
          <td style="min-width: 200px;">CHIP2_ADDRU1 (U2)</td>
          <td>P2W, P2B</td>
          <td>CH6</td>
        </tr>
      </tbody>
    </table>
  </div>
</div>


## 4. Voltage Converter

Two converters for voltage control in the primary of the transformer were included whose specifications are as follows:

|  CHIP  | CONVERTER | INPUT VOLTAGE | OUTPUT VOLTAGE |
|:------:|:--------:|:-------:|:-------:|
|   MIC29302   |    LINEAL     |   24V   | 12-24V |
|   LT8361   |    BOOST     |   24V   | 24-80V |
 
[![](images/VOLTAGE_CONVERTER.png)](images/VOLTAGE_CONVERTER.png)

>i **Note** 
>i
>i Both converters can be controlled from the digital potentiometer outputs, as the resistance of the voltage divider (signal control feedback) can increased or decreased. **The LT8361 converter is not recommended as it has issues working with an inductive load.** Nevertheless, the functionality of this circuit is not necessary for the proper operation of the high-voltage pulse generator.

## 5. Isolation Circuit

The ISO7760 isolation circuit is included to prevent distortion of the PWM signals from the ESP32 due to noise from the inverter's switching on the power ground.

[![](images/ISOLATION.png)](images/ISOLATION.png)

## 6. High Frequency Transformer

The high-frequency transformer model [EEL25](https://es.aliexpress.com/item/1005006887515509.html?spm=a2g0o.productlist.main.3.4853694bGhWuVQ&algo_pvid=8c980d17-1524-4090-b8db-4b72fd5a2d7b&utparam-url=scene%3Asearch%7Cquery_from%3A) is connected to each channel of the high-voltage pulse generator as follows:

[![](images/TRAFO.png)](images/TRAFO.png)

• *VP1CH1 and VP3CH1* are the outputs of each [inverter](Schematic.md).

• *HVCH1* is the high-voltage sinusoidal waveform of each channel.

• *VSCH1* is the monitoring voltage, which is 500 times lower than the high voltage.

• *PWRGND2* is the high voltage ground common to all channels.



>! **Caution** 
>!
>! Since the high voltage ground is common across all channels, there could be current returns that cause **voltage coupling** between different channels. After the tests performed, these differences remain within a safe operating range. However, you should consider these variations if you need all channels to operate at the same voltage level; in that case, you should set the digital potentiometers at different levels.



## 7. Isolated Amplifier

An isolated amplifier is included for high-voltage monitoring to prevent damage to the oscilloscope or ADC used.

[![](images/ISOLATED.png)](images/ISOLATED.png)


>i **Note** 
>i
>i You may omit this circuit at your own risk.





