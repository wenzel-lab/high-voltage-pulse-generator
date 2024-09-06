
# Program and Upload via Arduino Interface

To program and upload the program via the serial port to the [ESP32-DEVKITC-32E](https://www.mouser.cl/ProductDetail/Espressif-Systems/ESP32-DevKitC-32E?qs=GedFDFLaBXFpgD0kAZWDrQ%3D%3D), you can use the Arduino interface by following these instructions:

1.- **Install the Arduino IDE:**
   Download and install the Arduino IDE from the [Arduino website](https://www.arduino.cc/en/software).

2.- **Install the ESP32 Board Package:**
   
   - Open the Arduino IDE
   - Go to  <span style="color: red;">`File > Preferences`</span>. 
   - In the "Additional Board Manager URLs" field, add the following URL:<span style="color: red;"> `https://dl.espressif.com/dl/package_esp32_index.json`.</span>. 
   - Go to  <span style="color: red;">`Tools > Board > Board Manager`</span>.  
   - Search for "ESP32" and install the  <span style="color: red;">`esp32`</span> package by Espressif Systems.

3.- **Select the ESP32 Board:**
   - Go to <span style="color: red;">`Tools > Board`</span>. 
   - Select the appropriate ESP32 board from the list (e.g., <span style="color: red;">`ESP32 Dev Module)`.

4.- **Connect the ESP32 to Your Computer:**
   - Use a USB cable to connect the ESP32 to your computer.
   - Select the correct port in the Arduino IDE under <span style="color: red;">`Tools > Port`</span>. 

5.- **Upload the Program:**
   - Write your program or open an existing one.
   - Click the upload button (right arrow) in the Arduino IDE to compile and upload the program to the ESP32.

In the following subsections, you can find the details of the programmed functions that allow generating PWM signals and varying the position of the digital potentiometers.You can **download the code** from [here](images/POT_PWM/POT_PWM.ino). 

* [.](potentiometer.md){step}
* [.](pwm.md){step}



