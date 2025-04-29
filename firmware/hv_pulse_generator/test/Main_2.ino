#include <Arduino.h>
#include "driver/mcpwm.h"   //biblioteca "Motor Control PWM" nativa ESP32
#include "soc/mcpwm_periph.h"
#include <Wire.h>
#include <math.h>
#include <ESP32SPISlave.h>
// Direcciones I2C base de los tres chips con las direcciones variables 00, 10 y 11
#define BASE_ADDR 0x2C  // Dirección base del MCP4451 (01011 en los bits 7-3)
#define CHIP1_ADDR BASE_ADDR | 0x00 // Dirección del primer chip (A1A0 = 00)
#define CHIP2_ADDR BASE_ADDR | 0x02  // Dirección del segundo chip (A1A0 = 10)
#define CHIP3_ADDR BASE_ADDR | 0x03  // Dirección del tercer chip (A1A0 = 11)
// 4 Canales por cada chip 
#define CHANNEL0 0x00  // Dirección de memoria 00h
#define CHANNEL1 0x01  // Dirección de memoria 01h
#define CHANNEL2 0x06  // Dirección de memoria 06h
#define CHANNEL3 0x07  // Dirección de memoria 07h
//PWM
#define GPIO_U0_PWM0A_OUT 17   //Declara GPIO 17 como PWM0A_U0 CH1
#define GPIO_U0_PWM0B_OUT 16   //Declara GPIO 16 como PWM0B_U0 CH1
#define GPIO_U0_PWM1A_OUT 4    //Declara GPIO 4  como PWM1A_U0 CH2
#define GPIO_U0_PWM1B_OUT 0    //Declara GPIO 0  como PWM1B_U0 CH2
#define GPIO_U0_PWM2A_OUT 2    //Declara GPIO 2  como PWM2A_U0 CH3
#define GPIO_U0_PWM2B_OUT 15   //Declara GPIO 15 como PWM2B_U0 CH3
#define GPIO_U1_PWM0A_OUT 25   //Declara GPIO 25 como PWM0A_U1 CH4
#define GPIO_U1_PWM0B_OUT 26   //Declara GPIO 26 como PWM0B_U1 CH4
#define GPIO_U1_PWM1A_OUT 27   //Declara GPIO 27 como PWM1A_U1 CH5
#define GPIO_U1_PWM1B_OUT 14   //Declara GPIO 14 como PWM1B_U1 CH5
#define GPIO_U1_PWM2A_OUT 12   //Declara GPIO 12 como PWM2A_U1 CH6
#define GPIO_U1_PWM2B_OUT 13   //Declara GPIO 13 como PWM2B_U1 CH6
//SPI Slave
ESP32SPISlave slave;
static constexpr uint32_t BUFFER_SIZE {32};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];
uint8_t lastChannel = 7;  // Valor inicial fuera del rango válido
uint8_t lastPotValue = 256; // Valor inicial fuera del rango válido

void mcpwm(mcpwm_unit_t Unidade,
          mcpwm_io_signals_t Operador_A,
          mcpwm_io_signals_t Operador_B,
          mcpwm_timer_t TIMER,
          int GPIO_A,
          int GPIO_B,
          int freq) {

  mcpwm_gpio_init(Unidade, Operador_A, GPIO_A);
  mcpwm_gpio_init(Unidade, Operador_B, GPIO_B);

  mcpwm_config_t pwm_config;
  pwm_config.frequency = freq;    // Frequency = 100kHz
  pwm_config.cmpr_a = 50.0;       // Duty cycle of operator A = 50%
  pwm_config.cmpr_b = 50.0;       // Duty cycle of operator B = 50%
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  // Initialize MCPWM
  mcpwm_init(Unidade, TIMER, &pwm_config);

  // Set duty type for operator A and B
  mcpwm_set_duty_type(Unidade, TIMER, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(Unidade, TIMER, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);

  // Set complementary mode
  //mcpwm_sync_enable(Unidade, TIMER, MCPWM_SELECT_TIMER0, 0); // Enable synchronization for complementary mode
  mcpwm_deadtime_enable(Unidade, TIMER, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 0, 1); // Deadtime 20-200 ns

}

//Habilita señal PWM de cada canal hacia inversor
void activatePWMChannel(uint8_t channel, uint8_t Freq, uint8_t pulse_width) {
    uint32_t frequency = Freq * 1000;
    //Serial.print("\n");
    //Serial.print("Frequency [Hz]: ");
    //Serial.print(frequency, DEC);
    // Asignamos el canal actual
    switch (channel) {
        case 1:  // CH1
            mcpwm(MCPWM_UNIT_0, MCPWM0A, MCPWM0B, MCPWM_TIMER_0, GPIO_U0_PWM0A_OUT, GPIO_U0_PWM0B_OUT, frequency);
            delayMicroseconds(pulse_width);
            //mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_0); //deshabilita timer y configuracion ciclo trabajo
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);  // Apaga señal A
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);  // Apaga señal B
            break;
        case 2:  // CH2
            mcpwm(MCPWM_UNIT_0, MCPWM1A, MCPWM1B, MCPWM_TIMER_1, GPIO_U0_PWM1A_OUT, GPIO_U0_PWM1B_OUT, frequency);
            delayMicroseconds(pulse_width);
            //mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_1);
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
            break;
        case 3:  // CH3
            mcpwm(MCPWM_UNIT_0, MCPWM2A, MCPWM2B, MCPWM_TIMER_2, GPIO_U0_PWM2A_OUT, GPIO_U0_PWM2B_OUT, frequency);
            delayMicroseconds(pulse_width);
            //mcpwm_stop(MCPWM_UNIT_0, MCPWM_TIMER_2);
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
            break;
        case 4:  // CH4
            mcpwm(MCPWM_UNIT_1, MCPWM0A, MCPWM0B, MCPWM_TIMER_0, GPIO_U1_PWM0A_OUT, GPIO_U1_PWM0B_OUT, frequency);
            delayMicroseconds(pulse_width);
            //mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_0);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B);
            break;
        case 5:  // CH5
            mcpwm(MCPWM_UNIT_1, MCPWM1A, MCPWM1B, MCPWM_TIMER_1, GPIO_U1_PWM1A_OUT, GPIO_U1_PWM1B_OUT, frequency);
            delayMicroseconds(pulse_width);
            //mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_1);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
            break;
        case 6:  // CH6
            mcpwm(MCPWM_UNIT_1, MCPWM2A, MCPWM2B, MCPWM_TIMER_2, GPIO_U1_PWM2A_OUT, GPIO_U1_PWM2B_OUT, frequency);
            delayMicroseconds(pulse_width);
            //mcpwm_stop(MCPWM_UNIT_1, MCPWM_TIMER_2);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B);
            break;
        case 0:  // Todos los canales (activación en cascada)
            for (int i = 1; i <= 6; i++) {
                activatePWMChannel(i, Freq, pulse_width);
                //delayMicroseconds(50);  // Pequeño retardo entre activaciones, ajusta según sea necesario
            }
            return;
        default:
            Serial.println("Invalid channel selected.");
            break;
    }
}

void configurePotentiometer(uint8_t channel, uint8_t pot_value) {
    uint8_t response; // Declarar la variable fuera del switch
    uint8_t step_amp = 255 - pot_value; //max voltage step_amp = 0
    switch (channel) {
        case 1:
            assign_potentiometer(CHIP1_ADDR, CHANNEL3, step_amp);
            break;
        case 2:
            assign_potentiometer(CHIP3_ADDR, CHANNEL1, step_amp);
            break;
        case 3:
            assign_potentiometer(CHIP3_ADDR, CHANNEL2, step_amp);
            break;
        case 4:
            assign_potentiometer(CHIP1_ADDR, CHANNEL1, step_amp);
            break;
        case 5:
            assign_potentiometer(CHIP2_ADDR, CHANNEL3, step_amp);
            // Añadir un pequeño retraso antes de la lectura
            //delay(1000);
            //response = read_potentiometer2(CHIP2_ADDR, CHANNEL3); // chip, canal, response2 = return lowByte
            //Serial.print("\n");
            //Serial.print("posición potenciometro: ");
            //Serial.print(response, DEC);
            break;
        case 6:
            assign_potentiometer(CHIP2_ADDR, CHANNEL2, step_amp);
            break;
        case 0: // Todos los canales
            assign_potentiometer(CHIP1_ADDR, CHANNEL3, step_amp); // CH1
            assign_potentiometer(CHIP3_ADDR, CHANNEL1, step_amp); // CH2
            assign_potentiometer(CHIP3_ADDR, CHANNEL2, step_amp); // CH3
            assign_potentiometer(CHIP1_ADDR, CHANNEL1, step_amp); // CH4
            assign_potentiometer(CHIP2_ADDR, CHANNEL3, step_amp); // CH5
            assign_potentiometer(CHIP2_ADDR, CHANNEL2, step_amp); // CH6
            break;
        default:
            Serial.println("Invalid channel");
            break;
    }
}

void setup() {

  // Inicia la comunicación serie para depuración
  Serial.begin(9600);
  //SPI Slave
  slave.setDataMode(SPI_MODE0);
  slave.begin(VSPI);
  // clear buffers
  memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

  // Inicia la comunicación I2C
  Wire.begin(21, 22); // SDA = 21, SCL = 22 para ESP32
  Wire.setClock(1000);

  //delay(1000);
  //calibration_Range1(); //Usar esta funcion para determinar el rango de voltaje de operación de cada canal
  //Serial.print("\n");
  //Serial.print("Calibración finalizada");

}

void loop() {
    //Serial.print("\n");
    //Serial.println("Waiting for SPI data...");

    // Block until the transaction comes from master
    slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

    if (slave.available()) {
        unsigned long startTime = micros();
        uint8_t Freq = spi_slave_rx_buf[0];   // Primer byte: Frecuencia
        uint8_t pot_value = spi_slave_rx_buf[1];   // Segundo byte: Valor del potenciómetro
        uint8_t pulse_width = spi_slave_rx_buf[2]; // Tercer byte: Ancho de pulso
        uint8_t channel = spi_slave_rx_buf[3];     // Cuarto byte: Canal

        // Verificar si el canal o el valor del potenciómetro han cambiado
        if (channel != lastChannel || pot_value != lastPotValue) {
            configurePotentiometer(channel, pot_value); // Set amplitude
            lastChannel = channel;  // Actualizar canal actual
            lastPotValue = pot_value;  // Actualizar valor del potenciómetro
        }

        activatePWMChannel(channel, Freq, pulse_width); // Activate PWM signal durante un tiempo determinado en us
        unsigned long endTime = micros();
        unsigned long transferTime = endTime - startTime;
        Serial.print("SPI Transfer Time: ");
        Serial.println(transferTime);
    }
}


// Función para incrementar la posición del potenciómetro
void incrementPotentiometer(uint8_t chipAddr, uint8_t channel) {
  Wire.beginTransmission(chipAddr);

  // Comando de incremento INCR para el canal especificado (AD3:AD0 0 1 X X)
  uint8_t command = (channel << 4) | 0x04; // AD3:AD0 con 0 1 en los bits menos significativos

  Wire.write(command); // Enviar el comando de incremento

  Wire.endTransmission();
}

// Función para decrementar la posición del potenciómetro
void decrementPotentiometer(uint8_t chipAddr, uint8_t channel) {
  Wire.beginTransmission(chipAddr);

  // Comando de decremento DECR para el canal especificado (AD3:AD0 1 0 X X)
  uint8_t command = (channel << 4) | 0x08; // AD3:AD0 con 1 0 en los bits menos significativos

  Wire.write(command); // Enviar el comando de decremento

  Wire.endTransmission();
}

void assign_potentiometer(uint8_t chipAddr, uint8_t channel, uint8_t set_value){
  Wire.beginTransmission(chipAddr);
  // Comando de write para el canal especificado (AD3:AD0 0 0 X D8)
  uint8_t command = (channel << 4) | 0x00; // AD3:AD0 con 0 0 en los bits menos significativos

  // Dividir el valor en 9 bits: los primeros 8 bits (D7:D0) y el bit más significativo (D8)
  //uint8_t d8 = (set_value >> 8) & 0x01; // Extraer el bit D8
  //uint8_t data = set_value & 0xFF; // Extraer los bits D7:D0

  // Enviar el comando y los datos al dispositivo
  //Wire.write(command | d8); // Enviar el comando con el bit D8
  Wire.write(command);
  Wire.write(set_value); // valor a setear en el potenciometro para el canal especificado

  if (Wire.endTransmission() == 0) {
    //Serial.print("\n");
    //Serial.print("Valor ");
    //Serial.print(set_value);
    //Serial.println(" asignado correctamente.");
  } else {
    Serial.println("Error en la transmisión I2C para asignar valor.");
  }
}

uint8_t read_potentiometer2(uint8_t chipAddr, uint8_t channel){

  Serial.print("\n");
  Serial.println("Iniciando lectura del potenciómetro...");

  // Comando de lectura para el canal especificado (AD3:AD0 1 1 X)
  uint8_t command = (channel << 4) | 0x0C; // AD3:AD0 con 1 1 en los bits menos significativos
  uint16_t result = 0;
  // Iniciar transmisión al dispositivo
  Wire.beginTransmission(chipAddr);
  Wire.write(command); // Enviar el comando de lectura
  //Wire.endTransmission(false); // Terminar la transmisión con condición de reinicio
  if (Wire.endTransmission(false) != 0) { // Condición de reinicio (Repeated Start Condition)
    Serial.println("Error en la transmisión I2C.");
    return 0;
  } else {
    Serial.println("Comando de lectura enviado correctamente.");
  }

  // Leer los datos del potenciómetro
  Wire.requestFrom(chipAddr, (uint8_t)2);  // Solicitar 2 bytes de datos
  if (Wire.available() >= 2) {
    uint8_t highByte = Wire.read();  // Leer el primer byte (bits más significativos)
    uint8_t lowByte = Wire.read();   // Leer el segundo byte (bits menos significativos)
    //result = ((uint16_t)highByte << 8) | lowByte;  // Combinar ambos bytes en un solo valor de 16 bits
    Serial.print("Datos recibidos: 0x");
    Serial.print(highByte, HEX);
    Serial.print(" 0x");
    Serial.println(lowByte, HEX);
    //Wire.endTransmission();
    return lowByte;
  } else {
    Serial.print("\n");
    Serial.println("No se recibieron suficientes datos.");
    return 0;  // Si no hay datos disponibles, devolver 0
  }
}

void calibration_Range1(){

  // Lista de chips y canales: CHIP2-CHANNEL3 es el canal 5 del amplificador inversor
  uint8_t chips[] = {CHIP2_ADDR}; // CHIP1_ADDR, CHIP2_ADDR, CHIP3_ADDR
  uint8_t channels[] = {CHANNEL3}; // CHANNEL1, CHANNEL2, CHANNEL3
  uint8_t pos_init = 0;
  // Incrementa la posición inicial de los potenciómetros en cada chip y cada canal
  for (uint8_t i = 0; i < sizeof(chips) / sizeof(chips[0]); i++) {
    for (uint8_t j = 0; j < sizeof(channels) / sizeof(channels[0]); j++) {
      assign_potentiometer(chips[i], channels[j], pos_init); //parte el barrido desde la posición pos_init
      for (uint8_t k = 0; k < 255 - pos_init; k++){
        incrementPotentiometer(chips[i], channels[j]); //incrementa la posición del potenciometro variando desde un máximo a mínimo voltaje
        delay(1000);
      }
    }
  }
}
