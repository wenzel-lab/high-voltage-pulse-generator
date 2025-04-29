#include <Arduino.h>
#include "driver/mcpwm.h"   //biblioteca "Motor Control PWM" nativa ESP32
#include "soc/mcpwm_periph.h"
#include <Wire.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <Preferences.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define INTERNET_CONTROL true

// Direcciones I2C para digipot
#define DIGPOT_ADDR_1 0x2C  // Dirección base del MCP4451 (01011 en los bits 7-3)
#define DIGPOT_ADDR_2 0x2D  // Dirección base del MCP4451 (01011 en los bits 7-3)
#define DIGPOT_ADDR_3 0x2E  // Dirección base del MCP4451 (01011 en los bits 7-3)
#define DIGPOT_ADDR_4 0x2F  // Dirección base del MCP4451 (01011 en los bits 7-3)
#define DIGPOT_ADDR_5 0x2E  // Dirección base del MCP4451 (01011 en los bits 7-3)
#define DIGPOT_ADDR_6 0x2F  // Dirección base del MCP4451 (01011 en los bits 7-3)


// 4 Canales por cada chip
#define CHANNEL0 0x00  // Dirección de memoria 00h

//PWM
#define GPIO_U0_PWM0A_OUT 17   //Declara GPIO 17 como PWM0A_U0 CH1
#define GPIO_U0_PWM0B_OUT 16   //Declara GPIO 16 como PWM0B_U0 CH1
#define GPIO_U0_PWM1A_OUT 4    //Declara GPIO 4  como PWM1A_U0 CH2
#define GPIO_U0_PWM1B_OUT 0    //Declara GPIO 0  como PWM1B_U0 CH2
#define GPIO_U0_PWM2A_OUT 2    //Declara GPIO 2  como PWM2A_U0 CH3
#define GPIO_U0_PWM2B_OUT 15   //Declara GPIO 15 como PWM2B_U0 CH3
#define GPIO_U1_PWM0A_OUT 13   //Declara GPIO 13 como PWM0A_U1 CH4
#define GPIO_U1_PWM0B_OUT 12   //Declara GPIO 12 como PWM0B_U1 CH4
#define GPIO_U1_PWM1A_OUT 14   //Declara GPIO 14 como PWM1A_U1 CH5
#define GPIO_U1_PWM1B_OUT 27   //Declara GPIO 27 como PWM1B_U1 CH5
#define GPIO_U1_PWM2A_OUT 26   //Declara GPIO 26 como PWM2A_U1 CH6
#define GPIO_U1_PWM2B_OUT 25   //Declara GPIO 25 como PWM2B_U1 CH6

#define GPIO_TRIGGER_1  36
#define GPIO_TRIGGER_2  39
#define GPIO_TRIGGER_3  34
#define GPIO_TRIGGER_4  35
#define GPIO_TRIGGER_5  32
#define GPIO_TRIGGER_6  33


#define ID "HVPG-01"
#define VERSION  "HVPG-V1.2"
#define RESPONSE_OK     		"OK"
#define RESPONSE_ERROR  		"ERROR"
#define RESPONSE_ERROR_JSON  	"ERROR_INPUT_JSON"
#define RESPONSE_ERROR_CMD 		"ERROR_CMD"

#define NUMBER_CHANNELS 6
#define DEFAULT_AMP 10
#define DEFAULT_FREQ 100
#define DEFAULT_DUTY 50
#define DEFAULT_DURATION 100
#define DEFAULT_CHANNEL 1

#define WIFI_TIMEOUT 60
#define MQTT_TIMEOUT 5

//MQTT
#define PUBLISH_TOPIC  	"hvc/tx"
#define SUBSCRIBE_TOPIC "hvc/rx"

#define mqtt_server 	"35.223.234.244"
#define mqtt_user 		"iowlabs"
#define mqtt_password 	"!iow_woi!"
#define mqtt_port 1883

// Configuración de la red Wi-Fi
const char* ssid = "UDD";
const char* password = "udd_2019";


//Task to handel mqtt rcv commands
TaskHandle_t Task1;
// Configuración del cliente Wi-Fi y MQTT
WiFiClient wifiClient;
PubSubClient client(wifiClient);
//Handel save pre-sets for new instance
Preferences preferences;


struct Pulse
{
	uint8_t channel;
	int 	freq;
 	int 	amp;
	int 	duty;
	int 	duration;
};
Pulse pulseparameters[NUMBER_CHANNELS];

/*
	VARIABLES
*/

volatile bool triggered[6] = {false, false, false, false, false, false};

uint8_t lastChannel = 7;   // Valor inicial fuera del rango válido
uint8_t lastPotValue = 255;// Valor inicial fuera del rango válido

//cmd
const char* cmd;
const char* rcvd_id;
const char* response = RESPONSE_OK;
int rcvd_ch = DEFAULT_CHANNEL;

// commection
bool wifi_status = false;
int wifi_try = 0;
int mqtt_try = 0;
bool bussy_mqtt = false;


/*
	FUNCTIONS HEADRES
*/
void mcpwm(	mcpwm_unit_t Unidade,mcpwm_io_signals_t Operador_A,mcpwm_io_signals_t Operador_B,mcpwm_timer_t TIMER,int GPIO_A,int GPIO_B,int freq);
void activatePWMChannel(uint8_t channel, uint8_t Freq, uint8_t pulse_width);
void decrementPotentiometer(uint8_t chipAddr, uint8_t channel);
void configurePotentiometer(uint8_t channel, uint8_t pot_value);
void incrementPotentiometer(uint8_t chipAddr, uint8_t channel);
void assign_potentiometer(uint8_t chipAddr, uint8_t channel, uint8_t set_value);
void calibration_Range();

void savePreset(int index);
void loadPresets(int index);

void processCmdSerial();

void setup_wifi();
void reconnect();
void processCmd(byte* payload, unsigned int length);
void publishMqtt(char *payload);
void publishResponse();
void callback(char* topic, byte* payload, unsigned int length);
void Task1code(void *pvParameters);

//Trigger functions
void IRAM_ATTR keyChanged1();
void IRAM_ATTR keyChanged2();
void IRAM_ATTR keyChanged3();
void IRAM_ATTR keyChanged4();
void IRAM_ATTR keyChanged5();
void IRAM_ATTR keyChanged6();

void setup()
{
	// Inicia la comunicación serie para depuración
	Serial.begin(9600);

	Serial.println("\n -------------------------------------\n");
	Serial.println(" High Voltage Pulse controller firmware");
	Serial.print(" Version code : ");
	Serial.println(VERSION);
	Serial.println("\n -------------------------------------\n");


	// Inicia la comunicación I2C
  	Wire.begin(21, 22); // SDA = 21, SCL = 22 para ESP32
  	Wire.setClock(1000);

	pinMode(GPIO_TRIGGER_1, INPUT);
  	pinMode(GPIO_TRIGGER_2, INPUT);
  	pinMode(GPIO_TRIGGER_3, INPUT);
  	pinMode(GPIO_TRIGGER_4, INPUT);
  	pinMode(GPIO_TRIGGER_5, INPUT);
  	pinMode(GPIO_TRIGGER_6, INPUT);

	attachInterrupt(digitalPinToInterrupt(GPIO_TRIGGER_1), keyChanged1, RISING);
  	attachInterrupt(digitalPinToInterrupt(GPIO_TRIGGER_2), keyChanged2, RISING);
  	attachInterrupt(digitalPinToInterrupt(GPIO_TRIGGER_3), keyChanged3, RISING);
  	attachInterrupt(digitalPinToInterrupt(GPIO_TRIGGER_4), keyChanged4, RISING);
  	attachInterrupt(digitalPinToInterrupt(GPIO_TRIGGER_5), keyChanged5, RISING);
  	attachInterrupt(digitalPinToInterrupt(GPIO_TRIGGER_6), keyChanged6, RISING);

	//load data from previous session
	for (size_t i = 0; i < NUMBER_CHANNELS; i++)
	{
		loadPresets(i);
	}

	// Task 1 for handle mqtt reception
	xTaskCreatePinnedToCore(
			Task1code,   /* Task function. */
            "Task1",     /* name of task. */
            10000,       /* Stack size of task */
            NULL,        /* parameter of the task */
            1,           /* priority of the task */
            &Task1,      /* Task handle to keep track of created task */
            0);          /* pin task to core 0 */
	delay(500);

	if(INTERNET_CONTROL)
	{
		setup_wifi();
		client.setServer(mqtt_server, mqtt_port);
		client.setCallback(callback);
	}

	Serial.println("Setup Ready. Waiting for cmd");

}

void loop()
{
	//Handle cmd from USB

    while( Serial.available() > 0 ){ processCmdSerial(); }

	//TRIGGERS HANDLE
	for (int i = 0; i < 6; i++)
	{
    	if (triggered[i])
		{
      		triggered[i] = false;
      		Serial.printf("Trigger en GPIO %d\n", i);
      		// Trigger pulse
			activatePWMChannel( i , pulseparameters[i].freq, pulseparameters[i].duration);
    	}
  	}

/*
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
*/
}


/*
	Second loop to manage the mqtt connections
*/
void Task1code( void * pvParameters )
{
 	Serial.print("Task1 running on core ");
 	Serial.println(xPortGetCoreID());

	for(;;)
	{
		if(INTERNET_CONTROL)
		{
			if (bussy_mqtt == false)
			{
				if (!client.connected())
				{
			    	reconnect();
				}
			}
			client.loop();


  		}
	vTaskDelay(10);  // Cede el control por un tick del sistema
	}
}


void mcpwm(	mcpwm_unit_t Unidade,
          	mcpwm_io_signals_t Operador_A,
          	mcpwm_io_signals_t Operador_B,
          	mcpwm_timer_t TIMER,
          	int GPIO_A,
          	int GPIO_B,
          	int freq)
{
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
void activatePWMChannel(uint8_t _channel, uint8_t _freq, uint8_t _duration)
{
	Serial.println("Sending pulse ");
    uint32_t _frequency = _freq * 1000;
    switch (_channel)
	{
        case 1:  // CH1
            mcpwm(MCPWM_UNIT_0, MCPWM0A, MCPWM0B, MCPWM_TIMER_0, GPIO_U0_PWM0A_OUT, GPIO_U0_PWM0B_OUT, _frequency);
            //delayMicroseconds(_duration);
			delay(_duration*1000);
			mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);  // Apaga señal A
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);  // Apaga señal B
            break;
        case 2:  // CH2
            mcpwm(MCPWM_UNIT_0, MCPWM1A, MCPWM1B, MCPWM_TIMER_1, GPIO_U0_PWM1A_OUT, GPIO_U0_PWM1B_OUT, _frequency);
            delayMicroseconds(_duration);
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
            break;
        case 3:  // CH3
            mcpwm(MCPWM_UNIT_0, MCPWM2A, MCPWM2B, MCPWM_TIMER_2, GPIO_U0_PWM2A_OUT, GPIO_U0_PWM2B_OUT, _frequency);
            delayMicroseconds(_duration);
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
            break;
        case 4:  // CH4
            mcpwm(MCPWM_UNIT_1, MCPWM0A, MCPWM0B, MCPWM_TIMER_0, GPIO_U1_PWM0A_OUT, GPIO_U1_PWM0B_OUT, _frequency);
            delayMicroseconds(_duration);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B);
            break;
        case 5:  // CH5
            mcpwm(MCPWM_UNIT_1, MCPWM1A, MCPWM1B, MCPWM_TIMER_1, GPIO_U1_PWM1A_OUT, GPIO_U1_PWM1B_OUT, _frequency);
            delayMicroseconds(_duration);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B);
            break;
        case 6:  // CH6
            mcpwm(MCPWM_UNIT_1, MCPWM2A, MCPWM2B, MCPWM_TIMER_2, GPIO_U1_PWM2A_OUT, GPIO_U1_PWM2B_OUT, _frequency);
            delayMicroseconds(_duration);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_A);
            mcpwm_set_signal_low(MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM_OPR_B);
            break;
        case 0:  // Todos los canales (activación en cascada)
            for (int i = 1; i <= 6; i++)
			{
                activatePWMChannel(i, _freq, _duration);
                //delayMicroseconds(50);  // Pequeño retardo entre activaciones, ajusta según sea necesario
            }
            break;
        default:
            Serial.println("Invalid channel selected.");
            break;
    }
}

void configurePotentiometer(uint8_t channel, uint8_t pot_value)
{
    uint8_t response; // Declarar la variable fuera del switch
    uint8_t step_amp = 255 - pot_value; //max voltage step_amp = 0
    switch (channel)
	{
        case 1:
            assign_potentiometer(DIGPOT_ADDR_1, CHANNEL0, step_amp);
            break;
        case 2:
            assign_potentiometer(DIGPOT_ADDR_2, CHANNEL0, step_amp);
            break;
        case 3:
            assign_potentiometer(DIGPOT_ADDR_3, CHANNEL0, step_amp);
            break;
        case 4:
            assign_potentiometer(DIGPOT_ADDR_4, CHANNEL0, step_amp);
            break;
        case 5:
            assign_potentiometer(DIGPOT_ADDR_5,CHANNEL0, step_amp);
            break;
        case 6:
            assign_potentiometer(DIGPOT_ADDR_6, CHANNEL0, step_amp);
            break;
        case 0: // Todos los canales
            assign_potentiometer(DIGPOT_ADDR_1, CHANNEL0, step_amp); // CH1
            assign_potentiometer(DIGPOT_ADDR_2, CHANNEL0, step_amp); // CH2
            assign_potentiometer(DIGPOT_ADDR_3, CHANNEL0, step_amp); // CH3
            assign_potentiometer(DIGPOT_ADDR_4, CHANNEL0, step_amp); // CH4
            assign_potentiometer(DIGPOT_ADDR_5, CHANNEL0, step_amp); // CH5
            assign_potentiometer(DIGPOT_ADDR_6, CHANNEL0, step_amp); // CH6
            break;
        default:
            Serial.println("Invalid channel");
            break;
    }
}

// Función para incrementar la posición del potenciómetro
void incrementPotentiometer(uint8_t chipAddr, uint8_t channel)
{
	Wire.beginTransmission(chipAddr);
	// Comando de incremento INCR para el canal especificado (AD3:AD0 0 1 X X)
	uint8_t command = (channel << 4) | 0x04; // AD3:AD0 con 0 1 en los bits menos significativos
	Wire.write(command); // Enviar el comando de incremento

  	Wire.endTransmission();
}

// Función para decrementar la posición del potenciómetro
void decrementPotentiometer(uint8_t chipAddr, uint8_t channel)
{
	Wire.beginTransmission(chipAddr);
	// Comando de decremento DECR para el canal especificado (AD3:AD0 1 0 X X)
  	uint8_t command = (channel << 4) | 0x08; // AD3:AD0 con 1 0 en los bits menos significativos
	Wire.write(command); // Enviar el comando de decremento
	Wire.endTransmission();
}

void assign_potentiometer(uint8_t chipAddr, uint8_t channel, uint8_t set_value)
{
	Wire.beginTransmission(chipAddr);
  	// Comando de write para el canal especificado (AD3:AD0 0 0 X D8)
  	uint8_t command = (channel << 4) | 0x00; // AD3:AD0 con 0 0 en los bits menos significativos

  	Wire.write(command);
  	Wire.write(set_value); // valor a setear en el potenciometro para el canal especificado

  	if (Wire.endTransmission() == 0)
	{
    	//Serial.print("\n");
    	//Serial.print("Valor ");
    	//Serial.print(set_value);
    	//Serial.println(" asignado correctamente.");
  	}
	else
	{
    	Serial.println("Error en la transmisión I2C para asignar valor.");
  	}
}

void calibration_Range()
{
	uint8_t chips[] 	= {DIGPOT_ADDR_1}; // CHIP1_ADDR, CHIP2_ADDR, CHIP3_ADDR
	uint8_t channel 	= CHANNEL0; // CHANNEL1, CHANNEL2, CHANNEL3
	uint8_t pos_init	= 0;

  	// Incrementa la posición inicial de los potenciómetros en cada chip y cada canal
  	for (uint8_t i = 0; i < sizeof(chips) / sizeof(chips[0]); i++)
	{
    	assign_potentiometer(chips[i], channel, pos_init); //parte el barrido desde la posición pos_init
      	for (uint8_t k = 0; k < 255 - pos_init; k++)
		{
        	incrementPotentiometer(chips[i], channel); //incrementa la posición del potenciometro variando desde un máximo a mínimo voltaje
        	delay(1000);
      	}
    }
}


/*
uint8_t read_potentiometer2(uint8_t chipAddr, uint8_t channel)
{
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
*/


/*
	Save current presets to memory for motor index
*/
void savePreset(int index)
{
	String ns = "pulse_" + String(index);
  	preferences.begin(ns.c_str(), false);// begin in write mode
  	preferences.putInt("channel", 	pulseparameters[index].channel);
  	preferences.putInt("amp", 		pulseparameters[index].amp);
  	preferences.putInt("freq", 		pulseparameters[index].freq);
  	preferences.putInt("duty", 		pulseparameters[index].duty);
  	preferences.putInt("duration",	pulseparameters[index].duration);

  	preferences.end();
}

/*
	Load presets from the previous session for motor index
*/
void loadPresets(int index)
{
	String ns = "pulse_" + String(index);
	preferences.begin(ns.c_str(), true); // load memory in read mode
	pulseparameters[index].channel 	= preferences.getInt("channel", DEFAULT_CHANNEL);
	pulseparameters[index].amp 		= preferences.getInt("amp",		DEFAULT_AMP);
	pulseparameters[index].freq 	= preferences.getInt("freq",	DEFAULT_FREQ);
	pulseparameters[index].duty		= preferences.getInt("duty",	DEFAULT_DUTY);
	pulseparameters[index].duration = preferences.getInt("duration",DEFAULT_DURATION);

  	preferences.end();
}

/* mqtt callback*/
// Función de callback para manejar mensajes recibidos
void callback(char* topic, byte* payload, unsigned int length)
{
	Serial.print("Mensaje recibido en el tema: ");
	Serial.println(topic);
	Serial.print("Contenido: ");
	for (unsigned int i = 0; i < length; i++)
 	{
		Serial.print((char)payload[i]);
	}
	Serial.println("");
	processCmd(payload,length);
}

void publishMqtt(char *payload)
{
	if(wifi_status)
	{
		bussy_mqtt = true;
		Serial.print("sending data by mqtt . . .");
		client.publish(PUBLISH_TOPIC, payload , true);
		Serial.println(" done");
		bussy_mqtt = false;
	}

}

void setup_wifi()
{
	delay(10);
	Serial.println("Conectando a Wi-Fi...");
	WiFi.begin(ssid, password);
	wifi_try = 0;
 	while (WiFi.status() != WL_CONNECTED)
	{
    	delay(1000);
    	Serial.print(".");
		wifi_try +=1;
		if(wifi_try >= WIFI_TIMEOUT)break;
  	}
	if(WiFi.status()!= WL_CONNECTED)
	{
		wifi_status = false;
		Serial.println(" Wifi no conectado");
	}
	else
	{
		wifi_status = true;
		Serial.println("\nConexión Wi-Fi establecida. Dirección IP: ");
	  	Serial.println(WiFi.localIP());
	}


}

void reconnect()
{
	if(wifi_status)
	{
		mqtt_try = 0 ;
		while (!client.connected())
		{
	    	Serial.print("Conectando al broker MQTT...");
	    	if (client.connect(ID, mqtt_user, mqtt_password))
			{
	      		Serial.println("Conectado");
	      		// Suscribirse al tema con QoS 1
	      		client.subscribe(SUBSCRIBE_TOPIC, 1);
	      		Serial.println("Suscrito al tema: ");
	      		Serial.println(SUBSCRIBE_TOPIC);
	    	}
			else
			{
	      		Serial.print("Error: ");
	      		Serial.print(client.state());
	      		Serial.println(". Reintentando en 5 segundos...");
	      		delay(5000);
				mqtt_try +=1;
				if(mqtt_try >= MQTT_TIMEOUT)break;
	    	}
	  	}
	}

}

/*
  this function only send a respone type of data.
  used for communication.
*/
void publishResponse()
{
  	StaticJsonDocument<256> doc_tx;

	doc_tx["id"] 	= ID;
	doc_tx["cmd"]  	= cmd;
	doc_tx["resp"]  = response;

	String json;
	serializeJson(doc_tx, json);
	Serial.println(json);

	publishMqtt((char*) json.c_str());

}

/*
  this function only send a respone type of data.
  used for communication.
*/
void serialResponse()
{
  	StaticJsonDocument<256> doc_tx;

	doc_tx["id"] 	= ID;
	doc_tx["cmd"]  	= cmd;
	doc_tx["resp"]  = response;

	String json;
	serializeJson(doc_tx, json);
	Serial.println(json);

}


void processCmdSerial()
{
	StaticJsonDocument<1024> doc_rx;
  	//check for error
  	DeserializationError error_rx = deserializeJson(doc_rx, Serial);
  	if (error_rx)
  	{
    	Serial.println("testing");
    	Serial.print(F("deserializeJson() failed: "));
    	Serial.println(error_rx.c_str());
    	return;
  	}

	Serial.println("prosesing cmd from Serial");

	//parsing incoming msg
    rcvd_id = doc_rx["id"];
    JsonObject arg_js = doc_rx["arg"];

    cmd 	= doc_rx["cmd"];
 	rcvd_ch = doc_rx["ch"];

    //prossesing incoming command
    if(strcmp(cmd,"rst")==0)
    {
        response = RESPONSE_OK;
        serialResponse();
 		ESP.restart();
	}
	else if(strcmp(cmd,"v")==0)
	{
		response = VERSION;
		serialResponse();
	}
	else if(strcmp(cmd,"trigger")==0)// set diameter to motor
	{
		activatePWMChannel( rcvd_ch , pulseparameters[rcvd_ch].freq, pulseparameters[rcvd_ch].duration);
		response = RESPONSE_OK;
		serialResponse();
	}
	else if(strcmp(cmd,"set")==0)// set diameter to motor
	{
		int _freq  	= arg_js["freq"];
		int _amp 	= arg_js["amp"];
		int _duty 	= arg_js["duty"];
		int _pulse 	= arg_js["pulse"];

		pulseparameters[rcvd_ch].freq 		= _freq;
		pulseparameters[rcvd_ch].amp 		= _amp;
		pulseparameters[rcvd_ch].duty 		= _duty;
		pulseparameters[rcvd_ch].duration 	= _pulse;

		savePreset(rcvd_ch);
		response = RESPONSE_OK;
		serialResponse();
	}
	else
	{
      	Serial.println("Command not valid");
		response = RESPONSE_ERROR_CMD;
		serialResponse();
	}
	cmd = "";
}


/*
  Parse the received json message and procces the recived commands
*/
void processCmd(byte* payload, unsigned int length)
{
	StaticJsonDocument<1024> doc_rx;
    //const char* json_rx = "{\"id\":\"anw00\",\"cmd\":\"gps\",\"arg\":1}";
    DeserializationError error_rx;
    //check for error
    error_rx = deserializeJson(doc_rx, payload,length);
    if (error_rx)
    {
		Serial.print(F("deserializeJson() failed: "));
		response = RESPONSE_ERROR_JSON;
		publishResponse();
      	Serial.println(error_rx.c_str());
    }
	Serial.println("Prosesing cmd from mqtt");

    //parsing incoming msg
    rcvd_id = doc_rx["id"];

    if( strcmp(rcvd_id,ID) == 0 )
    {
		JsonObject arg_js = doc_rx["arg"];

      	cmd 	= doc_rx["cmd"];
		rcvd_ch = doc_rx["ch"];

      	//prossesing incoming command
      	if(strcmp(cmd,"rst")==0)
      	{
        	response = RESPONSE_OK;
        	publishResponse();
			ESP.restart();
      	}
      	else if(strcmp(cmd,"v")==0)
      	{
			response = VERSION;
        	publishResponse();

      	}
		else if(strcmp(cmd,"trigger")==0)// set diameter to motor
      	{
			activatePWMChannel( rcvd_ch , pulseparameters[rcvd_ch].freq, pulseparameters[rcvd_ch].duration);
			response = RESPONSE_OK;
        	publishResponse();
      	}
		else if(strcmp(cmd,"set")==0)// set diameter to motor
      	{

			int _freq  	= arg_js["freq"];
			int _amp 	= arg_js["amp"];
			int _duty 	= arg_js["duty"];
			int _pulse 	= arg_js["pulse"];

			pulseparameters[rcvd_ch].freq 	= _freq;
			pulseparameters[rcvd_ch].amp 	= _amp;
			pulseparameters[rcvd_ch].duty 	= _duty;
			pulseparameters[rcvd_ch].duration 	= _pulse;

			savePreset(rcvd_ch);
			response = RESPONSE_OK;
        	publishResponse();
      	}

      	else
      	{
        	Serial.println("Command not valid");
			response = RESPONSE_ERROR_CMD;
			publishResponse();
      	}
      	cmd = "";

    }
    else
    {
      Serial.println("msg not for me");
    }
}

void IRAM_ATTR keyChanged1() { triggered[0] = true; }
void IRAM_ATTR keyChanged2() { triggered[1] = true; }
void IRAM_ATTR keyChanged3() { triggered[2] = true; }
void IRAM_ATTR keyChanged4() { triggered[3] = true; }
void IRAM_ATTR keyChanged5() { triggered[4] = true; }
void IRAM_ATTR keyChanged6() { triggered[5] = true; }
