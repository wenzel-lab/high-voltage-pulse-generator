#include <SPI.h>
#include <ArduinoJson.h>

#define VSPI 3
#define TRIGGER_PIN 4       // Pin para leer el trigger externo (entrada)
#define GPIO_OUTPUT_PIN 16  // Pin para simular el trigger (salida)

const int VSPI_MISO = 19;
const int VSPI_MOSI = 23;
const int VSPI_SCLK = 18;
const int VSPI_SS = 5;
int dropsIdentified = 0;

static const int spiClk = 1000000;  // 1 MHz
uint32_t dataToSend = 0;  // Variable global para almacenar los datos a enviar

SPIClass vspi = SPIClass(VSPI);

void setup() {

  Serial.begin(9600); // Inicializa la comunicación serial con una velocidad de 9600 baudios
  delay(1000); // Espera para estabilizar el ESP32
  
  pinMode(TRIGGER_PIN, INPUT);    // Configura el pin de trigger como entrada
  pinMode(GPIO_OUTPUT_PIN, OUTPUT);  // Configura el pin de salida para simular el trigger
  pinMode(VSPI_SS, OUTPUT);  // Configurar SS como salida
  
  vspi.setDataMode(SPI_MODE0);    // El reloj es bajo (nivel lógico 0) cuando está inactivo
  vspi.setBitOrder(MSBFIRST);     // Orden de bits más significativo primero
  vspi.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS); // Inicializa el bus SPI

  //Serial.println("Master begin");
  //Serial.flush();
}

 
void loop() {
  //Serial.flush();
  // Verificar si hay datos disponibles en el puerto serial
  if (Serial.available()) {
    // Lee la entrada como una cadena
    String input = Serial.readStringUntil('\n');
    
    // Ignorar entradas vacías o solo con '\n' (que podrían ser bytes de sincronización)
    //if (input.length() == 0 || input == "\n") {
    //  return; // Salir si solo se recibe un byte de sincronización
    //}

    // Limpiar el buffer serial después de la lectura
    Serial.flush();
    
    // Crea un objeto DynamicJsonDocument con el tamaño adecuado
    DynamicJsonDocument doc(1024);
    
    // Deserializa el JSON de la cadena leída
    DeserializationError error = deserializeJson(doc, input);
    
    // Maneja errores de deserialización
    if (error) {
      Serial.print("Error parsing JSON (");
      Serial.print(error.c_str());
      Serial.print(") with input: ");
      Serial.println(input);
      return;
    }
    
    // Extrae los valores del JSON
    int frequency = doc["frequency"];
    int pot_value = doc["pot_value"];
    int pulse_width = doc["pulse_width"];
    String channel_str = doc["channel"];
    
    uint8_t channel = (channel_str == "ALL") ? 0 : channel_str.substring(2).toInt();
    
    // Empaqueta los datos para enviar a través de SPI
    dataToSend = (frequency << 24) | (pot_value << 16) | (pulse_width << 8) | channel;
    
    Serial.print("Data to send: 0x");
    Serial.println(dataToSend, HEX);
  }
  
  digitalWrite(GPIO_OUTPUT_PIN, HIGH);  // Enciende la señal
  //delayMicroseconds(100);
  //delay(1000);

  if(dataToSend != 0 && digitalRead(TRIGGER_PIN) == HIGH){ // triggerActivated && newDataAvailable
    //dropsIdentified++;
    
    unsigned long startTime = micros();
      // Iniciar la comunicación SPI
    vspi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    digitalWrite(VSPI_SS, LOW);
    
    // Envía los datos a través de SPI
    vspi.transfer((dataToSend >> 24) & 0xFF); // Byte más significativo
    vspi.transfer((dataToSend >> 16) & 0xFF);
    vspi.transfer((dataToSend >> 8) & 0xFF);
    vspi.transfer(dataToSend & 0xFF); // Byte menos significativo

    digitalWrite(VSPI_SS, HIGH);
    vspi.endTransaction();

    unsigned long endTime = micros();
    unsigned long transferTime = endTime - startTime;
    Serial.print("SPI Transfer Time: ");
    Serial.println(transferTime);
    // Imprimir los datos enviados por SPI
    //Serial.print("Sended Data (Frequency, Pot_Value, Pulse_width, Channel): 0x");
    //Serial.println(dataToSend, HEX);
    //Serial.print(" | Drops Identified: ");
    //Serial.println(dropsIdentified);

    //dataToSend = 0;
  }
  digitalWrite(GPIO_OUTPUT_PIN, LOW);   // Apaga la señal
  delayMicroseconds(5000);                          
}

