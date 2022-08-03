/* 
Sistema de Vigilancia casera para adultos mayores

Realizo: 
    José Lino Carrillo Villalobos, 
    Arturo Javier López Fausto y 
    Julio Cesar Ortiz Cornejo.

Fecha: 18 Julio 2022

Como parte del proyecto Final del Diplomado: Internet de las Cosas de Samsung Innovation Campus
impartido por: Codigo IoT (codigoiot.com)


Material en el GitHub : https://github.com/LinoCarrillo/proyectoCapstone


*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <MQ135.h>
#include  <DHT.h>
#include <PubSubClient.h> //Biblioteca para conexion MQTT

// Definiciones de Variables a Utilizar

//CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN 4
// generación de los pines con los que trabajaremos con los sensores
#define mq6_Pin 2  // Esta variable indica el pin en el que se conecta el sensor MQ-6
#define mq135_Pin 12  // Esta variable indica el pin en el que se conecta el sensor MQ-135
#define dht_Pin 15 // Esta variable indica el pin en el que se conecta el sensor DHT22
#define pinon 14 //Botón de Pánico
#define DHTTYPE DHT22   // Sensor DHT22

// Manejo de Variables a Utilizar

//Configuración de la Red a utilizar
const char* ssid = "AQUI VA EL NOMBRE DE TU RED ó SSID";
const char* password = "AQUI VA EL PASSWORD DE TU RED";
const int INTERVAL = 3000;  // Intervalo de espera entre lecturas
String chatId = "AQUI VA EL CHATID PERSONAL DE TELEGRAM";  //Chat ID para el manejo de Bot Personal, consulta la documentacion para ver como se genera
//String chatId = "-AQUI VA EL CHAT-ID DE UN GRUPO DE TELEGRAM";  // Chat ID para el manejo de Bot en un Grupo Previamente Creado, consulta la documentacion para ver como se genera

// Initialize Telegram BOT
String BOTtoken = "AQUI VA TU TOKEN DE TELEGRAM"; // Revisa la documentación para ver como se obtiene
bool sendPhoto = false;
//Datos del broker MQTT
const char* mqtt_server = "###.###.###.###"; // Si estas en una red local, coloca la IP asignada (la que muestre el sistema), en caso contrario, coloca la IP publica


// Objetos
IPAddress server(###,###,###,###); misma dirección IP que la linea anterior, separa por comas (,)


WiFiClient espClient; // Este objeto maneja los datos de conexion WiFi
PubSubClient client(espClient); // Este objeto maneja los datos de conexion al broker
WiFiClientSecure clientTCP;
DHT dht(dht_Pin, DHTTYPE);

UniversalTelegramBot bot(BOTtoken, clientTCP);

bool flashState = LOW;
// Motion Sensor
bool motionDetected = false;
int botRequestDelay = 1000;   // mean time between scan messages
long lastTimeBotRan;     // last time messages' scan has been done
int ValorSensorMq6=0;
int ValorSensorMq135=0;
int ValorSensorDht_1=0;
int ValorSensorDht_2=0;
String readings2="";
void handleNewMessages(int numNewMessages);
String sendPhotoTelegram();


// Indicates when motion is detected
static void IRAM_ATTR detectsMovement(void * arg){
  //Serial.println("MOTION DETECTED!!!");
  motionDetected = true;
}

int flashLedPin = 4;  // Para indicar el estatus de conexión
//int statusLedPin = 16; // Para ser controlado por MQTT
long timeNow, timeLast; // Variables de control de tiempo no bloqueante
int data = 0; // Contador
int estaon = LOW; //Para botón de pánico
int wait = 3000;  // Indica la espera cada 5 segundos para envío de mensajes MQTT
//============================== Void Setup  =====================
void setup()
{
  Serial.begin(115200);
  dht.begin();
  pinMode(dht_Pin, INPUT);
  pinMode(mq6_Pin,INPUT); // Se configura el pin 2 como entrada
  pinMode(mq135_Pin,INPUT); // Se configura el pin 16 como entrada
  pinMode (flashLedPin, OUTPUT);
  pinMode(pinon, INPUT);  //para el botón de Pánico
 // pinMode (statusLedPin, OUTPUT);
  digitalWrite (flashLedPin, LOW);
  //digitalWrite (statusLedPin, HIGH);
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState);
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Conectando a la red: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED) 
  {
//    digitalWrite (statusLedPin, HIGH);
 //   delay(500); //dado que es de suma importancia esperar a la conexión, debe usarse espera bloqueante
//    digitalWrite (statusLedPin, LOW);
    Serial.print(".");  // Indicador de progreso
    delay (1000);
  }
  Serial.println();
  Serial.println("WiFi conectado");
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());
  // Si se logro la conexión, encender led
 
  //ojo 
  if (WiFi.status () > 0){
//  digitalWrite (statusLedPin, LOW);
  }

  // Conexión con el broker MQTT
  client.setServer(server, 1883); // Conectarse a la IP del broker en el puerto indicado
  client.setCallback(callback); // Activar función de CallBack, permite recibir mensajes MQTT y ejecutar funciones a partir de ellos
  delay(1500);  // Esta espera es preventiva, espera a la conexión para no perder información

  timeLast = millis (); // Inicia el control de tiempo

  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if(psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } 
  else 
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) 
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(500);
    ESP.restart();
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA

  // PIR Motion Sensor mode INPUT_PULLUP
  //err = gpio_install_isr_service(0); 
  err = gpio_isr_handler_add(GPIO_NUM_13, &detectsMovement, (void *) 13);  
  if (err != ESP_OK)
  {
    Serial.printf("handler add failed with error 0x%x \r\n", err); 
  }
  err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
  if (err != ESP_OK)
  {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }
  delay(INTERVAL);
}//Final Void Setup
// =================  Void Loop  ==================
void loop(){
   estaon = digitalRead(pinon);

    if (estaon == HIGH) {
    // ENTONCES ENCENDEMOS EL LED
      delay(100);
      digitalWrite(flashLedPin, HIGH);
     int B_P=1; 
     Serial.println("Botón de Pánico Presionado");
     bot.sendMessage(chatId, "Se ha Presionado el Botón de Pánico Favor de comunicarse con sus Familiares :","");
     char dataString[8]; // Define una arreglo de caracteres para enviarlos por MQTT, especifica la longitud del mensaje en 8 caracteres
     dtostrf(B_P, 1, 2, dataString);  // Esta es una función nativa de leguaje AVR que convierte un arreglo de caracteres en una variable String
     Serial.print("Se Presiono el Botón de Pánico: "); // Se imprime en monitor solo para poder visualizar que el evento sucede
     Serial.println(dataString);
     client.publish("CodigoIoT/SIC/G5/B_P", dataString); // Esta es la función que envía los datos por MQTT, especifica el tema y el valor


  }
  else
  {
        // ENTONCES APAGAMOS EL LED

      digitalWrite(flashLedPin, LOW);

    }
  
  // Contro de la Temperatura y Humedad  Sensor DHT 22
  
  float h = dht.readHumidity(); //Leemos la Humedad
  float t = dht.readTemperature(); //Leemos la temperatura en grados Celsius
  ValorSensorDht_2=h;
  ValorSensorDht_1=t;      
  Serial.print("Humedad ");
  Serial.print(h);
  Serial.print(" %t  ");
  Serial.print("Temperatura: ");
  Serial.print(t);
  Serial.println(" *C ");
  if ((ValorSensorDht_1>=28)||(ValorSensorDht_2>=60))
     {
        bot.sendMessage(chatId, "Se han Detectado Temperatura o humedad altas en la Recamara Principal:","");
        char dataString[8]; // Define una arreglo de caracteres para enviarlos por MQTT, especifica la longitud del mensaje en 8 caracteres
        float temp=t;
        dtostrf(t, 1, 2, dataString);  // Esta es una función nativa de leguaje AVR que convierte un arreglo de caracteres en una variable String
        Serial.print("Temperatura: "); // Se imprime en monitor solo para poder visualizar que el evento sucede
        Serial.println(dataString);
        client.publish("CodigoIoT/SIC/G5/temp", dataString); // Esta es la función que envía los datos por MQTT, especifica el tema y el valor

        delay (100);
        dtostrf(h, 1, 2, dataString);
        Serial.print("Humedad: "); // Se imprime en monitor solo para poder visualizar que el evento sucede
        Serial.println(dataString);
        client.publish("CodigoIoT/SIC/G5/hum", dataString);
     }

 // Contro de la Calidad del Aire  Sensor MQ 135
  ValorSensorMq135 = digitalRead(mq135_Pin); //Leemos la terminal digital "12" del sensor
 
  if(ValorSensorMq135==1) //si la salida del sensor es 1
  {
   Serial.println("Sin presencia de gases en el ambiente");
   //Serial.println(mq135_Pin);
  }
  else //si la salida del sensor es 0
  {
   int MQ_135=ValorSensorMq135; 
   Serial.println("Gases detectados en el ambiente");
   bot.sendMessage(chatId, "Se han detectado Gases en el ambiente Favor de avisar a los Familiares :","");
   char dataString[8]; // Define una arreglo de caracteres para enviarlos por MQTT, especifica la longitud del mensaje en 8 caracteres
   dtostrf(MQ_135, 1, 2, dataString);  // Esta es una función nativa de leguaje AVR que convierte un arreglo de caracteres en una variable String
   Serial.print("Gases en el Ambiente: "); // Se imprime en monitor solo para poder visualizar que el evento sucede
   Serial.println(dataString);
   client.publish("CodigoIoT/SIC/G5/MQ_135", dataString); // Esta es la función que envía los datos por MQTT, especifica el tema y el valor

   //Serial.println(mq135_Pin);   
  }

  // Control de la busqueda de Gas Butano Sensor MQ 6
  ValorSensorMq6=digitalRead(mq6_Pin);
  int MQ_6=ValorSensorMq6;
  //Reporta por serial
  Serial.println("Lectura de gas proveniente del sensor MQ6");
  if (ValorSensorMq6==1)
        {
          Serial.print("No se ha detectado Gas proveniente de la Estufa ");  
          Serial.println(digitalRead(mq6_Pin));  
        }
        else
        {
         int MQ_6=ValorSensorMq6;
         Serial.print("Se ha detectado Gas proveniente de la Estufa");  
         Serial.println(digitalRead(mq6_Pin)); 
         char dataString[8]; // Define una arreglo de caracteres para enviarlos por MQTT, especifica la longitud del mensaje en 8 caracteres
         dtostrf(MQ_6, 6, 2, dataString);  // Esta es una función nativa de leguaje AVR que convierte un arreglo de caracteres en una variable String
         Serial.print("Gases LP en el Ambiente: "); // Se imprime en monitor solo para poder visualizar que el evento sucede
         Serial.println(dataString);
         client.publish("CodigoIoT/SIC/G5/MQ_6", dataString); // Esta es la función que envía los datos por MQTT, especifica el tema y el valor
 
        }
  

  if (ValorSensorMq6==0)
        {
          bot.sendMessage(chatId, "Se ha detectado Gas Proveniente de la estufa:","");
        }
 
  //sensorValue=0;
  delay (INTERVAL);  
  
  if (sendPhoto)
  {
    Serial.println("Preparing photo");
    sendPhotoTelegram(); 
    sendPhoto = false; 
  }

  if(motionDetected){
    bot.sendMessage(chatId, "Movimiento Detectado!!", "");
    Serial.println("Movimiento Detectado");
    sendPhotoTelegram();
    motionDetected = false;
  }
  
  if (millis() > lastTimeBotRan + botRequestDelay)
  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages)
    {
      Serial.println("Esperando Respuesta");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
  //Verificar siempre que haya conexión al broker
  if (!client.connected()) {
    reconnect();  // En caso de que no haya conexión, ejecutar la función de reconexión, definida despues del void setup ()
  }// fin del if (!client.connected())
  client.loop(); // Esta función es muy importante, ejecuta de manera no bloqueante las funciones necesarias para la comunicación con el broker
  
  timeNow = millis(); // Control de tiempo para esperas no bloqueantes
  
}// fin del void loop ()

// Funciones de usuario

// Esta función permite tomar acciones en caso de que se reciba un mensaje correspondiente a un tema al cual se hará una suscripción
void callback(char* topic, byte* message, unsigned int length) {

  // Indicar por serial que llegó un mensaje
  Serial.print("Llegó un mensaje en el tema: ");
  Serial.print(topic);

  // Concatenar los mensajes recibidos para conformarlos como una varialbe String
  String messageTemp; // Se declara la variable en la cual se generará el mensaje completo  
  for (int i = 0; i < length; i++) {  // Se imprime y concatena el mensaje
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  // Se comprueba que el mensaje se haya concatenado correctamente
  Serial.println();
  Serial.print ("Mensaje concatenado en una sola variable: ");
  Serial.println (messageTemp);

  // En esta parte puedes agregar las funciones que requieras para actuar segun lo necesites al recibir un mensaje MQTT

  // Ejemplo, en caso de recibir el mensaje true - false, se cambiará el estado del led soldado en la placa.
  // El ESP323CAM está suscrito al tema esp/output
  if (String(topic) == "joselinoCarrillo270271") {  // En caso de recibirse mensaje en el tema esp32/output
    if(messageTemp == "true"){
      Serial.println("Led encendido");
      digitalWrite(flashLedPin, HIGH);
    }// fin del if (String(topic) == "esp32/output")
    else if(messageTemp == "false"){
      Serial.println("Led apagado");
      digitalWrite(flashLedPin, LOW);
    }// fin del else if(messageTemp == "false")
  }// fin del if (String(topic) == "esp32/output")
}// fin del void callback

// Función para reconectarse
void reconnect() {
  // Bucle hasta lograr conexión
  while (!client.connected()) { // Pregunta si hay conexión
    Serial.print("Tratando de contectarse...");
    // Intentar reconexión
    if (client.connect("ESP32CAMClient")) { //Pregunta por el resultado del intento de conexión
      Serial.println("Conectado");
      client.subscribe("joselinoCarrillo270271"); // Esta función realiza la suscripción al tema
    }// fin del  if (client.connect("ESP32CAMClient"))
    else {  //en caso de que la conexión no se logre
      Serial.print("Conexion fallida, Error rc=");
      Serial.print(client.state()); // Muestra el codigo de error
      Serial.println(" Volviendo a intentar en 5 segundos");
      // Espera de 5 segundos bloqueante
      delay(5000);
      Serial.println (client.connected ()); // Muestra estatus de conexión
    }// fin del else
  }// fin del bucle while (!client.connected())
}// fin de void reconnect(




//Funciones para el manejo de los sensores y la conexión con Telegram
String sendPhotoTelegram()
{
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Captura de camara Fallida");
    delay(1000);
    ESP.restart();
    return "Captura de camara Fallida";
  }  
  
  Serial.println("Connect to " + String(myDomain));

  if (clientTCP.connect(myDomain, 443)) 
  {
    Serial.println("Conexión Realizada con Exito...");
    
    String head = "--ProyectoCapstone\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + chatId + "\r\n--ProyectoCapstone\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--ProyectoCapstone--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
  
    clientTCP.println("POST /bot"+BOTtoken+"/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=ProyectoCapstone");
    clientTCP.println();
    clientTCP.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0;n<fbLen;n=n+1024) 
    {
      if (n+1024<fbLen) 
      {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) 
      {
        size_t remainder = fbLen%1024;
        clientTCP.write(fbBuf, remainder);
      }
    }  
    
    clientTCP.print(tail);
    
    esp_camera_fb_return(fb);
    
    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;
    
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
      while (clientTCP.available()) 
      {
        char c = clientTCP.read();
        if (state==true) getBody += String(c);        
        if (c == '\n') 
        {
          if (getAll.length()==0) state=true; 
          getAll = "";
        } 
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length()>0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody="Conexion a la api.telegram.org fallo.";
    Serial.println("Conexion a la api.telegram.org fallo.");
  }
  return getBody;
}

void handleNewMessages(int numNewMessages)
{
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++)
  {
    // Chat id of the requester
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != chatId){
      bot.sendMessage(chat_id, "Usuario No Autorizado", "");
      continue;
    }
    
    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);

    String fromName = bot.messages[i].from_name;
    
    //Mensajes para controlar el Led Integrado a la Placa ESP32 CAM
    if (text == "/led") {
      flashState = !flashState;
  
      digitalWrite(FLASH_LED_PIN, flashState);
//      for (i=1;10;i++)
//      {
    delay(3000);flashState = !flashState;
//        digitalWrite(FLASH_LED_PIN, flashState);
    //    delay(100);      
//      }
    }
    
    //Mensaje para Tomar Foto de la camara de la Placa ESP 32 CAM
    if (text == "/foto") {
      sendPhoto = true;
      Serial.println("New photo  request");
    }

    //Mensajes para el Sensor MQ-6
      if (text == "/lecturaGas"){
        String readings = String(ValorSensorMq6);
        if (ValorSensorMq6==1)
        {
          bot.sendMessage(chatId, "No se Detecta Gas el valor esta en: ","");
        }
        if (ValorSensorMq6==0)
        {
          bot.sendMessage(chatId, "Se ha detectado Gas el valor esta en: ","");
        }
        bot.sendMessage(chatId, readings, "");
      }
      
       //Mensajes para el Sensor MQ-135
      if (text == "/lecturaCA"){
        String readings = String(ValorSensorMq135);
        if (ValorSensorMq135==1)
        {
          bot.sendMessage(chatId, "Sin Presencia de Gases en el Medio Ambiente, su valor Digital esta en: ","");
        }
        else
        {
          bot.sendMessage(chatId, "CUIDADO, Presencia de Gases en el Medio Ambiente, su valor Digital esta en:","");
        }
        bot.sendMessage(chatId, readings, "");
      }   

      
      //Mensajes para el Sensor DHT22
      if (text == "/lecturaTemHum"){
        String readings = String(ValorSensorDht_1);
        String readings2 = String(ValorSensorDht_2);
        
        if ((ValorSensorDht_1<=27)||(ValorSensorDht_2<=60))
        {
          bot.sendMessage(chatId, "Tempertura y Humedad Adecuada en Recamara Principal, sus valores están en : ","");
        }
        if ((ValorSensorDht_1>27)||(ValorSensorDht_2>60))
        {
          bot.sendMessage(chatId, "Cuidado, Tempertura o Humedad Altas en Recamara Principal, sus valores están en : ","");

        }
        bot.sendMessage(chatId, readings, "");
        bot.sendMessage(chatId, readings2, "");
      }   
    if (text == "/iniciar")
    {
      String welcome = "Bienvenidos al manejo de ESP32-CAM y Telegram bot.\n";
      welcome += "/foto : Para tomar Foto\n";
      welcome += "/led : Encender o apagar LED\n";
      welcome += "/lecturaTemHum : Recibir Lecturas de Tempertura y Humedad del Sensor DHT22\n";     
      welcome += "/lecturaCA : Recibir Lecturas de la Calidad de Aire del Sensor MQ-135\n";
      welcome += "/lecturaGas : Recibir Lecturas del sensorMQ-6\n\n";
      welcome += "Tambien recibiras Lecturas cuando se active el Sensor de movimiento.\n\n";
      bot.sendMessage(chatId, welcome, "Markdown");
    }
  }
}
