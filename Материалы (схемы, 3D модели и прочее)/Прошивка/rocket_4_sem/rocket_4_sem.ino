#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>
#include "math.h"


float g = 9.8155;
float R = 8.3145;
float mu = 0.002898;

static const int PIN_CHIP_SELECT = 17;
static const int GPSrx = 9, GPStx = 8, GSMrx = 1, GSMtx = 0;
static const int RED_LED_PIN = 21;
static const int GREEN_LED_PIN = 20;
static const int BUZZER_PIN = 22;

static const uint32_t GPSBaud = 9600;
static const uint32_t SerialBaud = 9600;
static const uint32_t GSMBaud = 9600;
int16_t ax, ay, az;
int16_t gx, gy, gz;
String responseAT = "";  // для хранения ответа от GSM
String _response;
String responseTextMode = ""; 
bool flagGSM = 1; // флаг для отправки сообщений о координатах 
bool FlagGreenLed;
bool FlagRedLed;
bool flagLastSMS = 1;
bool flagHmax = false;
bool flagSentHmax = false;
bool flagMillis = true;

char sms[300];
char sd[300];
char maxHsms[50];
int decLat;
int intLat;
int intLng;
int decLng;
int decT;
int intT;
int P0, p, p1, p2, p3;
float T1, T2, T3;
int intHmax, decHmax;
int timerP1, timerP2, timerP3;



// Настройки работы БК
int CntSMS = 0;
int TotalNumberSMS = 5;
int timerSendCoord;
int PeriodSMS = 1000;
int timeWorkBK = 40000;
int PeriodWriteSD = 100;
int PeriodBuzzer = 1000;
int frequencyBuzzer = 1500;
int timeBuzzer = 1000;



TinyGPSPlus gps;
MPU6050 accelgyro;
Adafruit_BMP085 bmp;

SoftwareSerial ss(GPSrx, GPStx);
SoftwareSerial SIM800(GSMrx, GSMtx);        

int TimeStartGPS;
int timeDisplayInfo;
int TimeGSM;
int timeSD;
int timeStopSD;
int timeRedLed;
int timeGreenLed;
int timeStartGSM;
int timeSendLastCoord;
int timerBuzzer;

void setup() {
  Serial.begin(SerialBaud);
  //while(!Serial);
  
  Wire.begin();

  // свеодиоды
  pinMode(RED_LED_PIN, OUTPUT);  
  pinMode(GREEN_LED_PIN, OUTPUT);  


  //sim800l
  /*timeRedLed = millis();
  FlagRedLed = false;
  SIM800.begin(GSMBaud);
  
  while(SIM800.available() <= 0)
  {
    if (millis() - timeRedLed >= 500)
    {
      Serial.println("Нет подключения к SIM800l");
      digitalWrite(RED_LED_PIN, FlagRedLed);
      digitalWrite(GREEN_LED_PIN, !FlagRedLed); 
      timeRedLed = millis();
      FlagRedLed = !FlagRedLed;
    }
  }*/  
  
  SIM800.begin(GSMBaud);
  
  //ждем 15 сек, чтоб gsm заработал
  int timerStartBK = millis();
  while(millis() - timerStartBK <= 5000)
  {;}      

  responseAT = sendATCommand("AT", true);  
  responseAT.trim();                       // Убираем пробельные символы в начале и конце

  responseTextMode = sendATCommand("AT+CMGF=1;&W", true); // Включаем текстовый режима SMS (Text mode) и сразу сохраняем значение (AT&W)!
  responseTextMode.trim();

   sendSMS("+79646360100", "GSM is connected");

  /*
  // Если GSM отвечает, мигаем в течение 5 с зеленым светодиодом раз в 500мс. Иначе красным и зеленым
  timeRedLed = millis();
  FlagRedLed = false;
  do {
    responseAT = sendATCommand("AT", true);  
    responseAT.trim();                       // Убираем пробельные символы в начале и конце

    responseTextMode = sendATCommand("AT+CMGF=1;&W", true); // Включаем текстовый режима SMS (Text mode) и сразу сохраняем значение (AT&W)!
    responseTextMode.trim();
    
    if (millis() - timeRedLed >= 200)
    {
      Serial.println("Нет подключения к SIM800l");
      Serial.print("Ответ GSM (AT, TextMode): ");
      Serial.print(responseAT);
      Serial.println(responseTextMode);
      digitalWrite(RED_LED_PIN, FlagRedLed);
      digitalWrite(GREEN_LED_PIN, !FlagRedLed); 
      timeRedLed = millis();
      FlagRedLed = !FlagRedLed;
    }
    
  } while (responseAT != "OK" && responseTextMode != "OK");              // Не пускать дальше, пока модем не вернет ОК
  
  Serial.print("Ответ GSM (AT, TextMode): ");
  Serial.print(responseAT);
  Serial.println(responseTextMode);

  digitalWrite(RED_LED_PIN, false);
  digitalWrite(GREEN_LED_PIN, false);

  
 
    if (responseAT == "OK" && responseTextMode == "OK")
    {
       timeGreenLed = millis();
       timeStartGSM = millis();
       FlagGreenLed = false;
      Serial.println("SIM800l работает");
      while(millis() - timeStartGSM <= 3000)
      {
        if (millis() - timeGreenLed >= 300)
        {
          digitalWrite(GREEN_LED_PIN, FlagGreenLed); 
          timeGreenLed = millis();
          FlagGreenLed = !FlagGreenLed;
        }
        
      } 

    }

digitalWrite(RED_LED_PIN, false);
digitalWrite(GREEN_LED_PIN, false);
 */   
  
  

 /* else 
  {
    timeRedLed = millis();
    FlagRedLed = false;
    SIM800.begin(GSMBaud);

    while(1)
    {
      if (millis() - timeRedLed >= 500)
      {
        Serial.println("Нет подключения к SIM800l");
        digitalWrite(RED_LED_PIN, FlagRedLed);
        digitalWrite(GREEN_LED_PIN, !FlagRedLed); 
        timeRedLed = millis();
        FlagRedLed = !FlagRedLed;
      } 
  
    }
  }*/

  

  // initialize bmp085
  timeRedLed = millis();
   FlagRedLed = false;
  // мигаем красным светодиодом раз в 1с
  while(!bmp.begin())
  {
    if (millis() - timeRedLed >= 1000)
    {
      Serial.println("Нет подключения к BMP");
      digitalWrite(RED_LED_PIN, FlagRedLed); 
      timeRedLed = millis();
      FlagRedLed = !FlagRedLed;
    }
  }
    
  digitalWrite(RED_LED_PIN, false);
  digitalWrite(GREEN_LED_PIN, false);
  
  // initialize mpu6050
  accelgyro.initialize();   
   timeRedLed = millis();
   FlagRedLed = false;
  // мигаем красным светодиодом раз в 0.5с
  while(!accelgyro.testConnection())
  {
    if (millis() - timeRedLed >= 500)
    {
      Serial.println("Нет подключения к MPU");
      digitalWrite(RED_LED_PIN, FlagRedLed); 
      timeRedLed = millis();
      FlagRedLed = !FlagRedLed;
    }
  }
  
  digitalWrite(RED_LED_PIN, false);
  digitalWrite(GREEN_LED_PIN, false);

 // initialize SD
  timeRedLed = millis();
  FlagRedLed = false;
 // Этот пин обязательно должен быть определен как OUTPUT
  pinMode(PIN_CHIP_SELECT, OUTPUT); 
  // мигаем красным светодиодом раз в 3с
  while(!SD.begin(PIN_CHIP_SELECT))
  {
    if (millis() - timeRedLed >= 3000)
    {
      Serial.println("Нет подключения к SD");
      digitalWrite(RED_LED_PIN, FlagRedLed); 
      timeRedLed = millis();
      FlagRedLed = !FlagRedLed;
    }
  }
    
   digitalWrite(RED_LED_PIN, false);
  digitalWrite(GREEN_LED_PIN, false); 


 // initialize GPS    
  
 ss.begin(GPSBaud);
    
  
 timeRedLed = millis();
 FlagRedLed = false;
 while(ss.available() <= 0 || !gps.encode(ss.read()) || !gps.location.isValid())
{
  if (millis() - timeRedLed >= 500)
  {
    Serial.println("Нет подключения к GPS");
    digitalWrite(RED_LED_PIN, FlagRedLed);
    digitalWrite(GREEN_LED_PIN, FlagRedLed); 
    timeRedLed = millis();
    FlagRedLed = !FlagRedLed;
  } 
}



digitalWrite(RED_LED_PIN, false);
digitalWrite(GREEN_LED_PIN, false);

  
  int timerReadyBK = millis();
  timeGreenLed = millis();
  timeStartGSM = millis();
  FlagGreenLed = false;
  Serial.println("Все работает");
  while(millis() - timerReadyBK <= 10000)
  {
    if (millis() - timeGreenLed >= 1000)
    {
      digitalWrite(GREEN_LED_PIN, FlagGreenLed); 
      timeGreenLed = millis();
      FlagGreenLed = !FlagGreenLed;
    }
  }    

   //начальное давлние
   P0 = bmp.readPressure();

  TimeStartGPS = millis();
  timeDisplayInfo = millis();
  TimeGSM = millis();
  timeSD = millis();
  timerSendCoord =  millis();
  timeSendLastCoord =  millis();
  timerBuzzer = millis();
  timerP1 = millis();
  timerP2 = millis();
  timerP3 = millis();

  digitalWrite(RED_LED_PIN, false);
  digitalWrite(GREEN_LED_PIN, false);

 

}


void loop() {

  if (flagMillis == 1)
  {
    timeStopSD = millis();
    flagMillis = 0;
    
  }

while(ss.available() > 0)
  gps.encode(ss.read());

/*
// ОТЛАДКА РАБОТЫ GSM

  if (SIM800.available())   {                   // Если модем, что-то отправил...
    _response = waitResponse();                 // Получаем ответ от модема для анализа
    _response.trim();                           // Убираем лишние пробелы в начале и конце
    Serial.println(_response);                  // Если нужно выводим в монитор порта
    //....
    if (_response.startsWith("+CMGS:")) {       // Пришло сообщение об отправке SMS
      int index = _response.lastIndexOf("\r\n");// Находим последний перенос строки, перед статусом
      String result = _response.substring(index + 2, _response.length()); // Получаем статус
      result.trim();                            // Убираем пробельные символы в начале/конце

      if (result == "OK") {                     // Если результат ОК - все нормально
        Serial.println ("Message was sent. OK");
      }
      else {                                    // Если нет, нужно повторить отправку
        Serial.println ("Message was not sent. Error");
      }
    }
  }
  if (Serial.available())  {                    // Ожидаем команды по Serial...
    SIM800.write(Serial.read());                // ...и отправляем полученную команду модему
  };



  

// TotalNumberSMS - количество смс, отправленных с периодичностью 1000 мс
  if (millis() - timerSendCoord >= PeriodSMS && CntSMS < TotalNumberSMS)
  {
    //отправляем смс   
    if (gps.location.isValid() && gps.time.isValid())
    {
      if (flagHmax == true && flagSentHmax == false)
      {
        splitFloat(gps.location.lat(), 6, intLat, decLat);
        splitFloat(gps.location.lng(), 6, intLng, decLng);
        sprintf(sms, "lat = %d.%d lng = %d.%d time = %d:%d:%d.%d Hmax = %d.%d m", intLat, decLat, intLng, decLng, gps.time.hour(), gps.time.minute(), 
        gps.time.second(), gps.time.centisecond(), intHmax, decHmax);
        sendSMS("+79646360100", sms);
        flagSentHmax = true;
      }
      else
      {
        splitFloat(gps.location.lat(), 6, intLat, decLat);
        splitFloat(gps.location.lng(), 6, intLng, decLng);
        sprintf(sms, "lat = %d.%d lng = %d.%d time = %d:%d:%d.%d", intLat, decLat, intLng, decLng, gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
        sendSMS("+79646360100", sms);
      }
    }
    else
    {
      sendSMS("+79646360100", "INVALID");
    }

    CntSMS = CntSMS + 1;
    timerSendCoord = millis();
  }
*/

  //Отправляем одну смс, чтоб нашли борткомп
  if (millis() - timeSendLastCoord >= 40000 && flagLastSMS == 1)
  {
    //отправляем смс   
    if (gps.location.isValid() && gps.time.isValid())
    {
      splitFloat(gps.location.lat(), 6, intLat, decLat);
      splitFloat(gps.location.lng(), 6, intLng, decLng);
      sprintf(sms, "last coord: lat = %d.%d lng = %d.%d time = %d:%d:%d.%d", intLat, decLat, intLng, decLng, gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
      sendSMS("+79646360100", sms);
    }
    else
    {
      sendSMS("+79646360100", "INVALID");
    }

    flagLastSMS = 0;
  }


// ПИЩАЛКА

if (millis() - timeSendLastCoord >= 40000 && millis() - timerBuzzer >=  PeriodBuzzer)
{
   tone(BUZZER_PIN, frequencyBuzzer, timeBuzzer);
   timerBuzzer = millis();
}


/*
  // ОТПРАВКА СМС НА МАКСИМАЛЬНОЙ ВЫСОТЕ
  if (millis() - timerP1 >= 60)
  {
    T1 = bmp.readTemperature();
    for (int i=0; i<10; i++)
    {
    p = bmp.readPressure();
    p1 = p1 + p;
    }
    timerP1 = millis();
  }

  if (millis() - timerP2 >= 80)
  {
    T2 = bmp.readTemperature();
    for (int i=0; i<10; i++)
    {
    p = bmp.readPressure();
    p2 = p2 + p;
    }
    timerP2 = millis();
  }

   if (millis() - timerP3 >= 100)
  {
    T3 = bmp.readTemperature();
    for (int i=0; i<10; i++)
    {
    p = bmp.readPressure();
    p3 = p3 + p;
    }
    timerP3 = millis();
  }
 
  
  if (p2>p1 && p3>p2)
  {
     int P1 = p1/10;
     float Hmax = log (P0/P1)*R*T1/mu/g;
     splitFloat(Hmax, 2, intHmax, decHmax);   
     flagHmax = true;
  }
    
    
*/



  // SD КАРТА

  if (millis() - timeStopSD <= timeWorkBK && millis() - timeSD >= PeriodWriteSD)
  {
      // Если файла с таким именем не будет, ардуино создаст его.
    File dataFile = SD.open("DATA_last.csv", FILE_WRITE);
  
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    splitFloat(bmp.readTemperature(), 1, intT, decT);
    int P = bmp.readPressure();

    // запись координаты 
    if (gps.location.isValid() && gps.time.isValid())
    {
      splitFloat(gps.location.lat(), 6, intLat, decLat);
      splitFloat(gps.location.lng(), 6, intLng, decLng);
      sprintf(sd, "%d.%d\t%d.%d\t%d:%d:%d.%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d.%d", intLat, decLat, intLng, decLng, gps.time.hour(), 
              gps.time.minute(), gps.time.second(), gps.time.centisecond(), ax, ay, az, gx, gy, gz, P, intT, decT);
      
    }
    else
    {sprintf(sd, "INVAILD \t INVAILD \t INVAILD \t%d\t%d\t%d\t%d\t%d\t%d\t%d%d.%d", ax, ay, az, gx, gy, gz, P, intT, decT);
    }
     
    
    // Если все хорошо, то записываем строку:
    if (dataFile) {
      dataFile.println(sd);
      dataFile.close();
      // Публикуем в мониторе порта для отладки
      Serial.print("SD: ");
      Serial.println(sd);
    }
    else {
    // Сообщаем об ошибке, если все плохо
      Serial.println("error opening datalog.csv");
    }

    timeSD = millis();    
  }
  
  
}


void splitFloat(float num, int precision, int &integerPart, int &decimalPart) {
  integerPart = int(num);
  float remainder = num - integerPart;
  decimalPart = int(remainder * pow(10, precision)); // Добавлено 0.5 для корректного округления
  
}


void displayInfo()
{
  // вывод координат
  if (gps.location.isValid())
  {
    // МИГАЕМ ЗЕЛЕНЫМ СВЕТОДИОДОМ, ДАННЫЕ ВСЕ БУДУТ
    Serial.print(gps.location.lat(), 6);
    Serial.print(F("\t"));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  //вывыод даты и времени
  Serial.print(F("\t"));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("\t"));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }



  // температура в *С и давление в Па
  Serial.print(F("\t"));
  Serial.print(bmp.readTemperature());
  Serial.print(F("\t"));
  Serial.print(bmp.readPressure());

  // данные с MPU
  Serial.print(F("\t"));
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  }



  String sendATCommand(String cmd, bool waiting) {
  String _resp = "";                            // Переменная для хранения результата
  Serial.println(cmd);                          // Дублируем команду в монитор порта
  SIM800.println(cmd);                          // Отправляем команду модулю
  if (waiting) {                                // Если необходимо дождаться ответа...
    _resp = waitResponse();                     // ... ждем, когда будет передан ответ
    // Если Echo Mode выключен (ATE0), то эти 3 строки можно закомментировать
    if (_resp.startsWith(cmd)) {  // Убираем из ответа дублирующуюся команду
     _resp = _resp.substring(_resp.indexOf("\r", cmd.length()) + 2);
    }
    Serial.println(_resp);                      // Дублируем ответ в монитор порта
  }
  return _resp;                                 // Возвращаем результат. Пусто, если проблема
}

String waitResponse() {                         // Функция ожидания ответа и возврата полученного результата
  String _resp = "";                            // Переменная для хранения результата
  long _timeout = millis() + 10000;             // Переменная для отслеживания таймаута (10 секунд)
  while (!SIM800.available() && millis() < _timeout)  {}; // Ждем ответа 10 секунд, если пришел ответ или наступил таймаут, то...
  if (SIM800.available()) {                     // Если есть, что считывать...
    _resp = SIM800.readString();                // ... считываем и запоминаем
  }
  else {                                        // Если пришел таймаут, то...
    Serial.println("Timeout...");               // ... оповещаем об этом и...
    _resp = "no answer";
  }
  return _resp;                                 // ... возвращаем результат. Пусто, если проблема
}


void sendSMS(String phone, String message)
{
  sendATCommand("AT+CMGS=\"" + phone + "\"", true);             // Переходим в режим ввода текстового сообщения
  sendATCommand(message + "\r\n" + (String)((char)26), true);   // После текста отправляем перенос строки и Ctrl+Z
}
