

/*
    Project:  EMACS -- Environmental Monitoring and Control System    
    File:     emacs_full_sensor_node_01.ino    
    Desc:     Customizable TX Node:   PIR with SEN-12642 version
        
    Coded by:     Jacob Romero
    Company:      Creative Engineering Solutions LLC
    email:        cesllc876@gmail.com
    date:         02/05/2025

    Copyright (C) 2025 
    Jacob Romero, Creative Engineering Solutions, LLC
    cesllc876@gmail.com

    EMACS COTS Hardware:    
      nRF24L01      2.4 GHz Transceiver
      DHT-11        Temp and Humidity Sensor
      K30           CO2 Sensor
      PIR Detector  Motion Sensor
      SEN-12642     Audio Sensor
      20x4 LCD      Display
      Arduino       MEGA, UNO or NANO
       
    This is a stable, customizable version of the TX Sensor Node in a
    TX/RX pair with options for all of the sensor types.
    
    This version of the customizable TX Node is set up for an 
    Arduino UNO PIR Motion Sensor with SEN-12642 Audio Sensor. 
    
    Sensor data is transmitted continuously by the
    TX module's nRF24L01 (2.4 GHz transceiver), which
    currently transmits packets of data from up to 4 sensors:
      DHT-11 -- Temp and Humidity Sensor
      K30 -- CO2 Sensor
      PIR -- Passive Infrared Motion Sensor
      SEN12642 -- Audio Sensor

    Credits:
    RF24SensorNet library
    https://github.com/szaffarano/RF24SensorNet
    NRF24L01 library
    TMRh20 https://github.com/TMRh20/RF24

    https://electronics.stackexchange.com/questions/94137
    /nrf24l01-power-amplifier-dbm-options-to-milliwatt
*/
 
//========= Libraries =========//

#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>

//#include <K30_I2C.h>
//#include <DHT.h>
//#include <DHT_U.h>
//#include <Adafruit_Sensor.h>
//#include <LiquidCrystal.h>

#include <printf.h>
#include <SPI.h>
#include "SPI.h"
#include <stdlib.h>
#include <stdio.h> 
#include <time.h>

#define SER_BAUDRATE  9600
#define LF            0x0A    // Line Feed        == 0x0A
#define CR            0x0D    // Carriage Return  == 0x0D
  
//========================
//======= LCD PINS =======
//========================
//#define D2          2 // LCD DB7
//#define D3          3 // LCD DB6
//#define D4          4 // LCD DB5
//#define D5          5 // LCD DB4
//#define EN_LCD      6 // LCD E
//#define RS_LCD      7 // LCD RS

//=============================
//======= nRF24L01 PINS =======
//=============================
/// Enable for UNO or NANO
#define MISO    12
#define MOSI    11
#define SCK     13
#define SS      10
#define CE      9
//#define DHTPIN  8 

#define MSG_PIN     4   // nRF24 interrupt
#define IRQ_MSG_IN  4   // nRF24 IRQ

/*
/// Enable for MEGA
#define MISO 50
#define MOSI 51
#define SCK 52
#define SS 53
#define CE 49

/// MEGA 2560 Interrupt Pins: 2, 3, 18, 19, 20, 21
#define MSG_PIN     19   // nRF24 interrupt
#define IRQ_MSG_IN  19   // nRF24 IRQ
*/
//========================
//======= PIR PINS =======
//========================
#define PIR_PIN       3   // PIR sensor input
#define IRQ_PIR_IN    3   // PIR IRQ
//========================

//==============================
//======= SEN-12642 PINS =======
//==============================
#define PIN_ENV_IN    A0  // SEN-12642 Envelope
#define PIN_AUDIO_IN  A2  // SEN-12642 Audio
#define PIN_GATE_IN   2   // SEN-12642 Gate
#define IRQ_GATE_IN   2   // SEN-12642 IRQ
//=============================

//===========================
//======= DHT-11 PINS =======
//===========================
// Uncomment whatever type of DHT you're using!
//#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)
//===========================

//=================================
//======= Custom Data types =======
//=================================

enum cmd_type {
    nada      = 0,   
    chan_chg  = 1, 
    rad_chg   = 2, 
    pa_chg    = 3,
    all_chg   = 4,
    tar_chg   = 5,
    response  = 6
};

enum PA_type {    // nRF24 Power Amplifier Setting
    PA_MIN  = 0,  // -18 dBm  == 7.0 mA
    PA_LOW  = 1,  // -12 dBm  == 7.5 mA
    PA_HIGH = 2,  // -6 dBm   == 9.0 mA
    PA_MAX  = 3   //  0 dBm   == 11.3 mA
};

enum Alarm_type{  
    ALM_NONE    = 0,
    HUM_LOW     = 1, 
    HUM_HIGH    = 2,
    TEMP_LOW    = 3, 
    CO2_LOW     = 4,  
    TEMP_HIGH   = 5,  
    CO2_HIGH    = 6,  
    CO2_GOOD    = 7, 
    LOUDNOISE   = 8,
    INTRUDER    = 9,   
    CO2_DNG     = 10,
    TEMP_FIRE   = 11 
};

enum Dng_level {  // Danger Level
    DNG_NONE  = 0,   
    DNG_LOW   = 1,
    DNG_MED   = 2,
    DNG_HIGH  = 3,
    DNG_MAX   = 4
};

struct pingCmd{
    cmd_type      my_cmd;
    int           ch_cmd;
    int           r_cmd;
    PA_type       pa_cmd;  
    int           tar_cmd;
    Alarm_type    alm_cmd;
    Dng_level     dng_cmd;
}myPing;

struct dataStruct{
    int     chNum;
    int     radNum;
    int     co2_val;
    int     h_val;
    int     c_val;
    int     f_val;
    int     pir_state;
    int     audio_gate;
    struct  pingCmd ping;
}myData;

/// We are allowed 6 address nodes: 1 for receive, and 5 for transmit
byte addresses[][6] = {"1Node","2Node","3Node","4Node","5Node","6Node"};

//=============================
//==== Instantiate modules ====
//=============================
RF24 radio(CE,SS);
//K30_I2C k30_i2c = K30_I2C(0x68);
//DHT dht(DHTPIN, DHTTYPE);
//LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
//=============================

//=================
//==== Globals ====
//=================

/// Interrupt variables must be volatile
volatile int audio_gate_val;
volatile int pir_pin_val;
volatile int msg_pin_val;
//volatile int alm_pin_val; // SAVE for MEGA development

/// Flags for triggering operations, etc.
bool      intruderFlag  = false;
bool      audioFlag     = false;
bool      respFlag      = false;
bool      almFlag       = false;
bool      dngFlag       = false;
//bool      validCO2Flag  = false;
//bool      rc		= false;

//////////////////////////////////////////
/// These belong in the SentMessage struct

/// Environmental variables
int    chanNum   = 0;
int    rNum      = 0;
int    co2Num    = 450;
int    pirNum    = 0;
int    gateNum   = 0;
float  hNum      = 40;
float  tNum      = 25;
float  fNum      = 72;

/// Security variables
cmd_type      myNum;
int           radNum;
int           chNum;
PA_type       pwrNum;
int           tarNum;
Alarm_type    almNum;
Dng_level     dngNum;

//////////////////////////////////////////
/// These belong in the PongMessage struct
cmd_type      myCmd;
int           radCmd;
int           chCmd;
PA_type       paCmd;
int           tarCmd;
Alarm_type    almCmd;
Dng_level     dngCmd;
//////////////////////////////////////////

//// Used to store asynchronous data messages sent by the TX to the Master
struct dataStruct SentMessage[1] = {0};

//// Used to store cmd ping messages received from the Master
struct pingCmd PongMessage[1] = {0}; 

//============================
//======= IRQ HANDLERS =======
//============================

/** https://reference.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

mode: defines when the interrupt should be triggered. 
Four constants are predefined as valid mode values:

LOW     to trigger the interrupt whenever the pin is low,
CHANGE  to trigger the interrupt whenever the pin changes value
RISING  to trigger when the pin goes from low to high,
FALLING for when the pin goes from high to low.
**/

//=====================================
//======= SEN-12642 IRQ HANDLER =======
//=====================================
//////////////////////////////
// audioISR()
// This function is installed as an interrupt service routine for the GATE pin
// change interrupt.  When digital input 2 changes state, this routine
// is called. It queries the PIN_GATE_IN and saves it in a global variable.

void audioISR()
{
  audio_gate_val = digitalRead(PIN_GATE_IN);
}

//===============================
//======= PIR IRQ HANDLER =======
//===============================
// https://forum.arduino.cc/t/reduce-the-sensitivity-of-a-pir-detector/673672/2

//////////////////////////////
// pirISR()
// This function is installed as an interrupt service routine for the PIR pin
// change interrupt.  When digital input 3 changes state, this routine
// is called. It queries the PIR_PIN and saves it in a global variable.

void pirISR()
{
  pir_pin_val = digitalRead(PIR_PIN);
}

//===================================
//======= MESSAGE IRQ HANDLER =======
//===================================

//////////////////////////////
// msgISR()
// This function is installed as an interrupt service routine for the nRF24 
// interrupt pin.  When digital input 4 changes state, this routine
// is called. It queries the MSG_PIN state and saves it in a global variable.

void msgISR()
{
  msg_pin_val= digitalRead(MSG_PIN);
}

//=========================
//========= Setup =========
//=========================                         
void setup() { 
  Serial.begin(SER_BAUDRATE);
  //dht.begin();
  //SPI.begin(); 
  
  pinMode(SCK,      OUTPUT);
  pinMode(MISO,     INPUT);
  pinMode(MOSI,     OUTPUT);
  pinMode(SS,       OUTPUT); 
  pinMode(CE,       OUTPUT);

  pinMode(IRQ_GATE_IN,  INPUT_PULLUP);    // Pin 2
  pinMode(IRQ_PIR_IN,   INPUT_PULLUP);    // Pin 3, PIR_PIN
  pinMode(IRQ_MSG_IN,   INPUT_PULLUP);    // Pin 4, MSG_PIN
  
  pinMode(PIN_GATE_IN, INPUT); // Pin 2
  attachInterrupt(digitalPinToInterrupt(IRQ_GATE_IN), audioISR, FALLING);
  
  pinMode(PIR_PIN, INPUT); // Pin 3 
  attachInterrupt(digitalPinToInterrupt(IRQ_PIR_IN), pirISR, FALLING);
  
  pinMode(MSG_PIN, INPUT); // Pin 4 
  attachInterrupt(MSG_PIN, msgISR, FALLING);
  //attachInterrupt(digitalPinToInterrupt(IRQ_MSG_IN), msgISR, FALLING);
  
  pinMode(PIN_ENV_IN,INPUT); // Pin A0
  pinMode(PIN_AUDIO_IN,INPUT); // Pin A2

  initRadioValues(5, 81, PA_MIN);
  setupTXRadio(radNum,chNum,pwrNum); // (radio number, channel number, power amp setting) 
}

//==========================
//======= Main Loop ========
//==========================

void loop() {   
  delay(10); 
  pir_pin_val = digitalRead(PIR_PIN);
  if(pir_pin_val == 1) intruderFlag = true;
  transmit_pir_state(intruderFlag);  
    
  delay(10); 
  audio_gate_val = digitalRead(PIN_GATE_IN);
  if (audio_gate_val == 1) audioFlag = true;
  transmit_audio_gate(audioFlag); 
   
  delay(10); 
  receive_msg();  
  serialCommOutput();

/*  readDHT();
  delay(2000);
  rc = k30_i2c.readCO2(co2);
  if (rc == 0) { 
    validCO2Flag = true;
    sendMessage(validCO2Flag);    
    //Serial.println("Channel inside main: " + String(radio.getChannel()));
    //radio.write(&SentMessage, sizeof(SentMessage));
    Serial.println("Message sent\n");
  }
  else{
    validCO2Flag = false;
    digitalWrite(speakerOut, LOW); 
  } */ 
}

//===========================
//=== Send Message Method ===
//===========================
void sendMessage(bool validCO2Flag){
  if (validCO2Flag){ // If we get a valid CO2 reading 
      transmit_response_to_RX(respFlag = true);
      delay(10000);
  }
}
 
//===============================
//======= Receive Message =======
//===============================
// Standard nRF24L01 Receive Message Handler
// Handles incoming pong messages from Master

void receive_msg(){ 
  if (radio.available()) {
      radio.read(&PongMessage, sizeof(PongMessage));
      Serial.println("DEBUG:\tMessage Received from Master");
      
      get_pong_cmd_RX(); // You need to run setupTXRadio() soon after this
      setupTXRadio(radNum,chNum,pwrNum);
      respFlag = true;
      Serial.println("DEBUG:\tMessage processed");      
      transmit_response_to_RX(respFlag);
      
      Serial.println("DEBUG:\tradNum "+String(radNum)+
      "  chNum " +String(chNum)+
      "  pwrNum "+String(pwrNum)+"\n");
  }
}

//=======================================
//=== Get Ping Command from RX Master ===
//=======================================
void get_pong_cmd_RX() { 
  
// You need to run setupTXRadio(radNum,chNum,pwrNum) soon 
// after running this because this sets up the parameters
// for setupTXRadio(radNum,chNum,pwrNum) to proceed.
 
    get_pong_cmd();
  
    if ((chCmd == chNum) && (tarCmd == radNum)) {
      Serial.println("\nSUCCESS: \nchCmd = " + String(chCmd)+
                      "\tchNum" + String(chNum)+
                      "\ntarCmd " + String(tarCmd)+
                      "\tradNum" + String(radNum)+
                      "\npaCmd " + String(paCmd)+
                      "\t\tpwrNum" + String(pwrNum)); 
            
      switch (myCmd) {
        case 0: // change nada
            checkRadioConn();
            break;
        case 1: // chan_chg only
            chNum = PongMessage[0].ch_cmd;
            break;
        case 2: // rad_chg only
            radNum = PongMessage[0].r_cmd;
            break;
        case 3: // pa_chg only
            pwrNum = PongMessage[0].pa_cmd;
            break;
        case 4: // change them all. CAUTION!!
            chNum = PongMessage[0].ch_cmd;
            radNum = PongMessage[0].r_cmd;
            pwrNum = PongMessage[0].pa_cmd;
            break;
        case 5: // received tar change
            SentMessage[0].ping.tar_cmd = PongMessage[0].tar_cmd;
            break;
        case 6: // response
            SentMessage[0].ping.tar_cmd = PongMessage[0].tar_cmd;
            break;
  
        default:
            break;
      }
   }
}

void get_pong_cmd() {  
    myNum  = SentMessage[0].ping.my_cmd;
    chNum  = SentMessage[0].ping.ch_cmd;
    radNum = SentMessage[0].ping.r_cmd;
    pwrNum = SentMessage[0].ping.pa_cmd;
    tarNum = SentMessage[0].ping.tar_cmd;
    almNum = SentMessage[0].ping.alm_cmd;
    dngNum = SentMessage[0].ping.dng_cmd;
    Serial.println("Sent Data: "+String(myNum)+" "+String(chNum)
                    +" "+String(radNum)+" "+String(pwrNum)
                    +" "+String(tarNum)+" "+String(almNum)
                    +" "+String(dngNum));

    myCmd  = PongMessage[0].my_cmd;
    chCmd  = PongMessage[0].ch_cmd;
    radCmd = PongMessage[0].r_cmd;
    paCmd  = PongMessage[0].pa_cmd;
    tarCmd = PongMessage[0].tar_cmd;
    almCmd = PongMessage[0].alm_cmd;
    dngCmd = PongMessage[0].dng_cmd;
    Serial.println("Pong Data 1: "+String(myCmd)+" "+String(chCmd)
                    +" "+String(radCmd)+" "+String(paCmd)
                    +" "+String(tarCmd)+" "+String(almCmd)
                    +" "+String(dngCmd));
}

//==========================
//=== Triage Sensor Data ===
//==========================
void triage_data(){
    //Serial.println("Begin Triage");
    
    Alarm_type    alm_temp = 0;
    Dng_level     dng_temp = 0;
  
    /// Make a copy of the SentMessage object
    struct dataStruct TempMessage[1] = {};
    TempMessage[0] = SentMessage[0];
 
    if (TempMessage[0].h_val < 10) {
      almNum = HUM_LOW;
      dngNum = DNG_NONE;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
      Serial.println("Alarm: " + String(almNum) + "\tHUM_LOW\t"
                    + String(TempMessage[0].h_val) 
                    + "\tDanger: " +String(dngNum));
    }
    if (TempMessage[0].h_val >= 85) {
      almNum = HUM_HIGH;
      dngNum = DNG_LOW;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
      Serial.println("Alarm: " + String(almNum) + "\tHUM_HIGH\t" + "" 
                    + String(TempMessage[0].h_val) 
                    + "\tDanger: " +String(dngNum));
    }
    if (TempMessage[0].c_val < 20) {
      almNum = TEMP_LOW;
      dngNum = DNG_LOW;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
//      Serial.println("alm_temp: " + String(alm_temp) + "\tdng_temp: " +String(dng_temp));
      Serial.println("Alarm: " + String(almNum) + "\tTEMP_LOW\t" + "" 
                    + String(TempMessage[0].c_val) 
                    + "\tDanger: " +String(dngNum));
    }
    if (TempMessage[0].c_val >= 40 && TempMessage[0].c_val < 100) {
      almNum = TEMP_HIGH;
      dngNum = DNG_HIGH;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
//      Serial.println("alm_temp: " + String(alm_temp) + "\tdng_temp: " +String(dng_temp));
      Serial.println("Alarm: " + String(almNum) + "\tTEMP_HIGH\t" + "" 
                    + String(TempMessage[0].c_val) 
                    + "\tDanger: " +String(dngNum));
    }  
    if (TempMessage[0].c_val >= 100) {
      almNum = TEMP_FIRE;
      dngNum = DNG_MAX;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
//      Serial.println("alm_temp: " + String(alm_temp) + "\tdng_temp: " +String(dng_temp));
      Serial.println("Alarm: " + String(almNum) + "\tTEMP_FIRE\t" + "" 
                    + String(TempMessage[0].c_val) 
                    + "\tDanger: " +String(dngNum));
    }
  
    if (TempMessage[0].co2_val <= 0) {
      almNum = CO2_LOW;
      dngNum = DNG_NONE;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
//      Serial.println("alm_temp: " + String(alm_temp) + "\tdng_temp: " +String(dng_temp));
      Serial.println("Alarm: " + String(almNum) + "\tCO2_LOW\t" + "" 
                    + String(TempMessage[0].co2_val) 
                    + "\tDanger: " +String(dngNum));
    }

    if (TempMessage[0].co2_val>=400 && TempMessage[0].co2_val<=1600) {
      almNum = CO2_GOOD;
      dngNum = DNG_NONE;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
//      Serial.println("alm_temp: " + String(alm_temp) + "\tdng_temp: " +String(dng_temp));
      Serial.println("Alarm: " + String(almNum) + "\tCO2_GOOD\t" + "" 
                    + String(TempMessage[0].co2_val) 
                    + "\tDanger: " +String(dngNum));
    }

    if (TempMessage[0].co2_val>=1800 && TempMessage[0].co2_val<3000) {
      almNum = CO2_HIGH;
      dngNum = DNG_HIGH;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
//      Serial.println("alm_temp: " + String(alm_temp) + "\tdng_temp: " +String(dng_temp));
      Serial.println("Alarm: " + String(almNum) + "\tCO2_HIGH\t" + "" 
                    + String(TempMessage[0].co2_val) 
                    + "\tDanger: " +String(dngNum));
    }
    if (TempMessage[0].co2_val>=3000) {
      almNum = CO2_DNG;
      dngNum = DNG_MAX;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
//      Serial.println("alm_temp: " + String(alm_temp) + "\tdng_temp: " +String(dng_temp));
      Serial.println("Alarm: " + String(almNum) + "\tCO2_DNG\t" + "" 
                    + String(TempMessage[0].co2_val) 
                    + "\tDanger: " +String(dngNum));
    } 

    //if (gateNum) {
    if (TempMessage[0].audio_gate == 1) {
      almNum = LOUDNOISE;
      dngNum = DNG_MED;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
//      Serial.println("alm_temp: " + String(alm_temp) + "\tdng_temp: " +String(dng_temp));
      Serial.println("Alarm: " + String(almNum) + "\tLOUDNOISE\t" + "" 
                    + String(TempMessage[0].audio_gate) 
                    + "\tDanger: " +String(dngNum));
    } 
    //if (pirNum) { 
    if (TempMessage[0].pir_state == 1) { 
      almNum = INTRUDER;
      dngNum = DNG_HIGH;
      if (almNum >= alm_temp) alm_temp = almNum;
      if (dngNum >= dng_temp) dng_temp = dngNum;
//      Serial.println("alm_temp: " + String(alm_temp) + "\tdng_temp: " +String(dng_temp));
      Serial.println("Alarm: " + String(almNum) + "\tINTRUDER\t" + "" 
                    + String(TempMessage[0].pir_state) 
                    + "\tDanger: " +String(dngNum));
    }

      
      SentMessage[0].ping.alm_cmd = alm_temp;  
      SentMessage[0].ping.dng_cmd = dng_temp;   
      almNum = alm_temp;
      dngNum = dng_temp;  
      Serial.println("\nSentMessage  Highest Alarm Type: "  
                    +String(SentMessage[0].ping.alm_cmd));
      Serial.println("SentMessage  Highest Danger Level: "
                    +String(SentMessage[0].ping.dng_cmd)+"\n");
 }

//================================
//======= Pack SentMessage =======
//================================
void pack_sent_msg() {
  
    SentMessage[0].chNum = chNum;
    SentMessage[0].radNum = radNum;
    SentMessage[0].co2_val = co2Num;
    SentMessage[0].h_val = hNum;
    SentMessage[0].c_val = tNum;
    SentMessage[0].f_val = fNum;
    SentMessage[0].pir_state = pirNum;
    SentMessage[0].audio_gate = gateNum;
    
    SentMessage[0].ping.my_cmd = myNum;
    SentMessage[0].ping.ch_cmd = chNum;
    SentMessage[0].ping.r_cmd = radNum;
    SentMessage[0].ping.pa_cmd = pwrNum;
    SentMessage[0].ping.tar_cmd = tarNum;
    SentMessage[0].ping.alm_cmd = almNum;
    SentMessage[0].ping.dng_cmd = dngNum;

    //printDebugMsg(SentMessage);
}

//========================================//
//============ Serial Commands ===========//
//========================================//
void serialCommOutput() {
  if (Serial.available()) {
    char c = Serial.read();  
    if (c == 'R') {
      Serial.println("\n*** CHECK MY RADIO CONNECTION");
      checkRadioConn();
    }
    else if (c == '1') {
      Serial.println("\n*** CHANGING NODE TO CH 81 Radio 1");
      setupTXRadio(1, 81, PA_MIN); 
      delay(5000);  
    }
    else if (c == '2') {
      Serial.println("\n*** CHANGING NODE TO CH 81 Radio 2");
      setupTXRadio(2, 81, PA_MIN); 
      delay(5000);
    } 
    else if (c == '3') {
      Serial.println("\n*** CHANGING NODE TO CH 81 Radio 3");
      setupTXRadio(3, 81, PA_MIN); 
      delay(5000);  
    }
    else if (c == '4') {
      Serial.println("\n*** CHANGING NODE TO CH 81 Radio 4");
      setupTXRadio(4, 81, PA_MIN); 
      delay(5000);
    }   
    else if (c == '5') {
      Serial.println("\n*** CHANGING NODE TO CH 81 Radio 5");
      setupTXRadio(5, 81, PA_MIN); 
      delay(5000);  
    }
    else if (c == 'E') {
      Serial.println("\n*** CHANGING NODE TO CH 76 Radio 1");
      setupTXRadio(1, 76, PA_MIN); 
      delay(5000);  
    }
    else if (c == 'S') {
      Serial.println("\n*** CHANGING MASTER TO CH 81");
      setupTXRadio(1, 81, PA_MIN); 
      delay(5000);
    } 
    Serial.println("\n*** Serial Command sent to TX...Good Luck...");
    radio.startListening(); 
  }
}

//==================================
//======= Transmit PIR State =======
//==================================
void transmit_pir_state(bool pirFlag){
  // if PIR is activated 
  if (pirFlag){ 
    pirNum = pirFlag;
    pack_sent_msg(); 
    triage_data(); 
    //SentMessage[0].pir_state = pirNum;
    //SentMessage[0].radNum = radNum;
    Serial.print("PIR State: "+String(pirFlag));
    Serial.print("\tRadio: " + String(SentMessage[0].radNum));
    Serial.println("\tChannel: " + String(radio.getChannel()));
    
    radio.stopListening(); // Sets radio to TX mode
    radio.write(&SentMessage, sizeof(SentMessage));
    radio.openReadingPipe(1,addresses[0]);
    radio.startListening(); // Sets radio back to RX mode
    
    delay(5000); 
    pirNum = false;
    pirFlag = false;
    intruderFlag = false; // must be reset to false before end of function
    SentMessage[0].pir_state = pirNum;  
    Serial.println("Msg pir_state Cleared: "+String(SentMessage[0].pir_state));
    Serial.println();
  }
}

//===================================
//======= Transmit Audio Gate =======
//===================================
void transmit_audio_gate(bool agate){
  // if Audio Alarm is Activated
  if (agate){   
    gateNum = agate;
    pack_sent_msg(); 
    triage_data(); 
    //SentMessage[0].audio_gate = gateNum;
    //SentMessage[0].radNum = radNum;
    Serial.print("Audio Detected: "+String(agate));
    Serial.print("\tRadio: " + String(SentMessage[0].radNum));
    Serial.println("\tChannel: " + String(radio.getChannel()));
    
    radio.stopListening(); // Sets radio to TX mode
    radio.write(&SentMessage, sizeof(SentMessage)); // Transmits the message
    radio.openReadingPipe(1,addresses[0]); // Sets radio back to RX mode
    radio.startListening(); // Sets radio back to RX mode
    
    delay(1000);  // Allow audio signal to subside
    gateNum = false;
    agate = false;
    audioFlag = false;  // must be reset to false before end of function
    SentMessage[0].audio_gate = gateNum; 
    Serial.println("Msg audio_gate cleared: "+String(SentMessage[0].audio_gate));
    Serial.println();
  }
}

//=========================================
//======= Transmit Response Message =======
//=========================================
void transmit_response_to_RX(bool respFlag){
  // if we received a pong command message...
  if (respFlag){ 
       
    pack_sent_msg(); 
    triage_data();    
    //printDebugMsg(SentMessage); 
  
    radio.stopListening(); // Sets radio to TX mode
    radio.write(&SentMessage, sizeof(SentMessage)); // Transmits the message
    radio.openReadingPipe(1,addresses[0]); // Sets radio back to RX mode
    radio.startListening(); // Sets radio back to RX mode
    
    delay(1000);  // Allow message to transmit
    Serial.println("DEBUG: Response Sent to Master "+String(respFlag));
    respFlag = false;  // Must be reset to false before end of function
    Serial.println();
  }
}

//=============================
//=== Setup TX Radio Method ===
//=============================
void setupTXRadio(int rn,int mychannel,PA_type pat){
  rn = ((rn > 0) && (rn < 6)) ? rn : 1; // ERROR CORRECTION
  radio.begin(); // Start the NRF24L01
  
  // Initialize radio parameters
  radio.setDataRate( RF24_250KBPS ); // RF24_1MBPS, RF24_2MBPS
  radio.setChannel(mychannel);
  setPowerLevel(pat);  
  //radio.setPALevel(RF24_PA_MIN); // MIN, LOW, HIGH, and MAX

  // Set Global radio parameters
  radNum = rn;
  chNum = mychannel;
  pwrNum = pat; 
  
  radio.openWritingPipe(addresses[rn]);
  radio.openReadingPipe(1,addresses[0]);
  radio.startListening(); // Initialize radio to RX mode

  checkRadioConn();
}

//======================================
//======= Check Radio Connection =======
//======================================
void checkRadioConn() {    
    printf_begin(); // Call this before calling printDetails()
    if (radio.isChipConnected()) {
      Serial.println("\nSensor Node Number "+ String(radNum) 
                    +" is connected to: Channel " + String(radio.getChannel()));
      radio.printDetails();  
      Serial.println("RF Comms Starting...");
    }
    else{
      Serial.println("\nRadio is not connected; showing Channel: " 
                      + String(radio.getChannel()));
      radio.printDetails();
    }  
}

//==========================
//======= Miscellany =======
//==========================
void setPowerLevel(PA_type pwr) {
  switch (pwr) {
    case 0:
        radio.setPALevel(RF24_PA_MIN);
        break;
    case 1:
        radio.setPALevel(RF24_PA_LOW);
        break;
    case 2:
        radio.setPALevel(RF24_PA_HIGH);
        break;
    case 3:
        radio.setPALevel(RF24_PA_MAX);
        break;
    default:
        break;
  }
}

void initRadioValues(int rn,int mychannel,PA_type pat) {  
    radNum = rn;
    chNum = mychannel;
    pwrNum = pat;
}

void printDebugMsg(dataStruct SM[1]) {
  
    Serial.println("\nDEBUG PRINT");
    Serial.println("Message chNum: \t"+String(SM[0].chNum));
    Serial.println("Message rn: \t"   +String(SM[0].radNum));
    Serial.println("Message CO2: \t"  +String(SM[0].co2_val));
    Serial.println("Message Humid: \t"+String(SM[0].h_val));    
    Serial.println("Message Deg C: \t"+String(SM[0].c_val));    
    Serial.println("Message Deg F: \t"+String(SM[0].f_val));    
    Serial.println("Message PIR: \t"  +String(SM[0].pir_state));    
    Serial.println("Message Audio: \t"+String(SM[0].audio_gate));

    Serial.println("\nMessage CMD Type: \t"  +String(SM[0].ping.my_cmd));
    Serial.println("Message Channel: \t"     +String(SM[0].ping.ch_cmd));
    Serial.println("Message Radio Number: \t"+String(SM[0].ping.r_cmd));
    Serial.println("Message Pwr Level: \t"   +String(SM[0].ping.pa_cmd));
    Serial.println("Message tar State: \t"   +String(SM[0].ping.tar_cmd));    
    Serial.println("\nMessage Alarm Type: \t"+String(SM[0].ping.alm_cmd));
    Serial.println("Message Danger Level: \t"+String(SM[0].ping.dng_cmd)+"\n");
}

/*--------------------------------------------------------------*/
/*------------ End of emacs_full_sensor_node_01.ino ------------*/
/*--------------------------------------------------------------*/
