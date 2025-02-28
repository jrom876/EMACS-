

/*
    Project:  EMACS -- Environmental Monitoring and Control System    
    File:     emacs_RX_Master_03.ino   
    Desc:     RX Master Node, stable version
        
    Coded by:     Jacob Romero
    Company:      Creative Engineering Solutions LLC
    email:        cesllc876@gmail.com
    date:         02/05/2025

    Copyright (C) 2025 
    Jacob Romero, Creative Engineering Solutions, LLC

    EMACS Hardware:    
      nRF24L01      2.4 GHz Transceiver
      DHT-11        Temp and Humidity Sensor
      K30           CO2 Sensor
      PIR Detector  Motion Sensor
      SEN-12642     Audio Sensor
      20x4 LCD      Display
      Arduino       MEGA, UNO or NANO
    
    This firmware module is a stable version of the 
    Master RX node in a TX/RX pair. 
    
    Sensor data is transmitted continuously by the
    TX module's nRF24L01 (2.4 GHz transceiver), which
    currently sends packets of data from up to 4 sensors:
      DHT-11 -- Temp and Humidity Sensor
      K30 -- CO2 Sensor
      PIR -- Passive Infrared Motion Sensor
      SEN12642 -- Audio Sensor
    
    Credits:
    RF24SensorNet library
    https://github.com/szaffarano/RF24SensorNet
    NRF24L01 library
    TMRh20 https://github.com/TMRh20/RF24

    https://how2electronics.com/stm32-nrf24l01-node-with-esp32-nrf24l01-gateway/
*/
 
//========= Libraries =========//
#include <RF24.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <LiquidCrystal.h>

#include <SPI.h>
#include <stdlib.h>
#include <stdio.h> 
#include <printf.h>
#include <Wire.h>
#include <time.h>

#define SER_BAUDRATE  9600
#define LF            0x0A    // Line Feed        == 0x0A
#define CR            0x0D    // Carriage Return  == 0x0D
  
//========================
//======= LCD PINS =======
//========================
#define D2          2 // LCD DB7
#define D3          3 // LCD DB6
#define D4          4 // LCD DB5
#define D5          5 // LCD DB4
#define EN_LCD      6 // LCD E
#define RS_LCD      7 // LCD RS

//=====================================
//======= nRF24L01 and IRQ PINS =======
//=====================================
/*
/// Enable for UNO or NANO
#define MISO      12
#define MOSI      11
#define SCK       13
#define SS        10
#define CE        9

#define BUZZR_PIN 8 

/// UNO/NANO Interrupt Pins: 2, 3
#define MSG_PIN     3   // nRF24 interrupt
#define IRQ_MSG_IN  3   // nRF24 IRQ
*/

/// Enable for MEGA 2560
#define MISO  50
#define MOSI  51
#define SCK   52
#define SS    53
#define CE    49

#define BUZZR_PIN 8 

/// MEGA 2560 Interrupt Pins: 2, 3, 18, 19, 20, 21
#define MSG_PIN     19   // nRF24 interrupt
#define IRQ_MSG_IN  19   // nRF24 IRQ

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
RF24 radio(CE, SS); // Uno pins 9 and 10, Mega pins 49 and 52
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
//=============================

//=================
//==== Globals ====
//=================

/// Interrupt variables must be volatile
volatile int msg_pin_val;

/// Flags for triggering operations
bool      intruderFlag  = false;
bool      audioFlag     = false;
bool      respFlag      = false;
//bool      almFlag       = false;
//bool      dngFlag       = false;

/// Variables for message passing
int           co2_msg, pir_msg, gate_msg, h_msg, c_msg, f_msg;
int           hif_msg, hic_msg;
int           cmd_msg, ch_msg, rad_msg;
PA_type       pa_msg;
int           tar_msg;
Alarm_type    alm_msg;
Dng_level     dng_msg;

// Used to store messages received by the Master
struct dataStruct ReceivedMessage[1] = {0};

// Used to store ping being sent by the Master to TX
struct pingCmd PingMessage[1] = {}; 

int   sensorOffset = 0;

//===============================
//======= MESSAGE HANDLER =======
//===============================
//////////////////////////////
// msgISR()
// This function is installed as an interrupt service routine for the nRF24 
// interrupt pin.  When digital input X changes state, this routine
// is called. It queries the MSG_PIN state and saves it in a global variable.

void msgISR()
{
  msg_pin_val= digitalRead(MSG_PIN);
}

//=========================
//========= Setup =========
//=========================
void setup(void) {
    Serial.begin(SER_BAUDRATE);
    SPI.begin();
    
    lcd.begin(20, 4);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Environmental Sensor");
    lcd.setCursor(0, 1);
    lcd.print("Standby for data");
    lcd.setCursor(0, 2);
    lcd.print("TEST DATA THIS LINE");
    lcd.setCursor(0, 3);
    lcd.print("CES, LLC.");
    delay(1000);
    
    pinMode(SCK, OUTPUT);           // 13
    pinMode(MISO, INPUT);           // 12
    pinMode(MOSI, OUTPUT);          // 11
    pinMode(SS, OUTPUT);            // 10
    pinMode(CE, OUTPUT);            // 9 
    //pinMode(BUZZR_PIN, OUTPUT);   // 8  
    pinMode(RS_LCD, OUTPUT);        // 7
    pinMode(EN_LCD, OUTPUT);        // 6
    
    pinMode(MSG_PIN, INPUT);        // Pin 3     
    pinMode(IRQ_MSG_IN, INPUT_PULLUP); // Pin 3, MSG_PIN        
    attachInterrupt(IRQ_MSG_IN, msgISR, RISING);
    
    setupMasterRadio(81,PA_MIN); // channel number and power level
}

//===========================
//======== Main Loop ========
//===========================
void loop(void) {
  receive_data();
  serialCommOutput();
}

//===========================
void receive_data() {  
    //ReceivedMessage[0] = {};
    if (radio.available()) {
      radio.read(&ReceivedMessage, sizeof(ReceivedMessage)); 
      parseRcvMsg();
      receive_rm(rad_msg);  // Runs TX radio specific message routines    
    } 
}

//===========================
/// Parses the incoming message and assigns its values
/// to global variables, and prints debug script.

void parseRcvMsg(){ 
    ch_msg    = ReceivedMessage[0].chNum;
    rad_msg   = ReceivedMessage[0].radNum;       
    co2_msg   = ReceivedMessage[0].co2_val + sensorOffset;
    h_msg     = ReceivedMessage[0].h_val;
    c_msg     = ReceivedMessage[0].c_val;
    f_msg     = ReceivedMessage[0].f_val;
    pir_msg   = ReceivedMessage[0].pir_state;
    gate_msg  = ReceivedMessage[0].audio_gate;
       
//    Serial.println("\nENV Data from TX Received: \n"  /// DEBUG PRINT
//                  + String(ch_msg) + " ch_msg\n"
//                  + String(rad_msg)  + " rad_msg\n"
//                  + String(co2_msg) + " co2_msg\n"
//                  + String(h_msg) + " h_msg\n"
//                  + String(c_msg) + " c_msg\n"
//                  + String(f_msg) + " f_msg\n"
//                  + String(pir_msg) + " pir_msg\n"
//                  + String(gate_msg) + " gate_msg\n"); 
                  
    cmd_msg   = ReceivedMessage[0].ping.my_cmd;
    ch_msg    = ReceivedMessage[0].ping.ch_cmd; 
    rad_msg   = ReceivedMessage[0].ping.r_cmd;
    pa_msg    = ReceivedMessage[0].ping.pa_cmd;
    tar_msg   = ReceivedMessage[0].ping.tar_cmd;
    alm_msg   = ReceivedMessage[0].ping.alm_cmd;
    dng_msg   = ReceivedMessage[0].ping.dng_cmd;     
   
//    Serial.println("Security Data from TX Received: \n"  /// DEBUG PRINT
//                  + String(cmd_msg) + " cmd_msg\n"
//                  + String(ch_msg)  + " ch_msg\n"
//                  + String(rad_msg) + " rad_msg\n"
//                  + String(pa_msg)  + " pa_msg\n"
//                  + String(tar_msg) + " tar_msg\n"
//                  + String(alm_msg) + " alm_msg\n"
//                  + String(dng_msg) + " dng_msg\n");     
}

//======================================
//======= Receive Message et al ========
//====================================== 
/*  This method allows us to run routines that extract specific data 
    from each of the 5 TX node responses according to its radio number.
    Nodes 1 and 2 are configured to receive environmental data, 
    while 3 through 5 are configured for security data.
*/
void receive_rm(int radnum) {
    switch (radnum) {
      
      case 0: // Master only mode, not used
          //do something when radNum equals 0 
          triage_msg();
//          alm_routine(alm_msg);
//          dng_routine(dng_msg); 
          compact_lcd_routine(); 
          break;
          
      case 1: // K30 + DHT11 main pipe
          triage_msg();
//          alm_routine(alm_msg);
//          dng_routine(dng_msg); 
          compact_lcd_routine();
          break;
          
      case 2: // K30 + DHT11 alternate pipe
          triage_msg();
//          alm_routine(alm_msg);
//          dng_routine(dng_msg); 
          compact_lcd_routine();
          break;
          
      case 3: // PIR + Audio detector      
          triage_msg();
//          alm_routine(alm_msg);
//          dng_routine(dng_msg);            
          compact_lcd_routine();
          if (pir_msg) {
            pir_routine();          
            //Serial.println("PIR detected: " + String(pir_msg) + "\n");
          }
          if (gate_msg) {
            audio_routine();        
            //Serial.println("Gate detected: " + String(gate_msg) + "\n");
          }
          break;
          
      case 4: // PIR + Audio detector     
          triage_msg();
//          alm_routine(alm_msg);
//          dng_routine(dng_msg);            
          compact_lcd_routine();
          if (pir_msg) {
            pir_routine();
            //buzz_routine_2();
            //Serial.println("PIR detected: " + String(pir_msg) + "\n");
          }
          if (gate_msg) {
            audio_routine();
            //buzz_routine_2();
            //Serial.println("Gate detected: " + String(gate_msg) + "\n");
          } 
          break;
          
      case 5: // PIR + Audio detector     
          triage_msg();
//          alm_routine(alm_msg);
//          dng_routine(dng_msg);            
          compact_lcd_routine();
          if (pir_msg) {
            pir_routine();
            //buzz_routine();
            //Serial.println("PIR detected: " + String(pir_msg) + "\n");
          }
          if (gate_msg) {
            audio_routine();
            //buzz_routine();
            //Serial.println("Gate detected: " + String(gate_msg) + "\n");
          }
          break;
      default:
          break;
    }
}

//============================
//======= LCD Routines =======
//============================

void compact_lcd_routine(void) { 

/// SAVE FOR DEBUG
//    Serial.println("\nCompact LCD Routine:");
//    Serial.println("RX Channel number: " + String(radio.getChannel()));
//    Serial.println("Input Radio number: " + String(rad_msg)); 
//    Serial.println("CO2 level: "+String(co2_msg));
//    Serial.println("Humidity: " + String(h_msg) + "%");
//    Serial.println("Degrees C: " + String(c_msg) + "C");
//    Serial.println("Degrees F: " + String(f_msg) + "F");
    
    lcd.begin(20, 4);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    lcd.setCursor(0, 0);
    lcd.print("Ch:" + String(ch_msg) + " R:"+String(rad_msg)+" CO2:"+String(co2_msg)); 
    
    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(0, 1);
    lcd.print("H:" + String(h_msg) + "% " 
              +String(c_msg) + "C " 
              +String(f_msg) + "F");
            
    lcd.setCursor(0, 2);
    lcd.print("                   ");
    lcd.setCursor(0, 2);
    lcd.print("PIR:" +String(pir_msg) 
              +" Aud:"+String(gate_msg) 
              +" Target:"+String(tar_msg));

    lcd.setCursor(0, 3);
    lcd.print("                   ");
    lcd.setCursor(0, 3);
    lcd.print("Alm:" +String(alm_msg) 
              +" Dng:"+String(dng_msg));
}

void pir_routine(void) {  
    if (pir_msg) {
      Serial.println("Intruder Detected" );
      //Serial.println("Time Stamp: " + String(100)); 
    }else {
      Serial.println("No Intruders" ); 
    }
    //delay(2000);
}
void audio_routine(void) {  
    if (gate_msg) {
      Serial.println("Audio Detected" );
      //Serial.println("Time Stamp: " + String(200));   
    }else {
      Serial.println("All Quiet" );  
    }
    //delay(2000);
}
void buzz_routine(void) { 
    Serial.println("BEEEP!!");  
    digitalWrite(BUZZR_PIN, HIGH);
    delay(500);  // 250, 500, 1000, 2000
    digitalWrite(BUZZR_PIN, LOW);         
}
void buzz_routine_2(void) { 
    Serial.println("BEEEP!!");  
    digitalWrite(BUZZR_PIN, HIGH);
    delay(250);  
    digitalWrite(BUZZR_PIN, LOW); 
    delay(250);  
    digitalWrite(BUZZR_PIN, HIGH);
    delay(250);  
    digitalWrite(BUZZR_PIN, LOW); 
}


//=======================================
//=== Compliance and Control Routines ===
//=======================================
/*  
 *  triage_msg() 
 *  alm_routine(Alarm_type alm)
 *  dng_routine(Dng_level dl)
 *   
 *  NOTE: These routines will provide control of all peripheral safety 
 *  and environmental control equipment and communication processes so that 
 *  regulatory compliance with Fire Departments, Local Building Codes, 
 *  and/or any other Regulatory Agencies can be achieved. 
 *  
 *  They will allow us to control equipment and comm processes such as:   
 *      CO2 Tank Valve
 *      Air Flow Vents
 *      Air Flow Fans
 *      Audio Alarms
 *      Visual Alarms
 *      Cameras
 *      LAN Server Messages
 *      Intruder Detection Alerts
 *      Environmental Updates and Alerts
 *      Text Messages, emails, etc.
*/   

void triage_msg() {  

    /// Instantiate and Initialize dummy objects
    Alarm_type    alm_temp = 0;
    Dng_level     dng_temp = 0;
    struct dataStruct TempMessage[1] = {};
    TempMessage[0] = ReceivedMessage[0];
    
    if (TempMessage[0].h_val < 10) {
      alm_msg = HUM_LOW;
      dng_msg = DNG_NONE;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg)  
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tHUM_LOW \t" 
                      + String(TempMessage[0].h_val)
                      + "\tDanger: " +String(dng_msg));
    }
    if (TempMessage[0].h_val >= 85) {
      alm_msg = HUM_HIGH;
      dng_msg = DNG_LOW;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg) 
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tHUM_HIGH \t" 
                      + String(TempMessage[0].h_val)
                      + "\tDanger: " +String(dng_msg));
    }
    if (TempMessage[0].c_val < 20) {
      alm_msg = TEMP_LOW;
      dng_msg = DNG_LOW;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg)  
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tTEMP_LOW \t" 
                      + String(TempMessage[0].c_val)
                      + "\tDanger: " +String(dng_msg));
    }
    if (TempMessage[0].c_val > 40) {
      alm_msg = TEMP_HIGH;
      dng_msg = DNG_HIGH;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg)  
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tTEMP_HIGH \t" 
                      + String(TempMessage[0].c_val)
                      + "\tDanger: " +String(dng_msg));
    }  
    if (TempMessage[0].c_val > 100) {
      alm_msg = TEMP_FIRE;
      dng_msg = DNG_MAX;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg)  
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tTEMP_FIRE \t" 
                      + String(TempMessage[0].c_val)
                      + "\tDanger: " +String(dng_msg));
    }  
    if (TempMessage[0].co2_val <= 0) {
      alm_msg = CO2_LOW;
      dng_msg = DNG_NONE;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg)  
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tCO2_LOW \t" 
                      + String(TempMessage[0].co2_val)
                      + "\tDanger: " +String(dng_msg));
    }
    if (TempMessage[0].co2_val>=400 && TempMessage[0].co2_val<2000) {
      alm_msg = CO2_GOOD;
      dng_msg = DNG_NONE;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg)  
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tCO2_GOOD \t" 
                      + String(TempMessage[0].co2_val)
                      + "\tDanger: " +String(dng_msg));
    }
    if (TempMessage[0].co2_val>=2000 && TempMessage[0].co2_val<3000) {
      alm_msg = CO2_HIGH;
      dng_msg = DNG_HIGH;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg)  
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tCO2_HIGH \t" 
                      + String(TempMessage[0].co2_val)
                      + "\tDanger: " +String(dng_msg));
    }
    if (TempMessage[0].co2_val>=3000) {
      alm_msg = CO2_DNG;
      dng_msg = DNG_MAX;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg)  
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tCO2_DNG \t" 
                      + String(TempMessage[0].co2_val)
                      + "\tDanger: " +String(dng_msg));
    }
    if (TempMessage[0].pir_state == 1) { 
      alm_msg = INTRUDER;
      dng_msg = DNG_HIGH;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg)  
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tINTRUDER \t" 
                      + String(TempMessage[0].pir_state)
                      + "\tDanger: " +String(dng_msg));
    }
    if (TempMessage[0].audio_gate == 1) {
      alm_msg = LOUDNOISE;
      dng_msg = DNG_MED;
      if (alm_msg >= alm_temp) alm_temp = alm_msg;
      if (dng_msg >= dng_temp) dng_temp = dng_msg;
      Serial.println("\nAlarm: " + String(alm_msg)  
                      + "\tRADIO \t" 
                      + String(TempMessage[0].radNum)
                      + "\tLOUDNOISE \t" 
                      + String(TempMessage[0].audio_gate)
                      + "\tDanger: " +String(dng_msg));
    }      
    ReceivedMessage[0].ping.alm_cmd = alm_temp;  
    ReceivedMessage[0].ping.dng_cmd = dng_temp;   
    alm_msg = alm_temp;
    dng_msg = dng_temp;  
//    Serial.println("\nHighest Alarm Type: "  
//                  +String(ReceivedMessage[0].ping.alm_cmd));
//    Serial.println("Highest Danger Level: "
//                  +String(ReceivedMessage[0].ping.dng_cmd)+"\n");    
}

void alm_routine(Alarm_type alm) {  
    if (alm == ALM_NONE) {
      alm_msg = ALM_NONE;
      //dng_msg = DNG_NONE;
      Serial.println("ALM_NONE Alarms Detected" );
      //Serial.println("Time Stamp: " + String(300)); 
    }
    else if(alm == HUM_LOW) {
      alm_msg = HUM_LOW;
      //dng_msg = DNG_NONE;
      Serial.println("HUM_LOW" );
      //Serial.println("Time Stamp: " + String(700));       
    }
    else if(alm == HUM_HIGH) {
      alm_msg = HUM_HIGH;
      dng_msg = DNG_LOW;
      Serial.println("HUM_HIGH: INVESTIGATE NOW" );
      //Serial.println("Time Stamp: " + String(800));       
    }
    else if(alm == TEMP_LOW) {
      alm_msg = TEMP_LOW;
      dng_msg = DNG_MED;
      Serial.println("TEMP_LOW: INVESTIGATE NOW" );
      //Serial.println("Time Stamp: " + String(900));       
    }
    else if(alm == TEMP_HIGH) {
      alm_msg = TEMP_HIGH;
      dng_msg = DNG_HIGH;
      Serial.println("TEMP_HIGH Detected: INVESTIGATE NOW" );
      //Serial.println("Time Stamp: " + String(1000));       
    }
    else if(alm == TEMP_FIRE) {
      alm_msg = TEMP_FIRE;
      dng_msg = DNG_MAX;
      Serial.println("FIRE, FIRE, FIRE!!" );
      //Serial.println("Time Stamp: " + String(1000));       
    }    
    else if(alm == CO2_LOW) {
      alm_msg = CO2_LOW;
      //dng_msg = DNG_NONE;
      Serial.println("CO2_LOW: INVESTIGATE NOW" );
      //Serial.println("Time Stamp: " + String(400));       
    }
    else if(alm == CO2_GOOD) {
      alm_msg = CO2_GOOD;
      //dng_msg = DNG_NONE;
      Serial.println("CO2_GOOD" );
      //Serial.println("Time Stamp: " + String(500));       
    }
    else if(alm == CO2_HIGH) {
      alm_msg = CO2_HIGH;
      dng_msg = DNG_HIGH;
      Serial.println("CO2_HIGH: CLEAR SPACES AND INVESTIGATE" );
      //Serial.println("Time Stamp: " + String(600));       
    }
    else if(alm == CO2_DNG) {
      alm_msg = CO2_DNG;
      dng_msg = DNG_MAX;
      Serial.println("CO2_DNG: DANGER!! CLEAR SPACES IMMEDIATELY!!" );
      //Serial.println("Time Stamp: " + String(600));       
    }
    else if(alm == INTRUDER) {
      alm_msg = INTRUDER;
      dng_msg = DNG_HIGH;
      Serial.println("INTRUDER Detected: DANGER HIGH!! INVESTIGATE NOW" );
      //Serial.println("Time Stamp: " + String(1100));       
    }
    else if(alm == LOUDNOISE) {
      alm_msg = LOUDNOISE;
      dng_msg = DNG_MED;
      Serial.println("LOUDNOISE Detected: INVESTIGATE NOW" );
      //Serial.println("Time Stamp: " + String(1200));       
    }
    else {
      Serial.println("No Alarms Detected" ); 
    }
}

void dng_routine(Dng_level dl) {  
    if (dl == DNG_NONE) {
      dng_msg = DNG_NONE;
      Serial.println("No Danger" );
      //Serial.println("Time Stamp: " + String(410));   
    }
    else if (dl == DNG_LOW) {
      dng_msg = DNG_LOW;
      Serial.println("Danger Low" );
      //Serial.println("Time Stamp: " + String(420)); 
    }
    else if (dl == DNG_MED) {
      dng_msg = DNG_MED;
      Serial.println("Danger Medium: Caution Advised!" );
      //Serial.println("Time Stamp: " + String(430));       
    }
    else if (dl == DNG_HIGH) {
      dng_msg = DNG_HIGH;
      Serial.println("Danger High: Perform Damage Control Protocol" );
      //Serial.println("Time Stamp: " + String(440));       
    }
    else if (dl == DNG_MAX) {
      dng_msg = DNG_MAX;
      Serial.println("Danger Maximum: Perform Emergency Response Protocol" );
      //Serial.println("Time Stamp: " + String(540));       
    }
    else {
      Serial.println("No Danger Detected" );  
    }
    delay(2000);
}

//================================
//======= Send Ping Method =======
//================================
void send_ping_cmd(cmd_type cmd, int ch, int rn, PA_type pat, int target){
    //PingMessage[1] = {};
    PingMessage[0].my_cmd = cmd;
    PingMessage[0].ch_cmd = ch;
    PingMessage[0].r_cmd = rn;
    PingMessage[0].pa_cmd = pat;
    PingMessage[0].tar_cmd = target;
    PingMessage[0].alm_cmd = ALM_NONE;
    PingMessage[0].dng_cmd = DNG_NONE;
    
    Serial.println("Ping CMD Type: "+String(PingMessage[0].my_cmd));
    Serial.println("Ping Channel: "+String(PingMessage[0].ch_cmd));
    Serial.println("Ping Radio Number: "+String(PingMessage[0].r_cmd));
    Serial.println("Ping Pwr Level: "+String(PingMessage[0].pa_cmd));
    Serial.println("Ping Target: "+String(PingMessage[0].tar_cmd)); 
    Serial.println("Ping Alm Level: "+String(PingMessage[0].alm_cmd));
    Serial.println("Ping Dng Level: "+String(PingMessage[0].dng_cmd)); 

    radio.stopListening(); // Change to TX role just long enough to ping cmd     
    radio.openWritingPipe(addresses[0]);
    radio.write(&PingMessage, sizeof(PingMessage));
    radio.startListening();
}
//========================================//
//============ Serial Commands ===========//
//========================================//
/* 
 *  CHANGE RADIO NUMBER WITH CAUTION!!
 * NOTE about calling rad_chg: It is a general 
 * rule not to change the radio number 
 * of a TX node when sending serial commands.   
 * This precludes possible pipe collisions, 
 * as TX pipes are assigned by radio number. 
 * 
 * Channel 76:  Environmental Monitor
 * Channel 81:  Security Monitor
 * 
*/

void serialCommOutput() {
  if (Serial.available()) {
    struct pingCmd TempMessage[1] = {};
    
    char c = Serial.read();
    recvWithEndMarker(c); // returns receivedChars[8] from Serial Monitor;
    showNewData();
    
    if (c == 'A' ) {
      Serial.println("\n*** CHECKING TX RADIO NUMBER 2 PARAMETERS");      
      send_ping_cmd(nada, 81, 5, PA_MIN, 5);
    }   
    else if (c == 'a' ) {
      Serial.println("\n*** CHECKING TX RADIO NUMBER 5 PARAMETERS");      
      send_ping_cmd(nada, 81, 2, PA_MIN, 2);
    }   
    ///////////////////////////
    /// Change tar settings
    else if (c == 'B') {
      Serial.println("\n*** SENDING TARGET TO RADIO 5 CH 81");      
      send_ping_cmd(tar_chg, 81, 5, PA_MIN, 5);
    } 
    else if (c == 'b') {
      Serial.println("\n*** SENDING NACK TO RADIO 5 CH 81");
      send_ping_cmd(tar_chg, 81, 2, PA_MIN, 5);
    }     
    else if (c == 'D') {
      Serial.println("\n*** SENDING WAIT TO RADIO 5 CH 81");
      send_ping_cmd(tar_chg, 81, 5, PA_MIN, 5);
    }
        
    /////////////////////////////////
    /// Change power amp settings ///
    /////////////////////////////////
    else if (c == 'M') {
      Serial.println("\n*** CHANGING INCOMING TX PA TO MIN");
      send_ping_cmd(pa_chg, 81, 5, PA_MIN, 5);
    } 
    else if (c == 'm') {
      Serial.println("\n*** CHANGING INCOMING TX PA TO MIN");
      send_ping_cmd(pa_chg, 81, 2, PA_MIN, 2);
    } 
    else if (c == 'L') {
      Serial.println("\n*** CHANGING INCOMING TX PA TO MIN");
      send_ping_cmd(pa_chg, 81, 5, PA_LOW, 5);
    } 
    else if (c == 'l') {
      Serial.println("\n*** CHANGING INCOMING TX PA TO LOW");
      send_ping_cmd(pa_chg, 81, 2, PA_LOW, 2);
    } 
    else if (c == 'H') {
      Serial.println("\n*** CHANGING INCOMING TX PA TO HIGH");
      send_ping_cmd(pa_chg, 81, 5, PA_HIGH, 5);
    }  
    else if (c == 'h') {
      Serial.println("\n*** CHANGING INCOMING TX PA TO HIGH");
      send_ping_cmd(pa_chg, 81, 2, PA_HIGH, 2);
    } 
    else if (c == 'X') {
      Serial.println("\n*** CHANGING INCOMING TX PA TO MAX");
      send_ping_cmd(pa_chg, 81, 5, PA_MAX, 5);
    } 
    else if (c == 'x') {
      Serial.println("\n*** CHANGING INCOMING TX PA TO MAX");
      send_ping_cmd(pa_chg, 81, 2, PA_MAX, 2);
    } 
    
    ////////////////////////////////////////////
    /// Change radio number of Slave TX Node ///
    ////////////////////////////////////////////
       
    //////////////////////////////////////////////////
    /// Back and Forth between Radios 1 and  2
    else if (c == '1') {
      Serial.println("\n*** CHANGING TX Radio to 1");
      send_ping_cmd(rad_chg, 81, 1, PA_MIN, (2));
    } 
    else if (c == '2') {
      Serial.println("\n*** CHANGING TX Radio to 2");
      send_ping_cmd(rad_chg, 81, 2, PA_MIN, (1));
    } 
    
    //////////////////////////////////////////////////
    /// Back and Forth between Radios 3 and 5
    else if (c == '#') {
      Serial.println("\n*** CHANGING TX Radio 3 to 5");
      send_ping_cmd(rad_chg, 81, 5, PA_MIN, (3));
    } 
    else if (c == '3') {
      Serial.println("\n*** CHANGING TX Radio 5 to 3");
      send_ping_cmd(rad_chg, 81, 3, PA_MIN, (5));
    }    
    //////////////////////////////////////////////////
    /// Back and Forth between Radios 4 and 5
    else if (c == '$') {
      Serial.println("\n*** CHANGING TX Radio 4 to 5");
      send_ping_cmd(rad_chg, 81, 5, PA_MIN, (4));
    } 
    else if (c == '4') {
      Serial.println("\n*** CHANGING TX Radio 5 to 4");
      send_ping_cmd(rad_chg, 81, 4, PA_MIN, (5));
    }     
    //////////////////////////////////////////////////
    /// Back and Forth between Radios 3 and 4
    else if (c == '^') {
      Serial.println("\n*** CHANGING TX Radio 3 to 4");
      send_ping_cmd(rad_chg, 81, 4, PA_MIN, (3));
    } 
    else if (c == '6') {
      Serial.println("\n*** CHANGING TX Radio 4 to 3");
      send_ping_cmd(rad_chg, 81, 3, PA_MIN, (4));
    }      
    //////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////   
    else if (c == 'e') {
      Serial.println("\n*** CHANGING TX Radio 5 channel TO 76");
      send_ping_cmd(chan_chg, 81, 5, PA_MIN, 5);
//      setupMasterRadio(81,PA_MIN); 
    }   
    else if (c == 's') {
      Serial.println("\n*** CHANGING TX Radio 5 channel TO 81");
      send_ping_cmd(chan_chg, 81, 5, PA_MIN, 5);
//      setupMasterRadio(81,PA_MIN); 
    } 
    else if (c == 'K') {
      Serial.println("\n*** CHECKING TX RADIO PARAMETERS");      
      send_ping_cmd(nada, 81, 5, PA_MIN, 5);
    } 
        
    /////////////////////////////////////////
    /// CHECK MY RADIO CONNECTION
    else if (c == 'R') {
      Serial.println("\n*** CHECK MY RADIO CONNECTION");
      checkRadioConn();
    } 
    
    /////////////////////////////////////////
    /// Change channel number of Master radio
    else if (c == 'E') {
      Serial.println("\n*** CHANGING MASTER TO CH 76");
      setupMasterRadio(76,PA_MIN); 
      delay(5000);  
    }
    else if (c == 'S') {
      Serial.println("\n*** CHANGING MASTER TO CH 81");
      setupMasterRadio(81,PA_MIN); 
      delay(5000);
    }
    Serial.println("\n*** Serial Command sent to TX...Good Luck...");
    radio.startListening();
  }
}

//================================
//================================
/// https://forum.arduino.cc/t/serial-input-from-multiple-characters/863789/6
const byte numChars = 8;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;

void recvWithEndMarker(char rc) {
    static byte ndx = 0;
    char endMarker = '\n';
    if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
            ndx = numChars - 1;
        }
    }
    else {
        receivedChars[ndx] = '\0'; // terminate the string
        ndx = 0;
        newData = true;
    } 
}

void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        newData = false;
    }
}

//===================================
//======= Setup Master Radio ========
//===================================

void setupMasterRadio(int mychannel,PA_type pat) { 
    radio.begin(); // Start the NRF24L01
    radio.setDataRate( RF24_250KBPS ); // R   F24_1MBPS, RF24_2MBPS
    setPowerLevel(pat); // MIN, LOW, HIGH, and MAX
    //radio.setPALevel(RF24_PA_MIN); // RF24_PA_MIN, LOW, HIGH, and MAX
    radio.setChannel(mychannel);
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1, addresses[1]);
    radio.openReadingPipe(2, addresses[2]); 
    radio.openReadingPipe(3, addresses[3]); 
    radio.openReadingPipe(4, addresses[4]); 
    radio.openReadingPipe(5, addresses[5]); 
    radio.startListening(); // Listen to see if information received
  //    Serial.println("Channel in setup: " + String(radio.getChannel())); // DBPRINT
    myData.chNum = mychannel;
    myData.radNum = 0; // 0 is the master controller's number
//    Serial.print("Master RF Comms Starting on channel "+String(myData.chNum));
//    Serial.println("\tusing radio "+ String(myData.radNum));
//    delay(1000);
    checkRadioConn();
}

//======================================
//======= Check Radio Connection =======
//======================================
void checkRadioConn() {    
    printf_begin(); // Call this before calling printDetails()
    if (radio.isChipConnected()) {
      Serial.println("\nMaster Radio is connected to Channel: " + String(radio.getChannel()));
      radio.printDetails();  
      Serial.println("RF Comms Starting...");
    }else{
      Serial.println("\nRadio is not connected; showing Channel: " + String(radio.getChannel()));
      radio.printDetails();
    }  
}

//===========================
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

/*-------------------------------------------------------*/
/*------------ End of emacs_RX_Master_03.ino ------------*/
/*-------------------------------------------------------*/

//=============================================== //
// ======= Comments and Experimental Code ======= //
//=============================================== //


/** https://reference.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/

SEN-12642 Audio Sensor

mode: defines when the interrupt should be triggered. 
Four constants are predefined as valid mode values:

LOW to trigger the interrupt whenever the pin is low,
CHANGE to trigger the interrupt whenever the pin changes value
RISING to trigger when the pin goes from low to high,
FALLING for when the pin goes from high to low.
**/

//================================
//======= Send Ping Method =======
//================================
void send_ping_cmd_target(pingCmd PM[1]){
    
    Serial.println("PM CMD Type: "      +String(PM[0].my_cmd));
    Serial.println("PM Channel: "       +String(PM[0].ch_cmd));
    Serial.println("PM Radio Number: "  +String(PM[0].r_cmd));
    Serial.println("PM Pwr Level: "     +String(PM[0].pa_cmd));
    Serial.println("PM Target: "        +String(PM[0].tar_cmd)); 
    Serial.println("PM Alm Level: "     +String(PM[0].alm_cmd));
    Serial.println("PM Dng Level: "     +String(PM[0].dng_cmd)); 

    radio.stopListening(); // Change to TX role just long enough to ping cmd     
    radio.openWritingPipe(addresses[0]);
    radio.write(&PM, sizeof(PM));
    radio.startListening();
}
   
//======================================//
//============ Decision Tree ===========//
//======================================//
/* 
 *  CHANGE RADIO NUMBER WITH CAUTION!!
 * NOTE about calling rad_chg: It is a general 
 * rule not to change the radio number 
 * of a TX node when sending serial commands.  * 
 * This precludes possible pipe collisions, 
 * as TX pipes are assigned by radio number. 
 * 
 * Channel 76:  Environmental Monitor
 * Channel 81:  Security Monitor
 * 
*/
       
//    TempMessage[0].my_cmd    = nada;
//    TempMessage[0].ch_cmd    = 0;
//    TempMessage[0].r_cmd     = 0;
//    TempMessage[0].pa_cmd    = 0;
//    TempMessage[0].tar_cmd   = 0;
//    TempMessage[0].alm_cmd   = ALM_NONE;
//    TempMessage[0].dng_cmd   = DNG_NONE;

    //receivedChars[];    
    //TempMessage[1] = PingMessage[0]; 
  
void cmd_decision_tree() {
  //if (Serial.available()) {

//    recvWithEndMarker(); // returns receivedChars[8] from Serial Monitor;
//    showNewData();
    int i = 0;
  
    /// Instantiate and initialize a temporary pingCmd struct
    struct pingCmd TempMessage[1] = {}; 
    
    char c = receivedChars[0];
    Serial.println("\n*** MSG CMD TYPE: nada\t" + String(c));
    /*
    switch (c) {
      case '0':
          TempMessage[0].my_cmd = nada;
          Serial.println("\n*** MSG CMD TYPE: nada\t" + String(c));  
          
          Serial.println("\n*** MSG CHANNEL NUMBER\t" + String(receivedChars[1])); 
          if (receivedChars[1] == '8') TempMessage[0].ch_cmd = 81;
          else if (receivedChars[1] == '7') TempMessage[0].ch_cmd = 76;
          else TempMessage[1].ch_cmd = 81;
          
          Serial.println("\n*** MSG RADIO NUMBER" + String(receivedChars[2])); 
          //receivedChars[2] = ((receivedChars[2] >= '1') && (receivedChars[2] <= '5')) ? receivedChars[2] : 1;
          TempMessage[0].r_cmd = receivedChars[2];
              
          Serial.println("\n*** MSG POWER NUMBER" + String(c)); 
          //receivedChars[3] = ((receivedChars[3] >= '0') && (receivedChars[3] <= '3')) ? receivedChars[3] : 1;
          TempMessage[0].pa_cmd = receivedChars[3]; 
          
          Serial.println("\n*** MSG TARGET NUMBER" + String(receivedChars[4])); 
          //receivedChars[4] = ((receivedChars[4] >= '1') && (receivedChars[4] <= '5')) ? receivedChars[4] : 1;
          TempMessage[0].tar_cmd = receivedChars[4]; 
          
          Serial.println("\n*** MSG ALARM NUMBER" + String(receivedChars[5])); 
          //receivedChars[5] = ((receivedChars[5] >= '0') && (receivedChars[5] <= '11')) ? receivedChars[5] : 1;
          TempMessage[0].tar_cmd = receivedChars[5];   
          
          Serial.println("\n*** MSG DANGER NUMBER" + String(receivedChars[6])); 
          //receivedChars[6] = ((receivedChars[6] >= '1') && (receivedChars[6] <= '5')) ? receivedChars[6] : 1;
          TempMessage[0].tar_cmd = receivedChars[6];              

          break;

          ///auto result = Test(a);
                      
//            PingMessage[0] = TempMessage[0];
//            send_ping_cmd_target(PingMessage); 
//          
//            Serial.println("\nNADA\nTempMessage[0].my_cmd:\t" + String(TempMessage[0].my_cmd)
//                          + "\nTempMessage[0].ch_cmd:\t" + String(TempMessage[0].ch_cmd)
//                          + "\nTempMessage[0].r_cmd:\t" + String(TempMessage[0].r_cmd)
//                          + "\nTempMessage[0].pa_cmd:\t" + String(TempMessage[0].pa_cmd)
//                          + "\nTempMessage[0].tar_cmd:\t" + String(TempMessage[0].tar_cmd)
//                          + "\nTempMessage[0].alm_cmd:\t" + String(TempMessage[0].alm_cmd)
//                          + "\nTempMessage[0].dng_cmd:\t" + String(TempMessage[0].dng_cmd));

      case '1':
          TempMessage[0].my_cmd = chan_chg;
          Serial.println("\n*** MSG CMD TYPE: chan_chg\t" + String(c));  
          
          Serial.println("\n*** MSG CHANNEL NUMBER\t" + String(receivedChars[1])); 
          if (receivedChars[1] == '8') TempMessage[0].ch_cmd = 81;
          else if (receivedChars[1] == '7') TempMessage[0].ch_cmd = 76;
          else TempMessage[1].ch_cmd = 81;
          
          Serial.println("\n*** MSG RADIO NUMBER" + String(receivedChars[2])); 
          //receivedChars[2] = ((receivedChars[2] >= '1') && (receivedChars[2] <= '5')) ? receivedChars[2] : 1;
          TempMessage[0].r_cmd = receivedChars[2];
              
          Serial.println("\n*** MSG POWER NUMBER" + String(c)); 
          //receivedChars[3] = ((receivedChars[3] >= '0') && (receivedChars[3] <= '3')) ? receivedChars[3] : 1;
          TempMessage[0].pa_cmd = receivedChars[3];  
          
          Serial.println("\n*** MSG TARGET NUMBER" + String(receivedChars[4])); 
          //receivedChars[4] = ((receivedChars[4] >= '1') && (receivedChars[4] <= '5')) ? receivedChars[4] : 1;
          TempMessage[0].tar_cmd = receivedChars[4]; 
          
          Serial.println("\n*** MSG ALARM NUMBER" + String(receivedChars[5])); 
          //receivedChars[5] = ((receivedChars[5] >= '0') && (receivedChars[5] <= '11')) ? receivedChars[5] : 1;
          TempMessage[0].tar_cmd = receivedChars[5];   
          
          Serial.println("\n*** MSG DANGER NUMBER" + String(receivedChars[6])); 
          //receivedChars[6] = ((receivedChars[6] >= '1') && (receivedChars[6] <= '5')) ? receivedChars[6] : 1;
          TempMessage[0].tar_cmd = int(receivedChars[6]);              

          break;

      case '2':
          TempMessage[0].my_cmd = rad_chg;
          Serial.println("\n*** MSG CMD TYPE: rad_chg\t" + String(c));  
          
          Serial.println("\n*** MSG CHANNEL NUMBER\t" + String(receivedChars[1])); 
          if (receivedChars[1] == '8') TempMessage[0].ch_cmd = 81;
          else if (receivedChars[1] == '7') TempMessage[0].ch_cmd = 76;
          else TempMessage[1].ch_cmd = 81;
          
          Serial.println("\n*** MSG RADIO NUMBER" + String(receivedChars[2])); 
          //receivedChars[2] = ((receivedChars[2] >= '1') && (receivedChars[2] <= '5')) ? receivedChars[2] : 1;
          TempMessage[0].r_cmd = receivedChars[2];
              
          Serial.println("\n*** MSG POWER NUMBER" + String(c)); 
          //receivedChars[3] = ((receivedChars[3] >= '0') && (receivedChars[3] <= '3')) ? receivedChars[3] : 1;
          TempMessage[0].pa_cmd = receivedChars[3];  
          
          Serial.println("\n*** MSG TARGET NUMBER" + String(receivedChars[4])); 
          //receivedChars[4] = ((receivedChars[4] >= '1') && (receivedChars[4] <= '5')) ? receivedChars[4] : 1;
          TempMessage[0].tar_cmd = receivedChars[4]; 
          
          Serial.println("\n*** MSG ALARM NUMBER" + String(receivedChars[5])); 
          //receivedChars[5] = ((receivedChars[5] >= '0') && (receivedChars[5] <= '11')) ? receivedChars[5] : 1;
          TempMessage[0].tar_cmd = receivedChars[5];   
          
          Serial.println("\n*** MSG DANGER NUMBER" + String(receivedChars[6])); 
          //receivedChars[6] = ((receivedChars[6] >= '1') && (receivedChars[6] <= '5')) ? receivedChars[6] : 1;
          TempMessage[0].tar_cmd = int(receivedChars[6]);              

          break;

      case '3':
          TempMessage[0].my_cmd = pa_chg;
          Serial.println("\n*** MSG CMD TYPE: pa_chg\t" + String(c));  
          
          Serial.println("\n*** MSG CHANNEL NUMBER\t" + String(receivedChars[1])); 
          if (receivedChars[1] == '8') TempMessage[0].ch_cmd = 81;
          else if (receivedChars[1] == '7') TempMessage[0].ch_cmd = 76;
          else TempMessage[1].ch_cmd = 81;
          
          Serial.println("\n*** MSG RADIO NUMBER" + String(receivedChars[2])); 
          //receivedChars[2] = ((receivedChars[2] >= '1') && (receivedChars[2] <= '5')) ? receivedChars[2] : 1;
          TempMessage[0].r_cmd = receivedChars[2];
              
          Serial.println("\n*** MSG POWER NUMBER" + String(c)); 
          //receivedChars[3] = ((receivedChars[3] >= '0') && (receivedChars[3] <= '3')) ? receivedChars[3] : 1;
          TempMessage[0].pa_cmd = receivedChars[3];  
          
          Serial.println("\n*** MSG TARGET NUMBER" + String(receivedChars[4])); 
          //receivedChars[4] = ((receivedChars[4] >= '1') && (receivedChars[4] <= '5')) ? receivedChars[4] : 1;
          TempMessage[0].tar_cmd = receivedChars[4]; 
          
          Serial.println("\n*** MSG ALARM NUMBER" + String(receivedChars[5])); 
          //receivedChars[5] = ((receivedChars[5] >= '0') && (receivedChars[5] <= '11')) ? receivedChars[5] : 1;
          TempMessage[0].tar_cmd = receivedChars[5];   
          
          Serial.println("\n*** MSG DANGER NUMBER" + String(receivedChars[6])); 
          //receivedChars[6] = ((receivedChars[6] >= '1') && (receivedChars[6] <= '5')) ? receivedChars[6] : 1;
          TempMessage[0].tar_cmd = int(receivedChars[6]);              

          break;

      case '4':
          TempMessage[0].my_cmd = all_chg;
          Serial.println("\n*** MSG CMD TYPE: all_chg\t" + String(c));  
          
          Serial.println("\n*** MSG CHANNEL NUMBER\t" + String(receivedChars[1])); 
          if (receivedChars[1] == '8') TempMessage[0].ch_cmd = 81;
          else if (receivedChars[1] == '7') TempMessage[0].ch_cmd = 76;
          else TempMessage[1].ch_cmd = 81;
          
          Serial.println("\n*** MSG RADIO NUMBER" + String(receivedChars[2])); 
          //receivedChars[2] = ((receivedChars[2] >= '1') && (receivedChars[2] <= '5')) ? receivedChars[2] : 1;
          TempMessage[0].r_cmd = receivedChars[2];
              
          Serial.println("\n*** MSG POWER NUMBER" + String(c)); 
          //receivedChars[3] = ((receivedChars[3] >= '0') && (receivedChars[3] <= '3')) ? receivedChars[3] : 1;
          TempMessage[0].pa_cmd = receivedChars[3];  
          
          Serial.println("\n*** MSG TARGET NUMBER" + String(receivedChars[4])); 
          //receivedChars[4] = ((receivedChars[4] >= '1') && (receivedChars[4] <= '5')) ? receivedChars[4] : 1;
          TempMessage[0].tar_cmd = receivedChars[4]; 
          
          Serial.println("\n*** MSG ALARM NUMBER" + String(receivedChars[5])); 
          //receivedChars[5] = ((receivedChars[5] >= '0') && (receivedChars[5] <= '11')) ? receivedChars[5] : 1;
          TempMessage[0].tar_cmd = receivedChars[5];   
          
          Serial.println("\n*** MSG DANGER NUMBER" + String(receivedChars[6])); 
          //receivedChars[6] = ((receivedChars[6] >= '1') && (receivedChars[6] <= '5')) ? receivedChars[6] : 1;
          TempMessage[0].tar_cmd = int(receivedChars[6]);              

          break;

      case '5':
          TempMessage[0].my_cmd = tar_chg;
          Serial.println("\n*** MSG CMD TYPE: tar_chg\t" + String(c));  
          
          Serial.println("\n*** MSG CHANNEL NUMBER\t" + String(receivedChars[1])); 
          if (receivedChars[1] == '8') TempMessage[0].ch_cmd = 81;
          else if (receivedChars[1] == '7') TempMessage[0].ch_cmd = 76;
          else TempMessage[1].ch_cmd = 81;
          
          Serial.println("\n*** MSG RADIO NUMBER" + String(receivedChars[2])); 
          //receivedChars[2] = ((receivedChars[2] >= '1') && (receivedChars[2] <= '5')) ? receivedChars[2] : 1;
          TempMessage[0].r_cmd = receivedChars[2];
              
          Serial.println("\n*** MSG POWER NUMBER" + String(c)); 
          //receivedChars[3] = ((receivedChars[3] >= '0') && (receivedChars[3] <= '3')) ? receivedChars[3] : 1;
          TempMessage[0].pa_cmd = receivedChars[3];  
          
          Serial.println("\n*** MSG TARGET NUMBER" + String(receivedChars[4])); 
          //receivedChars[4] = ((receivedChars[4] >= '1') && (receivedChars[4] <= '5')) ? receivedChars[4] : 1;
          TempMessage[0].tar_cmd = receivedChars[4]; 
          
          Serial.println("\n*** MSG ALARM NUMBER" + String(receivedChars[5])); 
          //receivedChars[5] = ((receivedChars[5] >= '0') && (receivedChars[5] <= '11')) ? receivedChars[5] : 1;
          TempMessage[0].tar_cmd = receivedChars[5];   
          
          Serial.println("\n*** MSG DANGER NUMBER" + String(receivedChars[6])); 
          //receivedChars[6] = ((receivedChars[6] >= '1') && (receivedChars[6] <= '5')) ? receivedChars[6] : 1;
          TempMessage[0].tar_cmd = int(receivedChars[6]);              

          break;

      case '6':
          TempMessage[0].my_cmd = response;
          Serial.println("\n*** MSG CMD TYPE: response\t" + String(c));  
          
          Serial.println("\n*** MSG CHANNEL NUMBER\t" + String(receivedChars[1])); 
          if (receivedChars[1] == '8') TempMessage[0].ch_cmd = 81;
          else if (receivedChars[1] == '7') TempMessage[0].ch_cmd = 76;
          else TempMessage[1].ch_cmd = 81;
          
          Serial.println("\n*** MSG RADIO NUMBER" + String(receivedChars[2])); 
          //receivedChars[2] = ((receivedChars[2] >= '1') && (receivedChars[2] <= '5')) ? receivedChars[2] : 1;
          TempMessage[0].r_cmd = receivedChars[2];
              
          Serial.println("\n*** MSG POWER NUMBER" + String(c)); 
          //receivedChars[3] = ((receivedChars[3] >= '0') && (receivedChars[3] <= '3')) ? receivedChars[3] : 1;
          TempMessage[0].pa_cmd = receivedChars[3];  
          
          Serial.println("\n*** MSG TARGET NUMBER" + String(receivedChars[4])); 
          //receivedChars[4] = ((receivedChars[4] >= '1') && (receivedChars[4] <= '5')) ? receivedChars[4] : 1;
          TempMessage[0].tar_cmd = receivedChars[4]; 
          
          Serial.println("\n*** MSG ALARM NUMBER" + String(receivedChars[5])); 
          //receivedChars[5] = ((receivedChars[5] >= '0') && (receivedChars[5] <= '11')) ? receivedChars[5] : 1;
          TempMessage[0].tar_cmd = receivedChars[5];   
          
          Serial.println("\n*** MSG DANGER NUMBER" + String(receivedChars[6])); 
          //receivedChars[6] = ((receivedChars[6] >= '1') && (receivedChars[6] <= '5')) ? receivedChars[6] : 1;
          TempMessage[0].tar_cmd = int(receivedChars[6]);              

          break; 
           
      default:
          break;        
    } */

    /////////////////////////////////////////
    /// CHECK MY RADIO CONNECTION
    if (c == 'R') {
      Serial.println("\n*** CHECK MY RADIO CONNECTION");
      checkRadioConn();
    }     
    /////////////////////////////////////////
    /// Change channel number of Master radio
    if (c == 'E') {
      Serial.println("\n*** CHANGING MASTER TO CH 76");
      setupMasterRadio(76,PA_MIN); 
      delay(5000);  
    }
    if (c == 'S') {
      Serial.println("\n*** CHANGING MASTER TO CH 81");
      setupMasterRadio(81,PA_MIN); 
      delay(5000);
    }
    PingMessage[0] = TempMessage[0];
    Serial.println("\n*** Serial Command sent to TX...Good Luck...");
    radio.startListening();
  //}
}
