# EMACS-
Environmental Monitoring and Control System

Written for Arduino MEGA, UNO, and NANO, and ESP32-WROOM

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
       
