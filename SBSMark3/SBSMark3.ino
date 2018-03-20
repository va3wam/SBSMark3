/*************************************************************************************************************************************
  SBS-MARK3 (SegbotSTEP generation 3) 
  This code is used to control a Segbot which is a two wheeled self balancing robot. This generation of the platform is built around
  the Adafriut Huzzah32 - ESP32 Featherboard. 

  WiFi Notes:
  - ESP32 cannot connect to 5.0GHz signals, only 2.4GHz signals
    
  Helpful links:
  Board Pinout details: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts
  Setting up Arduino IDE: https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/using-with-arduino-ide
  Async web server example: https://techtutorialsx.com/2017/12/01/esp32-arduino-asynchronous-http-webserver/
  
  History
  Version YYYY-MM-DD Description
*/   
  String my_ver="1.0";
/*
  ------- ---------- -----------------------------------------------------------------------------------------------------------------
  1.0     2018-03-17 Code base created
 *************************************************************************************************************************************/
#include <stdio.h>                                                           // Standard I/O library
#include <WiFiMulti.h>                                                       // Allows us to try connecting to multiple Access Points
#include <AsyncTCP.h>                                                        // https://github.com/me-no-dev/AsyncTCP 
                                                                             // Required for <ESPAsyncWebServer.h>
#include <ESPAsyncWebServer.h>                                               // https://github.com/me-no-dev/ESPAsyncWebServer#async-websocket-plugin
                                                                             // Async HTTP and WebSocket Server
#include <Wire.h>                                                            // Needed for I2C communication
#include <LiquidCrystal_I2C.h>                                               // https://github.com/marcoschwartz/LiquidCrystal_I2C
 
/*************************************************************************************************************************************
 Doug's shortcuts
 *************************************************************************************************************************************/
#define LINE(name,val) Serial.print(name); Serial.print("\t"); Serial.println(val); //Debug macro, prints current code line
#define sp Serial.print                                                      // Shortform print no carrige return
#define spl Serial.println                                                   // Shortform print with carrige return
#define spc Serial.print(", ")                                               // Shortform print comma and space
#define spf Serial.printf                                                    // Shortform formatted printing with no line return
#define spdl(label,val) sp(label); spl(val)                                  // displays one labelled variable, ending with new-line 
                                                                             // example: spdl(" param2= ",param2);     
#define spd(label,val) sp(label); sp(val)                                    // suitable for displaying multiple variables per line
                                                                             // example: spd("left motor= ",left_motor); spdl("  right motor= ",right_motor);
/*************************************************************************************************************************************
 Define network objects and services
 *************************************************************************************************************************************/
static const char ssid0[] = "MN_BELL418";                                    // The name of a Wi-Fi network AP to connect to
static const char ssid1[] = "MN_WORKSHOP_2.4GHz";                            // The name of a Wi-Fi network AP to connect to
static const char ssid2[] = "MN_DS_OFFICE_2.4GHz";                           // The name of a Wi-Fi network AP to connect to
static const char ssid3[] = "MN_OUTSIDE";                                    // The name of a Wi-Fi network AP to connect to
static const char password[] = "5194741299";                                 // The password for all of the Wi-Fi network APs
static const char *wsEvent[] = { "WStype_DISCONNECTED", "WStype_CONNECTED", "WStype_TEXT", "WStype_BIN"};
static WiFiMulti WiFiMulti;                                                  // Allows us to connect to one of a number of APs.
static AsyncWebServer server(80);                                            // Define object to handle websocket & web page requests 

/*************************************************************************************************************************************
 Define MPU6050 related variables
 *************************************************************************************************************************************/ 
#define MPU6050_WHO_AM_I 0x75                                                // Read only register on IMU with info about the device
#define MPU_address 0x68                                                     // MPU-6050 I2C address. Note that AD0 pin on the
                                                                             // board cannot be left flaoting and must be connected
                                                                             // to the Arduino ground for address 0x68 or be connected
                                                                             // to VDC for address 0x69.
byte IMU_FOUND = false;                                                      // Flag to see if MPU found on I2C bus
int acc_x;                                                                   // Read raw low and high byte to the MPU acc_x register
int acc_y;                                                                   // Read raw low and high byte to the MPU acc_y register
int acc_z;                                                                   // Read raw low and high byte to the MPU acc_z register
int gyro_x;                                                                  // Read raw low and high byte to the MPU gyro_x register
int gyro_y;                                                                  // Read raw low and high byte to the MPU gyro_y register
int gyro_z;                                                                  // Read raw low and high byte to the MPU gyro_z register
int temperature;                                                             // Read raw low and high byte to the MPU temperature register
int tmp;                                                                     // Used to do different calculations to extend the sign bit 
                                                                             // of raw data 
/*************************************************************************************************************************************
 Define PID and motor control variables and constants
 Observed top speed = 300 for step interval > 10.47 inches per second
 Observed slowest speed = 2300 step interval > 1.37 inches per second
 *************************************************************************************************************************************/
long pcnt;                                                                   // This is used for some JSON message ping test timing
int acc_calibration_value;                                                   // Balance point of robot when standing at 90 degrees.
                                                                             // To get this value stand the robot upright at 90 degrees
                                                                             // and watch the serial trace during boot up. Note that
                                                                             // the of this variable is set in the startWiFi()
                                                                             // function based on MAC address of ESP8266
                                                                             // Gets set during start up.
const volatile int speed = -1;                                               // for initial testing of interrupt driven steppers
                                                                             // speed = -1 enables IMU based balancing. 
                                                                             // speed = n enables fixed forward speed interval of n, 0 
                                                                             // is brakes on
const int bot_slow = 2300;                                                   // # of interrupts between steps at slowest workable bot speed
const int bot_fast = 250;                                                    // # of interrupts between steps at fastest workable bot speed
const float PID_I_fade = .80;                                                // How much of pid_i_mem history to retain

// ---------------------------------------------------------------------------- Various PID settings
float pid_p_gain = 30;                                                       // Gain setting for the P-controller (15)
float pid_i_gain = 1.2;                                                      // Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                                       // Gain setting for the D-controller (30)
float turning_speed = 30;                                                    // Turning speed (20)
float max_target_speed = 150;                                                // Max target speed (100)
const long usec_per_t0_int = 20;                                             // number of microseconds between t0 timer interrupts
const int pid_max = 400;                                                     // Upper bound for pid values
const int pid_min = -400;                                                    // Lower bound for pid values

// ---------------------------------------------------------------------------- Define the GPIO pins that perform functions on A4988 
                                                                             // controller below is corrected to reflect cross-wired DIR & 
                                                                             // STEP on MARK2 motherboard
#define pin_left_step 12                                                     // GPIO 12 initiates a step on motor on robot's left side (top A4988)
#define pin_left_dir 14                                                      // GPIO 14 controls the direction of left motor
#define pin_right_step 15                                                    // GPIO 15 initiates a step on motor on robot's right side (bottom A4988)
#define pin_right_dir 13                                                     // GPIO 13 controls the direction of right motor

/*************************************************************************************************************************************
 Declaring global variables and setting some values (YABR cut and paste)
 *************************************************************************************************************************************/ 
byte start, received_byte=0, low_bat;
volatile int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
volatile int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;
long gyro_yaw_calibration_value, gyro_pitch_calibration_value, balance_calibration_value;
unsigned long loop_timer;
float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;
float hold, hold1, hold2, hold3, hold4, hold5;
volatile long t0_per_sec;
int vertical_calibration;                                                    // This is calculated and displayed if speed == 0, which puts the brakes on
volatile float ftemp;                                                        // Temp replacement calculation to replace onestep
long mics, loop_mics, last_millis;                                           // Measure length of loop, and print occasionally
int t0_count;
int dump_count;                                                              // Need to differentiate multiple dumps form same compile
long t,t_old,d;
int millis_now, i=1;
long mil, mic;
volatile long int_time;
volatile int h_flag;                                                         // Flag to capture and hold one vector of debug interrupt times
                                                                                
/*************************************************************************************************************************************
 Define LCD related variables. 
 *************************************************************************************************************************************/ 
#define LCD_NO_MESSAGE ""                                                    // Blank message to scroll old messages off screen 
#define SCROLL 2                                                             // Tell LCD to scroll full screen, both lines 
#define LINE1 0                                                              // Tell LCD to diplay message on line 1
#define LINE2 1                                                              // Tell LCD to diplay message on line 2
const byte lcdAddr = 0x38;                                                   // LCD I2C address for old LCD 
//const byte lcdAddr = 0x3F;                                                   // LCD I2C address for new LCD 
const byte lcdCols = 16;                                                     // LCD number of characters in a row
const byte lcdRows = 2;                                                      // LCD number of lines
const unsigned int scrollDelay = 500;                                        // Miliseconds before scrolling next char
LiquidCrystal_I2C lcd(lcdAddr,lcdCols,lcdRows);                              // Define LCD object. Value re-assigned in WiFiStart()
byte LCD_FOUND = false;                                                      // Flag to see if LCD found on I2C bus
String LCDmsg0 = "";                                                         // Track message displayed in line 1 of LCD
String LCDmsg1 = "";                                                         // Track message displayed in line 2 of LCD

/*************************************************************************************************************************************
 Define non-device specific I2C related variables. Device specific variables like addresses are found in device specific sections.
 *************************************************************************************************************************************/ 
byte I2C_UNKNOWN = false;                                                    // Flag if unknown device found on I2C bus

/*************************************************************************************************************************************
 This function dumps a bunch of useful info to the terminal. This code is based on an example we found at this URL:
 https://stackoverflow.com/questions/14143517/find-the-name-of-an-arduino-sketch-programmatically                                   
 *************************************************************************************************************************************/
void display_Running_Sketch()
{                                 
  
   sp("[display_Running_Sketch] Sketch Name: ");spl(__FILE__);
   sp("[display_Running_Sketch] Version: "); spl(my_ver);
   sp("[display_Running_Sketch] Compiled on: ");sp(__DATE__);sp(" at ");spl(__TIME__);
   LINE("[display_Running_Sketch] Current line of code test: ", __LINE__);

} //display_Running_Sketch()

/*************************************************************************************************************************************
 This function connects to the local Access Point and then starts up a a socket server to listen for client connections. This code is 
 based on an exmaple we found at this URL: 
 https://stackoverflow.com/questions/14143517/find-the-name-of-an-arduino-sketch-programmatically      
 STATUS return codes: https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/include/wl_definitions.h    
 typedef enum {
    WL_NO_SHIELD        = 255 (for compatibility with WiFi Shield library)
    WL_IDLE_STATUS      = 0,
    WL_NO_SSID_AVAIL    = 1,
    WL_SCAN_COMPLETED   = 2,
    WL_CONNECTED        = 3,
    WL_CONNECT_FAILED   = 4,
    WL_CONNECTION_LOST  = 5,
    WL_DISCONNECTED     = 6
} wl_status_t;                         
 *************************************************************************************************************************************/
void startWiFi()
{  

   spl("[startWiFi] Scanning for and connecting to the strongest Access Point signal from a known list...");
   WiFiMulti.addAP(ssid0, password);                                         // Add Wi-Fi AP we may be able to connect to
   WiFiMulti.addAP(ssid1, password);                                         // Add Wi-Fi AP we may be able to connect to
   WiFiMulti.addAP(ssid2, password);                                         // Add Wi-Fi AP we may be able to connect to
   WiFiMulti.addAP(ssid3, password);                                         // Add Wi-Fi AP we may be able to connect to
   while(WiFiMulti.run() != WL_CONNECTED)                                    // Wait for connection to the strongest signalling Wi-Fi AP 
   {                                                               
      sp(".");                                                               // Send dot to console terminal to show the waiting process 
                                                                             // is active
      delay(100);                                                            // Wait a little before trying again
   } //while
   spl("");                                             
   sp("[startWiFi] Connected to Access Point ");                                        
   spl(WiFi.SSID());                                                         // Name of AP to which the ESP8266 is connected to
   sp("[startWiFi] IP address: ");                                   
   spl(WiFi.localIP());                                                      // IP address assigned to ESP8266 by AP
   sp("[startWiFi] MAC address: ");                                   
   spl(WiFi.macAddress());                                                   // IP address of the ESP8266

} //startWiFi()

/*************************************************************************************************************************************
 This function configures the route where the web service will be listening for incoming HTTP_GET requests as well as the functions 
 that will be executed when a request is received on that route as well as what to do if an invalid route is requested. Note that 
 the only vald rout being set up is "/".
 *************************************************************************************************************************************/
void startWebServer()
{

//  server.on("/", handleRoot);                                               // Attach function for handling root page (/)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(200, "text/plain", "Hello World");
  });
  server.begin();                                                           // Start web service
  spl("[startWebServer] Started web server");                               

} //startWebServer()

/*************************************************************************************************************************************
 This function configures the route where the web service will be listening for incoming HTTP_GET requests as well as the functions 
 that will be executed when a request is received on that route as well as what to do if an invalid route is requested. Note that 
 the only vald rout being set up is "/".
 *************************************************************************************************************************************/
void webSocketEvent()
{
  spl("[webSocketEvent] Web socket event occured");                               
} //webSocketEvent()

/*************************************************************************************************************************************
 This function scans the I2C bus for attached devices. This code was taken from http://playground.arduino.cc/Main/I2cScanner 
 *************************************************************************************************************************************/
void startI2Cbus()
{
      
   byte error, address;
   int nDevices;  
   spl("[startI2Cbus] Initialize I2C bus");
   Wire.begin(); 
   spl("[startI2Cbus] Scanning I2C bus...");
   nDevices = 0;
   for(address = 1; address < 127; address++ )
   {
      // The i2c_scanner uses the return value of the Write.endTransmisstion to see if a device did acknowledge to the address
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
     
      if (error == 0)
      {
         sp("[startI2Cbus] Device found at address 0x");
         if (address<16) 
         {
            sp("0");
         } //if
         sp(address,HEX);sp(" - ");
         nDevices++;
         switch(address)                                                     // Check each located device to see what it is
         {
            case MPU_address:                                                // Invensense MPU-6050
               spl("Invensense MPU-6050 IMU on GY-521 or ITG-MPU board");
               IMU_FOUND = true;                                             // Set flag indicating that MPU found on I2C bus
               break;
            case lcdAddr:                                                    // OpenSmart 1602 LCD Display 
               spl("OpenSmart 1602 LCD Display");
               LCD_FOUND = true;                                             // Set flag indicating that LCD found on I2C bus
               break;
            default:                                                         // Unknown websocket event
               spl("Unknown device");
               I2C_UNKNOWN = true;                                           // Indicate that an unknown I2C device has been found
               break;
         } //switch()          

       } //if
       else if (error==4)
       {
          sp("[startI2Cbus] Unknown error at address 0x");
          if (address<16) 
          {
             sp("0");
          } //if
          spl(address,HEX);spl(" ");
        } //else if    
   } //for
   if (nDevices == 0)
   {
      spl("[startI2Cbus] No I2C devices found");
   } //if
   else
   {
      spl("[startI2Cbus] I2C scan complete");
   } //else

   if(I2C_UNKNOWN)                                                           // Issue warning if I2C bus has unknown device 
   {
      spl("[startI2Cbus] WARNING - Unrecognized device detected on I2C bus. Boot sequence will continue."); 
   } //if

   if(!LCD_FOUND)                                                            // Issue error LCD not detected on I2C bus 
   {
      spl("[startI2Cbus] ERROR - LCD device not detected on I2C bus. Boot sequence halted."); 
   } //if

   if(!IMU_FOUND)                                                            // Issue error LCD not detected on I2C bus 
   {
      spl("[startI2Cbus] ERROR - IMU device not detected on I2C bus. Boot sequence halted."); 
   } //if

   while(!IMU_FOUND){};                                                      // If IMU is not found on I2C bus, halt boot up
   while(!LCD_FOUND){};                                                      // If LCD is not found on I2C bus, halt boot up

} //startI2Cbus()

/*************************************************************************************************************************************
 This function initializes the Open Smart 1602 LCD Display
 *************************************************************************************************************************************/
void initializeLCD()                                             
{

   lcd.clear();                                                              // Clear the LCD screen
   lcd.init();                                                               // Initialize the LCD object 
   lcd.backlight();                                                          // Turn on the LCD backlight
   flashLCD();                                                               // Flash the LCD backlight
  
} //initializeLCD()                                                          

/*************************************************************************************************************************************
 This function sends messages you pass to it to the LCD and displays it centered.
 *************************************************************************************************************************************/
void sendLCD(String LCDmsg, byte LCDline)
{

   byte textLen = LCDmsg.length();                                           // Length of message to send
   byte LCDcolumn = 0;                                                       // Starting column of message 
   if(LCDline > 1) LCDline=1;                                                // Ensure line argument is not too large                                   
   if(LCDline < 0) LCDline=0;                                                // Ensure line argument is not too small
   LCDcolumn=(lcdCols-textLen)/2;                                            // Figure out starting point to center message         

   // Clear line by sending blank message starting at first column, this ensures that previous messages are gone completely
   lcd.setCursor(0,LCDline);                                                 // Set cursor to correct location (line 1 or 2)
   lcd.print("                ");                                            // Send blank message to clear previous message 
   delay(5);                                                                 // Allow time for LCD to process message

   // Print actual message centered
   lcd.setCursor(LCDcolumn,LCDline);                                         // Set cursor to correct location (line 1 or 2)
   lcd.print(LCDmsg);                                                        // Send message to LCD 
   delay(5);                                                                 // Allow time for LCD to process message
   
   // Global variables track content of LCD line 1 & 2 to send to clients via JSON message
   if(LCDline == 0)                                                          // If this is the first line of the LCD
   {
      LCDmsg0 = LCDmsg;                                                      // Track current message in line 1 global variable
   } //if
   else                                                                      // If this is the second line of the LCD
   {
      LCDmsg1 = LCDmsg;                                                      // Track current message in line 2 global variable    
   } //else
   
} //sendLCD()

/*************************************************************************************************************************************
 This function scrolls a message from left to right on the LCD. Note that both lines of the display scroll. You can send a blank
 message to this function to scroll the current messages displayed on both lines off the LCD screen. Note that the second argument
 passed to this function is not used if a null message if passed.
*************************************************************************************************************************************/
void scrollLCD(String LCDmsg, byte LCDline)
{

   byte textLen = LCDmsg.length();                                           // Length of message to send
   byte LCDcolumn = 0;                                                       // Starting column of message 
   if(LCDline > 1) LCDline=1;                                                // Ensure line argument is not too large                                   
   if(LCDline < 0) LCDline=0;                                                // Ensure line argument is not too small
   if(LCDmsg != LCD_NO_MESSAGE)                                              // If this is not a blank message display it
   {
      lcd.setCursor(LCDcolumn,LCDline);                                      // Set LCD cursor to correct location 
      lcd.print(LCDmsg);                                                     // Send message to LCD
   } //if 
   for (byte positionCounter = 0; positionCounter < (textLen + lcdCols); positionCounter++) 
   {
      lcd.scrollDisplayRight();                                              // Scroll entire row to the right
      delay(scrollDelay);                                                    // Pause between scrolls
   } //for

} //scrollLCD()

/*************************************************************************************************************************************
 This function flashes the LCD backlight.
*************************************************************************************************************************************/
void flashLCD()   
{
   for (byte cnt = 0; cnt < 10; cnt++) 
   {
      lcd.backlight();                                                       // Turn on the LCD backlight
      delay(100);
      lcd.noBacklight();                                                     // Turn off backlight
      delay(100);
   } //for
   lcd.backlight();                                                          // Turn on the LCD backlight
   delay(100);

} //flashLCD()

/*************************************************************************************************************************************
 This function configures the MPU6050 using settings recommended by Joop Brokking
 *************************************************************************************************************************************/
void set_gyro_registers()
{

   spl("[set_gyro_registers] Configure the MPU6050...");                                                                          
   spl("[set_gyro_registers] Wake up MPU");                                  // By default the MPU-6050 sleeps. So we have to wake it up.
   Wire.beginTransmission(MPU_address);                                      // Start communication with the address found during search.
   Wire.write(0x6B);                                                         // We want to write to the PWR_MGMT_1 register (6B hex)
   Wire.write(0x00);                                                         // Set the register bits as 00000000 to activate the gyro
   Wire.endTransmission();                                                   // End the transmission with the gyro.

   // Set the full scale of the gyro to +/- 250 degrees per second
   spl("[set_gyro_registers] Set the full scale of the gyro to +/- 250 degrees per second");
   Wire.beginTransmission(MPU_address);                                      // Start communication with the address found during search.
   Wire.write(0x1B);                                                         // We want to write to the GYRO_CONFIG register (1B hex)
   Wire.write(0x00);                                                         // Set the register bits as 00000000 (250dps full scale)
   Wire.endTransmission();                                                   // End the transmission with the gyro
        
   // Set the full scale of the accelerometer to +/- 4g.
   spl("[set_gyro_registers] Set the full scale of the accelerometer to +/- 4g");
   Wire.beginTransmission(MPU_address);                                      // Start communication with the address found during search.
   Wire.write(0x1C);                                                         // We want to write to the ACCEL_CONFIG register (1A hex)
   Wire.write(0x08);                                                         // Set the register bits as 00001000 (+/- 4g full scale range)
   Wire.endTransmission();                                                   // End the transmission with the gyro

   // Set some filtering to improve the raw data.
   spl("[set_gyro_registers] Set Set Digital Low Pass Filter to ~43Hz to improve the raw data");
   Wire.beginTransmission(MPU_address);                                      // Start communication with the address found during search
   Wire.write(0x1A);                                                         // We want to write to the CONFIG register (1A hex)
   Wire.write(0x03);                                                         // Set the register bits as 00000011 (Set Digital Low Pass 
                                                                             // Filter to ~43Hz)
   Wire.endTransmission();                                                   // End the transmission with the gyro 

} //set_gyro_registers

/*************************************************************************************************************************************
 This function reads the accelerometer and gyro info from the MPU6050
 *************************************************************************************************************************************/
void read_mpu_6050_data()                                              
{

  Wire.beginTransmission(MPU_address);                                       // Start communicating with the MPU-6050
  Wire.write(0x3B);                                                          // Send the requested starting register
  Wire.endTransmission();                                                    // End the transmission
  Wire.requestFrom(MPU_address,14);                                          // Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                              // Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                        // Add the low and high byte to the acc_x variable
  if(acc_x > 32767) acc_x = acc_x - 65536;                                   // if it's really a negative number, fix it
  acc_y = Wire.read()<<8|Wire.read();                                        // Add the low and high byte to the acc_y variable
  if(acc_y > 32767) acc_y = acc_y - 65536;                                   // if it's really a negative number, fix it
  acc_z = Wire.read()<<8|Wire.read();                                        // Add the low and high byte to the acc_z variable
  if(acc_z > 32767) acc_z = acc_z - 65536;                                   // if it's really a negative number, fix it
  temperature = Wire.read()<<8|Wire.read();                                  // Add the low and high byte to the temperature variable
  if(temperature > 32767) temperature = temperature - 65536;                 // if it's really a negative number, fix it
  gyro_x = Wire.read()<<8|Wire.read();                                       // Add the low and high byte to the gyro_x variable
  if(gyro_x > 32767) gyro_x = gyro_x - 65536;                                // if it's really a negative number, fix it
  gyro_y = Wire.read()<<8|Wire.read();                                       // Add the low and high byte to the gyro_y variable
  if(gyro_y > 32767) gyro_y = gyro_y - 65536;                                // if it's really a negative number, fix it
  gyro_z = Wire.read()<<8|Wire.read();                                       // Add the low and high byte to the gyro_z variable
  if(gyro_z > 32767) gyro_z = gyro_z - 65536;                                // if it's really a negative number, fix it

} //read_mpu_6050_data()

/*************************************************************************************************************************************
 This function checks to see if the IMU is at the expected address on the I2C bus. If it is then initialize the IMU. Return codes for 
 speaking to the IMU at endTransmission are as follows:
 0:success
 1:data too long to fit in transmit buffer
 2:received NACK on transmit of address
 3:received NACK on transmit of data
 4:other error
 *************************************************************************************************************************************/
void initializeIMU()
{

  byte error, lowByte, highByte;
  int address;
  int receive_counter;
  int temp,tcnt1, tcnt2;
  String tmsg;

  sp("(initializeIMU): Checking to see if the IMU is found at expected I2C address of 0x");
  spl(MPU_address,HEX);
  Wire.beginTransmission(MPU_address);
  error = Wire.endTransmission();

  if (error == 0)
  {

     sp("[initializeIMU] I2C device found at address 0x");
     spl(MPU_address,HEX);
     spl("[initializeIMU] The IMU MPU-6050 found at expected address");
     
     Wire.beginTransmission(MPU_address);
     Wire.write(MPU6050_WHO_AM_I);
     Wire.endTransmission();
     
     spl("[initializeIMU] Send Who am I request to IMU...");
     Wire.requestFrom(MPU_address, 1);
     while(Wire.available() < 1);                                            // Wait for respy from IMU slave on I2C bus                                     
     lowByte = Wire.read();

     if(lowByte == MPU_address)
     {    
        sp("[initializeIMU] Who Am I responce is ok: 0x");
        spl(lowByte, HEX);        
        spl("[initializeIMU] Set up the Gyro registers in the IMU");
        set_gyro_registers();
        spl("[initializeIMU] Gyro started and configured");
        spl("[initializeIMU] Wait 10 seconds to allow MPU to settle down");
        for(int x=10; x > 1; x--)
        {
          tmsg = String(x) + " sec. init MPU";
          sendLCD(tmsg,LINE2);                                               // Send more boot message to LCD line 2
          delay(1000);
        } //for
        read_mpu_6050_data();                                                // Read MPU registers                
        spl("[initializeIMU] Create Gyro pitch and yaw offset values by averaging 500 sensor readings...");
        tcnt1 = 0; tcnt2 = 10;
        tmsg = String(tcnt2) + " Gyro Calc.";              
        sendLCD(tmsg,LINE2);                                                 // Send more boot message to LCD line 2
        for(receive_counter = 0; receive_counter < 500; receive_counter++)   // Create 500 loops
        {
           tcnt1 ++;
           if(tcnt1 > 50)
           {
              tcnt1 = 0;
              tcnt2 --;
              tmsg = String(tcnt2) + " Gyro Calc.";              
              sendLCD(tmsg,LINE2);                                           // Send more boot message to LCD line 2
           } //if
           Wire.beginTransmission(MPU_address);                              // Start communication with the gyro
           Wire.write(0x45);                                                 // Start reading the GYRO Y and Z registers
           Wire.endTransmission();                                           // End the transmission
           Wire.requestFrom(MPU_address, 4);                                 // Request 2 bytes from the gyro
           temp = Wire.read()<<8|Wire.read();                                // Combine the two bytes to make one integer, that could be negative
           if(temp > 32767) temp = temp - 65536;                             // if it's really a negative number, fix it
           gyro_pitch_calibration_value += temp;                             // 16 bit Y value from gyro, accumulating in 32 bit variable, sign extended
           temp = Wire.read()<<8|Wire.read();                                // Combine the two bytes to make one integer, that could be negative
           if(temp > 32767) temp = temp - 65536;                             // if it's really a negative number, fix it
           gyro_yaw_calibration_value += temp;                               // 16 bit Z value from gyro, accumulating in 32 bit variable, sign extended
           delay(20);
           Wire.beginTransmission(MPU_address);                              // Start communication with the IMU
           Wire.write(0x3F);                                                 // Get the MPU6050_ACCEL_ZOUT_H value
           Wire.endTransmission();                                           // End the transmission with the gyro
           Wire.requestFrom(MPU_address,2);
           temp = Wire.read()<<8|Wire.read();                                // Read the 16 bit number from IMU
           if(temp > 32767) temp = temp - 65536;                             // If it's really a negative number, fix it
           balance_calibration_value += temp;                                // 16 bit Z value from accelerometer, accumulating in 32 bit variable, sign extended
           delay(20);           
        } //for   
        if(gyro_pitch_calibration_value == -500 && 
           gyro_yaw_calibration_value == -500)                               // If calibration numbers are characteristically weird
        {
           while( 1==1)                                                      // request an IMU reset, because it's not working
           {
              spl("[setup] ******* IMU is stuck and needs a reset **********");
              delay(3000);
           } //while
        } //if
        gyro_pitch_calibration_value /= 500;                                 // Divide the total value by 500 to get the avarage gyro pitch offset
        gyro_yaw_calibration_value /= 500;                                   // Divide the total value by 500 to get the avarage gyro yaw offset
        balance_calibration_value /=500;                                     // Divide the total value by 500 to get the avarage balance value (Z accelerometer)
        sp("[setup] gyro_pitch_calibration_value= "); 
        spl(gyro_pitch_calibration_value);
        sp("[setup] gyro_yaw_calibration_value= "); 
        spl(gyro_yaw_calibration_value);
        sp("[setup] speed override value= "); spl(speed);    
        sp("[setupIMU): Balance value. NOTE - this is the balance value only if the robot is standing upright: ");
        spl(balance_calibration_value);
        sp("[setupIMU): acc_calibration_value currently set to ");
        spl(acc_calibration_value);
     } //if
     else
     {     
        sp("[initializeIMU] Wrong Who Am I responce: 0x");
        if (lowByte<16)Serial.print("0");
        spl(lowByte, HEX);
        spl("[initializeIMU] Initialization of IMU failed");    
     } //else
  } //if
  else
  {
     sp("[initializeIMU] MPU-6050 query returned error code ");
     spl(error);
     spl("[initializeIMU] ERROR! Robot will not be able to balance");
  } //else
      
} //initializeIMU()

/*************************************************************************************************************************************
 This function returns a String version of the local IP address
 *************************************************************************************************************************************/
String ipToString(IPAddress ip)
{

  String s="";
  for (int i=0; i<4; i++)
    s += i  ? "." + String(ip[i]) : String(ip[i]);
  return s;

} //ipToString()

/*************************************************************************************************************************************
 Initialization of all services, subsystems and program execution timing
 *************************************************************************************************************************************/
void setup() 
{
 
  Serial.begin(115200);                                                      // Start serial at 115200 baud
  delay(10);                                                                 // Allow time for serial to initialize
  spl(" ");spl("[setup] Start of sketch");                                   // Start fresh line in console and note start of sketch
  display_Running_Sketch();                                                  // Show environment details in console
  startI2Cbus();                                                             // Scan the I2C bus for connected devices
  initializeLCD();                                                           // Initialize the Open Smart 1602 LCD Display
  sendLCD("Boot Sequence",LINE1);                                            // Boot message to LCD line 1
  sendLCD("Init IMU",LINE2);                                                 // Send more boot message to LCD line 2
  initializeIMU();                                                           // Initialize MPU6050 IMU
  sendLCD("Start WiFi",LINE2);                                               // Send more boot message to LCD line 2
  startWiFi();                                                               // Start WiFi and connect to AP
  sendLCD("Start Web Server",LINE2);                                         // Send more boot message to LCD line 2
  startWebServer();                                                          // Start async web server
  sendLCD(ipToString(WiFi.localIP()),LINE1);                                 // Send IP address to LCD line 1
  sendLCD("SBSMark 3V" + my_ver,LINE2);                                      // Send version of code to LCD line 2
  spl("[setup] Initialization of the hardware complete");                    // Indicate in console that the robot is now ready  
 
} //setup()
 
/*************************************************************************************************************************************
 Main program loop
 *************************************************************************************************************************************/
void loop() 
{
 
} //loop()
