#include <Arduino.h>              // Required for platformIO Intellisense? Standard Arduino functions?
#include <WiFi.h>                 // Required to connect to WiFi network
#include <WiFiMulti.h>            // Allows us to try connecting to multiple Access Points
#include <ESP32WebServer.h>       // https://github.com/Pedroalbuquerque/ESP32WebServer
#include <ESPmDNS.h>              // Required for domain name service
#include <WebSocketsServer.h>     // https://github.com/Links2004/arduinoWebSockets
#include <Wire.h>                 // Needed for I2C communication
#include <LiquidCrystal_I2C.h>    // https://github.com/marcoschwartz/LiquidCrystal_I2C
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson

/***********************************************************************************************************
  Synopsis:
  This is the main sketch for the SegbotSTEP Mark3 robot. 

  Environment:
  - Platform: Espressif 32 (https://www.espressif.com/en/products/hardware/esp32/overview)
  - Framework: Arduino 
  - Embedded board: Adafruit ESP32 Feather (https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts)
  - SOC: WROOM32 module (https://dl.espressif.com/doc/esp-idf/latest/hw-reference/modules-and-boards.html)
  - IDE: Visual Studio Code (https://code.visualstudio.com/)
  - VSC plugin: platformIO (https://platformio.org/platformio-ide)
  - RTOS: FreeRTOS (https://www.freertos.org/Documentation/RTOS_book.html)
  - Language: C++ (Arduino)

  History
  Version YYYY-MM-DD Description
*/   
  String my_ver = "1.6";
/*
  ------- ---------- ---------------------------------------------------------------------------------------
  1.6     2018-04-13 The 1.5 fix made things worse. The error happend sooner. Commenting out WiFi all
          together to see if I2C becomes more reliable. The error is i2cWrite(): Busy Timeout! Addr: 68. 
          The I2C address 0x68 is assigned to the MPU6050 runing on the GY-521 board. This seems to have
          made the I2C bus communication reliable at last. Need to sort out how to have network services
          like WiFi, mDNS, websockets and web server running at the sae time without messing up the 
          reliability of the I2C bus. Also note that external 4.7K pull-up resistors were added around 
          version 1.4 and while they did not improve things they were left in and are part of the currently 
          stable configuration.
  1.5     2018-04-13 Pulled web service polling out of their own threads and put them in the main line in
          order to address an I2C error where the wire() calls result in a HAL error because the line is
          busy. The theory is that the WiFi activity is causing delays with writes and reads which cause 
          this error. There are others reporting issues like this and a new library fix is in the works to 
          address timing issues. 
  1.4     2018-04-13 Updated LINE macro to not TAB but rather label code line number more clearly. A cleanup
          of the short-cut macros would be handy as there is some redundancy now
  1.3     2018-04-02 Added motor control and PID logic from SBS Mark 2
  1.2     2018-04-01 Added DNS, Web and Websocket services
  1.1     2018-03-31 Moved init of IMU to its own FreeRTOS thread to and moved it up further in the boot 
          procss to allow for faster boot up by running the IMU init in parallel with other boot up tasks.
  1.0     2018-03-27 Code base created.
 ***********************************************************************************************************/

/***********************************************************************************************************
 Doug's shortcuts.
 ***********************************************************************************************************/
#define LINE(name,val) Serial.print(name); Serial.print(". Code Line: "); Serial.println(val); // Debug macro 
                                                                                               // prints 
                                                                                               // current 
                                                                                               // code line
#define sp Serial.print // Shortform print no carrige return
#define spl Serial.println // Shortform print with carrige return
#define spc Serial.print(", ") // Shortform print comma and space
#define spf Serial.printf // Shortform formatted printing with no line return
#define spdl(label,val) sp(label); spl(val) // Displays one labelled variable, ending with new-line 
                                            // example: spdl(" param2= ",param2);     
#define spd(label,val) sp(label); sp(val) // Suitable for displaying multiple variables per line
                                          // example: spd("left motor= ",left_motor); 
                                          // spdl("  right motor= ",right_motor);

/***********************************************************************************************************
 Define timer0 variables and objects
 ***********************************************************************************************************/
#define timer_number_0 0 // Timer number can be 0 to 3, since we have 4 hardware timers on ESP32
#define timer_prescaler_0 80 // Prescaler, divides 80 MHz by 80 = 1 MHz, or interrupt every microsecond
hw_timer_t * timer0 = NULL; // Pointer used to configure the timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // Synchronize variable updates shared by ISR and main
#define timer_cnt_up true // Used to tell timer to count up
#define timer_cnt_down false // Used to tell timer to count down 

/***********************************************************************************************************
 Define PID and motor control variables and constants
 ***********************************************************************************************************/
long pcnt; // This is used for some JSON message ping test timing
int acc_calibration_value; // Balance point of robot when standing at 90 degrees.
                           // To get this value stand the robot upright at 90 degrees
                           // and watch the serial trace during boot up. Note that
                           // the of this variable is set in the startWiFi()
                           // function based on MAC address of ESP32
                           // Gets set during start up.
const volatile int speed = -1; // for initial testing of interrupt driven steppers
                               // speed = -1 enables IMU based balancing. 
                               // speed = n enables fixed forward speed interval of n, 0 
                               // is brakes on
const int bot_slow = 2300; // # of interrupts between steps at slowest workable bot speed
const int bot_fast = 250; // # of interrupts between steps at fastest workable bot speed
const float PID_I_fade = .80; // How much of pid_i_mem history to retain
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Various PID settings
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain = 30; // Gain setting for the P-controller (15)
float pid_i_gain = 1.2; // Gain setting for the I-controller (1.5)
float pid_d_gain = 30; // Gain setting for the D-controller (30)
float turning_speed = 30; // Turning speed (20)
float max_target_speed = 150; // Max target speed (100)
const long usec_per_t0_int = 20; // Number of microseconds between t0 timer interrupts
const int pid_max = 400; // Upper bound for pid values
const int pid_min = -400; // Lower bound for pid values
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Define the GPIO pins that perform functions on A4988 controller
// below is corrected to reflect cross-wired DIR & STEP on MARK2 motherboard
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define pin_left_step 12  // GPIO 12 initiates a step on motor on robot's left side (top A4988)
#define pin_left_dir 14 // GPIO 14 controls the direction of left motor
#define pin_right_step 15 // GPIO 15 initiates a step on motor on robot's right side (bottom A4988)
#define pin_right_dir 13 // GPIO 13 controls the direction of right motor

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables and setting some values (YABR cut and paste)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, received_byte=0, low_bat;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// received_byte came from serial stuff I've omitted - suspect it is numchuk controller
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
int vertical_calibration; // This is calculated and displayed if speed == 0, which puts the brakes on
volatile float ftemp; // Temp replacement calculation to replace onestep
long mics, loop_mics, last_millis; // Measure length of loop, and print occasionally
int t0_count;
int dump_count; // Need to differentiate multiple dumps form same compile
long t,t_old,d;
int millis_now, i=1;
long mil, mic;
volatile long int_time;
volatile int h_flag; // Flag to capture and hold one vector of debug interrupt times

/***********************************************************************************************************
 Define on-board LED definitions. GPIO13 is where the onboard LED is located for the HUZZAH32 board. Other 
 board's (Adafruit ESP8266 for example) the LED might be on GPIO0.
 ***********************************************************************************************************/
const int LEDPIN = 13; // GPIO pin the the onboard LED is connected to
bool LEDStatus; // Current LED status
const char LEDON[] = "ledon"; // Turn onboard LED ON
const char LEDOFF[] = "ledoff"; // Turn onboard LED OFF
static void writeLED(bool); // Define function that sets the onbard LED, accepts 0 or 1

/***********************************************************************************************************
 Define network objects and services.
 ***********************************************************************************************************/
ESP32WebServer server(80); // Define web server object listening on port 80
WiFiMulti wifiMulti; // Allows us to connect to one of a number of known Access Points
WebSocketsServer webSocket = WebSocketsServer(81); // Define Websocket server object 
static const char ssid0[] = "MN_BELL418"; // The name of a WiFi network AP to connect to
static const char ssid1[]= "MN_WORKSHOP_2.4GHz"; // The name of a WiFi network AP to connect to
static const char ssid2[] = "MN_DS_OFFICE_2.4GHz"; // The name of a WiFi network AP to connect to
static const char ssid3[] = "MN_OUTSIDE"; // The name of a WiFi network AP to connect to
static const char password[] = "5194741299"; // The password for all of the WiFi network APs
MDNSResponder mdns; // DNS Service Discovery object used for client handshaking
static const char *wsEvent[] = { "WStype_DISCONNECTED", "WStype_CONNECTED", "WStype_TEXT", "WStype_BIN"};

/***********************************************************************************************************
 Define non-device specific I2C related variables. Device specific variables like addresses are found in 
 device specific sections.
 ***********************************************************************************************************/
byte I2C_UNKNOWN = false; // Flag if unknown device found on I2C bus

/***********************************************************************************************************
 Define MPU6050 related variables
 ***********************************************************************************************************/
#define MPU6050_WHO_AM_I 0x75 // Read only register on IMU with info about the device
#define MPU_address 0x68 // MPU-6050 I2C address. Note that AD0 pin on the board cannot be left floating and 
                         // must be connected to the Arduino ground for address 0x68 or be connected to VDC 
                         // for address 0x69.
byte IMU_FOUND = false; // Flag to see if MPU found on I2C bus
int acc_x; // Read raw low and high byte to the MPU acc_x register
int acc_y; // Read raw low and high byte to the MPU acc_y register
int acc_z; // Read raw low and high byte to the MPU acc_z register
int gyro_x; // Read raw low and high byte to the MPU gyro_x register
int gyro_y; // Read raw low and high byte to the MPU gyro_y register
int gyro_z; // Read raw low and high byte to the MPU gyro_z register
int temperature; // Read raw low and high byte to the MPU temperature register
volatile boolean imuReady = false; // flag used to note when imu boot sequence is complete

/***********************************************************************************************************
 Define LCD related variables. 
 ***********************************************************************************************************/
#define LCD_NO_MESSAGE "" // Blank message to scroll old messages off screen 
#define SCROLL 2 // Tell LCD to scroll full screen, both lines 
#define LINE1 0 // Tell LCD to diplay message on line 1
#define LINE2 1 // Tell LCD to diplay message on line 2
const byte lcdAddr = 0x38; // LCD I2C address for old LCD 
//const byte lcdAddr = 0x3F; // LCD I2C address for new LCD 
const byte lcdCols = 16; // LCD number of characters in a row
const byte lcdRows = 2; // LCD number of lines
const unsigned int scrollDelay = 500; // Miliseconds before scrolling next char
LiquidCrystal_I2C lcd(lcdAddr,lcdCols,lcdRows); // Define LCD object. Value re-assigned in WiFiStart()
byte LCD_FOUND = false; // Flag to see if LCD found on I2C bus
String LCDmsg0 = ""; // Track message displayed in line 1 of LCD
String LCDmsg1 = ""; // Track message displayed in line 2 of LCD

/***********************************************************************************************************
 Define main loop workflow related variables
 ***********************************************************************************************************/
#define JSON_DEBUG true // Flag to control JSON debug messages
#define LoopDelay 100000 // This is the target time in milliseconds that each
                         // iteration of Loop() should take. YABR used 4000 but
                         // testing shows that the ESP8266 cannot handle that
                         // speed but it can handle 20000. All angle calcuations
                         // will need to take this into account when porting from
                         // the YABR code base. 
String sendTelemetry = "flagoff"; // Flag to control if balance telemetry data is sent 
                                  // to connected clients

/***********************************************************************************************************
 Declare INDEX_HTML as a FLASH memory string containing a web page. Note that details on programming ESP32 
 PROGMEM are found here: http://arduino-esp8266.readthedocs.io/en/latest/PROGMEM.html. Details on how to 
 write a Websocket Javascript client can be found here: 
 https://www.tutorialspoint.com/websockets/websockets_send_receive_messages.htm. This is the code delivered 
 to any browser that connects to the robot. 
 ***********************************************************************************************************/
static const char PROGMEM INDEX_HTML[] = 
{
    R"rawliteral(
    <!DOCTYPE html>
    <html>
        <head>
            <meta charset="utf-8"/>
            <meta name = "viewport" content = "width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0">
            <title>SBS Mark2 Remote Home Page</title>
            <style>
                "body { background-color: #808080; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }"
            </style>
            <script type = "text/javascript" >
                var websock;
                ////////////////////////////////////////////////////////////////////////////////////////////
                // This function runs when the web page first loads in the browser. It defines websocket 
                // events which trigger during interactions with the server (robot). There is logging 
                // included in this code so if needed open a console in your browser. In Firefox select 
                // Tools/Web Developer/Toggle Tools then click the Console tab Use the garbage can icon to 
                // clear old messages.  
                ////////////////////////////////////////////////////////////////////////////////////////////
                function start() 
                {
    
                    ////////////////////////////////////////////////////////////////////////////////////////
                    // Define websock event functions other than "onmessage"
                    ////////////////////////////////////////////////////////////////////////////////////////
                    websock = new WebSocket('ws://' + window.location.hostname + ':81/'); // Define websocket object
                    websock.onopen = function(evt) { console.log('websock open'); }; // Log new connections
                    websock.onclose = function(evt) { console.log('websock close'); }; // Log closed connection
                    websock.onerror = function(evt) { console.log(evt); }; // Log connection errors
               
                    ////////////////////////////////////////////////////////////////////////////////////////
                    // Define the onmessage function for processing incoming server messages.
                    ////////////////////////////////////////////////////////////////////////////////////////
                    websock.onmessage = function(evt)                                           
                    {
                        console.log('[SegbotSTEP] evt = ' + evt.data); // Log incoming message
                        var msg = JSON.parse(evt.data); // Parse incoming message (JSON)
                        console.log('[SegbotSTEP] msg.item = ' + msg.item); // Log JSON msg element 1
                        if (msg.item === 'LED') // If this message is about the LED
                        {
                            var e = document.getElementById('ledstatus'); // Create handle for LED status
                            console.log('[SegbotSTEP] msg.value = ' + msg.value); // Log JSON msg element 3
                            if (msg.value === 'ledon') // If message sets LED on
                            {
                                e.style.color = 'red'; // Change LED text to RED
                                console.log('[SegbotSTEP] set ledstatus color to red'); // Log action
                            } //if
                            else if (msg.value === 'ledoff') // If message sets LED off
                            {
                                e.style.color = 'black'; // Change LED text to BLACK
                                console.log('[SegbotSTEP] set ledstatus color to black'); // Log action
                            } //else if
                            else // If you get here, command unknown
                            {
                                console.log('[SegbotSTEP] unknown LED value. evt.data = ' + evt.data);  // Log error item unknow
                            } // else
                        } //if
                        else if (msg.item === 'LCD') // If this message is about the LCD
                        {
                            var e1 = document.getElementById('lcd1'); // Create handle for LCD line 1 
                            var e2 = document.getElementById('lcd2'); // Create handle for LCD line 2 
                            console.log('[SegbotSTEP] update LCD line 1 with ' + msg.line1); // Log JSON msg line1 element
                            console.log('[SegbotSTEP] update LCD line 2 with ' + msg.line2); // Log JSON msg line1 element
                            e1.value = msg.line1; // Place JSON line1 to text box 1   
                            e2.value = msg.line2; // Place JSON line1 to text box 1   
                        } //else if
                        else if (msg.item === 'ping') // Turn around timing test message 
                        {
                            websock.send(evt.data); // Send message from server back
                        } //else if                  
                        else if (msg.item === 'balGraph') // If this is balancing data 
                        {
                            console.log('[SegbotSTEP] Web client does not support graphing, ignore message');      
                        } //else if                  
                        else // No idea what this ITEM type is
                        {
                            console.log('[SegbotSTEP] unknown item (case sensative). evt.data = ' + evt.data); // Log error item unknown                   
                        } //else
                    }; //websock.onmessage() 
                } //start()           
 
                ////////////////////////////////////////////////////////////////////////////////////////////
                // This function runs when either of the LED control buttons are pressed. These two buttons 
                // share the same HTML DIV class ID (ledstatus), which allows us to combine the ID of each 
                // button (ledon and ledoff) with the DIV class ID they belong to to message the JSON server 
                // (robot) what we want to do with the onboard LED of the Huzzah32    
                ////////////////////////////////////////////////////////////////////////////////////////////
                function ledControl(e) 
                {  
                    var msg =           // Construct JSON string
                    {
                        item:   "LED",  // JSON msg element 1
                        action: "set",  // JSON msg element 2
                        value:  e.id    // JSON msg element 3
                    }; //var msg
                    websock.send(JSON.stringify(msg)); // Send JSON message to server
                    console.log('[SegbotSTEP] sent this to server: ' + JSON.stringify(msg)); // Log message sent
                } //ledControl()

                ////////////////////////////////////////////////////////////////////////////////////////////
                // This function GETs or SETs the 2 lines of the LCD on the robot    
                ////////////////////////////////////////////////////////////////////////////////////////////
                function lcdControl(e) 
                {  
                    if (e.id === 'getlcd') // If we want to get the LCD values
                    {
                        var x = "get"; // Element 2 (action) will be GET                
                    } //if
                    else // If we want to set the LCD values
                    {
                        var x = "set"; // Element 2 (action) will be SET
                    } //else
                    var l1 = document.getElementById('lcd1'); // Create handle for LCD line 1 
                    var l2 = document.getElementById('lcd2'); // Create handle for LCD line 2 
                    var msg =               // Construct JSON GET string
                    {
                        item:   "LCD",      // JSON msg element 1
                        action: x,          // JSON msg element 2
                        line1:  l1.value,   // JSON msg element 3
                        line2:  l2.value    // JSON msg element 4                
                    }; //var msg
                    websock.send(JSON.stringify(msg)); // Send JSON message to server
                    console.log('[SegbotSTEP] sent this to server: ' + JSON.stringify(msg)); // Log message sent
                } //lcdControl()
            </script>
        </head>
        <body onload="javascript:start();">
            <h1>SBS Mark2 Web-Based Control Center</h1>
            Note: Balancing telemetry graph intentionally not included here due to complications serving up onjects 
            such as canvasjs from the ESP32 
            <div id="ledstatus"><b>LED</b></div>
            <button id="ledon"  type="button" onclick="ledControl(this);">On</button> 
            <button id="ledoff" type="button" onclick="ledControl(this);">Off</button>
            <p><b>1650 LCD</b><br>
            <input type="text" id="lcd1" style="background:GreenYellow; color:black;text-align:center;" maxlength="16"/><br>
            <input type="text" id="lcd2" style="background:GreenYellow; color:black;text-align:center;" maxlength="16"/><br>
            <button id="getlcd"  type="button" onclick="lcdControl(this);">Get</button> 
            <button id="setlcd" type="button" onclick="lcdControl(this);">Set</button>
        </body>
    </html>
    )rawliteral"
};

/***********************************************************************************************************
 This is the interrupt handler for hardware timer 0. This routine coordinates a variable with main.  
 ***********************************************************************************************************/
void IRAM_ATTR onTimer0() 
{

    portENTER_CRITICAL_ISR(&timerMux); // Prevent anyone else from updating the variable
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Left motor pulse calculations 
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    throttle_counter_left_motor ++; // Increase the throttle_counter_left_motor variable by 1 every time 
                                    // this routine is executed
    if(throttle_counter_left_motor > throttle_left_motor_memory) // If the number of loops is larger then the 
                                                                 // throttle_left_motor_memory variable
    {
        throttle_counter_left_motor = 0; // Reset the throttle_counter_left_motor variable
        throttle_left_motor_memory = throttle_left_motor; // Load the next throttle_left_motor variable
        if(throttle_left_motor_memory < 0) // If the throttle_left_motor_memory is negative
        {
            digitalWrite(pin_left_dir, LOW); // Change left wheel rotation to the reverse direction
            throttle_left_motor_memory *= -1; // negate the throttle_left_motor_memory variable
        } //if
        else digitalWrite(pin_left_dir, HIGH); // Otherwise set left wheel rotation to the forward direction
    } //if
    else if(throttle_counter_left_motor == 1)digitalWrite(pin_left_step, HIGH); // Set left motor step pin high 
                                                                                // to start a pulse for the 
                                                                                // stepper controller
    else if(throttle_counter_left_motor == 2)digitalWrite(pin_left_step, LOW);  // One interrupt later, lower 
                                                                                // the pin because the pulse 
                                                                                // only has to last for 20us 
                                                                                // Wiring of M1, M2, and M3 pins 
                                                                                // on A4988 controls step type - 
                                                                                // we default to SINGLE
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    // right motor pulse calculations 
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    throttle_counter_right_motor ++; // Increase the throttle_counter_right_motor variable by 1 every time 
                                     // the routine is executed
    if(throttle_counter_right_motor > throttle_right_motor_memory) // If the number of loops is larger then 
                                                                  // the throttle_right_motor_memory variable
    {
        throttle_counter_right_motor = 0; // Reset the throttle_counter_right_motor variable
        throttle_right_motor_memory = throttle_right_motor; // Load the next throttle_right_motor variable
        if(throttle_right_motor_memory < 0) // If the throttle_right_motor_memory is negative
        {
            digitalWrite(pin_right_dir, LOW); // Change right wheel rotation to the reverse direction
            throttle_right_motor_memory *= -1; // negate the throttle_right_motor_memory variable
        } //if
        else digitalWrite(pin_right_dir, HIGH); // Otherwise set right wheel rotation to the forward direction
    } //if 
    else if(throttle_counter_right_motor == 1)digitalWrite(pin_right_step, HIGH); // Set right motor step pin 
                                                                                  // high to start a pulse for 
                                                                                  // the stepper controller
    else if(throttle_counter_right_motor == 2)digitalWrite(pin_right_step, LOW); // One interrupt later, lower 
                                                                                 // the pin because the pulse 
                                                                                 // only has to last for 20us 
                                                                                 // Wiring of M1, M2, and M3 
                                                                                 // pins on A4988 controls 
                                                                                 // step type - we default to 
                                                                                 // SINGLE
    //timer0_write(ESP.getCycleCount() + t0_count -1 ); // Prime next interrupt to go off after proper interval
    //t0_per_sec++ ; // Count one more t0 int seen in this second
    portEXIT_CRITICAL_ISR(&timerMux); // Allow anyone else to update the variable
 
} //onTimer0()

/***********************************************************************************************************
 This function dumps a bunch of useful info to the terminal. This code is based on an example we found at 
 this URL: https://stackoverflow.com/questions/14143517/find-the-name-of-an-arduino-sketch-programmatically                                   
 ***********************************************************************************************************/
void display_Running_Sketch()
{                                 
  
    LINE("[display_Running_Sketch] Displaying basic running environment. Source code line: ", __LINE__);
    sp("[display_Running_Sketch] Sketch Name: ");spl(__FILE__);
    sp("[display_Running_Sketch] Sketch Version: "); spl(my_ver);
    sp("[display_Running_Sketch] Sketch compilation date: ");sp(__DATE__);sp(" at ");spl(__TIME__);
    spdl("[display_Running_Sketch] ESP32 SDK used: ", ESP.getSdkVersion());

} //display_Running_Sketch()

/***********************************************************************************************************
 This function controls the onboard Huzzah32 LED. Note inverted logic for Adafruit ESP8266 board does NOT 
 apply for the Feather32 board. THis logic needed to be revered as part of port to the Feather32.
 ***********************************************************************************************************/
void writeLED(bool LEDon)
{

    LINE("[writeLED] write value to LED. Source code line: ", __LINE__);
    LEDStatus = LEDon; // Track status of LED
    if (LEDon) // If request is to turn LED on
    {
        digitalWrite(LEDPIN, 1); // Set LED GPIO HIGH, which turn the LED on
        spl("[writeLED] Set LED GPIO HIGH (turn LED on)");
    } //if
    else 
    {
        digitalWrite(LEDPIN, 0); // Set LED GPIO LOW, which turn the LED off
        spl("[writeLED] Set LED GPIO LOW (turn LED off)");
    } //else

} //writeLED()

/***********************************************************************************************************
 This function scans the I2C bus for attached devices. This code was taken from 
 http://playground.arduino.cc/Main/I2cScanner 
 ***********************************************************************************************************/
void startI2Cbus()
{
      
    byte error, address;
    int nDevices;  
    LINE("[startI2Cbus] Initialize I2C bus. Source code line: ", __LINE__);
    Wire.setClock(400000); // choose 400 kHz I2C rate 
    Wire.begin();
    spl("[startI2Cbus] Scanning I2C bus...");
    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        // The i2c_scanner uses the return value of the Write.endTransmisstion to see if a device did acknowledge 
        // to the address
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
            switch(address) // Check each located device to see what it is
            {
                case MPU_address: // Invensense MPU-6050
                    spl("Invensense MPU-6050 IMU on GY-521 or ITG-MPU board");
                    IMU_FOUND = true; // Set flag indicating that MPU found on I2C bus
                    break;
                case lcdAddr: // OpenSmart 1602 LCD Display 
                    spl("OpenSmart 1602 LCD Display");
                    LCD_FOUND = true; // Set flag indicating that LCD found on I2C bus
                    break;
                default: // Unknown websocket event
                    spl("Unknown device");
                    I2C_UNKNOWN = true; // Indicate that an unknown I2C device has been found
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
   if(I2C_UNKNOWN) // Issue warning if I2C bus has unknown device 
   {
      spl("[startI2Cbus] WARNING - Unrecognized device detected on I2C bus. Boot sequence will continue."); 
   } //if
   if(!LCD_FOUND) // Issue error LCD not detected on I2C bus 
   {
      spl("[startI2Cbus] ERROR - LCD device not detected on I2C bus. Boot sequence halted."); 
   } //if
   if(!IMU_FOUND) // Issue error LCD not detected on I2C bus 
   {
      spl("[startI2Cbus] ERROR - IMU device not detected on I2C bus. Boot sequence halted."); 
   } //if

   while(!IMU_FOUND){}; // If IMU is not found on I2C bus, halt boot up
   while(!LCD_FOUND){}; // If LCD is not found on I2C bus, halt boot up

} //startI2Cbus()

/***********************************************************************************************************
 This function initializes the Open Smart 1602 LCD Display
 ***********************************************************************************************************/
void initializeLCD()                                             
{

    LINE("[initializeLCD] Initialize LCD. Source code line: ", __LINE__);
    lcd.clear(); // Clear the LCD screen
    lcd.init(); // Initialize the LCD object 
    lcd.backlight(); // Turn on the LCD backlight
    flashLCD(); // Flash the LCD backlight
  
} //initializeLCD()                                                          

/***********************************************************************************************************
 This function sends messages you pass to it to the LCD and displays it centered.
 ***********************************************************************************************************/
void sendLCD(String LCDmsg, byte LCDline)
{

    LINE("[sendLCD] Send text to LCD. Source code line: ", __LINE__);
    byte textLen = LCDmsg.length(); // Length of message to send
    byte LCDcolumn = 0; // Starting column of message 
    if(LCDline > 1) LCDline=1; // Ensure line argument is not too large                                   
    if(LCDline < 0) LCDline=0; // Ensure line argument is not too small
    LCDcolumn=(lcdCols-textLen)/2; // Figure out starting point to center message         
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Clear line by sending blank message starting at first column, this ensures that previous messages 
    // are gone completely
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    lcd.setCursor(0,LCDline); // Set cursor to correct location (line 1 or 2)
    lcd.print("                "); // Send blank message to clear previous message 
    delay(5); // Allow time for LCD to process message
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Print actual message centered
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    lcd.setCursor(LCDcolumn,LCDline); // Set cursor to correct location (line 1 or 2)
    lcd.print(LCDmsg); // Send message to LCD 
    delay(5); // Allow time for LCD to process message
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Global variables track content of LCD line 1 & 2 to send to clients via JSON message
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(LCDline == 0) // If this is the first line of the LCD
    {
        LCDmsg0 = LCDmsg; // Track current message in line 1 global variable
    } // if
    else // If this is the second line of the LCD
    {
        LCDmsg1 = LCDmsg; // Track current message in line 2 global variable    
    } // else
   
} //sendLCD()

/***********************************************************************************************************
 This function scrolls a message from left to right on the LCD. Note that both lines of the display scroll. 
 You can send a blank message to this function to scroll the current messages displayed on both lines off 
 the LCD screen. Note that the second argument passed to this function is not used if a null message if 
 passed.
 ***********************************************************************************************************/
void scrollLCD(String LCDmsg, byte LCDline)
{

    LINE("[scrollLCD] Scrolling text on LCD. Source code line: ", __LINE__);
    byte textLen = LCDmsg.length(); // Length of message to send
    byte LCDcolumn = 0; // Starting column of message 
    if(LCDline > 1) LCDline=1; // Ensure line argument is not too large                                   
    if(LCDline < 0) LCDline=0; // Ensure line argument is not too small
    if(LCDmsg != LCD_NO_MESSAGE) // If this is not a blank message display it
    {
        lcd.setCursor(LCDcolumn,LCDline); // Set LCD cursor to correct location 
        lcd.print(LCDmsg); // Send message to LCD
    } // if 
    for (byte positionCounter = 0; positionCounter < (textLen + lcdCols); positionCounter++) 
    {
        lcd.scrollDisplayRight(); // Scroll entire row to the right
        delay(scrollDelay); // Pause between scrolls
    } // for

} //scrollLCD()

/***********************************************************************************************************
 This function flashes the LCD backlight.
 ***********************************************************************************************************/
void flashLCD()   
{

    LINE("[flashLCD] Flashing back light of LCD. Source code line: ", __LINE__);
    for (byte cnt = 0; cnt < 10; cnt++) 
    {
        lcd.backlight(); // Turn on the LCD backlight
        delay(100);
        lcd.noBacklight(); // Turn off backlight
        delay(100);
    } // for
    lcd.backlight(); // Turn on the LCD backlight
    delay(100);

} //flashLCD()

/***********************************************************************************************************
 This function handles hexdumps that come in over a websocket.
 ***********************************************************************************************************/
void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) 
{

    LINE("[hexdump] HEX dump from websocket. Source code line: ", __LINE__);
    const uint8_t* src = (const uint8_t*) mem;
	Serial.printf("\n[hexdump] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
	for(uint32_t i = 0; i < len; i++) 
    {
	    if(i % cols == 0) 
        {
			Serial.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
		} // if
		Serial.printf("%02X ", *src);
		src++;
	} // for
	Serial.println("\n");

} //hexdump()

/***********************************************************************************************************
 This function connects to the local Access Point and then starts up a a socket server to listen for client 
 connections. This code is based on an exmaple we found at this URL: 
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
 ***********************************************************************************************************/
void startWiFi()
{
 
    LINE("[startWiFi] Scanning/connecting to strongest known AP signal. Source code line: ", __LINE__);
    //WiFi.mode();
    wifiMulti.addAP(ssid0, password); // Add Wi-Fi AP we may be able to connect to
    wifiMulti.addAP(ssid1, password); // Add Wi-Fi AP we may be able to connect to
    wifiMulti.addAP(ssid2, password); // Add Wi-Fi AP we may be able to connect to
    wifiMulti.addAP(ssid3, password); // Add Wi-Fi AP we may be able to connect to
    spl("[startWiFi] Connecting Wifi"); 
    sp(wifiMulti.run());
    while(wifiMulti.run() != WL_CONNECTED) 
    {                                                               
        sp(".\r"); // Send dot to console terminal to show the waiting process is active
        delay(100); // Wait a little before trying again
    } // while
    spl("");                                             
    sp("[startWiFi] Connected to Access Point ");                                        
    spl(WiFi.SSID()); // Name of AP to which the ESP32 is connected to
    sp("[startWiFi] IP address: ");                                   
    spl(WiFi.localIP());  // IP address assigned to ESP32 by AP
    sp("[startWiFi] MAC address: ");                                   
    spl(WiFi.macAddress()); // IP address of the ESP32

} //startWiFi()

/*************************************************************************************************************************************
 This function Start the mDNS service which handles mapping IP ports and provided services to connecting clients according to
 https://media.readthedocs.org/pdf/arduino-esp8266/docs_to_readthedocs/arduino-esp8266.pdf, mDNS implements a simple DNS server that 
 can be used in both STA and AP modes.  The DNS server currently supports only one domain (for all other domains it will reply with 
 NXDOMAIN or custom status code).  With it, clients can open a web server running on ESP8266 using a domain name, not an IP address.
 *************************************************************************************************************************************/
void startDNS()
{

    LINE("[startDNS] Start the DNS service. Source code line: ", __LINE__);
    if (mdns.begin("esp32")) // Start mDNS service
    {
        sp("[startDNS] MDNS responder started. ");
        sp("Adding HTTP service to port 80 ");
        mdns.addService("http", "tcp", 80); // If successfully started add web service on port 80
        spl("and WS service to port 81");                    
        mdns.addService("ws", "tcp", 81); // If successfully started add websocket service on port 81
        sp("[startDNS] Clients can connect to either ");
        sp("http://esp32.local or http://"); 
        spl(WiFi.localIP()); // Send message to console advising URI for clients to use

   } //if
   else // If mDNS service fails to start
   {
      spl("[startDNS] mdns.begin failed"); // Issue message
   } //else
  
} //startDNS()

/***********************************************************************************************************
 This function handles the web server service for valid HTTP GET and POST requests for the root (home) page. 
 The rely simply sends the web page defined in the EEPROM (non-volitile memory of the ESP8266 chip), pointed 
 to by INDEX_HTML[] 
 ***********************************************************************************************************/
volatile void handleRoot()
{

    LINE("[handleRoot] handleRoot web service event triggered. Source code line: ", __LINE__);
    server.send_P(200, "text/html", INDEX_HTML); // Send the HTML page defined in INDEX_HTML
    spl("[handleRoot] Home page  requested via HTTP request on port 80. Sent EEPROM defined document page to client");                              
   
} //handleRoot()

/***********************************************************************************************************
 This function handles the web server service for invalid HTTP GET and POST requests. If the requested file 
 or page doesn't exist, return a 404 not found error to the client 
 ***********************************************************************************************************/
volatile void handleNotFound()
{

    LINE("[handleNotFound] File not found web service event triggered. Source code line: ", __LINE__);
    String message = "File Not Found\n\n"; // Build string with 404 file not found message
        message += "URI: ";
        message += server.uri();
        message += "\nMethod: ";
        message += (server.method() == HTTP_GET)?"GET":"POST";
        message += "\nArguments: ";
        message += server.args();
        message += "\n";
    for (uint8_t i=0; i<server.args(); i++) // Append actual HTTP error elements to message
    {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n"; 
    } //for
    server.send(404, "text/plain", message); // Send 404 error message back to client
    spl("[handleNotFound] Sent 404 error to client");                              

} //handleNotFound()

/***********************************************************************************************************
 This function starts the web service which sends HTML and Javascript client code to remote Web browser clients
 ***********************************************************************************************************/
void startWebServer()
{

    LINE("[startWebServer] Starting Webserver service. Source code line: ", __LINE__);
    server.on("/", handleRoot); // Attach function for handling root page (/)
    server.on ( "/inline", []() // Attach simple inline page to test web server
    {
        server.send ( 200, "text/plain", "this works as well" );
    } ); //server.on(/inline)
    server.onNotFound(handleNotFound); // Attach function for handling unknown page
    server.begin(); // Start web service
    spl("[startWebServer] Started web server");                               

} //startWebServer()

/***********************************************************************************************************
 This function initializes a websocket service
 ***********************************************************************************************************/
void startWebsocket()
{
 
    LINE("[startWebsocket] Starting Websocket service. Source code line: ", __LINE__);
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);    

} //startWebsocket()

/***********************************************************************************************************
 This function handles websocket events
 ***********************************************************************************************************/
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) 
{

    LINE("[webSocketEvent] Websocket event detected. Source code line: ", __LINE__);
    if(JSON_DEBUG) // If JSON message debug flag set  
    {
        spf("[webSocketEvent] Event detected: "); // Show event details in terminal   
        spf("num = %d, type = %d (", num, type); // Show event details in terminal
        sp(wsEvent[type-1]);spl(")"); // Show event details in terminal
    } //if

    switch(type) // Handle each event by type
    {
        case WStype_DISCONNECTED: // Client disconnect event
            spf("[webSocketEvent] Client NUM [%u] disconnected\r\n", num);
            break;
        case WStype_CONNECTED: // Client connect event
        {
            IPAddress ip = webSocket.remoteIP(num);
            spf("[webSocketEvent] [%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            sendClientLEDState(num); // Send new client state of onboard LED
            sendClientLCDState(num); // Send new client text displayed on LCD
            sendClientVariableValue(num,"PID"); // Send new client PID tuning values
//            sendClientPing(num); // Send new client a ping message (test only)
        } //case
            break;                                                   
        case WStype_TEXT: // Client sent text event
            if(JSON_DEBUG) // If JSON message debug flag set  
            {
                spf("[webSocketEvent] Client NUM: [%u], sent TEXT: %s\r\n", num, payload);
            } //if
            process_Client_JSON_msg(num, type, payload, length); // Process recieved message using JSON library
            break;
        case WStype_BIN: // Client sent binary data event
            spf("[webSocketEvent] [%u] get binary length: %u\r\n", num, length);
            hexdump(payload, length); // Dump ESP8266 memory in hex to the console
            webSocket.sendBIN(num, payload, length); // echo data back to browser
            break;
        default: // Unknown websocket event
            spf("[webSocketEvent] Invalid WStype [%d]\r\n", type);
            break;
    } //switch()

} //webSocketEvent()

/*************************************************************************************************************************************
 This function processes incoming client messages. Messages are in JSON format. This code is based on
 https://techtutorialsx.com/2016/07/30/esp8266-parsing-json/
 Primer on JSON messaging format: http://www.json.org/
 About the JSON message format used. All messages have the first value:
    item     [variable/property name] - e.g. LED or LCD
 After this the JSON message format changes depending upon the value of ITEM. Each use case is detailed below:   

 1. ITEM = LED. Message format is as follows:
    action   [set,get]
    value    [ledon,ledoff]

 2. ITEM = LCD. Message format is as follows:
    action   [set,get]
    line1    [message]
    line2    [message]

 3. ITEM = ping. Message format is as follows:
    line     [32 bytes of data]

 4. ITEM = BAL-TEL. Message format is as follows:
    action   [set,get]
    value    [flagon,flagoff]

 5. ITEM = PID. Message format is as follows:
    action   [set,get]
    gain1    [pGain,pid_p_gain]
    gain2    [iGain,pid_i_gain]
    gain3    [dGain,pid_d_gain]

 6. ITEM = balGraph. Message format is as follows:
    action   [set,get]
    angle    [calculated value]. See sendClientBalanceGraph()
    pid      [calculated value]. See sendClientBalanceGraph()
    gspeed   [calculated value]. See sendClientBalanceGraph()
    pmem     [calculated value]. See sendClientBalanceGraph()
    
 Handy additional notes. How to handle data types: 
    Most of the time, you can rely on the implicit casts. In other case, you can do root["time"].as<long>();
 See also:
    The website arduinojson.org contains the documentation for all the functions used. It also includes an FAQ that will help you 
    solve any deserialization problem. Please check it out at: https://arduinojson.org/. The book "Mastering ArduinoJson" contains a 
    tutorial on deserialization. It begins with a simple example, like the one above, and then adds more features like deserializing 
    directly from a file or an HTTP request. Please check it out at: https://arduinojson.org/book/
 *************************************************************************************************************************************/
void process_Client_JSON_msg(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{

   String payload_str = String((char*) payload);
   StaticJsonBuffer<500> jsonBuffer;                                         // Use arduinojson.org/assistant to compute the capacity
   JsonObject& root = jsonBuffer.parseObject(payload_str);                   // Create JSON object tree with reference handle "root"
   if (!root.success())                                                      // Test if parsing succeeds 
   {
      sp("[process_Client_JSON_msg] WARNING, parseObject() failed on message from client [");
      sp(num);spl("]. Message ignored");
      return;
   } //if
   String item = root["item"];                                               // This is the item that the client is interested in
   if(JSON_DEBUG)                                                            // If JSON message debug flag set  
   {
      sp("[process_Client_JSON_msg] message from client regarding item: ");sp(item);
   } //if
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Message handling for LED related commands
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   if(item == "LED")                                                         // Check if client is interested in the LED
   {
      spf("[process_Client_JSON_msg] Client NUM [%u] messaging about LED\r\n", num);
      String action = root["action"];                                        // This is what the client wants to do with the LED
      String value = root["value"];                                          // This is the value detail for the LED
      if (action == "set")                                                   // If client wants to SET LED value
      {
         if (value == LEDON)                                                 // If client wants LEDON
         {
            if(JSON_DEBUG)                                                   // If JSON message debug flag set  
            {
               sp("[process_Client_JSON_msg] Client requested LED on: ");
            } //if
            writeLED(true);                                                  // Call function to turn GPIO LED off
            sendClientLEDState(99);                                          // Request broadcast (99) of the current value of the LED
         } //if
         else if(value == LEDOFF)                                            // If client wants LEDOFF 
         {
            if(JSON_DEBUG)                                                   // If JSON message debug flag set  
            {
               sp("[process_Client_JSON_msg] Client requested LED off: ");
            } //if
            writeLED(false);                                                 // Call function to turn GPIO LED on
            sendClientLEDState(99);                                          // Request broadcast (99) of the current value of the LED
         } //else if
         else                                                                // If client wants any other value it is invalid
         {
            sp("[process_Client_JSON_msg] Client [");sp(num);sp("] has sent unrecognized VALUE [");sp(value);spl("] for LED. Message ignored");
         } //else       
      } //if ACTION
      else if (root["action"] == "get")                                      // If client wants to GET LED value
      {
         spf("[process_Client_JSON_msg] Client [%u] requesting current LED value\r\n", num);
         sendClientLEDState(num);                                            // Send client the current value of the LED
      } //else if ACTION
      else                                                                   // Any other action the client wants for LED is invalid
      {
         sp("[process_Client_JSON_msg] Client [");sp(num);sp("] has sent unrecognized ACTION [");sp(action);spl("]. Message ignored");
      } //else ACTION
   } //if
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Message handling for LCD related commands
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   else if(item == "LCD")                                                    // Check if client is interested in the LCD
   {
      spf("[process_Client_JSON_msg] Client NUM [%u] messaging about LCD\r\n", num);
      String action = root["action"];                                        // This is what the client wants to do with the LCD
      String line1 = root["line1"];                                          // Message text for top line of LCD
      String line2 = root["line2"];                                          // Message text for bottom line of LCD
      if (action == "set")                                                   // If client wants to SET LCD value
      {
         spf("[process_Client_JSON_msg] Client [%u] wants to set LCD message\r\n", num);         
         lcd.clear();                                                        // Clear the LCD screen
         sendLCD(line1, 0);                                                  // Update line 1 of LCD with client message
         sendLCD(line2, 1);                                                  // Update line 2 of LCD with client message
         sendClientLCDState(99);                                             // Send client the current value of the LED        
      } // if
      else if(action == "get")                                               // If client wants to GET LCD value
      {
         spf("[process_Client_JSON_msg] Client [%u] requesting current LCD message\r\n", num);
         sendClientLCDState(num);                                            // Send client the current value of the LED        
      } //else if
      else                                                                   // If ACTION is not one of the above values
      {
         sp("[process_Client_JSON_msg] Client [");sp(num);sp("] has sent unrecognized ACTION [");sp(action);spl("]. Message ignored");        
      } //else
   } //else if ITEM LCD
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Message handling for PING related commands. Server echoes messages back to client. Useful when developing a new client. NOTE:
   // Client MUST send JSON formatted message with at least one element defined (item). When building a new client you can cause the 
   // server to send the PING JSON message to the client by uncommenting the call to the ping function in the start() function.
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   else if(item == "ping")                                                   // Check if client is sending ping test message
   {
      spf("[process_Client_JSON_msg] Client NUM [%u] sent ping message\r\n", num);
      sendClientPing(num);                                                   // Send client the current value of the LED        
   } //else if ITEM ping
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Message handling for balance telemetry related commands
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   else if(item == "BAL-TEL")                                                // Check if client is sending telemetry config message
   {
      spf("[process_Client_JSON_msg] Client NUM [%u] messaging about telemetry data\r\n", num);
      String action = root["action"];                                        // This is what the client wants to do with the flag
      String value = root["value"];                                          // This is the value detail for the flag
      if (action == "set")                                                   // If client wants to SET telemetry flag value
      {
         spf("[process_Client_JSON_msg] Client [%u] wants to set telemetry flag\r\n", num);   
         if(value == "flagon")                                               // If client wants to recieve telemetry data
         {
            sp("[process_Client_JSON_msg] Client [");sp(num);sp("] has requested telemetry data be sent. Setting flag ON"); 
            sendTelemetry = "flagon";       
         } //if
         else                                                                // Any other value other then ON turns off telemetry data
         {
            sp("[process_Client_JSON_msg] Client [");sp(num);sp("] has requested telemetry data be sent. Setting flag OFF"); 
            sendTelemetry = "flagoff";               
         } //else if               
      } // if
      else if(action == "get")                                               // If client wants to GET telemetry flag value
      {
         spf("[process_Client_JSON_msg] Client [%u] requesting current telemetry flag value message\r\n", num);
         sendClientVariableValue(num,"sendTelemetry");                       // Send client the current value of the BAL-TEL flag        
      } //else if
      else                                                                   // If ACTION is not one of the above values
      {
         sp("[process_Client_JSON_msg] Client [");sp(num);sp("] has sent unrecognized ACTION [");sp(action);spl("]. Message ignored");        
      } //else
   } //else if ITEM BAL-TEL
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Message handling for balance telemetry related commands
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   else if(item == "PID") // Check if client is sending PID config message
   {
        spf("[process_Client_JSON_msg] Client NUM [%u] messaging about PID values\r\n", num);
        String action = root["action"]; // This is what the client wants to do with the flag
        if (action == "set") // If client wants to SET PID values
        {
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // am: For now just log what would be done without actually changing the values. Need some logic around verifying values.
   // Example of how to convert string (JSON message format) to float using toFloat() below.
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
   String s_val1 = "123.456789";
   float f;
   Serial.print("Taking String with 3 digit and 6 decimal places and putting in float: ");
   f = s_val1.toFloat();
   Serial.println(f,6);  // the number in the second argument tells how many digits of precision to use
*/
         spf("[process_Client_JSON_msg] Client [%u] wants to set the following PID variables:\r\n", num);
         spf("pid_p_gain = [%s]",root["pGain"]);                             // Assign value sent from cleint for P gain
         spf("pid_i_gain = [%s]",root["iGain"]);                             // Assign value sent from cleint for I gain
         spf("pid_d_gain = [%s]",root["dGain"]);                             // Assign value sent from cleint for D gain
      } //if
      else if(action == "get")                                               // If client wants to GET PID values
      {
         spf("[process_Client_JSON_msg] Client [%u] requesting current PID variable values\r\n", num);
         sendClientVariableValue(num,"sendPID");                            // Send client the current value of the PID variables        
      } //else if
      else                                                                   // If ACTION is not one of the above values
      {
         sp("[process_Client_JSON_msg] Client [");sp(num);sp("] has sent unrecognized ACTION [");sp(action);spl("]. Message ignored");        
      } //else
    
   } //else if ITEM PID
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // If the item field has not been accounted for in the IF/ELSE structure here then handle as an unknown command
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   else                                                                      // Unknown item being referenced by client
   {
      sp("[process_Client_JSON_msg] Client NUM [");sp(num);sp("] messaging about unknown ITEM [");sp(item);spl("]. Message ignored");
   } //else ITEM UNKNOWN

} //process_Client_JSON_msg()

/*************************************************************************************************************************************
 This function is used to test roung trip timing of messages to the client.    
 *************************************************************************************************************************************/
void sendClientPing(uint8_t num)
{
   pcnt++;
   String msg = "";                                                          // String to hold JSON message to be transmitted
   StaticJsonBuffer<200> jsonBuffer;                                         // Memory pool for JSON object tree
                                                                             // Use arduinojson.org/assistant to compute the capacity
   JsonObject& root = jsonBuffer.createObject();                             // Create the root of the object tree 
   root["item"] = "ping";                                                    // Element 1 of JSON message
   root["line"] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ123456";                        // Element 2 of JSON message
   root.printTo(msg);                                                        // Convert JSON object tree to string
   if(pcnt%1000==0) 
   {
       sp("[sendClientPing] time after 1000 pings = ");spl(millis());
   } //if
   webSocket.sendTXT(num, msg);                                              // Send JSON message to server (robot)

} //sendClientPing()

/*************************************************************************************************************************************
 This function sends the current state of the onboard LED a specific client (or all clients if argumwnt is 99).    
 *************************************************************************************************************************************/
void sendClientLEDState(uint8_t num)
{

   String msg = "";                                                          // String to hold JSON message to be transmitted
   StaticJsonBuffer<200> jsonBuffer;                                         // Memory pool for JSON object tree
                                                                             // Use arduinojson.org/assistant to compute the capacity
   JsonObject& root = jsonBuffer.createObject();                             // Create the root of the object tree 
   root["item"] = "LED";                                                     // Element 1 of JSON message
   root["action"] = "set";                                                   // Element 2 of JSON message
   if (LEDStatus)                                                            // Check flag that tracks current state of the onboard LED  
   {
      root["value"] = "ledon";                                               // Element 3 of JSON message
   } //if
   else                                                                      // If client wants to turn LED off
   {
      root["value"] = "ledoff";                                              // Element 3 of JSON message
   } //else
   root.printTo(msg);                                                        // Convert JSON object tree to string
   if(num==99)                                                               // If client num is 99 we want to multicast 
   {
      sp("[sendClientLEDState] broadcast this to all clients: ");spl(msg);
      webSocket.broadcastTXT(msg);                                           // Send payload data to all connected clients
   } //if
   else                                                                      // Otherwise we unicast to a specific client
   {
      sp("[sendClientLEDState] unicast this to [");sp(num);sp("]: ");spl(msg);
      webSocket.sendTXT(num, msg);                                           // Send JSON message to server (robot)
   } //else

} //sendClientLEDState()

/*************************************************************************************************************************************
 This function sends the current state of the LCD to a specific client (or all clients if argumwnt is 99)
 *************************************************************************************************************************************/
void sendClientLCDState(uint8_t num)
{

   String msg = "";                                                          // String to hold JSON message to be transmitted
   StaticJsonBuffer<200> jsonBuffer;                                         // Memory pool for JSON object tree
                                                                             // Use arduinojson.org/assistant to compute the capacity
   JsonObject& root = jsonBuffer.createObject();                             // Create the root of the object tree 
   root["item"] = "LCD";                                                     // Element 1 of JSON message
   root["action"] = "set";                                                   // Element 2 of JSON message
   root["line1"] = LCDmsg0;                                                  // Element 3 of JSON message
   root["line2"] = LCDmsg1;                                                  // Element 4 of JSON message    
   root.printTo(msg);                                                        // Convert JSON object tree to string
   if(num==99)                                                               // If client num is 99 we want to multicast 
   {
      sp("[sendClientLCDState] broadcast this to all clients: ");spl(msg);
      webSocket.broadcastTXT(msg);                                           // Send payload data to all connected clients
   } //if
   else                                                                      // Otherwise we unicast to a specific client
   {
      sp("[sendClientLCDState] unicast this to [");sp(num);sp("]: ");spl(msg);
      webSocket.sendTXT(num, msg);                                           // Send JSON message to server (robot)
   } //else

} //sendClientLCDState()

/*************************************************************************************************************************************
 This function calculates data that can be used to assess how well the robot is balancing and sends data to the client in order to 
 graph balance performance. This is a key tool for PID tuning. Expnation of the data calculated is as follows:
 
 num = client wesocket ID
 a1 = Bot variable angle_gyro
 a2 = pid_output_left, the value from PID formula calculation
 a3 = # of clock interrupts between steps
 a4 = Bot variable pid_i_mem, the "I" part of PID

 About b3 (ground speed) logic below:
 Spreadsheet formula is =IF(D9=0,0,ROUND($F$1 * (1000000/(D9*20))*3.1415926*4/200+25,1))
 There are 2 cases (avoiding a division by zero by treating them separately)
    if motor is stopped, i.e. left_motor is zero, then ground speed is zero. easy.
    if motor is stepping after the specified number of clock interrupts...
       speed = rotations per second> * <distance per rotation>
       graphable_speed = scale_factor * <steps per second> * <distance per step>
       = 100 * < (usec in a second) / (usec for a step) > * < ( wheel circumference ) * (circle fraction per step)
       = 100 * < ( 1,000,000 ) / left_motor * 20 usec per interrupt) > * < ( pi * diameter ) * ( 1/200) >
       = 100 * < ( 1,000,000 ) / left_motor * 20 usec per interrupt) > * < ( 3.1415... * 4 inches ) * ( 1/200) >
 *************************************************************************************************************************************/
void sendClientBalanceGraph(uint8_t num,float a1, float a2, float a3, float a4)
{

   String msg = "";                                                          // String to hold JSON message to be transmitted
   StaticJsonBuffer<200> jsonBuffer;                                         // Memory pool for JSON object tree
                                                                             // Use arduinojson.org/assistant to compute the capacity
   JsonObject& root = jsonBuffer.createObject();                             // Create the root of the object tree 
   float b1;                                                                 // Robot's current forward/backward angle, in degrees, 
                                                                             // scaled by 100 to make it easily visible in graph 
   float b2;                                                                 // PID from bot's calculations
   float b3;                                                                 // Robot ground speed 
   float b4;                                                                 // PID from bot's calculations

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Doug's calculations 
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   b1 = a1 *100;                                                             // Robot's current forward/backward angle, in degrees, 
                                                                             // scaled by 100 to make it easily visible in graph 
   b2 = a2;                                                                  // PID from bot's calculations. No further processing needed
   if ( a3 == 0 )                                                            // If ground speed is 0 
   { 
      b3 = 0; 
   } //if
   else                                                                      // If motor is stepping after the specified number of clock interrupts
   { 
      b3 = 100 * ( ( 1000000 / (a3 * 20) ) * ( (3.1415926 * 4 ) / 200) ) ;
   } //else 
   b4 = a4;                                                                  // PID from bot's calculations. No further processing needed

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Build JSON message. Note that floating values are converted to string and get an 'f' appended to them. This is done because our
   // JSON library cannot handle floating point variables. All floating point variables are given 4 digits of precision via the 
   // second argument of the String() function.
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   root["item"] = "balGraph";                                                // Element 1 of JSON message is type of JSON message
   root["angle"] = String(b1, 4)+"f";                                        // Element 2 of JSON message is angle of robot times 100
   root["pid"] = String(b2, 4)+"f";                                          // Element 3 of JSON message is PID value
   root["gspeed"] = String(b3, 4)+"f";                                       // Element 4 of JSON message is calculated ground speed
   root["pmem"] = String(b4, 4)+"f";                                         // Element 5 of JSON message is PID memory, the "I" part of PID
   root.printTo(msg);                                                        // Convert JSON object tree to string

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Send JSON message to specified target client(s)
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   if(num==99)                                                               // If client num is 99 we want to multicast 
   {
      sp("[sendClientLEDState] broadcast this to all clients: ");spl(msg);
      webSocket.broadcastTXT(msg);                                           // Send payload data to all connected clients
   } //if
   else                                                                      // Otherwise we unicast to a specific client
   {
      sp("[sendClientLEDState] unicast this to [");sp(num);sp("]: ");spl(msg);
      webSocket.sendTXT(num, msg);                                           // Send JSON message to server (robot)
   } //else
 
} //sendClientBalanceGraph()

/*************************************************************************************************************************************
 This function sends the client the current value of a variable
 *************************************************************************************************************************************/
void sendClientVariableValue(uint8_t num, String variable)
{

   String msg = "";                                                          // String to hold JSON message to be transmitted
   StaticJsonBuffer<200> jsonBuffer;                                         // Memory pool for JSON object tree
                                                                             // Use arduinojson.org/assistant to compute the capacity
   JsonObject& root = jsonBuffer.createObject();                             // Create the root of the object tree 

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Build JSON message. Note that floating values are converted to string and get an 'f' appended to them. This is done because our
   // JSON library cannot handle floating point variables. 
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   if(variable == "sendTelemetry")                                           // Send client the current value of the telemetry flag  
   {
      root["item"] = "BAL-TEL";                                              // Element 1 of JSON message
      root["action"] = "get";                                                // Element 2 of JSON message
      root["value"] = sendTelemetry;                                         // Element 3 of JSON message (is type Boolean)     
   } //if
   else if(variable == "PID")                                                // Send client the PID tuning data
   {
      spl("[sendClientVariableValue] PID values are float, which cannot be sent via JSON message, so we convert them to Strings");
      sp("[sendClientVariableValue] pid_p_gain = ");spl(pid_p_gain,4);       // Print float to 4 places of precision in console
      sp("[sendClientVariableValue] pid_i_gain = ");spl(pid_i_gain,4);       // Print float to 4 places of precision in console
      sp("[sendClientVariableValue] pid_d_gain = ");spl(pid_d_gain,4);       // Print float to 4 places of precision in console
      root["item"] = "PID";                                                  // Element 1 of JSON message
      root["action"] = "get";                                                // Element 2 of JSON message
      root["pGain"] = String(pid_p_gain, 4)+"f";                             // Element 3 of JSON message is pid_p_gain with 4 digits of precision and an f appended
      root["iGain"] = String(pid_i_gain, 4)+"f";                             // Element 4 of JSON message is pid_p_gain with 4 digits of precision and an f appended
      root["dGain"] = String(pid_d_gain, 4)+"f";                             // Element 5 of JSON message is pid_p_gain with 4 digits of precision and an f appended
   } //else if
   else                                                                      // If the variable specified is not coded for
   {
      root["item"] = "unknownVariable";                                      // Element 1 of JSON message
      root["action"] = "get";                                                // Element 2 of JSON message
      root["value"] = "server bug: see sendClientVariableValue()";           // Element 3 of JSON message (is type String)         
   } //else
   root.printTo(msg);                                                        // Convert JSON object tree to string

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // Send JSON message to specified target client(s)
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   if(num==99)                                                               // If client num is 99 we want to multicast 
   {
      sp("[sendClientLEDState] broadcast this to all clients: ");spl(msg);
      webSocket.broadcastTXT(msg);                                           // Send payload data to all connected clients
   } //if
   else                                                                      // Otherwise we unicast to a specific client
   {
      sp("[sendClientLEDState] unicast this to [");sp(num);sp("]: ");spl(msg);
      webSocket.sendTXT(num, msg);                                           // Send JSON message to server (robot)
   } //else
   
} //sendClientVariableValue()

/***********************************************************************************************************
 This function returns a String version of the local IP address
 ***********************************************************************************************************/
String ipToString(IPAddress ip)
{

    LINE("[ipToString] Converting IP address to String. Source code line: ", __LINE__);
    String s="";
    for (int i=0; i<4; i++)
    s += i  ? "." + String(ip[i]) : String(ip[i]);
    return s;

} //ipToString()

/***********************************************************************************************************
 This function checks to see if the IMU is at the expected address on the I2C bus. If it is then initialize 
 the IMU. Return codes for speaking to the IMU at endTransmission are as follows:
 0:success
 1:data too long to fit in transmit buffer
 2:received NACK on transmit of address
 3:received NACK on transmit of data
 4:other error
 ***********************************************************************************************************/
void initializeIMU()
{

    byte error, lowByte, highByte;
    int address;
    int receive_counter;
    int temp,tcnt1, tcnt2;
    String tmsg;
    LINE("[initializeIMU] Initializing the MPU6050 IMU. Source code line: ", __LINE__);
    sp("[initializeIMU]: Checking to see if the IMU is found at expected I2C address of 0x");
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
        while(Wire.available() < 1); // Wait for reply from IMU slave on I2C bus                                     
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
                tmsg = "[initializeIMU] Delay countdown: " + String(x);
                spl(tmsg);
                delay(1000);
            } //for
            read_mpu_6050_data(); // Read MPU registers                
            spl("[initializeIMU] Create Gyro pitch and yaw offset values by averaging 500 sensor readings...");
            tcnt1 = 0; tcnt2 = 10;
            for(receive_counter = 0; receive_counter < 500; receive_counter++) // Create 500 loops
            {
                tcnt1 ++;
                if(tcnt1 > 50)
                {
                    tcnt1 = 0;
                    tcnt2 --;
                    tmsg = "[initializeIMU] Gyro calc countdown: " + String(tcnt2);
                    spl(tmsg);              
                } //if
                Wire.beginTransmission(MPU_address); // Start communication with the gyro
                Wire.write(0x45); // Start reading the GYRO Y and Z registers
                Wire.endTransmission(); // End the transmission
                Wire.requestFrom(MPU_address, 4); // Request 2 bytes from the gyro
                temp = Wire.read()<<8|Wire.read(); // Combine the two bytes to make one integer, that could be negative
                if(temp > 32767) temp = temp - 65536; // if it's really a negative number, fix it
                gyro_pitch_calibration_value += temp; // 16 bit Y value from gyro, accumulating in 32 bit variable, sign extended
                temp = Wire.read()<<8|Wire.read(); // Combine the two bytes to make one integer, that could be negative
                if(temp > 32767) temp = temp - 65536; // if it's really a negative number, fix it
                gyro_yaw_calibration_value += temp; // 16 bit Z value from gyro, accumulating in 32 bit variable, sign extended
                delay(20);
                Wire.beginTransmission(MPU_address); // Start communication with the IMU
                Wire.write(0x3F); // Get the MPU6050_ACCEL_ZOUT_H value
                Wire.endTransmission(); // End the transmission with the gyro
                Wire.requestFrom(MPU_address,2);
                temp = Wire.read()<<8|Wire.read(); // Read the 16 bit number from IMU
                if(temp > 32767) temp = temp - 65536; // If it's really a negative number, fix it
                balance_calibration_value += temp; // 16 bit Z value from accelerometer, accumulating in 32 bit variable, sign extended
                delay(20);           
            } //for   
            if(gyro_pitch_calibration_value == -500 && 
                gyro_yaw_calibration_value == -500) // If calibration numbers are characteristically weird
            {
                while( 1==1) // request an IMU reset, because it's not working
                {
                    spl("[setup] ******* IMU is stuck and needs a reset **********");
                    delay(3000);
                } //while
            } //if
            gyro_pitch_calibration_value /= 500; // Divide the total value by 500 to get the avarage gyro pitch offset
            gyro_yaw_calibration_value /= 500; // Divide the total value by 500 to get the avarage gyro yaw offset
            balance_calibration_value /=500; // Divide the total value by 500 to get the avarage balance value (Z accelerometer)
            sp("[initializeIMU] gyro_pitch_calibration_value= "); 
            spl(gyro_pitch_calibration_value);
            sp("[initializeIMU] gyro_yaw_calibration_value= "); 
            spl(gyro_yaw_calibration_value);
            sp("[initializeIMU] speed override value= "); spl(speed);    
            sp("[initializeIMU]: Balance value. NOTE - this is the balance value only if the robot is standing upright: ");
            spl(balance_calibration_value);
            sp("[initializeIMU]: acc_calibration_value currently set to ");
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
    spl("[initializeIMU] IMU initialization complete. Ending FreeRTOS task thread");
    imuReady = true;
    //vTaskDelete( NULL );
      
} //initializeIMU()

/***********************************************************************************************************
 This function configures the MPU6050 using settings recommended by Joop Brokking
 ***********************************************************************************************************/
void set_gyro_registers()
{

    LINE("[set_gyro_registers] Configure the MPU6050. Source code line: ", __LINE__);
    spl("[set_gyro_registers] Wake up MPU"); // By default the MPU-6050 sleeps. So we have to wake it up.
    Wire.beginTransmission(MPU_address); // Start communication with the address found during search.
    Wire.write(0x6B); // We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00); // Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission(); // End the transmission with the gyro.

    // Set the full scale of the gyro to +/- 250 degrees per second
    spl("[set_gyro_registers] Set the full scale of the gyro to +/- 250 degrees per second");
    Wire.beginTransmission(MPU_address); // Start communication with the address found during search.
    Wire.write(0x1B); // We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x00); // Set the register bits as 00000000 (250dps full scale)
    Wire.endTransmission(); // End the transmission with the gyro
        
    // Set the full scale of the accelerometer to +/- 4g.
    spl("[set_gyro_registers] Set the full scale of the accelerometer to +/- 4g");
    Wire.beginTransmission(MPU_address); // Start communication with the address found during search.
    Wire.write(0x1C); // We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x08); // Set the register bits as 00001000 (+/- 4g full scale range)
    Wire.endTransmission(); // End the transmission with the gyro

    // Set some filtering to improve the raw data.
    spl("[set_gyro_registers] Set Set Digital Low Pass Filter to ~43Hz to improve the raw data");
    Wire.beginTransmission(MPU_address); // Start communication with the address found during search
    Wire.write(0x1A); // We want to write to the CONFIG register (1A hex)
    Wire.write(0x03); // Set the register bits as 00000011 (Set Digital Low Pass 
                      // Filter to ~43Hz)
    Wire.endTransmission(); // End the transmission with the gyro 

} //set_gyro_registers

/***********************************************************************************************************
 This function reads the accelerometer and gyro info from the MPU6050. Note that we are expweriencing 
 issues with writing to the I2C bus
 ***********************************************************************************************************/
void read_mpu_6050_data()                                              
{

    LINE("[read_mpu_6050_data] Read MPU6050 registers. Source code line: ", __LINE__);
    Wire.beginTransmission(MPU_address); // Start communicating with the MPU-6050
    Wire.write(0x3B); // Send the requested starting register
    Wire.endTransmission(); // End the transmission
    Wire.requestFrom(MPU_address,14); // Request 14 bytes from the MPU-6050
    while(Wire.available() < 14); // Wait until all the bytes are received
    acc_x = Wire.read()<<8|Wire.read(); // Add the low and high byte to the acc_x variable
    if(acc_x > 32767) acc_x = acc_x - 65536; // if it's really a negative number, fix it
    acc_y = Wire.read()<<8|Wire.read(); // Add the low and high byte to the acc_y variable
    if(acc_y > 32767) acc_y = acc_y - 65536; // if it's really a negative number, fix it
    acc_z = Wire.read()<<8|Wire.read(); // Add the low and high byte to the acc_z variable
    if(acc_z > 32767) acc_z = acc_z - 65536; // if it's really a negative number, fix it
    temperature = Wire.read()<<8|Wire.read(); // Add the low and high byte to the temperature variable
    if(temperature > 32767) temperature = temperature - 65536; // if it's really a negative number, fix it
    gyro_x = Wire.read()<<8|Wire.read(); // Add the low and high byte to the gyro_x variable
    if(gyro_x > 32767) gyro_x = gyro_x - 65536; // if it's really a negative number, fix it
    gyro_y = Wire.read()<<8|Wire.read(); // Add the low and high byte to the gyro_y variable
    if(gyro_y > 32767) gyro_y = gyro_y - 65536; // if it's really a negative number, fix it
    gyro_z = Wire.read()<<8|Wire.read(); // Add the low and high byte to the gyro_z variable
    if(gyro_z > 32767) gyro_z = gyro_z - 65536; // if it's really a negative number, fix it

} //read_mpu_6050_data()

/***********************************************************************************************************
 Setup for timer0 interrupt
 ***********************************************************************************************************/
void startTimer0()
{

    LINE("[startTimer0] Initialize and start timer0. Source code line: ", __LINE__);
    timer0 = timerBegin(timer_number_0, timer_prescaler_0, timer_cnt_up); // Pointer to hardware timer 0
    timerAttachInterrupt(timer0, &onTimer0, true); // Bind onTimer function to hardware timer 0
    timerAlarmWrite(timer0, 1000000, true); // Interrupt generated is of edge type not level (third arg) at 
                                            // count of 1000000 (second argument)
    timerAlarmEnable(timer0); // Enable hardware timer 0

} //startISR()

/***********************************************************************************************************
 Setup for A4988 motor controllers
 ***********************************************************************************************************/
void initializeMotorControllers()
{
  
    LINE("[initializeMotorControllers] Set up GPIO pins connected to motors. Source code line: ", __LINE__);
    pinMode(pin_left_dir,OUTPUT); // Note that our motor control pins are Outputs
    pinMode(pin_left_step,OUTPUT);
    pinMode(pin_right_dir,OUTPUT);
    pinMode(pin_right_step,OUTPUT);
    digitalWrite(pin_left_dir, HIGH); // Init direction and step to known starting points
    digitalWrite(pin_left_step, LOW); // i.e. Forward (High) and not stepping (Low)
    digitalWrite(pin_right_dir, HIGH);
    digitalWrite(pin_right_step, LOW);

} //initializeMotorControllers()

/***********************************************************************************************************
 This is the setup function for Arduino sketches
 ***********************************************************************************************************/
void setup() 
{

    Serial.begin(115200); // Open a serial connection at 115200bps
    Serial.println("");
    LINE("[setup] Start of sketch. Source code line: ", __LINE__);
    sp("[setup] PIN attached to onboard LED = "); spl(LED_BUILTIN);
    pinMode(LED_BUILTIN, OUTPUT); // Take control on onbaord LED
    writeLED(false); // Turn onboard LED off
    display_Running_Sketch(); // Show environment details in console
    Serial.setDebugOutput(true); // http://esp8266.github.io/Arduino/versions/2.0.0/doc/reference.html
    for(uint8_t t = 4; t > 0; t--) // Allow time for ESP32 serial to initialize 
    {
        spf("[setup] Boot wait %d...\r\n", t); // Count down message to console
        Serial.flush(); // Wait for message to clear buffer
        delay(1000); // Allow time to pass
    } //for   
    startI2Cbus(); // Scan the I2C bus for connected devices
    spl("[setup] Initialize IMU");
    initializeIMU();
    /*
    xTaskCreate(initializeIMU, // Create FreeRTOS task. This will get the IMU going in a parallel thread
        "InitIMU", // String with name of task for debug purposes
        10000, // Stack size in words
        NULL, // Parameter passed as input of the task
        2, // Priority of the task
        NULL); // Task handle
    */
    initializeLCD(); // Initialize the Open Smart 1602 LCD Display
    sendLCD("Boot Sequence",LINE1); // Boot message to LCD line 1
    sendLCD("Start WiFi",LINE2); // Starting WiFi message to LCD line 2
    startWiFi(); // Start WiFi and connect to AP
    startDNS(); // Start domain name service
    sendLCD("Start Webserver",LINE2); // Starting Webserver message to LCD line 2
    startWebServer(); // Start web service 
    sendLCD("Start Websockets",LINE2); // Starting Websockets message to LCD line 2
    startWebsocket(); // Include libaries in Websockets.h missing. Need to fix this.
 /*
    if(imuReady == false)
    {
        spl("[setup] Waiting for IMU init to complete");
        sendLCD("Waiting for IMU",LINE2); // Starting IMU message to LCD line 2
        while(imuReady == false); // Wait for IMU initialization to complete (done in seperate FreeRTOS thread)
    } //if
*/
    xTaskCreatePinnedToCore(monitorWebsocket, // Create FreeRTOS task. This will monitor websocket activity in a parallel thread
        "monitorWebsocket", // String with name of task for debug purposes
        10000, // Stack size in words. Tried lower number but causes reboots
        NULL, // Parameter passed as input of the task
        2, // Priority of the task
        NULL, // Task handle
        0); // Specify which of the two CPU cores to pin this task to
    xTaskCreatePinnedToCore(monitorWeb, // Create FreeRTOS task. This will monitor web activity in a parallel thread
        "monitorWeb", // String with name of task for debug purposes
        10000, // Stack size in words. Tried lower number but causes reboots
        NULL, // Parameter passed as input of the task
        2, // Priority of the task
        NULL, // Task handle
        0); // Specify which of the two CPU cores to pin this task to
    xTaskCreatePinnedToCore(balanceRobot, // Create FreeRTOS task. This will keep the robot balanced
        "balanceRobot", // String with name of task for debug purposes
        10000, // Stack size in words.
        NULL, // Parameter passed as input of the task
        1, // Priority of the task
        NULL, // Task handle
        1); // Specify which of the two CPU cores to pin this task to
/*
    xTaskCreate(monitorWebsocket, // Create FreeRTOS task. This will monitor websocket activity in a parallel thread
        "monitorWebsocket", // String with name of task for debug purposes
        10000, // Stack size in words. Tried lower number but causes reboots
        NULL, // Parameter passed as input of the task
        2, // Priority of the task
        NULL); // Task handle
    xTaskCreate(monitorWeb, // Create FreeRTOS task. This will monitor web activity in a parallel thread
        "monitorWeb", // String with name of task for debug purposes
        10000, // Stack size in words. Tried lower number but causes reboots
        NULL, // Parameter passed as input of the task
        2, // Priority of the task
        NULL); // Task handle
*/
    sendLCD("Init Motors",LINE2); // Send more boot message to LCD line 2
    initializeMotorControllers(); // Initialize the motor controllers
    sendLCD(ipToString(WiFi.localIP()),LINE1); // Send IP address to LCD line 1
    sendLCD("Version: " + my_ver,LINE2); // Send version of code to LCD line 2
    startTimer0();
    spl("[setup] initialization process complete");
    loop_timer = micros() + LoopDelay; // Set the loop_timer variable at the next end loop time

} //setup()

/***********************************************************************************************************
 This function monitors the websocket service for client activity. It runs as its own FreeRTOS task. 
 Variables and objects used in here need to be declared volitile in order to be accessible by the main
 FreeRTOS task.
 ***********************************************************************************************************/
void monitorWebsocket(void * parameter) 
{

    while(1)
    {
        webSocket.loop(); // Poll for websocket client events
        delay(10); // required delay to prevent watchdog timer gping off
    } //while
    vTaskDelete( NULL );

} //monitorWebsocket()

/***********************************************************************************************************
 This function monitors the web service for client activity. It runs as its own FreeRTOS task. 
 Variables and objects used in here need to be declared volitile in order to be accessible by the main
 FreeRTOS task.
 ***********************************************************************************************************/
void monitorWeb(void * parameter) 
{

    while(1)
    {
        server.handleClient(); // Poll for web server client
        delay(10); // required delay to prevent watchdog timer gping off
    } //while
    vTaskDelete( NULL );

} //monitorWebsocket()

/***********************************************************************************************************
 This function balances the robot. It runs as its own FreeRTOS task. Variables and objects used in here need 
 to be declared volitile in order to be accessible by the main FreeRTOS task.
 ***********************************************************************************************************/
void balanceRobot(void * parameter)
{

    while(1)
    {

        int temp; // Interim placeholder variable
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // Note time at start of loop
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        mics = micros(); // Spot check the length of the main loop
        millis_now = millis();
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // Check to see if telemetry data needs to be sent to connected clients
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        if(sendTelemetry == "flagon") // Check to see if any client has requested telemetry info
        {
            sendClientBalanceGraph(99,            // Send data to clients used to assess balancing performance
                                angle_gyro,       // AM: target specific client instead of broadcast?
                                pid_output_left,
                                left_motor,pid_i_mem
                                ); 
        } //if                         
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // angle calculations   
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        Wire.beginTransmission(MPU_address); // Start communication with the gyro
        Wire.write(0x3F); // Start reading at register 3F (ACCEL_ZOUT_H)
        Wire.endTransmission(); // End the transmission
        Wire.requestFrom(MPU_address, 2); // Request 2 bytes from the gyro
        temp = Wire.read()<<8|Wire.read(); // Combine the two bytes to make one integer
        if(temp > 32767) temp = temp - 65536; // if it's really a negative number, fix it
        accelerometer_data_raw = temp;
        accelerometer_data_raw += acc_calibration_value; // Add the accelerometer calibration value
        if(accelerometer_data_raw > 8192)accelerometer_data_raw = 8192; // Prevent division by zero by 
                                                                        // limiting the acc data to +/-8200;
        if(accelerometer_data_raw < -8192)accelerometer_data_raw = -8192; // Prevent division by zero by 
                                                                        // limiting the acc data to +/-8200;
        angle_acc = asin((float)accelerometer_data_raw/8192.0)* 57.296; // Calculate the current angle 
                                                                        // according to the accelerometer
        if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5) // If the accelerometer angle is almost 0
        {
            angle_gyro = angle_acc; // Load the accelerometer angle in the angle_gyro variable
            start = 1;                             // Set the start variable to start the PID controller
        } //if
        Wire.beginTransmission(MPU_address); // Start communication with the gyro
        Wire.write(0x43); // Start reading at register 43
        Wire.endTransmission(); // End the transmission
        Wire.requestFrom(MPU_address, 4); // Request 4 bytes from the gyro
        temp = Wire.read()<<8|Wire.read(); // Combine the two bytes read to make one 16 bit signed integer
        if(temp > 32767) temp = temp - 65536; // if it's really a negative number, fix it
        gyro_yaw_data_raw = temp; // and use result as raw data, which is yaw degrees/sec * 65.5
        temp = Wire.read()<<8|Wire.read(); // Combine the two bytes read to make one 16 bit signed integer
        if(temp > 32767) temp = temp - 65536; // if it's really a negative number, fix it
        gyro_pitch_data_raw = temp; // and use result as raw data, which is pitch degrees/sec * 65.5
        gyro_pitch_data_raw -= gyro_pitch_calibration_value; // Add the gyro calibration value
        angle_gyro += gyro_pitch_data_raw * 0.000030534; // Calculate the traveled during this loop angle 
                                                        // and add this to the angle_gyro variable
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // MPU-6050 offset compensation. Not every gyro is mounted 100% level with the axis of the robot. This 
        // can be cause by misalignments during manufacturing of the breakout board. As a result the robot
        // will not rotate at the exact same spot and start to make larger and larger circles. To compensate
        // for this behavior a VERY SMALL angle compensation is needed when the robot is rotating. Try  
        // 0.0000003 or -0.0000003 first to see if there is any improvement.
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        gyro_yaw_data_raw -= gyro_yaw_calibration_value; // Add the gyro calibration value
        // Uncomment the following line to make the compensation active
        // re-comment the line below to see if angle calibration gets more accurate
        angle_gyro -= gyro_yaw_data_raw * 0.0000003; // Compensate the gyro offset when the robot is rotating
        //angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004; // Correct the drift of the gyro angle with 
                                                                 // the accelerometer angle
        angle_gyro = angle_gyro * 0.996 + angle_acc * 0.004; // Correct the drift of the gyro angle with the 
                                                             // accelerometer angle
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // PID controller calculations. The balancing robot is angle driven. First the difference between the 
        // desired angel (setpoint) and actual angle (process value) is calculated. The self_balance_pid_setpoint 
        // variable is automatically changed to make sure that the robot stays balanced all the time. The 
        // (pid_setpoint - pid_output * 0.015) part functions as a brake function.
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
        if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015;
        //try to reduce longevity of pid_i_mem, which gets big, and stays big
        //pid_i_mem += pid_i_gain * pid_error_temp; // Calculate the I-controller value and add it to the  
                                                    // pid_i_mem variable
        temp = pid_i_gain * pid_error_temp; // current I controller value
        hold2 = pid_i_mem; // grab it for debugging before it gets changed
        pid_i_mem =temp + PID_I_fade * pid_i_mem; // allow impact of past pid_i_mem history to fade out over time
        if(pid_i_mem > pid_max)pid_i_mem = pid_max; // Limit the I-controller to the parameterized maximum  
                                                    // controller output
        else if(pid_i_mem < pid_min)pid_i_mem = pid_min;
        //Calculate the PID output value
        pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
        hold3 = pid_output;
        if(pid_output > pid_max)pid_output = pid_max; // Limit the PI-controller to the maximum controller output
        else if(pid_output < pid_min)pid_output = pid_min;
        pid_last_d_error = pid_error_temp; // Store the error for the next loop

        if(pid_output < 5 && pid_output > -5)pid_output = 0; // Create a dead-band to stop the motors when the robot 
                                                            // is balanced

        if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1) // If the robot tips over or the start 
                                                                            // variable is zero or the battery is 
                                                                            // empty
        {
            pid_output = 0; // Set the PID controller output to 0 so the motors stop moving
            pid_i_mem = 0; // Reset the I-controller memory
            start = 0; // Set the start variable to 0
            self_balance_pid_setpoint = 0; // Reset the self_balance_pid_setpoint variable
            throttle_left_motor = 0; // stop the wheels from moving
            throttle_right_motor = 0;
        } //if

        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // Control Calculations (does nothing without numchuk controller to provide "received_byte", which  
        // stays at zero
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        pid_output_left = pid_output;  // Copy the controller output to the pid_output_left variable for the  
                                       // left motor
        pid_output_right = pid_output; // Copy the controller output to the pid_output_right variable for the 
                                       // right motor
        if(received_byte & B00000001)  // If the first bit of the receive byte is set change the left and right 
        {                              // variable to turn the robot to the left
            pid_output_left += turning_speed; // Increase the left motor speed
            pid_output_right -= turning_speed; // Decrease the right motor speed
        } //if
        if(received_byte & B00000010) // If the second bit of the receive byte is set change the left and right 
        {                             // variable to turn the robot to the right
            pid_output_left -= turning_speed; // Decrease the left motor speed
            pid_output_right += turning_speed; // Increase the right motor speed
        } //if
        if(received_byte & B00000100) // If the third bit of the receive byte is set change the left and right 
        {                             // variable to turn the robot to the right
            if(pid_setpoint > -2.5)pid_setpoint -= 0.05; // Slowly change the setpoint angle so the robot starts 
                                                        // leaning forewards
            if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005; // Slowly change the setpoint angle so 
                                                                        // the robot starts leaning forewards
        } //if
        if(received_byte & B00001000) // If the forth bit of the receive byte is set change the left and right 
        {                             // variable to turn the robot to the right
            if(pid_setpoint < 2.5)pid_setpoint += 0.05; // Slowly change the setpoint angle so the robot starts 
                                                        // leaning backwards
            if(pid_output < max_target_speed)pid_setpoint += 0.005; // Slowly change the setpoint angle so the 
                                                                    // robot starts leaning backwards
        } //if   
        if(!(received_byte & B00001100)) // Slowly reduce the setpoint to zero if no foreward or backward command 
                                        // is given
        {
            if(pid_setpoint > 0.5)pid_setpoint -=0.05; // If the PID setpoint is larger then 0.5 reduce the setpoint 
                                                    // with 0.05 every loop
            else if(pid_setpoint < -0.5)pid_setpoint +=0.05; // If the PID setpoint is smaller then -0.5 increase 
                                                            // the setpoint with 0.05 every loop
            else pid_setpoint = 0; // If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint 
                                // to 0
        } //if
        //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. 
        // This way the robot will always find it's balancing point
        if(pid_setpoint == 0) // If the setpoint is zero degrees
        {
            //if(pid_output < 0)self_balance_pid_setpoint += 0.0015; // Increase the self_balance_pid_setpoint if the 
                                                                    // robot is still moving forewards
            //if(pid_output > 0)self_balance_pid_setpoint -= 0.0015; // Decrease the self_balance_pid_setpoint if the 
                                                                    // robot is still moving backwards
        } //if
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // Motor Pulse calculations. To compensate for the non-linear behaviour of the stepper motors the 
        // following calculations are needed to get a linear speed behaviour.
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
        // else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;
        // if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
        // else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;
        // Calculate the needed pulse time for the left and right stepper motor controllers
        // if(pid_output_left > 0)left_motor = 400 - pid_output_left;
        // else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
        // else left_motor = 0;
        // if(pid_output_right > 0)right_motor = 400 - pid_output_right;
        // else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
        // else right_motor = 0;
        // save a value for once a second debugging before we overwrite it
        hold = pid_output_left;
        // modifications for A4988 based motor controllers:
        //  - ignore linearity considerations. Don't think the difference is substantial
        //  - map the pid output directly to speed range of motors.
        //  - reliable speeds are from 300 steps/sec (fast) to 2300 steps/sec (slow)
        //  - pid range is 1(slow) to 400 (fast)
        //  - all of this applies to both directions, + and - values
        //  - so, linearly map (400 > 0) to (300 > 2300)
        //
        // Calculate the needed pulse time for the left and right stepper motor controllers
        if(pid_output_left > 0)left_motor = bot_slow - (pid_output_left/400)*(bot_slow - bot_fast);
        else if(pid_output_left < 0)left_motor = -1*bot_slow - (pid_output_left/400)*(bot_slow - bot_fast);
        else left_motor = 0;
        if(pid_output_right > 0)right_motor = bot_slow - (pid_output_right/400)*(bot_slow - bot_fast);
        else if(pid_output_right < 0)right_motor = -bot_slow - (pid_output_right/400)*(bot_slow - bot_fast);
        else right_motor = 0;
        //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
        if (speed >= 0) // if we're overriding IMU to force a constant test speed
        {
            noInterrupts(); // ensure interrupt can't happen when only one wheel is updated
            throttle_left_motor = speed; // overwrite the calculated wheel intervals
            throttle_right_motor= speed; // ...with fixed value, maybe zero to brake for vertical calibration
            interrupts();                // by briefly disabling interrupts
        } //if
        else // restructure conditional so we don't have a brief wrong setting
        {
            noInterrupts(); // ensure interrupt can't happen when only one wheel is updated
            throttle_left_motor = -1* left_motor;   // oops - corections were in wrong direction
            throttle_right_motor = -1 *right_motor; // ..so need to negate the throttle values
            interrupts();                           // ..by briefly disabling interrupts
        } //else
        // do some debug output to serial monitor to see whats going on
        //count 4 millesecond loops to get to a second, and dump debug info once a second
        if (i++ > 250) // if a full second has passed, display debug info
        {
            i = 0; // prepare to count up to next second
            float dtmp = angle_gyro; // temp cheat to help tune balance point
            spd("--pid_error_temp= ",pid_error_temp); spd("  angle_gyro= ",angle_gyro); spd("  dtmp= ",dtmp); spd(" start= ",start);
            spd("  throttle_left_motor= ",throttle_left_motor); spd("  left_motor= ",left_motor); spl();        
        } //if
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // Poll network services for client activity. Note each of these polls are meant to run in a seperate 
        // thread, which may be commented out in setup()
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        //server.handleClient(); // Poll for web server client
        //webSocket.loop(); // Poll for websocket client events

        //////////////////////////////////////////////////////////////////////////////////////////////////////
        // loop_timer: The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop 
        // is exactly 4 milliseconds a wait loop is created by setting the loop_timer variable to +4000 
        // microseconds every loop.
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        last_millis = millis()-millis_now; // track previous loop's length as well
        if(loop_timer > micros()) // if the target loop end time is still in the future..
        {
            while(loop_timer > micros()) {}; // spin in this while until micros() catches up with loop_timer
        } //if                               // no spin time needed if we're already past old target loop end
        loop_timer = micros() + 4000;        // time next target loop end time is 4 msec from now.    
    } //while    

} //balanceRobot()

/***********************************************************************************************************
 This is the main function for Arduino sketches
 ***********************************************************************************************************/
void loop() 
{

} //loop()