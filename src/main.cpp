/** @file main.cpp
 *    This file contains things that need to be documented
 * 
 *    
 * 
 *  @author  Simon Schoennauer, Deven Frauenhofer, Lynette Cox
 *  @date    4 Nov 2020 Original file
 *  @date    1 Dec 2020 Final file
 */
 
#include <Arduino.h>
#include <PrintStream.h>
#if (defined STM32L4xx || defined STM32F4xx)
    #include <STM32FreeRTOS.h>
#endif
 
#include <Wire.h>
#include "Adafruit_FXOS8700.h"
#include <math.h>
#include "taskqueue.h"
#include "taskshare.h"
#include <analogWrite.h>
#include <cmath>
#include "TOD.h"
#include "P13.h"
 
/// Define Shares & Queues Below
Share<float> elevation ("ELEV");             // current elevation in milidegrees
Share<float> azimuth ("AZI");                // current azimuth in milidegrees
Share<bool> clear_elevation ("CLEAR ELEV");  // make this true to set current elevation to 0deg
Share<bool> clear_azimuth ("CLEAR AZI");     // make this true to set current elevation to 0deg
Share<bool> elev_direction("ELEV DIR");      // either a 1 or 0 to represent direction of the elev motor
Share<bool> azi_direction("AZI DIR");        // either a 1 or 0 to represent direction of the elev motor
Share<float> elevation_sp("SP ELEV");        // elevation setpoint in milidegrees
Share<float> azimuth_sp("SP AZI");           // azimuth setpoint in milidegrees
Share<float> mag("MAG");                     // queue of latest magnetometer data
Share<float> accel ("ACCEL");                // queue of latest accelerometer data
Share<bool> start("start");                  // true if we're ready to begin the tracking sequence
// Ephemeris Shares, contain the time data used to track satellite
Queue<uint8_t> starttime_queue(6);           // contains starting date details: year/month/day/hour/minute/second
Queue <uint8_t> currenttime_queue(48);       // contains the current year/month/day/hour/minute/second
Share<float> elevation_coords("ELEV_CORDS"); // contains the next desired elevation position
Share<float> azimuth_coords("AZI_CORDS");    // contains the next desired azimuth position
 
 
/** @brief   Task which talks to the FXOS8700 accelerometer/magnetometer, and  
 *           outputs inclination angle and magnetic heading.
 *  @details This task reads raw 3-axis data from both the accelerometer
 *           and magnetometer. The magnetometer data is calibrated before it is
 *           converted into a magnetic heading. The accelerometer data is directly
 *           converted into a tilt angle. The magnetic heading (azimuth angle) 
 *           and elevation angle are placed into shared data variables.
 *           This task uses the Adafruit FXOS8700 library at 
 *           @c https://github.com/adafruit/Adafruit_FXOS8700.git
 *  @param   p_params Pointer to parameters passed to this function; we don't
 *           expect to be passed anything and so ignore this pointer
 *  @param   ax variable that contains the raw X-axis accelerometer reading
 *  @param   ay variable that contains the raw Y-axis accelerometer reading
 *  @param   az variable that contains the raw Z-axis accelerometer reading
 *  @param   mx variable that contains the raw X-axis magnetometer reading
 *  @param   my variable that contains the raw Y-axis magnetometer reading
 *  @param   mz variable that contains the raw Z-axis magnetometer reading
 *  @param   azi variable that contains the current azimuth heading, in deg.
 *  @param   el variable that contains the current elevation angle, in deg.
 */
void task_accelmag (void* p_params)
{
    (void)p_params;
    // Initialize Variables
    double ax;        // acceleration in x, y, z
    double ay;
    double az;
    double mx;        // magnetic orientation in x, y, z
    double my;
    double mz;
    float azi = 0;    // magnetic heading CW in degrees from north (N = 0 deg)
    float el = 0;     // current elevation angle, in deg from horizontal
 
    // Initialize the I2C bus and accelerometer driver
    Wire.begin ();
    Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);      // Create instance of the FXOS class
 
    // Initialize the sensor 
    if (!accelmag.begin(ACCEL_RANGE_4G)) {
        /* There was a problem detecting the FXOS8700 ... check your connections */
        Serial.print("Ooops, no FXOS8700 detected ... Check connections. Assigned Address from FXOS8700.h: ");
        Serial.println(FXOS8700_ADDRESS);
        while (1);
    }
    
    // If the accel/mag works, ask it for accel and mag data
    for (;;)
    {
        // Create event variable
        sensors_event_t aevent, mevent;
 
        // Get a new sensor event, store ordered results in shares
        accelmag.getEvent(&aevent, &mevent);
        ax = aevent.acceleration.x;
        ay = aevent.acceleration.y;
        az = aevent.acceleration.z;
        mx = mevent.magnetic.x;
        my = mevent.magnetic.y;
        mz = mevent.magnetic.z;
 
        // Calibrate magnetometer results
	// The calibration was done by pointing each axis roughly in the direction of the magnetic field, so that the sensor reading                       for that axis was at a minimum/maximum, and recording the reading. Then, the PCB was flipped 180 degrees and the reading was taken again. The average of these two readings for each axis represents the error/bias of the sensor. This error is subtracted from the magnetometer readings in the code.
        mx = mx + 57.5;    
        my = my + 66.75;
        mz = mz - 51;
        
        // Calculate magnetic heading
        el = atan2(-az,ay);
        azi = -atan2(mx, -my*sin(el) - mz*cos(el) );
        
        // Convert to deg
        el = el*57.295;
        azi = azi*57.295;
 
        accel.put(el);     // put current elevation into share
        mag.put(azi);      // put current azimuth into share
 
        delay(30);
    }
}
 
 
/** @brief   Task that interfaces with optical transmission sensors to implement an
 *           encoder, this task updates the current position read from the encoders
 *  @details This task counts ticks for both the elevation and azimuth motor encoders.
 *           This task resets encoder count if it is flagged to do so. This task also 
 *           converts total encoder count into angular position data for the elevation
 *           and azimuth axes using a calibration constant defined by the gear 
 *           reductions in the mechanical system.
 *  @param   p_params Pointer to parameters passed to this function; we don't
 *           expect to be passed anything and so ignore this pointer
 *  @param   elevation This is a share that is updated to contain the current elevation 
 *           pointing position, in millidegrees, as read from the encoders. 
 *  @param   azimuth This is a share that is updated to contain the current azimuth 
 *           magnetic heading, in millidegrees, as read from the encoders. 
 *  @param   clear_elevation A bool, if set true, the task sets @param elevation to zero and sets 
 *           @param clear_elevation to false
 *  @param   clear_azimuth A bool, if set true, the task sets @param azimuth to zero and sets 
 *           @param clear_azimuth to false
 *  @param   elev_direction This contains the elevation motor direction so the
 *           encoder can determine when to increment or decrement the total encoder count
 *  @param   azi_direction This contains the azimuth motor direction so the  
 *           encoder can determine when to increment or decrement the total encoder count
 */
void task_position (void* p_params)
{
    (void)p_params;             // does nothing, clears a compiler warning
    // Initialize Variables
    bool el_prev_state = false;
    bool azi_prev_state = false;
    int azi_tics = 0;
    int el_tics = 0;
    // Variables to hold values pulled from shares
    bool el_clear = false;
    bool azi_clear = false;
    bool elev_dir;              
    bool azi_dir;              
    float el_deg;
    float azi_deg;
    float el_val;
    float azi_val;
    // Redundant variables needed to update shares
    bool false_var = false;     
 
    // Set up ADC pins for encoders
    pinMode(26, INPUT);         //Setup for elevation ADC pin
    pinMode(27, INPUT);         //Setup for azimuth ADC pin
 
    // Encoder Position Task Loop
    for (;;)
    {   
        // Grab values from shares
        clear_elevation >> el_clear;           //grab clear_elevation bool
        clear_azimuth >> azi_clear;            //grab clear_azimuth bool
        elev_direction >> elev_dir;            //grab current elevation motor direction
        azi_direction >> azi_dir;              //grab current azimuth motor direction
        
        // Runs if the elevation encoder position must be reset
        if (el_clear){
            el_tics = 0;                       //reset elevation encoder position to 0
            elevation >> el_val;
            el_tics = el_val*75.79;
            clear_elevation << false_var;      //resets clear_elevation flag for the next call
        }
        // Runs if the azimuth encoder position must be reset
        if (azi_clear){
            azi_tics = 0;                      //reset azimuth encoder position to 0
            azimuth >> azi_val;
            azi_tics = azi_val*197.35;
            clear_azimuth << false_var;        //resets clear_azimuth flag for the next call
        }
        // Runs if the elevation encoder reads high and if it has changed states since the last function call
        if ((analogRead(26) > 3500) && el_prev_state==false){
            el_prev_state = true;
            // If the motor is spinning forward, increment encoder count
            if (elev_dir){
                el_tics++;
            }
            // If the motor is spinning in reverse, decrement encoder count
            else if (!elev_dir){  
                el_tics--;
            }
            else{
                Serial << "Elevation encoder increment issue" << endl;
            }
        }
        // Determine if elev encoder reads low and if it has not changed posiiton since the last read
        else if((analogRead(26) < 3000) && (el_prev_state==true)){
            el_prev_state = false;
        }
        // Runs if the azimuth encoder reads high and if it has changed states since the last function call
        if ((analogRead(27) > 3500) && azi_prev_state==false){
            azi_prev_state = true;
            // If the motor is spinning forward, increment encoder count
            if (azi_dir){
                azi_tics++;
            }
            // If the motor is spinning in reverse, decrement encoder count
            else if (!azi_dir){  
                azi_tics--;
            }
            else{
                Serial << "Azimuth encoder increment issue" << endl;
            }
        }
        // Determine if azi encoder reads low and if it has not changed posiiton since the last read
        else if((analogRead(27) < 3000) && (azi_prev_state==true)){
            azi_prev_state = false;
        }
        // Convert encoder ticks to position (deg)
        el_deg = (13.194*el_tics);       // convert encoder ticks to millidegrees of rotation (12.5 millideg/elev tic)
        elevation << el_deg;             // Place current position into elevation share
        azi_deg = (5.067*azi_tics);      // converts encoder ticks to millidegrees of rotation (4.81?)
        azimuth << azi_deg;              // Place current position into elevation share
               
        // Task delay
        vTaskDelay(1); //delay of 1
    }
}
 
 
/** @brief   Task which handles the 3 main states of the satellite tracker 
 *           positioning system.
 *  @details This task handles the following 3 states:
 *               state 0 - HOME STATE, this state is meant to run at the initial power 
 *                         up of the satellite tracking system. In this state, the 
 *                         system position itself toward the initial (start) location of  
 *                         the next satellite tracking sequence. Then moves into state 1.
 *               state 1 - WAIT STATE, this state occurs when the tracker is waiting to begin
 *                         its track sequence. The tracker is aligned with the initial start
 *                         position of the next satellite track sequence. This state checks
 *                         current time (from the internet) and compares it to the inital
 *                         start position sequence time. If the next satellite sequence start
 *                         time matches the current time, moves into state 2.  
 *               state 2 - TRACK STATE, this state occurs while the tracker is carrying out 
 *                         a tracking sequence. The tracker will sweep across the sky using
 *                         coordinates it receives 
 *  @param   p_params Pointer to parameters passed to this function; we don't
 *           expect to be passed anything and so ignore this pointer.
 */
void task_supervisor (void* p_params) //this task is actually the master task
{
    (void)p_params;                   //shuts up compiler warning
    /// Initialize Variables
    uint8_t state = 0;                //contains current state of supervisor task, begin in HOME state
    // Redundant variables that have to exist in order to place stuff into shares
    bool false_var = false;
    bool true_var = true;
    float float_zero;
    // These are local variables that hold values pulled from shares
    float el_sp; 
    float azi_sp;
    float elevation_val = 0;
    float azimuth_val = 0;
    // Variables holding start time data from the starttime_queue
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    // Variables for recording current time
    uint8_t day_now;
    uint8_t month_now;
    uint8_t year_now;
    uint8_t hour_now;
    uint8_t minute_now;
    uint8_t second_now;
    float azi_heading = 150;
    uint8_t wait = 0;
    
    // Place initial values into queues
    elevation_sp << float_zero;       // sets initial elevation setpoint = 0
    azimuth_sp << float_zero;         // sets initial azimuth   setpoint = 0
    elevation << float_zero;          // sets initial elevation position to 0 
    azimuth << float_zero;            // sets initial azimuth   position to 0   
    clear_elevation << false_var;     // does not reset elevation count
    elev_direction << false_var;      // sets initial elevation motor direction
    clear_azimuth << false_var;
    azi_direction << false_var;
 
    // Motor driver setup for PH/EN Mode
    pinMode(15, OUTPUT); //gpio 15 is the mode set (low for PH/EN high for PWM)
    digitalWrite(15,0);
    pinMode(12, OUTPUT); //gpio 12 is the PH for motor B (Azimuth)
    digitalWrite(12, 1);
    pinMode(14, OUTPUT); //gpio 14 is the EN for motor B (Azimuth)
    analogWrite(14, 0);
    pinMode(18, OUTPUT); //gpio 18 is the PH for motor A (Elevation)
    digitalWrite(18,1);
    pinMode(19, OUTPUT); //gpio 19 is the EN for motor A (Elevation)
    analogWrite(19, 100);
 
    // Supervisor task loop
    for (;;){
        // Grab values from shares
        elevation >> elevation_val;   // grab current elevation from share 'elevation', place in 'elevation_val'
        elevation_sp >> el_sp;        // grab elevation setpoint from share 'elevation_sp', place in 'el_sp'
        azimuth >> azimuth_val;       // grab current azumuth from share 'azimuth', place in 'azimuth_var'
        azimuth_sp >> azi_sp;         // grab azimuth setpoint from share 'azimuth_sp', place in 'azi_sp'
 
        Serial << "State " << state << " | Current Elev: " << elevation_val << " | Elev sp: " << el_sp << " | Current Azi: " << azimuth_val << " | Azi sp: " << azi_sp << endl;
 
        /// 0-2: SUPERVISOR TASK STATE MACHINE
        // 0. HOME STATE (run at initial power-up; read magnetometer and move to start location of next sattrack sequence. then move to state 1)
        if(state == 0){
            elevation_sp << elevation_val;
            if (wait < 4){
                wait++;
            }
            else{
                accel >> elevation_val;
                mag >> azimuth_val;
                elevation << elevation_val;
                clear_elevation << true_var;
                azimuth << azi_heading;
                clear_azimuth << true_var;
                elevation_coords >> el_sp;
                elevation_sp << el_sp;
                azimuth_coords >> azi_sp;
                azimuth_sp << azi_sp;
                state = 1;
            }
            
        } // end of state 0
        
        // 1. WAIT STATE (tracker is aligned at start position, checks current time from internet, compares to initial start time of sequence. move into state 2 when true.)
        else if(state == 1){
            // Old code for testing (bypasses internet time start flag)
            // if(wait < 120){
            //     wait++;
            // }
            // else{
            //     azi_sp = 1000; //test
            //     azimuth_sp << azi_sp;
            //     start << true_var;       // start tracking sequence (activate motor control)
            //     state = 2;
            // }
            // } // end of state 1
 
            // Pull desired sequence start time & current time from queues
            starttime_queue.get(year);
            starttime_queue.get(month);
            starttime_queue.get(day);
            starttime_queue.get(hour);
            starttime_queue.get(minute);
            starttime_queue.get(second);
            currenttime_queue.get(year_now);
            currenttime_queue.get(month_now);
            currenttime_queue.get(day_now);
            currenttime_queue.get(hour_now);
            currenttime_queue.get(minute_now);
            currenttime_queue.get(second_now);
 
            // Check if we're ready to begin the sequence by comparing current time to desired start time
            if( (day_now == day) && (month_now == month) && (year_now == year) ) {                //first verify the correct date
                if( (hour_now == hour) && (minute_now == minute) && (second_now >= second) ) {    //next verify the correct time, seconds are triggered if time is >= desired ???
                    start << true_var;                                                            // begin tracking
                    state = 2;                                                                    // start time reached, move into tracking state
                    Serial << "Date(Month/Day/Year): " << month_now << "/" << day_now << "/" << year_now << "|  Time(hr/min/sec): " << hour_now << "/" << minute_now << "/" << second_now <<  endl;
                    Serial << "Tracking State Begin. " << endl;
                }
                else{
                    // do nothing, we aren't ready to change state yet (not time yet)
                }
            }
            else{
                // do nothing, we aren't ready to change state yet (not right date)
                Serial << "Month/day/year does not match desired start month/day/year" << endl;
            }
 
        } // end of state 1
 
        // 2. TRACK STATE (tracker carries out tracking sequence. Sweeps across sky.)
        else if(state == 2){
            // synchronizes 2 sets of shares
            elevation_coords >> el_sp;    // sets elevation_sp = elevation_coords
            elevation_sp << el_sp;
            azimuth_coords >> azi_sp;     // sets azimuth_sp = azimuth_coords
            azimuth_sp << azi_sp;
        }
        else {
            Serial << "Supervisor task state error" << endl;
        } // end of state 2
 
        // Supervisor task time delay 
        vTaskDelay(1000);
    }
} // end supervisor task
 
 
/** @brief   Function which controls the azimuth and elevation motors using
 *           closed-loop speed control.
 *  @details This function calculates and updates both the elevation 
 *           and azimuth motor speeds based on current and desired 
 *           position information. 
 *  @param   p_params Pointer to parameters passed to this function; we don't
 *           expect to be passed anything and so ignore this pointer.
 */
void task_control (void* p_params) 
{
    (void)p_params;                   //shuts up compiler warning
    // Initialize Variables
    uint8_t n = 0; //IMPORTANT this keeps track of the timing it's incremented each time the task is entered (ie. every 5ms)
    float el_gain = -50;    // IMPORTANT make sure negative for stability, adjust this to change el control loop
    float azi_gain = -50;   // adjust this to change azi control loop
    uint8_t el_duty_cycle;
    uint8_t azi_duty_cycle;
    float el_speed = 0;
    float azi_speed = 0;
    // Variables that hold data from shares
    float elv_sp = 0;
    float azi_sp = 0;
    float elv;
    float azi;
    // Redundant varables needed to update shares
    bool false_var = false;
    bool true_var = true;
 
    // Motor Control Task Loop
    for (;;){
        //pull data from shares
        elevation_sp >> elv_sp;
        azimuth_sp >> azi_sp;
        elevation >> elv;
        azimuth >> azi;
 
        // Calculate new speed based on desired and current position data
        el_speed = el_gain*(elv_sp - elv)/(1000-n*5);   //the times 5 is from the task timing, the 1000 comes from sup task timing
        azi_speed = azi_gain*(azi_sp - azi)/(1000-n*5); //the times 5 is from the task timing, the 1000 comes from sup task timing
        n++;
        if(n == 200){ // can't divide by zero haha
            n=0;
        }
 
        // Set direction of rotation of elevation motor
        if (el_speed < 0){
            digitalWrite(18,0);
            elev_direction << true_var;
        }
        else if (el_speed > 0){
            digitalWrite(18,1);
            elev_direction << false_var;
        }
        else{
            analogWrite(19,0);
        }
        // Set duty cycle of elevation motor
        if (abs(el_speed) > 255){
            el_duty_cycle = 255;
        }
        else{
            el_duty_cycle = abs(el_speed);
        }
        analogWrite(19, el_duty_cycle);    // update motor speed
 
        // Set direction of rotation of azimuth motor
        if (azi_speed < 0){
            digitalWrite(12,0);
            azi_direction << true_var;
        }
        else if (azi_speed > 0){
            digitalWrite(12,1);
            azi_direction << false_var;
        }
        else{
            analogWrite(14,0);
        }
        // Set duty cycle of azimuth motor
        if (abs(azi_speed) > 255){
            azi_duty_cycle = 255;
        }
        else{
            azi_duty_cycle = abs(azi_speed);
        }
        analogWrite(14, azi_duty_cycle);    // update motor speed
 
    // Task delay
    vTaskDelay(5);
    }
}
 
 
void task_new_coords (void* p_params)
{
    (void)p_params;                     // shuts up compiler warning
    
    bool start_flag = false;
    start << start_flag;
 
    int year = 2020;
    uint8_t month = 11;
    uint8_t day = 29;
    uint8_t h = 1;
    uint8_t m = 59;
    uint8_t s = 0;
 
    DateTime local_time(year, month, day, h, m, s);	//Creates a date time object
 
    float LA = 35.6368759;	//Paso Robles Coordinates
    float LO = -120.6545022;
    float HT = 732; //223m	//Feet or meters?
 
    Observer local_coords(LA,LO,HT); //Create observer object
 
    const char * l1 = "1 33591U 09005A   20334.64686490  .00000028  00000-0  40559-4 0  9997"; //NOAA 19 TLE
    const char * l2 = "2 33591  99.1943 344.2907 0013582 336.4368  23.6179 14.12441211608710";
 
    Satellite Sat(l1,l2);	//Create Satellite object with NOAA19 TLE
 
    Sat.predict(local_time);	//Compute Satellite Coordinate Predictions 
 
    float alt;
    float az;
    float range;
    float range_rate;
 
    Sat.topo(&local_coords, alt, az, range, range_rate);	//Generate look angles using observer object
 
    for (;;){
 
        start >> start_flag;	//Update start flag share
        
        if (start_flag){	//Keep track of time using task timing
            s++;
        }
 
        if (s == 60){
            s = 0;
            m++;
        }
        if (m == 60){		//No need to go over an hour
            m = 0;
            h++;
        }
        local_time.settime(year, month, day, h, m, s); //UTC update the time
        Sat.predict(local_time); //Compute satellite coordinates
        Sat.topo(&local_coords, alt, az, range, range_rate); //Compute look angles relative to observer
 
        az *= 1000;	//We use milidegrees
        alt *= 1000;
 
        elevation_coords << alt; //Stow in the share
        azimuth_coords << az;
 
        //Serial << "azimuth " << az << " altitude " << alt << endl;
        vTaskDelay(1000);
    }
}
 
 
/** @brief   Task which returns current UTC time from the internet.
 *  @details This task takes in your network SSID and password, establishes
 *           a connection with the internet, and then pulls UTC time. It stores
 *           current time data into a queue.
 *  @param   p_params Pointer to parameters passed to this function; we don't
 *           expect to be passed anything and so ignore this pointer
 *  @param   currenttime_queue This queue holds current time data in the form of
 *           year/month/day/hour/minute/second.
 */
void task_time (void* p_params)
{
    // Initialize Variables
    // manually enter current day/month/year
    uint8_t year = 20;    //only input last 2 digits (e.g. 20 = 2020)
    uint8_t month = 11;
    uint8_t day = 28;
    
    TOD RTC; //Instantiate Time of Day class TOD as RTC
 
    uint8_t lastminute=0;
    uint8_t lastsecond=0;
    char printbuffer[50];
    bool TODvalid=false;
 
    char ssid[] = "netID";                 // your network SSID (name)
    char password[] = "password";          // your network password
    
    //Internet access setup
    if(RTC.begin(ssid,password))TODvalid=true;   //.begin(ssid,password) requires SSID and password of your home access point
                                               //All the action is in this one function.  It connects to access point, sends
                                               //an NTP time query, and receives the time mark. Returns TRUE on success.
    lastminute=RTC.minute();
 
    // Time task loop
    for (;;){
        // TIME STUFF (TOD library)
        if(RTC.second()!=lastsecond && TODvalid){ //We want to perform this loop on the second, each second
            lastsecond=RTC.second();
            currenttime_queue.put(year);
            currenttime_queue.put(month);
            currenttime_queue.put(day);
            currenttime_queue.put(RTC.hour());
            currenttime_queue.put(RTC.minute());
            currenttime_queue.put(RTC.second());
        }
        vTaskDelay(1000);  //1 second delay, needed for proper task timing.
    }
}
 
 
/** @brief   Arduino setup function which runs once at program startup.
 *  @details This function sets up a serial port for communication and creates 
 *           the tasks which will be run.
 */
void setup () 
{
    // Start the serial port, wait a short time, then say hello. 
    Serial.begin (115200);
    delay (2000);
    Serial << endl << endl << "Begin SatTrack" << endl;
  
    // Create a task which reads accel/magnetometer data
    xTaskCreate (task_accelmag,
                 "AccelMag",                      // Name for printouts
                 2000,                            // Stack size
                 NULL,                            // Parameter(s) for task fn.
                 3,                               // Priority
                 NULL);                           // Task handle
    xTaskCreate (task_supervisor,
                 "Printy",                        // Name for printouts
                 4000,                            // Stack size
                 NULL,                            // Parameter(s) for task fn.
                 2,                              // Priority
                 NULL);                           // Task handle 
    xTaskCreate (task_position,
                 "position",                      // Name for printouts
                 2000,                            // Stack size
                 NULL,                            // Parameter(s) for task fn.
                 1,                               // Priority
                 NULL);                           // Task handle
    xTaskCreate (task_control,
                 "control",                       // Name for printouts
                 2000,                            // Stack size
                 NULL,                            // Parameter(s) for task fn.
                 1,                               // Priority
                 NULL);                           // Task handle
    xTaskCreate (task_new_coords,
                 "coordinates",                  // Name for printouts
                 15000,                          // Stack size
                 NULL,                           // Parameter(s) for task fn.
                 3,                             // Priority
                 NULL);        
    xTaskCreate (task_time,                      // keeps track of internet time
                 "timey",
                  4000,
                 NULL,
                 4,
                 NULL);
 
    // If using an STM32, we need to call the scheduler startup function now;
    // if using an ESP32, it has already been called for us
    #if (defined STM32L4xx || defined STM32F4xx)
        vTaskStartScheduler ();
    #endif
}
 
 
/** @brief   Arduino's low-priority loop function, which we don't use.
 *  @details A non-RTOS Arduino program runs all of its continuously running
 *           code in this function after @c setup() has finished. When using
 *           FreeRTOS, @c loop() implements a low priority task on most
 *           microcontrollers, and crashes on some others, so we'll not use it.
 */
void loop () {
}
 
 

