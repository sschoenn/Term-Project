/** @file main.cpp
 *    This file contains things that need to be documented
 * 
 *    This example uses the Adafruit FXOS8700 library at 
 *    @c https://github.com/adafruit/Adafruit_FXOS8700.git
 *    to communicate with an accelerometer on a custom shoe PCB made for the
 *    DOIT ESP32 DEVKIT V1. 
 * 
 *  @author  Simon Schoennauer
 *  @date    4 Nov 2020 Original file
 *  @date    21 Nov 2020
 *  This version of our main file has implemented the control loop, encoder reading, and FXO8700 driver
 *  tasks. Only the elevation axis is supported. 
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
#include "TimeLib.h"


/// Define Shares & Queues Below
Share<float> elevation ("ELEV");             // current elevation in milidegrees
Share<float> azimuth ("AZI");                // current azimuth in milidegrees
Share<bool> clear_elevation ("CLEAR");       // make this true to set current elevation to 0deg
Share<bool> elev_direction("DIR");           // either a 1 or 0 to represent direction
Share<float> elevation_sp("SP ELEV");        // elevation setpoint in milidegrees
Share<float> azimuth_sp("SP AZI");           // azimuth setpoint in milidegrees
Share<bool> start_condition("start");        // true if we're ready to begin the tracking sequence
Share<bool> home_condition("homing");        // true if the device is homing itself to first position
Share<bool> first_position("1st POSITION");  // true if we're trying to move to the first position of the sequence (part of the home state)
Queue<double> accelmag_queue (30);           // accel/mag queue of latest data
// Ephemeris Shares, contain the time data used to track satellite
Share<uint8_t> desired_day("DAY NUM");       // contains the day of the month (0-31) 
Share<uint8_t> desired_month("MONTH NUM");   // contains the month (1-12) 
Share<uint8_t> desired_year("YEAR NUM");     // contains the 4-digit year (2020, 2008, etc) 
Share<uint8_t> desired_hour("HOUR NUM");     // contains hour (0-23) 
Share<uint8_t> desired_minute("MINUTE NUM"); // contains minutes (0-59) 
Share<uint8_t> desired_second("SECOND NUM"); // contains seconds (0-59) 


/** @brief   Task which talks to the FXOS8700 accelerometer/magnetometer, and  
 *           outputs inclination angle to the serial monitor once per second.
 *  @param   p_params Pointer to parameters passed to this function; we don't
 *           expect to be passed anything and so ignore this pointer
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
    double pitch = 0;     // pitch, roll, and yaw (radians)
    double roll = 0;
    //double yaw = 0;
    double mag_heading = 0;

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
        sensors_event_t aevent, mevent;

        // Get a new sensor event, store ordered results in queue
        accelmag.getEvent(&aevent, &mevent);
        ax = aevent.acceleration.x;
        ay = aevent.acceleration.y;
        az = aevent.acceleration.z;
        mx = mevent.magnetic.x;
        my = mevent.magnetic.y;
        mz = mevent.magnetic.z;
        
        // Calculate pitch and roll (from accel)
        pitch = atan2 (ay ,( sqrt ((ax * ax) + (az * az))));
        roll = atan2(-ax ,( sqrt((ay* ay) + (az * az))));

        // Calculate yaw (from mag)
        //float Yh = (my * cos(roll)) - (mz * sin(roll));
        //float Xh = (mx * cos(pitch))+(my * sin(roll)*sin(pitch)) + (mz * cos(roll) * sin(pitch));
        //yaw =  atan2(Yh, Xh);
        
        // Calculate magnetometer heading (THIS ONLY WORKS WHEN MAG IS HORIZONTAL TO THE EARTH)
        //old code, this doesn't work.
        if( (az > 9.8) && (az < 9.9) ){                 // check if mag is horizontal by reading gravity TEST THIS CODE
            if (my < 0){
                mag_heading = 270 - (atan2(my,mx) * 180/PI) ;    // heading in degrees CW from North    
            }
            else if (my > 0){
                mag_heading = 90 - (atan2(my,mx) * 180/PI) ;    // heading in degrees CW from North
            }
            else if ((my == 0) && (mx < 0)){
                mag_heading = 180 ;
            }
            else if ((my == 0) && (mx > 0)){
                mag_heading = 0 ;
            }
        }
        else{
            mag_heading = 666;  //if accelerometer is not horizontal with ground
        }
        
        // Convert to deg
        roll = roll*57.295;
        pitch = pitch*57.295;
        
        accelmag_queue.put(ax);       // ax
        accelmag_queue.put(ay);       // ay
        accelmag_queue.put(az);       // az
        accelmag_queue.put(mx);       // mx
        accelmag_queue.put(my);       // my
        accelmag_queue.put(mz);       // mz
        accelmag_queue.put(roll);     // put roll in queue
        accelmag_queue.put(pitch);    // put pitch value in queue
        accelmag_queue.put(mag_heading); //put heading in queue

        delay(30);
    }
}

/** @brief   Task that interfaces with transmission sensors to implement an
 *           encoder, this task updates the current position read from the encoders
 *  @param   p_params Pointer to parameters passed to this function; we don't
 *           expect to be passed anything and so ignore this pointer
 *  @param   elevation This is a share that is updated to contain the current elevation 
 *           pointing position, in millidegrees, as read from the encoders. 
 *  @param   clear_elevation A bool, if set true, the task sets @param elevation to zero and sets 
 *           @param clear_elevation to false
 *  @param   elev_dir This contains the elevation motor direction so the encoder can 
 *           determine when to increment or decrement the total encoder count
 */
void task_position (void* p_params)
{
    (void)p_params;             // does nothing, clears a compiler warning
    
    // Initialize Variables
    bool el_prev_state = false;
    bool clear = false;
    bool false_var = false;     //you can't just pass true or false into a share it needs to be in a var
    bool true_var = true;
    int zero = 0;
    int az_tics = 0;
    int el_tics = 0;
    int int_dummy = 0;
    bool elev_dir;              //elevation direction
    float el_deg;

    bool az_prev_state = false;

    pinMode(26, INPUT);         //Setup for elevation ADC pin
    pinMode(27, INPUT);         //Setup for azimuth ADC pin

    // Encoder Position Task Loop
    for (;;)
    {   
        // Serial << analogRead(26) << " previous state " << el_prev_state << endl;      //debug code
        clear_elevation >> clear;              //grab clear_elevation bool (either true or false?)
        elev_direction >> elev_dir;            //set elevation motor direction
        
        // Runs if the elevation encoder position must be reset
        if (clear){
            el_tics = 0;                       //reset elevation encoder position to 0
            clear_elevation << false_var;      //resets clear_elevation flag for the next call
        }

        // Runs if the encoder reads high and if it has changed states since the last function call
        if ((analogRead(26) > 3500) && el_prev_state==false){
            
            el_prev_state = true;
            
            // If the motor is spinning forward, increment encoder count
            if (elev_dir){
                el_tics++;
                //Serial<<el_tics<< "if" <<elev_dir << endl;
            }
            
            // If the motor is spinning in reverse, decrement encoder count
            else if (!elev_dir){  
                el_tics--;
                //Serial<<el_tics<< "if else" <<elev_dir << endl;
            }

            else{
                Serial << "Encoder increment issue" << endl;
            }
        }

        // Determine if encoder reads low and if it has not changed posiiton since the last read
        else if((analogRead(26) < 3000) && (el_prev_state==true)){
            el_prev_state = false;
        }
        
        // Convert encoder ticks to position (deg)
        el_deg = (12.5*el_tics);
        elevation << el_deg;              // Place current position into elevation share
        
        // Task delay
        vTaskDelay(1);
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
 *           expect to be passed anything and so ignore this pointer
 */
void task_supervisor (void* p_params) //this task is actually the master task
{
    (void)p_params;                   //shuts up compiler warning

    /// Initialize Variables
    uint8_t state = 0;                //contains current state of supervisor task, begin in HOME state
    //these variables hold ephemeris tracking data pulled from shares
    uint8_t day;
    uint8_t month;
    uint8_t year;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;  
    // Redundant variables that have to exist in order to place stuff into shares
    bool false_var = false;
    bool true_var = true;
    int zero = 0;
    float float_zero;
    // These are local variables that hold values pulled from shares
    float el_sp; 
    float azi_sp;
    float elevation_val = 0;
    float azimuth_val = 0;
    bool home_bool;
    bool first_position_var;          // holds bool from share first_position
    // Variables for recording current time
    uint8_t year_now;
    uint8_t month_now;
    uint8_t day_now;
    uint8_t hour_now;
    uint8_t minute_now;
    uint8_t second_now;
    
    // Place initial values into queues
    elevation_sp << float_zero;       // sets initial elevation setpoint = 0
    azimuth_sp << float_zero;         // sets initial azimuth   setpoint = 0
    elevation << float_zero;          // sets initial elevation position to 0 
    azimuth << float_zero;            // sets initial azimuth   position to 0   
    clear_elevation << false_var;     // does not reset elevation count
    elev_direction << false_var;      // sets initial elevation motor direction
    home_condition << true_var;       // initialize homing sequence first
    first_position << false_var;      // we can't move to 1st sequence position until homing first

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
        home_condition >> home_bool;  // grab bool from share 'home_condition' and place into home_bool
        first_position >> first_position_var; // grab bool from share 'first_position', place into first_position_var

        //Serial << "State " << state << " | Current Elev: " << elevation_val << " | Elev sp: " << el_sp << " | Current Azi: " << azimuth_val << " | Azi sp: " << azi_sp << endl;

        /// 0-2: SUPERVISOR TASK STATE MACHINE
        // 0. HOME STATE (run at initial power-up; read magnetometer and move to start location of next sattrack sequence. then move to state 1)
        if(state == 0){
            //elevation_sp << (sp_lut[increment]);      //old code, idk what this does
            if(home_bool == false){                      // Check if the home task is still homing the tracker
                if(first_position_var == true){         // Azimuth encoders are homed to north, but we still need to check if we're in the starting position
                    //move to the initial track position
                    //turn on motor control and feed it the very first elev/azi setpoint.
                    // ?????????
                }
                else if(first_position_var == false){   //we've zeroed the azimuth encoders to north, AND we're in the initial position
                    state = 1;                          // move into state 1 if home_bool is set to false. This flag is set to false only in the homing task.
                }
            }
            else if(home_bool == true){
                // do nothing, we're still homing azimuth encoders
            }
        } //end of state 0
        
        // 1. WAIT STATE (tracker is aligned at start position, checks current time from internet, compares to initial start time of sequence. move into state 2 when true.)
        else if(state == 1){
            // Check and store internet time
            // year_now = year();
            // month_now = month();
            // day_now = day();
            // hour_now = hour();
            // minute_now = minute();
            // second_now = second();

            // time_t time = now(); // store the current time in time variable 'time'
            // hour(time);          // returns the hour for the given time 'time' (0-23)
            // minute(time);        // returns the minute for the given time 'time' (0-59)
            // second(time);        // returns the second for the given time 'time' (0-59)
            // day(time);           // the day for the given time 'time' (1-31_)
            // month(time);         // the month for the given time 'time' (1-12)
            // year(time);          // the year for the given time 'time' (2008, 2020, etc)

            // Compare internet time to desired sequence start time
            //TBD here, need to know format of deven's code. FOr now I'm pulling from these shares.
            desired_day >> day;
            desired_month >> month;
            desired_year >> year;
            desired_hour >> hour;
            desired_minute >> minute;
            desired_second >> second;

            // // Check if we're ready to begin the sequence by comparing current time to desired start time
            // if( (day(time) == day) && (month(time) == month) && (year(time) == year) ) {                    //first verify the correct date
            //     if( (hour(time) == hour) && (minute(time) == minute) && (second(time) >= second) ) {    //next verify the correct time, seconds are triggered if time is >= desired ???
            //         state = 2;    // start time reached, move into tracking state
            //         Serial << "Date(Month/Day/Year): " << month(time) << "/" << day(time) << "/" << year(time) << "|  Time(hr/min/sec): " << hour(time) << "/" << minute(time) << "/" << second(time) <<  endl;
            //         Serial << "Tracking State Begin. " << endl;
            //     }
            //     else{
            //         // do nothing, we aren't ready to change state yet (not right time)
            //     }
            // }
            // else{
            //     // do nothing, we aren't ready to change state yet (not right date)
            // }
        } //end of state 1

        // 2. TRACK STATE (tracker carries out tracking sequence. Sweeps across sky.)
        else if(state == 2){
            start_condition << true_var;       // start tracking sequence (activate motor control)
            // Changes increment (for calculating next position in coords task)
            increment++;
            if(increment == 180){
                start_condition << false_var;
            }
        }

        // Supervisor task time delay
        vTaskDelay(1000);
        
    }
}




/** @brief   Function which finds true north using magnetometer data, then resets
 *           azimuth encoders so that 0 = north. 
 *  @details TBD
 */
void task_home (void* p_params) {
    // Initialize Variables
    //vars to hold accel/mag data
    double ax;
    double ay;
    double az;
    double mx;
    double my;
    double mz;
    double roll;
    double pitch;
    double mag_heading = 666;            // some obscure default value that will never be an actual reading
    // Need these redundant things for shares
    bool false_var = false;
    bool true_var = true;
    bool home_var;
    float float_zero = 0;
    //temporary while we don't have azimuth calibration working
    uint8_t countdown = 60000;

    // Task loop
    for(;;){
        home_condition >> home_var;     //grab bool from share "home_condition"
        if(home_var){
            //grab current magnetometer data
            accelmag_queue.get(ax);          // ax
            accelmag_queue.get(ay);          // ay
            accelmag_queue.get(az);          // az
            accelmag_queue.get(mx);          // mx
            accelmag_queue.get(my);          // my
            accelmag_queue.get(mz);          // mz
            accelmag_queue.get(roll);        // roll
            accelmag_queue.get(pitch);       // pitch
            accelmag_queue.get(mag_heading); // current heading CW from north

            //THIS NEEDS TO BE TESTED
            // determine if elevation is 0 deg
            if ((az >= -0.1) && (az <= 0.1)){
                // arm is level, reset elevation encoder to 0, and then calibrate azimuth
                analogWrite(19,0);                          // stop elevation motor 
                elevation << float_zero;                    // reset elevation encoder 
                Serial << "Elevation is appx level, encoders have been reset. Initiating azimuth calibration:" << endl;
                // if(mag_heading == 0){                    // we *should* be facing north here for this to be true
                //     azimuth << float_zero;               //reset azimuth encoder to 0
                //     home_condition << false_var;         // move out of elev/azimuth homing!
                //     first_position << true_var;          // move into first position of tracking sequence
                // }
                // else if (mag_heading > 0){               // azimuth isn't zeroed, move azimuth until north is reached
                //     analogWrite(14,50);                  // rotate azimuth in negative direction until 0
                // }
                // else if (mag_heading < 0){               // azimuth isn't zeroed, rotate azimuth until north is reached
                //     analogWrite(14,-50);                 // rotate azimuth in positive direction until 0
                // }
                // else if (mag_heading == 666){
                //     Serial << "Error with homing, accel/mag does not read horizontal...fix the code probably" << endl;  //how did we get here if the prev if statement determined mag is level w/ground
                //     delay(10000);
                // } 
                
                // temporary azimuth calibration solution
                countdown --;
                Serial << "Please calibrate azimuth by hand. Point the dish northward." << endl;
                Serial << "Countdown until azimuth encoder reset: " << countdown << endl;
                delay(1000);
                if (countdown <= 0){
                    azimuth << float_zero;               // reset azimuth encoder to 0
                    home_condition << false_var;         // move out of elev/azimuth homing!
                    first_position << true_var;          // move into first position of tracking sequence 
                    Serial << "Azimuth encoders have been reset. Moving to first position of tracking sequence" << endl;
                    delay(5000)                   
                }

            }
            else if (az > 0.1){
                digitalWrite(18,0);
                analogWrite(19,50);                      //turn elev motor to make pitch 0
                Serial << "Turning Elevation Motor FWD to increase pitch. az = " << az << endl;
            }
            else if (az < -0.1){
                digitalWrite(18,1);
                analogWrite(19,50);                     //turn elev motor the other way to make pitch 0
                Serial << "Turning Elevation Motor BWD to decrease pitch. az = " << az << endl;
            }
            // Task delay (moved delay to within the if statement so that there's only a delay if are currently running this task)
            vTaskDelay(5);
        }
        
        //vTaskDelay(5);  //moved to above, see note. It should work like that....
    }
}




/** @brief   Function which generates the tracking sequence heading positions.
 *  @details This function generates the correct heading, producing timing, latitude, 
 *           & longitudinal position coordinates for the particular satellite to be
 *           tracked.
 */
void task_control (void* p_params) 
{
    // Initialize Variables
    bool start;              // contains value pulled from share "start"
    float elv_sp = 0;
    float azi_sp = 0;
    float elv;
    float azi;
    float gain = -50;        //IMPORTANT make sure its negative for stability, adjust this to change control loop
    bool false_var = false;
    bool true_var = true;
    uint8_t duty_cycle;
    int prev_elv = 0;       //some of these vars aren't used...
    int prev_elv_sp = 0;
    float speed = 0;
    int old_speed;
    uint8_t n = 0; //IMPORTANT this keeps track of the timing it's incremented each time the task is entered (ie. every 5ms)

    // Motor Control Task Loop
    for (;;){
        //check if we're supposed to be tracking at this moment in time
        start_condition >> start;                  // check flag from share "start_condition"
        if (start){
            //pull data from shares
            elevation_sp >> elv_sp;
            azimuth_sp >> azi_sp;
            elevation >> elv;
            azimuth >> azi;

            speed = gain*(elv_sp - elv)/(1000-n*5); //the times 5 is from the task timing, the 1000 comes from sup task timing
            n++;
            if(n == 200){ // can't divide by zero haha
                n=0;
            }

            //Serial << speed << endl;

            if (speed < 0){
                digitalWrite(18,1);
                elev_direction << true_var;
            }
            else if (speed > 0){
                digitalWrite(18,0);
                elev_direction << false_var;
            }
            else{
                analogWrite(19,0);
            }
            
            if (abs(speed) > 255){
                duty_cycle = 255;
            }
            else{
                duty_cycle = abs(speed);
            }
            analogWrite(19, duty_cycle);
        } // runs if start = true

        // Task delay
        vTaskDelay(5);
    }
}


/** @brief   Function which generates the tracking sequence heading positions.
 *  @details This function generates the correct heading, producing timing, latitude, 
 *           & longitudinal position coordinates for the particular satellite to be
 *           tracked.
 *           Inputs are GPS cords of sat tracker, GPS coods of the sat at some point
 *           at the current time (init conditions of longitude, latitude, and time),
 *           and velocty of sat (constant).
 */
void task_coords (void* p_params)
{
    (void)p_params;                     // shuts up compiler warning

    float lates = 35.49;                // 
    float lones = -120.67;              // 
    float latsat;                       // 
    float lonsat = 55.3;                // 
    float latsat0 = 44.41;              //
    long starttime = 69260;             // 
    long endtime = 69320;               //
    float vsat = 7.42;                  // 
    float earthradius = 6370;           // [km] radius of the earth
    float altitude = 863.25;            // [km?] altitude of SatTrack
    float gamma;                        //
    float el;                           //
    float alpha;                        //
    float pi = 3.141592653589793;       // contains value of pi
    float timestep = 1;                 // [s?] value of time step
    float elevation_calc;               // contains final calc of desired elevation
    float azimuth;                      // contains the desired azimuth position
    int currenttime;                    // contains current time
    bool start;                         // true if we are ready to start tracking

    // Initialize date variables to be placed inside shares
    uint8_t day = 23;                   // contains the day of the month (0-31) 
    uint8_t month = 11;                 // contains the month (1-12) 
    uint8_t year = 2020;                // contains the 4-digit year (2020, 2008, etc) 
    uint8_t hour = 15;                  // contains hour, military time (0-23)
    uint8_t minute = 30;                // contains minutes (0-59)
    uint8_t second = 30;                // contains seconds (0-59)

    // Task Loop
    for (;;){
        latsat = latsat0 + vsat/(earthradius + altitude)*currenttime;
        gamma = acos(sin(latsat)*sin(lates)+cos(latsat)*cos(lates)*cos(lonsat-lones));
        el = acos(sin(gamma)/sqrt(1+pow(earthradius/(earthradius+altitude),2)-2*(earthradius/(earthradius+altitude))*cos(gamma)));
        elevation_calc = 1000*el*180/pi;
        alpha = asin(sin(abs(lones-lonsat))*cos(latsat)/sin(gamma));
        if(latsat<=lates && lonsat<=lones)
        {
            azimuth = 180 + alpha*180/pi;
        }
        if(latsat<=lates && lonsat>lones)
        {
            azimuth = 180 - alpha*180/pi;
        }
        if(latsat>lates && lonsat<=lones)
        {
            azimuth = 360 - alpha*180/pi;
        }
        if(latsat>lates && lonsat>lones)
        {
            azimuth = alpha*180/pi;
        }
        start_condition >> start;
        if (start){
            currenttime += timestep;
        }
        currenttime += timestep;

        //Serial << "az: " << azimuth << "  " << "el: " << elevation_calc << endl;
        //place tracking data into shares for use with other tasks. If this doesn't change at every iteration maybe we can move this into the "init" part of the task
        elevation_sp << elevation_calc;      //place desired elevation setpoint into share
        azimuth_sp << azimuth;               //place desired azimuth setpoint into share
        desired_day << day;
        desired_month << month;
        desired_year << year;
        desired_hour << hour;                // military time (0-23)
        desired_minute << minute;
        desired_second << second;

        // task delay
        vTaskDelay(1000);
    }
}


/** @brief   Arduino setup function which runs once at program startup.
 *  @details This function sets up a serial port for communication and creates 
 *           the tasks which will be run.
 */
void setup () 
{
    // Start the serial port, wait a short time, then say hello. Use the
    // non-RTOS delay() function because the RTOS hasn't been started yet.
    // The "\033[2J" causes some serial terminals to clear their screens
    Serial.begin (115200);
    delay (2000);
    Serial << endl << endl << "Begin SatTrack main.cpp" << endl;

    // Specify which pin is to be used to send the PWM and make a pointer to it
    // The variable must be static so it exists when the task function runs

    // Create a task which reads accel/magnetometer data
    xTaskCreate (task_accelmag,
                 "AccelMag",                      // Name for printouts
                 2000,                            // Stack size
                 NULL,                            // Parameter(s) for task fn.
                 4,                               // Priority
                 NULL);                           // Task handle
    //Create a task which prints accelerometer data in a pretty way
    xTaskCreate (task_supervisor,
                 "Printy",                        // Name for printouts
                 4000,                             // Stack size
                 NULL,                            // Parameter(s) for task fn.
                 8,                               // Priority
                 NULL);                           // Task handle 
    xTaskCreate (task_position,
                 "position",                        // Name for printouts
                 2000,                             // Stack size
                 NULL,                            // Parameter(s) for task fn.
                 5,                               // Priority
                 NULL);                           // Task handle
    xTaskCreate (task_control,
                 "control",                        // Name for printouts
                 2000,                             // Stack size
                 NULL,                            // Parameter(s) for task fn.
                 6,                               // Priority
                 NULL);                           // Task handle
    xTaskCreate (task_coords,
                 "coordinates",                   // Name for printouts
                 10000,                           // Stack size
                 NULL,                            // Parameter(s) for task fn.
                 9,                               // Priority
                 NULL);        
    xTaskCreate (task_home,                       // homes the azimuth and elevation encoders
                 "homebound",
                 2000,
                 NULL,
                 6,
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

