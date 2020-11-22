/** @file main.cpp
 *    This file contains things that need to be documented
 * 
 *    This example uses the Adafruit FXOS8700 library at 
 *    @c https://github.com/adafruit/Adafruit_FXOS8700.git
 *    to communicate with an accelerometer on a custom shoe PCB made for the
 *    DOIT ESP32 DEVKIT V1. 
 * 
 *  @author  Simon Schoennauer
 * //make it go to 20 degrees from where it's at (add direction)
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


// Define shares and queues here

//Shares & Queue currently being used 
Share<float> elevation ("ELEV");          // current elevation in milidegrees
Share<bool> clear_elevation ("CLEAR");    // make this true to set current elevation to 0deg
Share<bool> elev_direction("DIR");        // either a 1 or 0 to represent direction
Share<float> elevation_sp("SP");          // elevation setpoint in milidegrees
Share<bool> start_condition("start");
Queue<double> accelmag_queue (30);      // S.F of 3 x [(9 data items per pull) + 3 address spots]

// Shares that do nothing
Share<int> azimuth ("AZI");    
Share<int> azimuth_sp("SP");





/** @brief   Task which talks to the FXOS8700 accelerometer/magnetometer, and  
 *           outputs inclination angle once per second.
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
    double yaw = 0;

    // Initialize the I2C bus and accelerometer driver
    Wire.begin ();
    Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);      // Create instance of the FXOS class


    /* Initialise the sensor */
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
        float Yh = (my * cos(roll)) - (mz * sin(roll));
        float Xh = (mx * cos(pitch))+(my * sin(roll)*sin(pitch)) + (mz * cos(roll) * sin(pitch));
        yaw =  atan2(Yh, Xh);
        
        // Convert to deg
        roll = roll*57.295;
        pitch = pitch*57.295;
        yaw = yaw*57.295;
        
        accelmag_queue.put(1000);     // event number address, accel/mag data address = 1000
        accelmag_queue.put(ax);       // ax
        accelmag_queue.put(ay);       // ay
        accelmag_queue.put(az);       // az
        accelmag_queue.put(mx);       // mx
        accelmag_queue.put(my);       // my
        accelmag_queue.put(mz);       // mz
        accelmag_queue.put(roll);     // put roll in queue
        accelmag_queue.put(pitch);    // put pitch value in queue
        accelmag_queue.put(yaw);      // put yaw value in queue

        delay(30);
    }
}

/** @brief   Task that interfaces with transmission sensors to implement an
 *           encoder, this task updates the current position read from the encoders
 *  @param   p_params Pointer to parameters passed to this function; we don't
 *           expect to be passed anything and so ignore this pointer
 *  @param   elevation this share is updated to contain the current position in milidegrees
 *           from the encoders. 
 *  @param   clear_elevation if set to true the task sets @param elevation to zero and sets 
 *           @param clear_elevation to false
 *  @param   elev_direction this function uses the motor direction to increment or decrement
 *           the total encoder count
 */
void task_position (void* p_params)
{
    (void)p_params;
    // Initialize Variables
    bool el_prev_state = false;
    bool clear = false;
    bool false_var = false;     //you can't just pass true or false into a share it needs to be in a var
    bool true_var = true;
    int zero = 0;
    int az_tics = 0;
    int el_tics = 0;
    int int_dummy = 0;
    bool elev_dir;  //set initial elevation direction = 1
    float el_deg;

    bool az_prev_state = false;

    pinMode(26, INPUT); //Setup for elevation ADC pin
    pinMode(27, INPUT); //Setup for azimuth ADC pin

    
    for (;;)
    {   
        //Serial << analogRead(26) << " previous state " << el_prev_state << endl;
        clear_elevation >> clear; //grab clear_elevation bool (either true or false?)
        elev_direction >> elev_dir;      //set elevation motor direction

             
        if (clear){
            el_tics = 0;
            clear_elevation << false_var;
        }
        if ((analogRead(26) > 3500) && el_prev_state==false){
            el_prev_state = true;
            if (elev_dir){  //encoder count fwd
                el_tics++;
                //Serial<<el_tics<< "if" <<elev_dir << endl;
            }
            else if (!elev_dir){  //encoder count reverse
                el_tics--;
                //Serial<<el_tics<< "if else" <<elev_dir << endl;
            }
            else{
                Serial << "Encoder increment issue" << endl;
            }
        }
        else if((analogRead(26) < 3000) && (el_prev_state==true)){
            el_prev_state = false;
        }

        el_deg = (12.5*el_tics);

        elevation << el_deg;

        vTaskDelay(1);
    }
}


/** @brief   Task which receives data from a queue and prints it nicely.
 *  @param   p_params Pointer to parameters passed to this function; we don't
 *           expect to be passed anything and so ignore this pointer
 */
void task_supervisor (void* p_params) //this task is actually the master task
{
    // Initialize Variables
    float elevation_val = 0;
    int azimuth_val = 0;
    double address;
    double ax;
    double ay;
    double az;
    double mx;
    double my;
    double mz;
    double pitch;
    double roll;
    double yaw;
    bool elev_dir;     //contains rotational direction of elevation motor
    uint8_t state = 0;
    float sp_lut [] = { 0, 0, 0, 100, 200, 300, 600, 900, 600, 400}; //IMPORTANT is the temp LUT that the system follows keep 1st few at 0  
    int increment = 0;
    int int_dummy;
    bool false_var = false;
    bool true_var = true;
    int zero = 0;
    float float_zero;
    float el_sp;
    uint32_t number = 0;


    azimuth_sp << zero;
    elevation << float_zero;
    clear_elevation << false_var;
    azimuth << zero;  // current azimuth queue  
    elev_direction << false_var;
    elevation_sp << float_zero;

    // Motor driver setup for PH/EN Mode
    pinMode(15, OUTPUT); //gpio 15 is the mode set (low for PH/EN high for PWM)
    digitalWrite(15,0);
    pinMode(12, OUTPUT); //gpio 12 is the PH for motor B
    digitalWrite(12, 1);
    pinMode(14, OUTPUT); //gpio 14 is the EN for motor B
    analogWrite(14, 0);
    pinMode(18, OUTPUT); //gpio 18 is the PH for motor A
    digitalWrite(18,1);
    pinMode(19, OUTPUT); //gpio 19 is the EN for motor A (Elevation)
    analogWrite(19, 100);

    for (;;){
        //Grab data from queue and check for event address in first spot
        accelmag_queue.get(address);
        if(address == 1000){               // grabbing accel/mag data
            accelmag_queue.get(ax);
            accelmag_queue.get(ay);
            accelmag_queue.get(az);
            accelmag_queue.get(mx);
            accelmag_queue.get(my);
            accelmag_queue.get(mz);
            accelmag_queue.get(pitch);
            accelmag_queue.get(roll);
            accelmag_queue.get(yaw);
        }
        else{
            Serial.println("ERROR: Bad event address in queue, reset program.");
            vTaskDelay(50000);
        }
        
        //Display the accel results (acceleration units in m/s^2) 
        //Serial.println("--------------------------------------------------------");
        //Serial << "Accel   X: " << ax << "         Y: " << ay << "        " << " Z: " << az << "  m/s^2" << endl;

        //Position test code
        //Grabs a desired position, checks current position, and then moves to desired position

       
        elevation >> elevation_val;
        elevation_sp >> el_sp;

        Serial << "State " << state << " | Current Position: " << elevation_val << " | sp: " << el_sp << endl;

        // right now only state 0 is used, all we do is increment the setpoint once per second

        if(state == 0){
            //elevation_sp << (sp_lut[increment]);
            if (elevation_val == el_sp){
                start_condition << true_var;
                state = 1;
                }
        }
        else if(state ==1){
            increment++;
            if(increment == 180){
                start_condition << false_var;
            }
            
        
        }


        vTaskDelay(1000);
    }
}

void task_control (void* p_params) 
{

    float elv_sp = 0;
    int azi_sp = 0;
    float elv;
    int azi;
    float gain = -50;        //IMPORTANT make sure its negative for stability, adjust this to change control loop
    bool false_var = false;
    bool true_var = true;
    uint8_t duty_cycle;
    int prev_elv = 0;       //some of these vars aren't used...
    int prev_elv_sp = 0;
    float speed = 0;
    int old_speed;
    uint8_t n = 0; //IMPORTANT this keeps track of the timing it's incremented each time the task is entered (ie. every 5ms)



    for (;;){

        elevation_sp >> elv_sp;
        azimuth_sp >> azi_sp;
        elevation >> elv;
        azimuth >> azi;
        //Serial<< speed <<  endl;

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

        vTaskDelay(5);
    }
}

void task_coords (void* p_params)
{
    (void)p_params;

    float lates = 35.49;
    float lones = -120.67;
    float latsat;
    float lonsat = 55.3;
    float latsat0 = 44.41;
    long starttime = 69260;
    long endtime = 69320;
    float vsat = 7.42;
    float earthradius = 6370;
    float altitude = 863.25;
    float gamma;
    float el;
    float alpha;
    float pi = 3.141592653589793;
    float timestep = 1;
    float elevation_calc;
    float azimuth;
    int currenttime;
    bool start;

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

        Serial << "az: " << azimuth << "  " << "el: " << elevation_calc << endl;
        elevation_sp << elevation_calc;
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
                 "coordinates",                        // Name for printouts
                 10000,                             // Stack size
                 NULL,                            // Parameter(s) for task fn.
                 9,                               // Priority
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
void loop () 
{
}

