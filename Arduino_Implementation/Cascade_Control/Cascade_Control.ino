///////////////////////////////// Include all the required libraries ////////////////////////////////
#include <Wire.h>
#include <PID_v1.h>
#include <My_Motors.h>
#include <Encoder.h>
#include <TimerOne.h>

///////////////////////////////// MPU-6050 parameters //////////////////////////////////////////////

long accelX, accelY, accelZ, gyroX, gyroY, gyroZ; // Parameters to record the raw accelerometer / gyro data
float omega_x_gyro, omega_x_calculated;// Parameter to store raw gyro data to converted data into [deg/s]
float mpu_calib[6] = {-2043, 108, 1293,  48, -12, 19}; // Array storing MPU Offset values [accX, accY,accZ,gyroX,gyroY,gyroZ]
float pitch, Theta_prev, Theta_now ; // Parameters for computing angle data from Accelerometer and gyro 
float Theta_correction = 2.5; // Angle to be added to the Theta_now for correcting the upright robot angle to account for total caliberation [deg]                 
double dt_gyro; // Variable to store time difference values for gyro angle calculations 
uint32_t t_gyro_prev, t_gyro_now; // timer for gyro unit
float alpha = 0.98; // Complimentary filter control parameter
float rad2deg = 57.3, deg2rad = 0.01745; // Angle conversion factors

////////////////////////////// MOTOR CONTROL PARAMATERS ////////////////////////////////////////////

short Rmot1 = 7; short Rmot2 = 8; // Pins for Right motor FW/BCK
short Rmot3 = 9; // Pin for Right motor PWM
short Lmot1 = 4; short Lmot2 = 5; // Pins for Left motor FW/BCK
short Lmot3 = 6; // Pins for Right motor PWM
short R_enc_pin1 = 2; short R_enc_pin2 = 3; // right motor encoder pins
short L_enc_pin1 = 18;  short L_enc_pin2 = 19; // left motor encoder pins 

float rpm_limit = 1.0; // RPM below this is considered 0, this value is in RPM and NOT [rad/s]
float avg_pt = 10.0;  // Number of points used for exponentially averaging the RPM signal
short PPR = 330; // Number of pulses per revolution of the encoder (for a gearbox 1:30, this value is 330 seen from the website)
float Final_Rpm_r, Final_Rpm_l; // Motor final averaged out RPM, units can be selected while calling get_RPM function
My_Motors Rmot(&Final_Rpm_r, rpm_limit, avg_pt, PPR); // Right motor object for calculating rotational velocities from encoder data
My_Motors Lmot(&Final_Rpm_l, rpm_limit, avg_pt, PPR); // Left motor object for calculating rotational velocities from encoder data
Encoder myEnc_r(R_enc_pin2, R_enc_pin1); // Make encoder objects to calculate motor velocties
Encoder myEnc_l(L_enc_pin1, L_enc_pin2); // Make encoder objects to calculate motor velocties
double Output_lmot, Output_rmot; // Variables for storing PWM outputs seperately for left and right motors

///////////////////////////////// Balancing PID parameters ///////////////////////////////////////////////////

double Input_bal, Output_bal, Setpoint_bal, error_bal; // Input output and setpoint variables defined
double Out_min_bal = -255, Out_max_bal = 255; // PID Output limits, this is the output PWM value
double Kp_bal = 46.0, Ki_bal = 0.0, Kd_bal = 0.80; // Initializing the Proportional, integral and derivative gain constants
double Output_lower_bal = 30.0; // PWM Limit at which the motors actually start to move
PID bal_PID(&Input_bal, &Output_bal, &Setpoint_bal, Kp_bal, Ki_bal, Kd_bal, P_ON_E, DIRECT); // PID Controller for balancing

///////////////////////////////// TRANSLATION PID parameters ///////////////////////////////////////////////////

double Input_trans, Output_trans, Setpoint_trans; // Input output and setpoint variables defined
double Out_min_trans = -25, Out_max_trans = 25; // PID Output limits, this output is in degrees
double Kp_trans = 20.0, Ki_trans = 0.0, Kd_trans = 0.00; // Initializing the Proportional, integral and derivative gain constants
PID trans_PID(&Input_trans, &Output_trans, &Setpoint_trans, Kp_trans, Ki_trans, Kd_trans, P_ON_E, DIRECT); // PID Controller for translating

///////////////////////////////// ROBOT PHYSICAL PROPERTIES ////////////////////////////////////////////

/*Both motors have different characteristics, the right motor spins faster when going in forward direction but slower in backward direction.
Therefore, we need correction factors to drive straight*/
float motor_corr_fac_fwd = 0.95, motor_corr_fac_bck = 0.97;// factor to correct for the difference b/w the motor characteristics
float r_whl = 0.5 * 0.130; // Wheel radius [m]
short fall_angle = 45; // Angles at which the motors must stop rotating [deg]
float full_speed = 350.0 * (2.0*3.14 / 60.0) * r_whl; // Full linear speed of the robot @ motor rated RPM [here 350 RPM @ 12 V]
float frac_full_speed = 0.40; // Fraction of full speed allowed 
float V_max = frac_full_speed * full_speed; // Maximum speed allowed [m/s]
float V_max_fwd = 0.01, V_min_bck = -0.01; // Variables used for storing minimum and maximum values of translation speed for applying brakes

/*Ratio when the mode_prev must be set to "balance". This is the ratio between 
instantaneous translation velocity and max / min (fwd / back) velocities. This ratio decides
when the mode must be set to balance. Lower values mean we must wait for longer time for 
the speeds to decrease. Higher value mean we donot wait for longer time. The higher the value, 
the smoother the stopping of the robot*/

float speed_ratio_mode_change = 0.40; 
float speed_steps = 0.08; // Steps in which speed should be incremented in order to get to the full speed
float brake_steps = 0.04; // Steps in which speed should be decremented in order to apply brakes, the smaller the value, the longer the duration of brake application
String mode_prev = "balance", mode_now = "balance"; // To set balancing, moving fwd and moving backward modes on the robot
String rotation_direction = ""; // To set rotation direction
double Rot_Max = 150.0, rot_steps = 20.0; // Max rotation speed and the steps in which speed is decreased to "0"
double Rot_Speed = Rot_Max;

////////////// All boolean switching PARAMETERS/////////////////////////

bool switch_bal_controller = false; // Variable to switch to a softer controller when the robot is moving to make stopping / starting behavior smooth
bool switch_trans_controller = false; // Variable to switch trans controller behavior once moving commands are issued
bool lock = true; // Variable to prevent accidental changing of parameters by bluetooth app
bool rotating = false;  // To set rotation mode on the robot
bool start_again; // Boolean to reset Rot_Speed = Rot_max once Rot_Speed decreases from Rot_Max to 0
bool led_state = 0; // Parameter to turn LED from ON / OFF

////////////// LED BLINKING / Loop time PARAMETERS/////////////////////////

int pin = 13; // PIN where LED is attached
double t_loop_prev, t_loop_now, dt_loop, t_mode_switch; // Time parameters to log times for main control loop
double t_loop = 20.0; // Overall loop time [millis]

void setup() {

    Serial.begin(115200);  
    pinMode(pin, OUTPUT);
    
    /////////////////////////////// Motor initialization ///////////////////////////////////////////
  
    pinMode(Rmot1,OUTPUT);pinMode(Rmot2,OUTPUT);pinMode(Rmot3,OUTPUT); // Declaring right motor pins as output  
    pinMode(Lmot1,OUTPUT);pinMode(Lmot2,OUTPUT);pinMode(Lmot3,OUTPUT); // Declaring left motor pins as output
    
    ////////////////////////// BALANCING PID  initialization ////////////////////////////////////////////////////////
        
//  bal_PID.SetSampleTime(t_loop); // Set Loop time for PID [milliseconds]    
    bal_PID.SetMode(AUTOMATIC); // Set PID mode to Automatic
    bal_PID.SetOutputLimits(Out_min_bal, Out_max_bal); // Set upper and lower limits for the maximum output limits for PID loop
    
    ////////////////////////// TRANSLATION PID initialization ////////////////////////////////////////////////////////        

    trans_PID.SetSampleTime(t_loop); // Set Loop time for PID [milliseconds]
    trans_PID.SetMode(AUTOMATIC); // Set PID mode to Automatic        
    trans_PID.SetOutputLimits(Out_min_trans, Out_max_trans); // Set upper and lower limits for the maximum output limits for PID loop
 
    ////////////////////////// MPU initialization ///////////////////////////////////////////////////
    
    Wire.begin(); // Start wire library    
    setupMPU(); // Initializing MPU6050  
    get_MPU_data(); // Get initial angles of the MPU  
    pitch = (atan2(accelY-mpu_calib[1], accelZ+mpu_calib[2]))*rad2deg; //  Calculate initial pitch angle [deg] and caliberated for error in accY and accZ
    Theta_prev = pitch; // set the total starting angle to this pitch 
    t_gyro_prev = millis(); // Log time for gyro calculations [ms]  
    t_loop_prev = millis(); // Log time for overall control loop [ms]
   Timer1.initialize(150000);
   Timer1.attachInterrupt(Blink_Led);
    delay(50);  
}

void loop() {

  t_loop_now = millis();
  dt_loop = t_loop_now - t_loop_prev; // Calculate time change since last loop [millis]
  /*Begin the main computing loop, enter the loop only if the minimum loop time is elapsed*/
  if (dt_loop>=t_loop){  
  
    read_BT(); // Read data from the bluetooth
    Get_Tilt_Angle(); // Update the angle readings to get updated omega_x_calculated, Theta_now
    Lmot.getRPM(myEnc_l.read(), "rad/s"); // Get current encoder counts & compute left motor rotational velocity in [rad/s] 
    Rmot.getRPM(myEnc_r.read(), "rad/s"); // Get current encoder counts & compute right motor rotational velocity in [rad/s]
    float V_trans = 0.5 * (Final_Rpm_r + Final_Rpm_l) * r_whl;// Calculate the total Robot linear translation velocity [m/s]
    
    ////////////////////////////////////////// COMPUTE BALANCING PID OUTPUT/ //////////////////////////////////////////////////
    
    if (mode_now == "go fwd"){ 
      // If we change mode to forward now, switch controllers and start increasing the setpoint slowly to avoid jerky behaviour
      switch_bal_controller = true;
      switch_trans_controller = true;
      rotating = false;
      Setpoint_trans = Setpoint_trans + speed_steps;
      mode_prev = "go fwd";
      if (Setpoint_trans > V_max){Setpoint_trans = V_max;}
      if (V_max_fwd < V_trans) {V_max_fwd = V_trans;} // If the current velocity is more than V_max_fwd, then this is the new max velocity
    }
    else if (mode_now == "go bck"){
      // If we change mode to back now, switch controllers and start decreasing the setpoint slowly to avoid jerky behaviour
      switch_bal_controller = true;
      switch_trans_controller = true;
      rotating = false;
      Setpoint_trans = Setpoint_trans - speed_steps;
      mode_prev = "go bck";
      if (Setpoint_trans < -V_max){Setpoint_trans = -V_max;}
      if (V_min_bck > V_trans) {V_min_bck = V_trans;} // If the current velocity is less than V_min_bck, then this is the new min velocity
      }
    else if (mode_now == "balance"){ // If we change mode to balance now, we need to apply brakes      
      if (mode_prev == "go fwd"){
        Setpoint_trans = Setpoint_trans - brake_steps; // If going in fwd direction, apply brakes by setting the trans setpoint to opposite value
        switch_trans_controller = false; // Reset trans_PID to original value to get increased braking
        /*if the ratio of current velocity and the maximum velocity measured since the robot started moving forward
        is less than speed_ratio_mode_change, change mode_prev to balance*/
        if (V_trans/V_max_fwd<=speed_ratio_mode_change){mode_prev = "balance"; t_mode_switch = millis();} // Set mode_prev to balance so that the robot goes to balancing mode totally
        }
      else if (mode_prev == "go bck"){
        Setpoint_trans = Setpoint_trans + brake_steps; // If going in bck direction, apply brakes by setting the trans setpoint to opposite value
        switch_trans_controller = false; // Reset trans_PID to original value to get increased braking
        /*if the ratio of current velocity and the minimum velocity measured since the robot started moving backward
        is less than speed_ratio_mode_change, change mode_prev to balance*/
        if (V_trans/V_min_bck<=speed_ratio_mode_change){mode_prev = "balance"; t_mode_switch = millis();} // Set mode_prev to balance so that the robot goes to balancing mode totally
        }
      else if (mode_prev == "balance"){
        Setpoint_trans = 0.0;
        rotating = false;
        V_min_bck = -0.01; // Re-initialise the variables
        V_max_fwd = 0.01;
        /*Switch to a stiffer balancing controller 2 seconds after stopping*/
        double dt_mode_switch = millis() - t_mode_switch;
        if (dt_mode_switch > 2000){switch_bal_controller = false;}
      }
    }
    
    ////////////////////////////////////////// COMPUTE 1st loop/ //////////////////////////////////////////////////

    Input_trans = V_trans; // Measured value / Input value [m/s]
    if (switch_trans_controller = false){trans_PID.SetTunings(Kp_trans, Ki_trans, Kd_trans);} // Keep original value to increase braking
    else if (switch_trans_controller = true){trans_PID.SetTunings(0.3 * Kp_trans, Ki_trans, Kd_trans);} // Change trans_PID to 0.3 * original value to reduce bending angle
    trans_PID.Compute(); // Compute Output_trans of the 1st loop [degrees]
    
    ////////////////////////////////////////// COMPUTE 2nd loop/ //////////////////////////////////////////////////
 
    Setpoint_bal = Output_trans; // Set the output [angle in deg] of the translation PID as Setpoint to the balancing PID loop
    Input_bal = Theta_now + Theta_correction; // Set Theta_now as the input / current value to the PID algorithm (The correction is added to correct for the error in MPU calculated angle)             
    error_bal = Setpoint_bal - Input_bal; // To decide actuator / motor rotation direction 
    /* If balancing use a harder / stronger controller, but while moving use a softer controller for smooth stopping / starting*/
    if (switch_bal_controller == false){bal_PID.Compute_For_MPU(Kp_bal, Ki_bal, Kd_bal, omega_x_gyro);} // Compute motor PWM using harder balancing PID
    else if (switch_bal_controller == true){bal_PID.Compute_For_MPU(0.5 * Kp_bal, Ki_bal, Kd_bal, omega_x_gyro);} // Compute motor PWM using softer balancing PID     
     
    /*Scale the output from 0-255 to 30-255 and then negate it if the initial output computed by PID loop was negetive*/
    double Output_bal_scaled = map(abs(Output_bal), 0, Out_max_bal, Output_lower_bal, Out_max_bal);
    
    if (Output_bal<0){Output_bal_scaled*=-1;} // Negate the Output_bal_scaled if the output was negative 

    Output_rmot = Output_bal_scaled; Output_lmot = Output_bal_scaled; // Seperate the output computed for both motors 
    
    /*It is to be noted here that the Output_bal (& hence Output_lmot, Output_rmot) is -ve if the robot moves in 
    fwd direction and +ve if the robot moves in backward direction*/
    
    if (rotating == true){
      if (start_again == true){Rot_Speed = Rot_Max; start_again = false;}          	
    	Rot_Speed-=rot_steps;
    	if (Rot_Speed<0){Rot_Speed = 0; rotating == true;}    	
    	if (rotation_direction == "clockwise"){
    		Output_lmot -=  Rot_Speed;
    		Output_rmot +=  Rot_Speed;
    	}
    	else if (rotation_direction == "anti_clockwise"){
    		Output_lmot +=   Rot_Speed;
    		Output_rmot -=  Rot_Speed;
    	}
    }
    
    else if (rotating == false){ // Account for motor speed differences, if not rotating
      if (Final_Rpm_l>0 && Final_Rpm_r>0){Output_rmot*=motor_corr_fac_fwd;}
      else if (Final_Rpm_l<0 && Final_Rpm_r<0){Output_lmot*=motor_corr_fac_bck;}      

      // To prevent continuous jerky behaviour, the robot starts balancing outside +- 0.2 deg      
      if (abs(error_bal)<0.2 && mode_now == "balance"){Output_rmot = 0.0; Output_lmot = 0.0;} 
    }    
    
    ///////////////////////////////////////// If robot has fallen then stop the motors /////////////////////////////////////////////

    if (abs(error_bal)>=fall_angle){
       Output_rmot = 0.0;
       Output_lmot = 0.0; // Stop the robot
       rotating = false;
       Rot_Speed = 0.0;
       switch_bal_controller = false;
       switch_trans_controller = false;
       mode_now = "balance"; // Change mode to balance
       mode_prev = "balance"; // Change mode to balance
       }
    ///////////////////////////////////////// Apply motor controls /////////////////////////////////////////////
   
    mot_cont(); // Apply the calculated output to control the motor
    t_loop_prev = t_loop_now; // Set prev loop time equal to current loop time for calculating dt for next loop
  }  
}

///////////////////////// Function for initializing / getting MPU Data ////////////////////////////////////////////////////////

void Get_Tilt_Angle(){  
  t_gyro_now = millis(); // Log time now [millis]
  get_MPU_data(); // Update / Get Raw data acelX acelY acelZ giroX giroY giroZ  
  dt_gyro = (t_gyro_now - t_gyro_prev) / 1000.0; // calculate time difference since last loop for gyro angle calculations [seconds]
  omega_x_gyro = (gyroX-mpu_calib[3]) / 131.0; // Compute Angular velocity from raw gyroXreading [deg/s] and also correct for the error;    
  /*Since we will only need the ratios of accelerometer readings to calculate accelerometer angles, 
  we do not need to convert raw data to actual data */  
  pitch = (atan2(accelY-mpu_calib[1], accelZ+mpu_calib[2]))*rad2deg; // Angle calculated by accelerometer readings about X axis in [deg]  and caliberated for error in accY and accZ
  Theta_now = alpha * (Theta_prev + (omega_x_gyro * dt_gyro)) + (1-alpha) * pitch; // Calculate the total angle using a Complimentary filter
  omega_x_calculated = (Theta_now - Theta_prev) / dt_gyro; // Calculated omega_x from complimentary filter
  Theta_prev = Theta_now;
  t_gyro_prev = t_gyro_now;  
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Accelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

///////////////////////// Function for getting MPU data ///////////////////////////////////////////////////////

void get_MPU_data(){

  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
//  while(Wire.available() < 6); // This line is commented out because Wire.requestFrom() is blocking, so there is no point in waiting again
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
//  while(Wire.available() < 6); // This line is commented out because Wire.requestFrom() is blocking, so there is no point in waiting again
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ

}

///////////////////////// Function for motor control ////////////////////////////////////////////////////////

void Blink_Led(){
  if (led_state ==0){digitalWrite(pin, 1);led_state = 1;}
  else if(led_state == 1){digitalWrite(pin, 0);led_state = 0;}
}

/*
Now we have 4 possibilities
1. Both motor outputs are negetive => move motors forward
2. Both motor outputs are positive => move motors backward
3. Left motor negative and right motor positive => Clockwise rotation
4. Left motor positive and right motor negative => Anti - Clockwise rotation
*/

void mot_cont(){

  if (Output_lmot<=0 & Output_rmot<=0){ // => move motors forward
    digitalWrite(Lmot1, LOW);
    digitalWrite(Lmot2, HIGH);
    digitalWrite(Rmot1, LOW);
    digitalWrite(Rmot2, HIGH);
    analogWrite(Lmot3,abs(Output_lmot));    
    analogWrite(Rmot3,abs(Output_rmot));
  }
  else if (Output_lmot>0 & Output_rmot>0){ // => move motors backward
    digitalWrite(Lmot1, HIGH);
    digitalWrite(Lmot2, LOW);
    digitalWrite(Rmot1, HIGH);
    digitalWrite(Rmot2, LOW);
    analogWrite(Lmot3,abs(Output_lmot));
    analogWrite(Rmot3,abs(Output_rmot));
  }

  else if (Output_lmot<=0 & Output_rmot>=0){ // => Move clockwise
    digitalWrite(Lmot1, LOW);
    digitalWrite(Lmot2, HIGH);
    digitalWrite(Rmot1, HIGH);
    digitalWrite(Rmot2, LOW);
    analogWrite(Lmot3,abs(Output_lmot));    
    analogWrite(Rmot3,abs(Output_rmot));
  }
  else if (Output_lmot>=0 & Output_rmot<=0){ // => Move anti - clockwise
    digitalWrite(Lmot1, HIGH);
    digitalWrite(Lmot2, LOW);
    digitalWrite(Rmot1, LOW);
    digitalWrite(Rmot2, HIGH);
    analogWrite(Lmot3,abs(Output_lmot));
    analogWrite(Rmot3,abs(Output_rmot));
  }
}

///////////////////////////////// READ BLUETOOTH ////////////////////////

void read_BT(){
  if (Serial.available()>0){
    char c = Serial.read();
    if (c == 'm'){lock = false;Serial.print("Unlocked");}
    if (c == 'n'){lock = true;Serial.print("Locked");}
    else if (c =='0' & lock == false){mode_now = "balance";Serial.print(mode_now);} 
    else if (c =='1' & lock == false){mode_now = "go fwd";Serial.print(mode_now);} 
    else if (c =='2' & lock == false){mode_now = "go bck";Serial.print(mode_now);} 
    else if (c =='3' & lock == false){Kp_bal+=1.0;Serial.print("Kp_bal = "+String(Kp_bal));}
    else if (c =='4' & lock == false){Kp_bal-= 1.0;Serial.print("Kp_bal = "+String(Kp_bal));}
    else if (c =='5' & lock == false){Kd_bal+=0.05;Serial.print("Kd_bal = "+String(Kd_bal));}
    else if (c =='6' & lock == false){Kd_bal-=0.05;Serial.print("Kd_bal = "+String(Kd_bal));}
    else if (c =='7' & lock == false){Kp_trans+=0.5;trans_PID.SetTunings(Kp_trans, Ki_trans, Kd_trans);Serial.print("Kp_trans = "+String(Kp_trans));} 
    else if (c =='8' & lock == false){Kp_trans-=0.5;trans_PID.SetTunings(Kp_trans, Ki_trans, Kd_trans);Serial.print("Kp_trans = "+String(Kp_trans));} 
    else if (c =='9' & lock == false){motor_corr_fac_fwd+=0.01;Serial.print("MoFac = "+String(motor_corr_fac_fwd));} 
    else if (c =='a' & lock == false){motor_corr_fac_fwd-=0.01;Serial.print("MoFac = "+String(motor_corr_fac_fwd));} 
    else if (c =='b' & lock == false & mode_now == "balance"){
      rotating = true;
      start_again = true;
      rotation_direction = "anti_clockwise";
      Serial.print("Rot anticlk");
    }
    else if (c =='c' & lock == false & mode_now == "balance"){
      rotating = true;
      start_again = true;
      rotation_direction = "clockwise";
      Serial.print("Rot clk");
    }
    else if (c =='d' & lock == false){Rot_Max+=2.0;Serial.print("Rot_Max = "+String(Rot_Max));} 
    else if (c =='e' & lock == false){Rot_Max-=2.0;Serial.print("Rot_Max = "+String(Rot_Max));}
//    else if (c =='f' & lock == false){Kp_path_cor+=0.1;Serial.print("Kp_PaCo = "+String(Kp_path_cor));} 
//    else if (c =='g' & lock == false){Kp_path_cor-=0.1;Serial.print("Kp_PaCo = "+String(Kp_path_cor));} 
    else if (c =='h' & lock == false){frac_full_speed+=0.05;V_max = frac_full_speed * full_speed; Serial.print("FrFs = "+String(frac_full_speed));} 
    else if (c =='i' & lock == false){frac_full_speed-=0.05;V_max = frac_full_speed * full_speed; Serial.print("FrFs = "+String(frac_full_speed));} 
    else if (c =='j' & lock == false){Theta_correction+=0.1;Serial.print("Theta_Cor = "+String(Theta_correction));} 
    else if (c =='k' & lock == false){Theta_correction-=0.1;Serial.print("Theta_Cor = "+String(Theta_correction));} 
    else if (c =='l' & lock == false){
      //Reset all parameters to default values
      Serial.print("Reset"); 
      mode_now = "balance";mode_prev = "balance"; rotating = false;
      switch_bal_controller = false;
      switch_trans_controller = false;
      Kp_bal = 46.0; Kd_bal = 0.8;
      Kp_trans = 20.0;
      trans_PID.SetTunings(Kp_trans, Ki_trans, Kd_trans);
      motor_corr_fac_fwd = 0.95;motor_corr_fac_bck = 0.97;
      speed_ratio_mode_change = 0.40;
      speed_steps = 0.08;brake_steps = 0.04;
      frac_full_speed = 0.40;
      Theta_correction = 2.5;
      Rot_Max = 150;

    } 
   
    }  
}

