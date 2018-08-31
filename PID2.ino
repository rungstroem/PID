//Includes
//**************************************************
#include <math.h>
#include <stdio.h>
//**************************************************

//Pin naming
//**************************************************
int LM35_temp_sensor = A1; //Analog input from LM35 sensor
//int PT100_temp_sensor = A0; //Analog input from PT100 sensor 
int SSR_heater      = 11;  //Digital output for SSR
//int heater = 13;
//**************************************************

//Variables
//**************************************************
int temp_set    = 0;  //Desired temperature
int temp        = 0;  //Actual temperature
int PID_signal  = 0;  //The output from the PID controller
double out_min;
double out_max;
//**************************************************

//Variables for the PID
//**************************************************
unsigned long last_time;
int sample_time = 1000;
unsigned long NOW;
double input;
double output;
double set_point;
double err_sum;
double last_err;
double time_change;
double error;
double d_err;
double i_term;

//controller gains
double kp;
double ki;
double kd;
//*************************************************
//du er simpelthen bare så sød og dejlig!
//For serial sommunication
//*************************************************
int serial_input = 0;
char serial_byte_in;
char temporary = 'f';
//*************************************************

//Setup loop
//*************************************************
void setup() {
//Serial setup
Serial.begin(9600);

//Pin setup
pinMode(LM35_temp_sensor, INPUT); //Configure A1 to analog input for temp sensor
//pinMode(PT100_temp_sensor, INPUT); //Configure A0 to analog input for temp sensor
pinMode(SSR_heater, OUTPUT); //Configure D11 to PWM output
//pinMode(heater, OUTPUT); //Configure D13 to digital output

//PID gains
tuning(2, 2, 2);    //PID gains for regulator function

//Max - Min output parameters
output_limit(100, 0); //First max output, second min output
}
//*************************************************

//Super loop
//*************************************************
void loop() {
  serial_in();    //Seriel communication (input numbers defines temperature setpoint, input 't' returns actual temperature
  
  temp_read();    //PT100 temperature probe reading and converstion to celsius
                  //Hej min elskede - jeg synes du er rigtig dejlig
  regulator();

  heat();
}
//*************************************************

void serial_in(){
  if(Serial.available() > 0){
    serial_input = 0;
    
    while(1){
      serial_byte_in = Serial.read();
      
      if(serial_byte_in == '\n'){
        break;
      }
      if(serial_byte_in == -1){
        continue;
      }
      if(serial_byte_in == 't'){ //Input 't', actual temperature is written to the serial port
        serial_out('T', temp);
        break;  
      }
      if(serial_byte_in == 'p'){  //Input 'p', the state of the PID_signal is written to the serial port
        serial_out('P', PID_signal);
        break;
      }
      if(serial_byte_in == 'h'){  //Input 'h', the actual state of the PID output is written to the serial port
        serial_out('H', output);
        break;
      }
      if(serial_byte_in =='s'){ //Input 's', the setpint is written to the serial port
        serial_out('S', temp_set);
      }
      else{
        serial_input = serial_input * 10;
        serial_input = ((serial_byte_in - 48) + serial_input);
        temporary = 't';
      }
    }
    
    if(temporary == 't'){
      if(serial_input < 101){
        temp_set = serial_input;
        serial_out('S', temp_set);
      }
      else{
        Serial.println("Invalid input");
      }
      temporary = 'f';
    }
  }
}

void serial_out(char C, int I){
  if(C == 'T'){
    Serial.println("Temperature");
    Serial.println(I);
  }
  if(C == 'S'){
    Serial.println("Set point");
    Serial.println(I);
  }
  if(C == 'P'){
    Serial.println("PID output");
    Serial.println(I);
  }
  if(C == 'H'){
    Serial.println("PID calculation");
    Serial.println(I);
  }
}

void temp_read(){
//LM35 temperature probe 
  temp = (analogRead(LM35_temp_sensor)*0.48828125);

//PT100 temperature probe
  //temp = (analogRead(PT100_temp_sensor));
}

void regulator(){
    NOW = millis();
    
    time_change = (double)(NOW - last_time);

    if(time_change >= sample_time){
          
      //Error for propotional
      //error = set_point - input;
      error = (double)temp_set - temp;
      
      //Error for integrator
      err_sum = err_sum + error;
      if(error == 0){
        err_sum = 0;
      }
      if(err_sum < -100){
        err_sum = -100;
      }
      if(err_sum > 100){
        err_sum = 100;
      }
      
      //Error for derivative
      d_err = error - last_err;

      //Anti-windup
      i_term = err_sum * ki;
      if(i_term > out_max){
        i_term = out_max;
      }
      if(i_term < out_min){
        i_term = out_min; 
      }
  
      //Output
      output = ((kp * error)+i_term+(kd*d_err));
  
      //Update variables
      last_err = error;
      last_time = NOW;

      //Saturation
      if(output < out_min){
        //output = 0;
        PID_signal = 0;
      }else if(output > out_max){
         //output = 100; 
        PID_signal = 100;
      }else {
        //PID_signal = (double)output;
        PID_signal = (int)output;
      }
    }
}

void heat(){
//PWM output for Solid-State relay  
  analogWrite(SSR_heater, PID_signal);  

//Output for Standard relay  
//  if(PID_signal == 100 && temp_set != 0){
//    digitalWrite(heater, HIGH);  
//  }
//  else{
//    digitalWrite(heater, LOW);
//  }
}

void tuning(double Kp, double Ki, double Kd){
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void output_limit(double outmax, double outmin){
  out_max = outmax;
  out_min = outmin;
}

