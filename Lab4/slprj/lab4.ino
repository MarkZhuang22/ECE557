#include "Timer.h"

#include <BasicLinearAlgebra.h>

using namespace BLA;

 

/** -----------------------------------------------------------

* SECTION 1

*

 * @students

* TODO: Add your definitions for system, controller, and observer

 * matrices below

*

* ----------------------------------------------------------- **/

// Creating vectors for the observer and integrator states

Matrix<4, 1> x_hat; // initialized to 0 in setup()

Matrix<4, 1> x_hat_dot;

Matrix<4, 1> x_d;

 

Matrix<1, 1> u = {0}; // initializing the control input

Matrix<2, 1> y; // measured output

 

/** YOUR MATRICES HERE

 *  Below is an example of how to declare a 2 by 2 identity matrix,

*  you can delete this identityMatrix as it is not used in the

*  rest of the code

*/

 

double z_measured; // in metres

double theta_measured; // in radians

 

Matrix<4, 4> A = {

  0,  1, 0, 0,

  0, -8.81538074049198, 1.89236839895895,  0,

  0, 0, 0, 1,

  0, -26.6970949136644, 35.4099588096879,  0

};

 

Matrix<4, 1> B = {

  0,

  1.44950046175804,

  0,

  4.38976517794682

};

 

Matrix<2, 4> C = {

    1,  0, 0, 0,

    0, 0, 1, 0

};

 

Matrix<4, 4> Actrl = {

  -71.1939,  1, 0.000843726739062380,  0,

  -950.876750871020, 17.3489101635682,  -69.4121727531871, -12.3986501360486,

  26.6972119405078,  0, -80.0107474155998, 1,

  1966.97440804652,  52.5406122458154,  -1810.74444705018, -37.5489101636844

};

 

Matrix<4, 6> Bctrl = {

  71.1938718469023,  -0.000843726739062380, 0, 0, 0, 0,

  972.769010871533,  1.86605050043335,  -21.8922600005128, -26.1642909040602, 69.4384906517127,  12.3986501360486,

  -26.6972119405078, 80.0107474155998,  0, 0, 0, 0,

  -1900.67440804496, 1635.86218704790,  -66.3000000015530, -79.2377071594797, 210.292218811971,  37.5489101636844

};

 

Matrix<1, 4> Cctrl = {

    15.1033,   18.0506,  -47.9051,   -8.5537

};

 

Matrix<1, 6> Dctrl = {

    0,     0,  -15.1033,  -18.0506,   47.9051,    8.5537

};

 

Matrix<1,4> K = {

    -15.1033,  -18.0506,   47.9051,    8.5537

};

 

Matrix<1,5> Kbar = {

   -14.8316,  -19.0652,   52.6325,    9.3647,    1.7321

};

 

Matrix<1,4> Kx = {

   -14.8316,  -19.0652,   52.6325,    9.3647

};

 

Matrix<1,1> Ki = {1.7321};

 

Matrix<4,2> L = {

  71.1938718469023,  -0.000843726739062380,

  972.769010871533, 1.86605050043335,

  -26.6972119405078, 80.0107474155998,

  -1900.67440804496, 1635.86218704790

};

 

double error;

 

// Signal generator parameters

double a_sg= 0.1; //signal generator amplitude (m)

double f_sg= 0.1; // signal generator frequency (Hz)

double T_sg= 1/f_sg; //signal generator period

double z_d= -a_sg; //signal generator initial

 

/** -----------------------------------------------------------

* SECTION 2

* Do NOT modify

*

 * Sets up timer, encoder, and motor pin constants

* ----------------------------------------------------------- **/

Timer t;

double T_sample = 0.001;  // Controller sample time in seconds

double T_plot = 0.03;     // writing to serial every T_plot seconds

double T_tot = 0; // Total elapsed time in seconds

char serial_delim = ',';  // delimeter for serial interface communications

 

enum PinAssignments { //Cart and Pendulum Encoder pins

  //Cart Encoder signals A and B

  encoderPinA = 2,// brown wire, blk wire gnd

  encoderPinB = 3,//green wire, red wire 5v

 

  //Pendulum Encoder signals A and B

  encoderPinA_P = 18, //lab Din5 blk wire

  encoderPinB_P = 19, //lab Din5 orange wire

};

 

volatile int encoderPos = 0; //count cart ticks

volatile int encoderPos_P = 0; //count pendulum ticks

 

//Constants to map encoder clicks to position and angle respectively

const float pi = 3.14;

double K_encoder = 2.2749*0.00001;

double K_encoder_P = 2*pi*0.000244140625;

 

// motor control

int PWM_PIN = 11;   // pin to send the magnitude of applied motor voltage

int DIR_PIN = 8;    // pin to send the polarity of applied motor voltage

int duty_cycle;   // duty cycle of pwm to send to motor

 

//Boolean variables for determining encoder position changes

bool A_set = false;

bool B_set = false;

bool A_set_P = false;

bool B_set_P = false;

 

bool play = true; //used to stop experiment when pendulum falls

int calibrated = 0; //Calibration variable for pendulum angle

 

/** -----------------------------------------------------------

* SECTION 3

* Do NOT modify

*

 * Setup function which sets the mode for Arduino pins

* ----------------------------------------------------------- **/

void setup() {

  Serial.begin (115200); // open serial connection for writing

  x_hat.Fill(0); // initialize observer states to 0

 

  configurePins(); // sets all pins and modes

 

  t.every(T_sample*1000, outputFeedbackControl);  // update observer and controller every T_sample seconds

  t.every(T_plot*1000, writeToSerial);            // perform writeToSerial every T_plot seconds

  t.every(T_sg*1000/2, toggleSquareWave);         // switches signal amplitude between + and -

                                                  // t.every takes in ms

}

 

/** -----------------------------------------------------------

* SECTION 4

* Do NOT modify

*

 * main loop() which executes over and over again on the Arduino

* ----------------------------------------------------------- **/

void loop(){

  if(experimentFailed()){ //Stop experiment when pendulum falls (safety)

    stopExperiment();

  }

  t.update(); // update timer, which triggers scheduled functions

              // according the t.every(...) set in the setup() function

}

 

/** -----------------------------------------------------------

* SECTION 5

*

 * @students

* TODO: Add the steps to update the observer state estimate

 *       and the feedback control input

*

*       Make sure to update z_measured, y, x_hat_dot, x_hat,

 *       and u

* ----------------------------------------------------------- **/

void outputFeedbackControl(){

  if (isCalibrated()){ // checks if the start conditions

                       // for the experiment have been met

 

    /* YOUR CODE HERE

     *  make sure to update the value of the global variable u

     *  before the function call to mapControlInputToMotorShield()

    */

    z_measured = encoderPos * K_encoder;

    theta_measured = encoderPos_P * K_encoder_P;

 

    /*

    Matrix<4,1> x_d = {z_d,     0,  0, 0};

    Matrix<2,1> y = {z_measured, theta_measured};

    Matrix<6,1> con = {z_measured, theta_measured, z_d,     0,  0, 0};

    x_hat_dot = Actrl * x_hat + Bctrl * con;

    x_hat = x_hat + x_hat_dot * T_sample;

    u = -K * x_hat + K * x_d;

    */

 

   

    y = {z_measured, theta_measured};

    error = z_d - z_measured;

    x_hat_dot = A * x_hat + B * u + L *(y - C*x_hat);

    x_hat = x_hat + x_hat_dot * T_sample;

    u = -Kx * x_hat - Ki * error;

 

    mapControlInputToMotorShield();

 

  } else if(reachedCalibration()) {

      setCalibration(); // lift up position, just reached calibration

  }

} 

 

/** -----------------------------------------------------------

* SECTION 6

* Do NOT modify

*

 * Utility functions for creating the square wave reference signal,

 * writing to the serial port, and interfacing with the motor shield

* ----------------------------------------------------------- **/

void toggleSquareWave(){

  z_d = -z_d;

}

 

void writeToSerial(){

  Serial.print(y(0), 5);

  Serial.print(serial_delim);

 

  Serial.print(y(1), 5);

  Serial.print(serial_delim);

 

  Serial.print(z_d, 5);

  Serial.print(serial_delim);

 

  Serial.print(x_hat(1), 5);  // observer estimate of z_dot

  Serial.print(serial_delim);

 

  Serial.print(x_hat(3), 5);  // observer estimate of theta_dot

  Serial.print(serial_delim);

 

  Serial.print(u(0), 5); //control input

 

  Serial.println();

}

 

void mapControlInputToMotorShield(){

  //Mapping between required voltage and 11.75V motor shield

  duty_cycle = round(u(0)/11.75*1024);

  if (duty_cycle>512 ){ //Saturation to not exceed motor voltage limits of 5.875 V

    duty_cycle=512; //motor moves left

  } else if (duty_cycle < -512 ){

    duty_cycle=-512;//motor moves right

  }

  

  if (duty_cycle > 0){ // Update the direction of motor based on the sign of duty_cycle 

    digitalWrite(DIR_PIN, HIGH);

    OCR1A = duty_cycle;

  } else if (duty_cycle <= 0){

    digitalWrite(DIR_PIN, LOW);

    OCR1A = -duty_cycle;

  }

}

 

void configurePins(){

  pinMode(encoderPinA, INPUT);  // cart encoder pins

  pinMode(encoderPinB, INPUT);

  digitalWrite(encoderPinA, HIGH);  // turn on pullup resistor

  digitalWrite(encoderPinB, HIGH);  // turn on pullup resistor

  attachInterrupt(0, doEncoderA, CHANGE);

  attachInterrupt(1, doEncoderB, CHANGE);

 

  pinMode(encoderPinA_P, INPUT); // pendulum encoder pins

  pinMode(encoderPinB_P, INPUT);

  digitalWrite(encoderPinA_P, HIGH);  // turn on pullup resistor

  digitalWrite(encoderPinB_P, HIGH);  // turn on pullup resistor

  attachInterrupt(5, doEncoderA_P, CHANGE); //int.5 on pin18

  attachInterrupt(4, doEncoderB_P, CHANGE); //int.4 on pin 19

 

  pinMode(PWM_PIN, OUTPUT); // motor pins

  pinMode(DIR_PIN, OUTPUT);

  

  //Clock setup to increase PWM frequency controlling applied voltage on PIN 11

  TCCR1A = _BV(COM1A1) | _BV(WGM21) | _BV(WGM20);

  TCCR1B = _BV(CS10);

  OCR1A = 0;

}

 

bool isCalibrated(){

  /**

  Returns true if the pendulum has been previously calibrated at the

  upright position and false otherwise

  */

  return calibrated == 1;

}

 

bool reachedCalibration(){

  /**

  Returns true if the pendulum has just reached calibrated position

  from the uncalibrated position, and false if it has not. This should

  only be called if isCalibrated() returns false.

  */

  return abs(encoderPos_P)>=2048;

}

 

void setCalibration(){

  /**

  Sets initial values for global variables upon calibration.

  */

  int lift_direction;

  if (encoderPos_P < 0){

    lift_direction = -1;

  } else {

    lift_direction = +1;

  }

  encoderPos_P=encoderPos_P - (lift_direction * 2048);

  calibrated = 1;

  encoderPos = 0; //negative moves right

}

 

/** -----------------------------------------------------------

* SECTION 7

* Do NOT modify

*

 * Interrupts for reading cart position/pendulum angle

 * encoders and updating encoderPos / encoderPos_P

* ----------------------------------------------------------- **/

 

void doEncoderA(){ // Interrupt on A changing state

  // Test transition

  A_set = digitalRead(encoderPinA) == HIGH;

  // and adjust counter + if A leads B

  encoderPos += (A_set != B_set) ? +1 : -1;

}

 

void doEncoderB(){ // Interrupt on B changing state

  // Test transition

  B_set = digitalRead(encoderPinB) == HIGH;

  // and adjust counter + if B follows A

  encoderPos += (A_set == B_set) ? +1 : -1;

}


void doEncoderA_P(){ // Interrupt on A changing state

  // Test transition

  A_set_P = digitalRead(encoderPinA_P) == HIGH;

  // and adjust counter + if A leads B

  encoderPos_P += (A_set_P != B_set_P) ? +1 : -1;

}

 

void doEncoderB_P(){ // Interrupt on B changing state

    // Test transition

    B_set_P = digitalRead(encoderPinB_P) == HIGH;

    // and adjust counter + if B follows A

    encoderPos_P += (A_set_P == B_set_P) ? +1 : -1;

  }

 

bool experimentFailed(){

  /**

  Returns true if the pendulum has fallen down and false otherwise

  */

  return (play == false) || (calibrated==1 && abs(encoderPos_P)>=1000);

}

 

void stopExperiment(){ 

  /**

  Stops the experiment

  */

  OCR1A = 0; //Turn off applied voltage to stop experiment

  play = false;

  delay(90000000);

}

 

 
