#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;
Servo headServo; //Servo object


//-------------------------Servo--------------------------------//
//switches
const boolean HEAD_DEBUG = false;

//Head Servo Timing
unsigned long headCm;
unsigned long headPm;
const unsigned long HEAD_MOVEMENT_PERIOD =100;

//Head esrvo constants
const int HEAD_SERVO_PIN = 0;
const int NUM_HEAD_POSITIONS = 7;

//adjusted the angles so US was facing straight foward.at the middle element
//{140,130,120,110,100,25,25}
//{113,98,83}
const int HEAD_POSITITIONS[NUM_HEAD_POSITIONS] ={130,120,110,25,25,25,25};

int wallAngle = 25;

//distances at the head positions
//{0,0,0,0,0,0,0}
//{0,0,0}
double DISTANCES[NUM_HEAD_POSITIONS] = {0,0,0,0,0,0,0};

//head servo data
boolean headDirectionClcokwise = true;
int currHeadPos = 0;

//head servo data
boolean headDirectionClockwise = true;


//----------------------Ultra Sound------------------------------//
//Initialize UltraSound
const int ECHO_PIN = 30;
const int TRIG_PIN = 17;

//Ultrasonic Mic
const int MAX_DISTANCE = 200; //(200 cm / 2 meters)

// Determine the normalization factor based on the MAX_DISTANCE
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 10;

//Ultrasonic timing
unsigned long usCurrMil;
unsigned long usPrevMil;
const unsigned long US_PERIOD = 50; // Time to wait for 1st ultra sound to activate

//------------------Motor--------------------------//

//Motor Debg
boolean MOTOR_DEBUG = false;

//Motor constants
const float MOTOR_BASE_SPEED = 75;
const int MOTOR_MIN_SPEED = 50;
const int MOTOR_MAX_SPEED = 100;

//start out with the MOTOR_BASE_SPEED
float leftSpeed = MOTOR_BASE_SPEED;
float rightSpeed = MOTOR_BASE_SPEED;

// Determine the normalization factor based on Motor_Based_Speed
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

//Motor timing
unsigned long motorCurrMil;
unsigned long motorPrevMil;
const unsigned long MOTOR_PERIOD = 20; //Time to wait for adjusting the motor speed


//------------PID------------------//
//Target Distance from the wall
double target = 30.0;
double aviodDist = 20.0;
//PID debugging flag
boolean PID_DEBUG = false;

//Flag indicating whether the distance is infront of the robot or the wall 
//true = wallDist  false = in front
boolean distType = false;

//PID constants
double kp = 2;
//.5
double ki = 0;
double kd = 5000.0;

//PID calulated values
double pidSum;

//Object constant
double op = 3;
double opSum;

//I inits
double integrator = 0.0;
double kiTotal = 0.0;

//initializes so there is a value the first time moveHead runs
double newDist = target;

//error
double err = 0;
double prevErr = 0;

//time
long currTime;
long prevTime = millis();
long passTime;

//integral limits
double limitMax = 100;
double limitMin = 0;

float magnitude;

//----------------------Set Up-------------------------------//

void setup() {
  Serial.begin(57600);

  // init head postion
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(120);
  
  //Prep US
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  
  //start delay
  delay(3000);
  buzzer.play("c32");  
}
//-------------------Loop---------------------------------//
void loop() {
  //Tells set motors if the value is a newDist or the Distance from the wall 
  //or to aviod the object in front of it
  if(moveHead(newDist)){
     pidSum = updatePID(newDist);
     distType = true;
    }
    else{
    opSum = frontPID(newDist);  
    }
    
    setMotors(newDist, pidSum, opSum);

  newDist = usReadCm();
}

double getError(double dist){
  return dist - target;
  }//getError
  
//returns the controllerOutput
//Helps determine the motor speed when the wall distance is in mind
double updatePID(double wallDist){

    double updateValue;
    //positive err --> Left
    //negative err --> Right
    err = getError(wallDist);
    

    //----------PROPORTION------//
    double proportional = kp* err;

     //----------INTEGRAL------//
    integrator = integrator + err;
    //integral limits
    if(integrator > limitMax)
       integrator = limitMax;
    else if(integrator < limitMin)
      integrator = limitMin;
    double kiRes = ki * integrator;

    //----------DIRIVATIVE------//
    //milliseconds
    currTime = millis();
    passTime = currTime - prevTime;
    double dirivative  = kd* ((err - prevErr)/passTime);

    //resets to the new previous values
    prevErr = err;
    prevTime = currTime;

    if(PID_DEBUG){
      Serial.println("------PID-----");
      Serial.print("WallDist:");
      Serial.println(wallDist);
      Serial.print("P: ");
      Serial.println(proportional);
      Serial.print("I: ");
      Serial.println(kiRes);
      Serial.print("D: ");
      Serial.println(dirivative);
      Serial.println("--------------");
      Serial.print("PassTime: ");
      Serial.print(passTime);
      Serial.println(" ");
      }//if
    
    updateValue = proportional + kiRes +dirivative ;

    return updateValue;
  }//Update PID

//Determines speed when an object is in front of it
//Solves corner problem
double frontPID(double dist){
  double objtPro = 0;
  if(dist < target){
    objtPro =op * getError(dist);
  }//if
  return objtPro;
  }//frontPID

//moves the US mounted on the servo 
//determines whether the US is facing a wall or object
boolean moveHead(double dist){
    headCm = millis();
    boolean newError = false;
    
    if(headCm > headPm + HEAD_MOVEMENT_PERIOD){

        //head debug output
        if(HEAD_DEBUG){
            Serial.print(currHeadPos);
            Serial.print(" - ");
            Serial.println(HEAD_POSITITIONS[currHeadPos]);
            printDist();
          }//if

          //position head to currrent postion in array
          headServo.write(HEAD_POSITITIONS[currHeadPos]);
          //adding the distance to the array
          DISTANCES[currHeadPos] = dist;

          //(sizeof(DISTANCES)/ sizeof(long)) - 1 == currHeadPos
          //wallAngle == HEAD_POSITITIONS[currHeadPos]
          
          //determines when the wall is being looked at
          if(NUM_HEAD_POSITIONS - 2 <= currHeadPos)
          { 
            newError = true;
          }//if
              
        
          /**
           * See next head posttion
           * Moves Servo to the next head Position and changes directions when needed
           */
           if(headDirectionClockwise){
            if(currHeadPos >= (NUM_HEAD_POSITIONS - 1)){
                headDirectionClockwise = !headDirectionClockwise;
                currHeadPos--;
              }//if
             else{
              currHeadPos++;
              }
           }//if

           else{
            if(currHeadPos <= 0){
                headDirectionClockwise = !headDirectionClockwise;
                currHeadPos++;
              }//if

              else{
                  currHeadPos--;
                }
            }//else

            //reset previous millis
            headPm = headCm; 
      }//if
      return newError;
  }//moveHead
  
  //Prints the array of distnaces
  void printDist(){
    Serial.print("{");
    //sizeof determines the size of bytes
    //A long is 4 bytes
    //Divide the bytes of the array by its data type to get the  array length
    for(int i = 0; i < sizeof(DISTANCES)/ sizeof(long); i++)
    {
        Serial.print(" ");
        Serial.print(DISTANCES[i]);
        Serial.print(",");
    }//for
    Serial.print("}");
    Serial.println("");
 }//printDist

  //Interprets the reading on the Ultra Sonic as a distance
  double usReadCm(){
  usCurrMil = millis();
  if(usCurrMil > usPrevMil + US_PERIOD){
      
      //Clears the TRIG_PIN (set low)
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);

      //Sets the TRIG_PIN HIGH(Active) for 10 microseconds
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      //Reads the ECHO_PIN, returns the sound wave travel time in microseconds
      //note the durration (38000 microsec) that will allow for reading up max distance supported by the sensor
      double duration = pulseIn(ECHO_PIN, HIGH, 38000);
      
      //calc the distance
      double distance = duration *0.034/2; //Time of flight equation: Speed of sound wave divided by 2

      //apply limits
      if(distance > MAX_DISTANCE) distance = MAX_DISTANCE;
      if(distance == 0) distance = MAX_DISTANCE;

      //update the prevMillis
      usPrevMil = usCurrMil;

      return distance;
      
    }//if
  }//usReadCm

 //Adjusts the motor speeds by taking the distance of the wall and nearby objects in consideration
 void setMotors(double dist, double controllerOutput, double objectOutput){
      motorCurrMil = millis();
      
      if(motorCurrMil > motorPrevMil + MOTOR_PERIOD)
      {
          //check to see if the most current distnace measurement is less then / equal to MAX_DISTANCE
          if(dist <= MAX_DISTANCE){

              //determine the magnitude of the distance by taking the difference (short distnace = high distance)
              //divide by the DISTANCE_FACTOR to ensure uniform response as MAX_DISTNACE changes
              //This maps the distnace(1 - MAX_RANGE) to 0-100 for the magnitude
               magnitude = (float) (MAX_DISTANCE - dist)/ DISTANCE_FACTOR;
              //ex: MAX_DISTANCE = 80, distnace = 40: 80 - 40 = 40/.8 = 50(midrange)
              //ex: MAX_DISTNACE = 160, distance = 40: 160- 40 = 120/1.6 = 75 (top 1/4)

              if(distType){
                //too far from wall (go right)
                if(controllerOutput < 0){
                   //CO is negative 
                   leftSpeed = getWheel(controllerOutput , 1);
                   rightSpeed = getWheel(controllerOutput, -1);  
                }
                //too claose to wall (go left)
                else{
                   //CO is positive
                   rightSpeed = getWheel(controllerOutput , -1);
                   leftSpeed = getWheel(controllerOutput, 1 ); 
                }//else

                //lower limit check
                if(leftSpeed < MOTOR_MIN_SPEED) leftSpeed = MOTOR_MIN_SPEED;
                if(rightSpeed < MOTOR_MIN_SPEED) rightSpeed = MOTOR_MIN_SPEED;
                //max limit check
                if(leftSpeed > MOTOR_MAX_SPEED) leftSpeed = MOTOR_MAX_SPEED;
                if(rightSpeed > MOTOR_MAX_SPEED) rightSpeed = MOTOR_MAX_SPEED;
                 
              }//if WallDist
              else{
                  //OverRides the Speed when there is an incoming object
                  //&& NUM_HEAD_POSITIONS - 5 > currHeadPos
                  if(dist < aviodDist ){
                    Serial.println("Here");
                    Serial.println(currHeadPos);
                    rightSpeed = getWheel(objectOutput, -1);
                    leftSpeed = getWheel(objectOutput, 1);
                  }//if
               // Applying to rigthSpeed because (ideally) the wall should be on our right side.
               // Therefore, we can assume that there is more room on the left side of the robot.
              }//else  not WallDist
                        
          }//if
         
           motors.setSpeeds(leftSpeed,rightSpeed);
           if(MOTOR_DEBUG){
              Serial.print("Current Distance: ");
              Serial.println(dist);
              Serial.print("Magnitude: ");
              Serial.println(magnitude);
              Serial.print("Error: ");
              Serial.println(err);
              Serial.print("PID output: ");
              Serial.println(controllerOutput);
              Serial.print("Left: ");
              Serial.println(leftSpeed);
              Serial.print("Right: ");
              Serial.println(rightSpeed);
           }//if
          
          motorPrevMil = motorCurrMil;
          //distType reset 
          distType = false;
        }//if
    }//setMotors
    
 //calculates the wheelspeed      
 double getWheel(double co, int sign){
      //Multiple the magnitude by the MOTOR_FACTOR to map the magnitude range(0 - 100) at the motors
     //(0 - MOTOR_BASED_SPEED)
     //Added Min Speed as a constant to increase the speed
     double wheelSpeed = (MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR)) + (co * sign)+ (MOTOR_MIN_SPEED);
     if(MOTOR_DEBUG){
       Serial.println("Base, Mag, and factor");
       Serial.println((MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR)));
       Serial.println("pid output:");
       Serial.println(co * sign);
       Serial.println("True Wheel Speed:  ");
       Serial.println(wheelSpeed);
     }//if
     return wheelSpeed;
  }//getWheel
   



                   
