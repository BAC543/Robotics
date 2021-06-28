#include <Pololu3piPlus32U4.h>

  using namespace Pololu3piPlus32U4;
  
  Encoders encoders;
  Buzzer buzzer;
  Motors motors;
  ButtonA buttonA;

//-----------------------DISTANCE CALCULATIONS-----------------------//
  unsigned long currentMilllis;
  unsigned long prevMillis;
  const unsigned long PERIOD = 20;
  long countsLeft = 0;
  long countsRight = 0;
  long prevLeft  = 0;
  long prevRight = 0;
  //All calculations are done in cm
  
  //CLICKS_PER_ROTATION * GEAR_RATIO =  Total # of clicks that happen in the encoder per wheel rotaiton
  const float CLICKS_PER_ROTATION = 12;
  const float GEAR_RATIO = 75.81F;
  //3.2 cm
  const float WHEEL_DIAMETER = 3.2;
  const float WHEEL_CIRCUMFRENCE = 10.0531;

//-----------------LOCATION INITS---------------------//

  //location debugger
  boolean debug;
  boolean locDebug = true;
  boolean changeDebugger = true;
  
  //right and left wheel constants
  float b = 8.5;
  float prevSl = 0.0F;
  float prevSr = 0.0F;
  float tempSl = 0.0F;
  float tempSr = 0.0F;
  float Sl = 0.0F;
  float Sr  = 0.0F;
  float x = 0.0;
  float y = 0.0;
  float theta = 0;
  float pos[3] = {x, y, theta};

  //change constants
  float travelDist = 0;
  float sChange = 0;
  float thetaChange = 0;
  float xChange = 0;
  float yChange = 0;

  // goals
  const int NUMBER_OF_GOALS = 4;
  float xGoals[NUMBER_OF_GOALS] = {60, 80, -60,0};
  float yGoals[NUMBER_OF_GOALS] = {0, 50, -30, 0};
  int count = 0;
  float xGoal = xGoals[count];
  float yGoal = yGoals[count];
  float endDist;
  boolean isEndGame = false;

  //Size of Way Points in cm
  int leeWay = 2;

  //Did I convert MAX_DISTANCE, DISTANCE_FACTOR, & magniude correctly? 

  //distance from origin
  float MAX_DISTANCE = sqrt(sq(xGoal - x)+sq(y -yGoal));
  float slowDist = 40;
  float dist = MAX_DISTANCE;
  
  // Determine the normalization factor based on the MAX_DISTANCE
  const float DISTANCE_FACTOR = MAX_DISTANCE / 2;
    
//------------------Motor--------------------------//
//Motor Debg
boolean MOTOR_DEBUG = false;

//Motor constants
const float MOTOR_BASE_SPEED = 75;
const int MOTOR_MIN_SPEED = 50;
const int MOTOR_MAX_SPEED = 150;

//start out with the MOTOR_BASE_SPEED
float leftSpeed = MOTOR_BASE_SPEED;
float rightSpeed = MOTOR_BASE_SPEED;

// Determine the normalization factor based on Motor_Based_Speed
const float MOTOR_FACTOR = MOTOR_BASE_SPEED / 100;

//Motor timing
unsigned long motorCurrMil;
unsigned long motorPrevMil;
//Time to wait for adjusting the motor speed
const unsigned long MOTOR_PERIOD = 20; 
//------------PID------------------//

//PID debugging flag
boolean PID_DEBUG = false;

//PID constants
double kp = 100;

//I inits
double ki = 1.0;
double integrator = 1.0;
double kiTotal = 0.0;

//integral limits
double limitMax = 100;
double limitMin = 0;


//PID calulated values
double pidSum = 1;

//error
double err = 0;

float magnitude;

//----------------------Set Up-------------------------------//
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay(1000);
  buzzer.play("c32");
}

void loop() {
      if(count < NUMBER_OF_GOALS)
      {
        checkEncoders();
        setMotors(pidSum);
        isFinished();
      }//if

      //Allows me to reset the location for debugging purposes
      if(buttonA.isPressed()){
        Sl = 0;
        Sr = 0;
        x = 0;
        y = 0;
        theta = 0;
      }//if

}//loop

void checkEncoders()
{
  currentMilllis =  millis();
  
 if(currentMilllis > prevMillis + PERIOD)
 {
    countsLeft += encoders.getCountsAndResetLeft();
    countsRight += encoders.getCountsAndResetRight();

    //Saves the previous values of Sl and Sr
    prevSl = Sl;
    prevSr = Sr;
    
    //Calculates the distance of the left and right wheel
    Sl += ((countsLeft - prevLeft)/(CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);
    Sr += ((countsRight - prevRight)/(CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFRENCE);

    tempSl = Sl - prevSl;
    tempSr = Sr - prevSr; 

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMilllis;

    //udpates when the time period passes
    updateChange();
    pidSum = updatePID();

    if(locDebug){
      Serial.print("Left: ");
      Serial.print(Sl);
      Serial.print("Right: ");
      Serial.println(Sr);

      Serial.print("Temp Sl");
      Serial.println(tempSl);
      Serial.print("Temp Sr");
      Serial.println(tempSr);
      Serial.print("Travel Distance: ");
      Serial.println(travelDist);

      Serial.print("Goal(x,y): (");
      Serial.print(xGoal);
      Serial.print(",");
      Serial.print(yGoal);
      Serial.println(")");
      
      Serial.print("Current position (x,y, theta):  ");
      Serial.print("(");
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.print(",");
      Serial.print(theta);
      Serial.println("%)");
      Serial.print("Distance from goal :");
      Serial.println(dist); 
      Serial.println("############################################");
    }//if
  }//if
}//CheckEncoders

void updateChange(){

     //Do you need to the total distance or the traveled distance from the last check? --Traveled
     
     //traveled
     travelDist =(tempSl + tempSr)/2;
    
     thetaChange = (tempSr -  tempSl)/b;
    
     xChange = travelDist*cos(theta + thetaChange/2);
     yChange = travelDist*sin(theta + thetaChange/2);
     //updates the current location
     x+= xChange;
     y+= yChange;
     theta+=thetaChange;
     dist= sqrt(sq(xGoal - x) + sq(y - yGoal));

      
     if(changeDebugger){
        Serial.print("sChange :");
        Serial.println(sChange);
        Serial.print("thetaChange :");
        Serial.println(thetaChange);
        Serial.print("xChange :");
        Serial.println(xChange);
        Serial.print("yChange :");
        Serial.println(yChange);
      }//if
   
}//updateChange
  
double getError(){
  float error =  theta - atan2(yGoal - y, xGoal - x);
  return error;
  }//getError
  
//returns the controllerOutput
//Helps determine the motor speed when the wall distance is in mind
double updatePID(){

    double updateValue;
    //positive err --> Left
    //negative err --> Right
    err = getError();
   
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

    if(PID_DEBUG){
      Serial.println("------PID-----");
      Serial.print("currAngle:");
      Serial.println(theta);
      Serial.print("P: ");
      Serial.println(proportional);
      Serial.println(" ");
      }//if
    
    updateValue = proportional + kiRes;

    return updateValue;
 }//Update PID

//Identifies when the robot reaches a way point
void isFinished (){
  boolean fin = false;
  
  if((xGoal - leeWay <= x && xGoal + leeWay >= x) && (yGoal - leeWay <= y && yGoal + leeWay >= y))
      fin = true;

  if(fin){
      count++;
      xGoal = xGoals[count];
      yGoal = yGoals[count];

      //Sets a new Max distance from the new way point
      MAX_DISTANCE = sqrt(sq(xGoal - x) + sq(y - yGoal));
      
      if(count < NUMBER_OF_GOALS)
        buzzer.play("c32");
     
      if(count == NUMBER_OF_GOALS-1)
         isEndGame = true;  

      else if(count == NUMBER_OF_GOALS){
            buzzer.play("c32");
            buzzer.play("abc");
            motors.setSpeeds(0,0);
        }//if
    }//if
           
}//isFinished

 //Adjusts the motor speeds by taking the angle and x/y goals into consideration
 void setMotors(double controllerOutput){
      motorCurrMil = millis();
      
      if(motorCurrMil > motorPrevMil + MOTOR_PERIOD)
      {
          
          //check to see if the most current distnace measurement is less then / equal to MAX_DISTANCE
          //determine the magnitude of the distance by taking the difference (short distnace = high distance)
          //divide by the DISTANCE_FACTOR to ensure uniform response as MAX_DISTNACE changes
          //This maps the distnace(1 - MAX_RANGE) to 0-100 for the magnitude
           magnitude = (float) (MAX_DISTANCE - dist)/ DISTANCE_FACTOR;
          //ex: MAX_DISTANCE = 80, distnace = 40: 80 - 40 = 40/.8 = 50(midrange)
          //ex: MAX_DISTNACE = 160, distance = 40: 160- 40 = 120/1.6 = 75 (top 1/4)
  
        
        //too far from target (go right)
        if(controllerOutput < 0){
           //CO is negative 
           leftSpeed = getWheel(controllerOutput , 1);
           rightSpeed = getWheel(controllerOutput, -1);  
        }
        //too close to target (go left)
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
        
         motors.setSpeeds(leftSpeed,rightSpeed);
         if(MOTOR_DEBUG){
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
        }//if
    }//setMotors
    
 //calculates the wheelspeed      
 double getWheel(double co, int sign){
      //Multiple the magnitude by the MOTOR_FACTOR to map the magnitude range(0 - 100) at the motors
     //(0 - MOTOR_BASED_SPEED)
     //Added Min Speed as a constant to increase the speed
     double wheelSpeed = (MOTOR_BASE_SPEED - (magnitude * MOTOR_FACTOR)) + (co * sign)+ (MOTOR_MIN_SPEED);    
     
     //slows the robot down when it is approaching the end goal
     if(isEndGame){ 
        //Wheel to gradually slow down when the slow dist is reached
        if(dist < slowDist)
            wheelSpeed = wheelSpeed*(dist/slowDist);
      }//if
      
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


  
