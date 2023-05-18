#include <Wire.h>
#include <LIDARLite.h>
#include <Servo.h>
#include <SPI.h>

#define WheelCW 7 //Wheel CW is defined as pin #7//

#define WheelCCW 8 //Wheel CCW is defined as pin #8//

#define SteeringCW 3

#define SteeringCCW 4

// Globals
LIDARLite lidarLite;
int cal_cnt = 0;
int rotation = 0; //LIDAR ROTATION
int rotationDirection = -1; // -1 = LEFT  1 = RIGHT
int wheelRotation = 0; //-1 = LEFT 0 = MIDDLE 1 = RIGHT
int maxDetectDist = 200;
Servo myservo;
int maxRotation = 70;
int rotationAmount = 10;
int distSize = maxRotation / rotationAmount; //PUT distSize in dist and lidar detections ex: dist[distSize] and lidarDetections[distSize]
int dist[7]; 
int distMiddle = distSize / 2;
bool lidarDetections[7]; //TRUE = OBJECT DETECTED; FALSE = NO OBJECT DETECTED; Booleans representing whether the lidar detected an object within the detection distance at each lidar position
int numDetections = 0;
int numLeftDetections = 0;
int numRightDetections = 0;
int lidarTurnDelay = 0;
//LIDAR STARTS FROM RIGHT AT 0 degrees if you are the car
//LIDAR 180 is full left

//dist[0] is far right, whichi s 0 degrees.
//dist[17] is far left, which is 180 degrees

void setup()
{

  pinMode(WheelCW, OUTPUT); //Set CW as an output//
  pinMode(WheelCCW, OUTPUT); //Set CCW as an output//
  pinMode(SteeringCW,OUTPUT);
  pinMode(SteeringCCW,OUTPUT);
  
  Serial.begin(9600); // Initialize serial connection to display distance readings
  lidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz
  lidarLite.configure(0); // Change this number to try out alternate configurations
  //myservo.attach(9); // attach the servo to our servo object
  //myservo.write(90);
  wheelMotorOff();
  turnWheelsMiddle();
  //delay(1000);
}


void loop()
{
   //goReverse();
// goForward();
// wheelMotorOff();
Serial.print("Hi");
Serial.println(random(0,1000));
 ///For adjusting the lidar activate this method and move the lidar to the middle
  //lidarToMiddle();

 // wheelMotorOff();
  goForward();
  delay(1000);

  //wheelMotorOff();
  goReverse();
  delay(1000);

//turnWheelsRight();

//turnWheelsMiddle();

//turnWheelsLeft();

//turnWheelsMiddle();
  /* 
  updateLidar();
  
  
  if(lidarDetections[distMiddle - 2] || lidarDetections[distMiddle - 1] || lidarDetections[distMiddle] || lidarDetections[distMiddle + 1] || lidarDetections[distMiddle + 2]){
    
    if(numDetections > (distSize - 2)){
       //STUCK IN A CORNER OR NO WAY FORWARD CODE
       goReverse();
      if(numLeftDetections > numRightDetections){
        //TURN RIGHT
        turnWheelsRight();
      }else{
        //TURN LEFT
        turnWheelsLeft();
      }
      
    }else{
      goForward();
      
      if(numLeftDetections > numRightDetections){
        //TURN RIGHT
        turnWheelsRight();
      }else{
        //TURN LEFT
        turnWheelsLeft();
      }
    }
  }else{
    goForward();
    turnWheelsMiddle();
  }
*/
  
//  LIDAR DETECTIONS IS UPDATEWITHIN UPDATE LIDAR
  
  
  

}

void updateLidar(){
//  PRINTING THE DISTANCES
  
   Serial.print("dist: "  );
    for(int i = 0; i < distSize; i++){
      Serial.print(dist[i]);
      Serial.print(", ");
    }
   Serial.println();
  
  
  if(rotationDirection > 0){
    double distance = turnLidarRight();
    dist[(rotation / rotationAmount) - 1] = distance;
  }else{
    double distance = turnLidarLeft();
    dist[(rotation / rotationAmount) - 1] = distance;
  }

 //CHANGING LIDAR ROTATION DIRECTION
  if(rotation >= maxRotation){
    rotationDirection = 1;
  }else if(rotation <= 10){
    rotationDirection = -1; 
  }

 //TRANSFORMNG THE LIDAR DISTANCES TO A BOOLEAN VALUE AND COUNTING NUMBER OF DETECTIONS
    numDetections = 0;
    numLeftDetections = 0;
    numRightDetections = 0;
  for(int lidarPosToCheck = 0; lidarPosToCheck < distSize; lidarPosToCheck++){
    if(dist[lidarPosToCheck] < maxDetectDist){
      lidarDetections[lidarPosToCheck] = true;
      numDetections ++;
      if(lidarPosToCheck < (distSize / 2) -1){
        numLeftDetections ++;
      }else if(lidarPosToCheck > (distSize / 2) + 1){
        numRightDetections ++;
      }
    }else{
      lidarDetections[lidarPosToCheck] = false;
    }
    
  }
  
  
}


//LIDAR CONTROL CODE 

int lidar(){// returns readings from the lidar
  int dist;

  // At the beginning of every 100 readings,
  // take a measurement with receiver bias correction
  if ( cal_cnt == 0 ) {
    dist = lidarLite.distance();      // With bias correction
  } else {
    dist = lidarLite.distance(false); // Without bias correction
  }

  // Increment reading counter
  cal_cnt++;
  cal_cnt = cal_cnt % 100;

  Serial.println(cal_cnt);
  return dist;
}

int turnLidarRight(){// moves and records right
    int distance = lidar();
    myservo.write(rotation);
    rotation -= rotationAmount;
    delay(lidarTurnDelay);
    
    return(distance);
}

int turnLidarLeft(){// moves and records left
    int distance = lidar();
    myservo.write(rotation); 
    rotation += rotationAmount;
    delay(lidarTurnDelay);
    
    return(distance);
}

int lidarToMiddle(){ //moves the lidar to the "middle" position for adjustment purposes; FOR DEVELOPMENT PURPOSES ONLY
  myservo.write(maxRotation / 2);
}


//MOTOR CONTROL CODE

void turnWheelsLeft(){
  if(wheelRotation == 0){
    steeringMotorCW();
    delay(105);
  }else if(wheelRotation == 1){
    steeringMotorCW();
    delay(210);
  }
  steeringMotorOff();
  wheelRotation = -1;
 //Serial.println("Wheels Left");
}
void turnWheelsRight(){
  if(wheelRotation == 0){
    steeringMotorCCW();
    delay(105);
  }else if(wheelRotation == -1){
    steeringMotorCCW();
    delay(210);
  }
  steeringMotorOff();
  wheelRotation = 1;
 //Serial.println("Wheels Rightt");
}
void turnWheelsMiddle(){
  if(wheelRotation == -1){
    steeringMotorCCW();
    delay(105);
  }else if(wheelRotation == 1){
    steeringMotorCW();
    delay(105);
  }
  steeringMotorOff();
  wheelRotation = 0;
  //Serial.println("Wheels Middle");
}
void goForward(){
  wheelMotorCCW();
  //Serial.println("Wheels Forward");
}
void goReverse(){
  wheelMotorCW();
  //Serial.println("Wheels Reverse");
}
//forward
void wheelMotorCCW(){
  digitalWrite(WheelCW,HIGH);
  digitalWrite(WheelCCW, HIGH);
}
//back
void wheelMotorCW(){
  digitalWrite(WheelCW, LOW);
  digitalWrite(WheelCCW, LOW);

}

void wheelMotorOff(){
  digitalWrite(WheelCW,LOW);
  digitalWrite(WheelCCW, LOW);
  Serial.println("Wheels Off");
}


void steeringMotorCCW(){
  //RIGHT TURN 
  
  digitalWrite(SteeringCW,HIGH);
  digitalWrite(SteeringCCW, LOW);
}

void steeringMotorOff(){
  
  digitalWrite(SteeringCW,LOW);
  digitalWrite(SteeringCCW, LOW);

  //delay(1000);
}

void steeringMotorCW(){
  //LEFT TURN
  
  digitalWrite(SteeringCW, LOW);
  digitalWrite(SteeringCCW, HIGH);
}
