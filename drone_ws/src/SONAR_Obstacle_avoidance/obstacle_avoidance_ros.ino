#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;
geometry_msgs::Twist sonarDistance;
geometry_msgs::Twist warning;
ros::Publisher chatter1("Dist_Value", &sonarDistance);
ros::Publisher warning_chatter("Warning", &warning);

const int frontEchoPin = 7;
const int frontTriggerPin = 6;
const int leftEchoPin = 11;
const int leftTriggerPin = 10;
const int rightEchoPin = 9;
const int rightTriggerPin = 8;

float maxFrontDistance = 25.00;
float frontDistanceCm, rearDistanceCm, leftDistanceCm, upDistanceCm, downDistanceCm, rightDistanceCm = 4000.00;
float frontDuration, rearDuration, upDuration, leftDuration, rightDuration, downDuration;
float maxLeftDistance, maxRightDistance = 20.00;

int  frontWarnFlag = 0;
int  rearWarnFlag = 0;
int  leftWarnFlag = 0; 
int  rightWarnFlag = 0;
int  downWarnFlag = 0; 
int  upWarnFlag = 0;



void resetFlags(){
 frontWarnFlag = 0;
 rearWarnFlag = 0;
 leftWarnFlag = 0;
 rightWarnFlag = 0;
 warning.angular.x=0;
 warning.angular.x=0;
 warning.angular.x=0;
 warning.linear.x=0;
 warning.linear.y=0;
 warning.linear.z=0;
}


void updateDist()
{
  sonarDistance.linear.x=frontDistanceCm;
  sonarDistance.linear.x=rearDistanceCm;
  sonarDistance.linear.x=rightDistanceCm;
  sonarDistance.linear.x=leftDistanceCm;
  sonarDistance.linear.x=upDistanceCm;
  sonarDistance.linear.x=downDistanceCm;  
}

void checkFrontDistance() {
  digitalWrite(frontTriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(frontTriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(frontTriggerPin, LOW);
  frontDuration = pulseIn(frontEchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
  frontDistanceCm = frontDuration * 10 / 292 / 2;  //convertimos a distancia, en cm
  //Serial.print("Distance: ");
  //Serial.print(frontDistanceCm);
  //Serial.println(" cm");
  if (frontDistanceCm < maxFrontDistance) {
    //Serial.println("Front too close");
    frontWarnFlag = 1;
    delay(20); 
  }
}

void checkLeftDistance() {
  digitalWrite(leftTriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(leftTriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(leftTriggerPin, LOW);
  leftDuration = pulseIn(leftEchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
  leftDistanceCm = leftDuration * 10 / 292 / 2;  //convertimos a distancia, en cm
  //Serial.print("Left distance: ");
  //Serial.print(leftDistanceCm);
  //Serial.println(" cm");
  if (leftDistanceCm < maxLeftDistance) {
    //Serial.println("Left too close");
    leftWarnFlag = 1;
    delay(20); 
  }
}

void checkRightDistance() {
  digitalWrite(rightTriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(rightTriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(rightTriggerPin, LOW);
  rightDuration = pulseIn(rightEchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
  rightDistanceCm = rightDuration * 10 / 292 / 2;  //convertimos a distancia, en cm
  //Serial.print("Right distance: ");
  //Serial.print(rightDistanceCm);
  //Serial.println(" cm");
  if (rightDistanceCm < maxRightDistance) {
    //Serial.println("Right too close");
    rightWarnFlag = 1;
    delay(20); 
  }
}


void moveRear() {
  //Serial.println("Backward.");
  warning.linear.y=1;
}

void moveFront() {
  //Serial.println("Forward.");
  warning.linear.x=1;
}

void moveLeft() {
  //Serial.println("Left.");
  warning.angular.x=1;
}

void moveRight() {
  //Serial.println("Right.");
  warning.linear.z=1;
}


void moveUp() {
  //Serial.println("Up.");
  warning.angular.y=1;
}


void moveDown() {
  //Serial.println("Down.");
  warning.angular.z=1;
}


void takeAction(){
  if(frontWarnFlag == 1)
    {
      if(rightWarnFlag == 0)
        moveRight();
      else if(leftWarnFlag == 0)
        moveLeft();
      else if(rearWarnFlag == 0)
        moveRear();
      else if(downWarnFlag == 0)
        moveDown();
      else 
        moveUp();
    }
  else if(rightWarnFlag == 1 || leftWarnFlag == 1 || rearWarnFlag == 1)
    moveFront();    
}

void setup() {
  // serial
  Serial.begin(9600);

  pinMode(frontTriggerPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(leftTriggerPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(rightTriggerPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  nh.initNode();
  nh.advertise(chatter1);

}

void loop() {

  // front distance check
  checkFrontDistance();
  // left distance check
  checkLeftDistance();
  // right distance check
  checkRightDistance();
  takeAction();
  updateDist();
  resetFlags();
  chatter1.publish(&sonarDistance);
  warning_chatter.publish(&warning);

  nh.spinOnce();

}
