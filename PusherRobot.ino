#include <AccelStepper.h>
#include <MultiStepper.h>
float positions[] = {4.5, 9, 13.5, 18, 22.5, 27, 31.5, 36, 40.5}; //positions in inches measured from "home" position
    // [NOTE] Only using 11 positions to start with right now. 
    //positions are 4.5" (11.43 cm) apart, on center************THESE POSITIONS WILL NEED TO BE ADJUSTED BY OFFSET FROM HOME POSITION->POSITION 1
    //position index values stored as: 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14
int arraySize = 10; //?????? *NOTE: this seems to be working, but may need to change to " int arraySize=15; " ???????
int revolutions = 0;
float distancePerRevolution=2.3622; //distance (inches) of timing belt travel per one 360deg (200step) rotation of 22-tooth, 3mm pitch drive gear **actual value = 6 cm
int stepsPerRevolution=200; //motor's rated # steps per one revolution
int interval = 2000;

AccelStepper stepper1( AccelStepper::DRIVER, 10, 11);

void setup() {
  Serial.begin(9600);
  Serial.println("Setting current position.");
  // Set initial stepper properties. 
  stepper1.setCurrentPosition(0);
  stepper1.setMaxSpeed(500.0);
  stepper1.setAcceleration(500.0);
  Serial.println("Setup complete.");
}

void loop() {
  delay(5000);
  
  // Calculate a random position. 
  int randomPos = random(arraySize);
  float distance = positions[randomPos];
  int numSteps = distance/distancePerRevolution*stepsPerRevolution;
  
  Serial.print("Position, Distance, NumSteps ");
  Serial.print(randomPos);
  Serial.print(", ");
  Serial.print(distance);
  Serial.print(", ");
  Serial.println(numSteps);
  
  
  // Move to that random position.
  stepper1.move(numSteps);
  stepper1.runToPosition();

  Serial.println("Reached. Moving back to 0th position.");

  // Move back to initial position. 
  stepper1.move(-numSteps);
  stepper1.runToPosition();

  Serial.println("At 0th position.");
}

