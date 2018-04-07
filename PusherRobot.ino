#include <AccelStepper.h>
#include <MultiStepper.h>

float positions[] = {4.5, 9, 13.5, 18, 22.5, 27, 31.5, 36, 40.5, 45}; //positions in inches measured from "home" position
    // [NOTE] Only using 11 positions to start with right now. 
    //positions are 4.5" (11.43 cm) apart, on center************THESE POSITIONS WILL NEED TO BE ADJUSTED BY OFFSET FROM HOME POSITION->POSITION 1
    //position index values stored as: 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14
int arraySize = 10; //?????? *NOTE: this seems to be working, but may need to change to " int arraySize=15; " ???????
float distancePerRevolution=2.6006;  //2.3622; //distance (inches) of timing belt travel per one 360deg (200step) rotation of 22-tooth, 3mm pitch drive gear **actual value = 6 cm
int stepsPerRevolution=400; //motor's rated # steps per one revolution

// Keep track of where I am, so I can calculate where I'd be going next. 
float currentXPosition = 0;

// Feedback pin that gets activated when the y-stepper hits the object.
const int yFeedbackPin = 2;
const int xHomePin = 3; 
const int yHomePin = 4; 

// States of the system. 
enum State {
  SEARCH,
  FIND
};

// For Home, we always do that in setup.
State currentState = FIND;

// Initialize X & Y stepper motors. 
AccelStepper xStepper( AccelStepper::DRIVER, 10, 11);
AccelStepper yStepper( AccelStepper::DRIVER, 5, 6);

void setup() {
  Serial.begin(9600);

  // Set output pins. 
  pinMode(yFeedbackPin, INPUT);
  pinMode(xHomePin, INPUT);
  pinMode(yHomePin, INPUT);

  // Configure y-stepper.
  yStepper.setCurrentPosition(0);
  //yStepper.setSpeed(1);
  yStepper.setMaxSpeed(2500.0);
  yStepper.setAcceleration(15000.0);

  // Home Y stepper motor first.  
  //homeYStepper();

  // Configure x-stepper.
  xStepper.setCurrentPosition(currentXPosition);
  xStepper.setMaxSpeed(1500.0);
  xStepper.setAcceleration(500.0);

  // Home X Stepper motor. 
  //homeXStepper();

  Serial.println("Setup complete.");
}

void loop() {
//  if (currentState == SEARCH) {
//      // Calculate a random position, distance to go, and numSteps to get to that position.
//      int randomIdx = random(arraySize);
//      float distanceToGo = positions[randomIdx] - currentXPosition; // It could be +ve/-ve and that'll determine direction to go to.
//      int numSteps = distanceToGo/distancePerRevolution*stepsPerRevolution;
//    
//      // Print the current position. 
//      printPos(randomIdx, positions[randomIdx], numSteps);
//      
//      // Move to that random position. 
//      xStepper.move(numSteps);
//      xStepper.runToPosition();
//    
//      // Update my current position.
//      currentXPosition = positions[randomIdx]; 
//
//      // Change the state to FIND from here, so we come and move the yStepper from here. 
//  }
//
//  if (currentState == FIND) {
//     int ySteps = 1; 
//     // Move the motor into the slot.  
//     while (digitalRead(yFeedbackPin) == LOW) {
//        yStepper.moveTo(ySteps);
//        yStepper.run();
//        ySteps++;
//     }
//
//     // Switch is pressed. Move 1 full revolution into the slot. 
//     ySteps = stepsPerRevolution; 
//     yStepper.moveTo(ySteps);
//     while(yStepper.distanceToGo() != 0) {
//        yStepper.run();
//     }
//
//     // Home the stepper motor. 
//     homeYStepper();
//
//     // Change the state back to SEARCH from here, so we come and move the xStepper from here. 
//  }
//    Serial.println("Starting yStepping");
    int ySteps = 8000;
    yStepper.move(-ySteps);
    yStepper.runToPosition();
//    Serial.println("Completed 10 revolutions.");
//    Serial.println("Starting yStepping");
    ySteps = 8000;
    yStepper.move(ySteps);
    yStepper.runToPosition();
//    Serial.println("Completed 10 revolutions.");
//    delay(1000);
}

void homeXStepper() {
  // Move in the positive direction.
  int setupHome = 1; 
  while (digitalRead(xHomePin) == LOW) {  // Make the stepper move counterclockwise until the homeSwitch is activated.
    xStepper.moveTo(setupHome);  // Set the position to move to
    xStepper.run();
    setupHome++;  // decrease by 1 for next move if needed
  }

  // Reset it to go backward so it releases the switch. 
  setupHome = -1; 

  while (digitalRead(xHomePin) == HIGH) { // Make the Stepper move clockwise until the homeSwitch is deactivated
    xStepper.moveTo(setupHome);
    xStepper.run();
    setupHome--; // increase by one for next move if needed
  }

  // Now set the current position. 
  xStepper.setCurrentPosition(0);

  // Now set maxSpeed, etc. 
}

void homeYStepper() {
  // Move in the positive direction.
  int setupHome = -1; 
  while (digitalRead(yHomePin) == LOW) {  // Make the stepper move counterclockwise until the homeSwitch is activated.
    yStepper.moveTo(setupHome);  // Set the position to move to
    yStepper.run();
    setupHome--;  // decrease by 1 for next move if needed
  }

  // Reset it to go backward so it releases the switch. 
  setupHome = 1; 

  while (digitalRead(yHomePin) == HIGH) { // Make the Stepper move clockwise until the homeSwitch is deactivated
    yStepper.moveTo(setupHome);
    xStepper.run();
    setupHome++; // increase by one for next move if needed
  }

  // We have found our 0 position. Boom. 
  yStepper.setCurrentPosition(0);

  // Now set maxSpeed, acceleration, etc for yStepper. 
}

// Helper method to print where I am. 
void printPos(int idx, float distance, int numSteps) {
  Serial.print("Destination: Position, Distance, NumSteps ");
  Serial.print(idx);
  Serial.print(", ");
  Serial.print(distance);
  Serial.print(", ");
  Serial.println(numSteps);
}

