#include <AccelStepper.h>
#include <MultiStepper.h>

float positions[] = {4.5, 9, 13.5, 18, 22.5, 27, 31.5, 36, 40.5, 45}; //positions in inches measured from "home" position
    // [NOTE] Only using 11 positions to start with right now. 
    //positions are 4.5" (11.43 cm) apart, on center************THESE POSITIONS WILL NEED TO BE ADJUSTED BY OFFSET FROM HOME POSITION->POSITION 1
    //position index values stored as: 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14
int arraySize = 10; //?????? *NOTE: this seems to be working, but may need to change to " int arraySize=15; " ???????
float distancePerRevolution=2.6006; //distance (inches) of timing belt travel per one 360deg (200step) rotation of 22-tooth, 3mm pitch drive gear **actual value = 6 cm
int stepsPerRevolution=400; //motor's rated # steps per one revolution

// Keep track of where I am, so I can calculate where I'd be going next. 
float currentXPosition = 0;

// Feedback pin that gets activated when the y-stepper hits the object.
const int yFeedbackPin = 4;
const int xHomePin = 2; 
const int yHomePin = 3; 
const int yForwardPin = 8; 

boolean yFeedbackState = false;

// States of the system. 
enum State {
  SEARCH,
  FIND
};

// For Home, we always do that in setup.
State currentState = SEARCH;

// Initialize X & Y stepper motors. 
AccelStepper xStepper( AccelStepper::DRIVER, 10, 11);
AccelStepper yStepper( AccelStepper::DRIVER, 5, 6);

void setup() {
  Serial.begin(9600);

  // Set output pins. 
  pinMode(yFeedbackPin, INPUT_PULLUP);
  pinMode(xHomePin, INPUT_PULLUP);
  pinMode(yHomePin, INPUT_PULLUP);
  pinMode(yForwardPin, INPUT_PULLUP);

  Serial.println("yStepper homing begin.");

  // Home Y stepper motor.   
  homeYStepper();

  Serial.println("yStepper homing complete.");

  // Configure y-stepper for the rest of the system. 
  yStepper.setMaxSpeed(3000.0);
  yStepper.setAcceleration(15000.0);

  delay(2000);

  Serial.println("xStepper homing begin.");

  // Home X Stepper motor. 
  //homeXStepper();

  // Configure x-stepper initial speeds. 
  xStepper.setMaxSpeed(3000.0);
  xStepper.setAcceleration(10500.0);

  Serial.println("yStepper homing complete");
}

void loop() {
    boolean xHomeState = digitalRead(xHomePin);
    boolean yHomeState = digitalRead(yHomePin);
    boolean yFeedbackState = !digitalRead(yFeedbackPin); 
    boolean yForwardState = digitalRead(yForwardPin);

    // Forward. 
    if (yForwardState == LOW) {
      moveYForward();
    }

    Serial.print("xHome, yHome, yFeedback, yFowardPin: "); 
    Serial.print(xHomeState);
    Serial.print(", ");
    Serial.print(yHomeState);
    Serial.print(", ");
    Serial.print(yFeedbackState);
    Serial.print(", ");
    Serial.println(yForwardState);

    

    delay(500);
    
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
}

void homeXStepper() {
  xStepper.setCurrentPosition(0);
  xStepper.setMaxSpeed(500.0);
  xStepper.setAcceleration(100.0);
  
  // Move in the positive direction.
  int xSteps = 0; 

  // This is PULLUP so it's HIGH if it's not pressed. 
  while (digitalRead(xHomePin) == HIGH) {  // Make the stepper move counterclockwise until the homeSwitch is activated.
    xStepper.move(xSteps);  // Set the position to move to
    xStepper.run();
    xSteps--;  // Move all the way to the left for calibration. 
  }

  delay(1000);

  xStepper.setCurrentPosition(0);
  xSteps = 0; 

  // This is PULLUP so it's LOW when it's pressed.
  while (digitalRead(xHomePin) == LOW) { // Make the Stepper move clockwise until the homeSwitch is deactivated
    xStepper.move(xSteps);
    xStepper.run();
    xSteps++; // increase by one for next move if needed
  }

  // Now set the current position. 
  xStepper.setCurrentPosition(0);
}

void homeYStepper() {
  // Set slower speeds for the yStepper. 
  yStepper.setCurrentPosition(0);
  yStepper.setMaxSpeed(1000.0);
  yStepper.setAcceleration(200.0);
  
  // Steps to take in the y direction.
  int ySteps = 0; 
  
  while (digitalRead(yHomePin) == HIGH) {  // Make the stepper move backward until the homeSwitch is activated.
    yStepper.move(ySteps);  // Set the position to move to
    yStepper.run();
    ySteps--;  // Keep going backwards
  }

  delay(1000);

  yStepper.setCurrentPosition(0);
  ySteps = 0;
  
  while (digitalRead(yHomePin) == LOW) { // Make the Stepper move forward until the homeSwitch is deactivated
    yStepper.move(ySteps);
    yStepper.run();
    ySteps++; // increase by one for next move if needed
  }

  // We have found our 0 position. Boom. 
  yStepper.setCurrentPosition(0); 
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

void moveYForward() {
  int ySteps = 8000; 
  yStepper.move(ySteps);
  yStepper.runToPosition();
}

// Some yStepper tuning code that we could use. 

//    int ySteps = 16000;
//    yStepper.move(-ySteps);
//    yStepper.runToPosition();
//    
//   // Move the motor into the slot.  
//   while (!digitalRead(yFeedbackPin) == LOW) {
//      yStepper.move(ySteps);
//      yStepper.run();
//      ySteps++;
//   }
//  
//   int distanceToGo = yStepper.distanceToGo();
//   int stepsToGoBack = ySteps - distanceToGo;
//  
//   Serial.println(stepsToGoBack);
//   Serial.println("Hit something.");
//   yStepper.stop();
//  
//   delay(2000);
//  
//   yStepper.setMaxSpeed(500);
//  
//   int newPushSteps = 2000;
//   yStepper.move(newPushSteps);
//   yStepper.runToPosition();
//  
//   delay(2000);
//  
//   yStepper.setMaxSpeed(3000);
//   yStepper.moveTo(-(stepsToGoBack + newPushSteps));
//   Serial.println(yStepper.targetPosition());
//   yStepper.runToPosition();
//  
//   delay(2000);

