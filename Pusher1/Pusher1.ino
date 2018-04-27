#include <AccelStepper.h>
#include <MultiStepper.h>

// 0 - 14 indices for all the position 
const float positions[] = {0, 4.5, 9.0, 13.5, 18, 22.5, 27, 31.5, 36, 40.5, 45, 49.5, 54, 59.5, 64}; //positions in inches measured from "home" position
    // positions are 4.5" (11.43 cm) apart,
const int arraySize = 15; 
const float distancePerRevolution=1.57927; // distance (inches) per revolution (360). This is calculated during calibration of the motor for this machine. 

const long xStepsPerRevolution=3200; // X motor's rated # steps per one revolution (1/32nd Microstep)
const long maxYSteps=33500; // Max steps we want to go out before we decide to come back. 
const long yStepsPerRevolution=400; // Y motor's rated # steps per one revolution (1/4th Microstep)

// Keep track of where I am, so I can calculate where I'd be going next. 
float currentXPosition = 0;

// Switches. 
const int yFeedbackPin = 4; // y-feedback pin. 
const int xHomePin = 2; // x-home pin. 
const int yHomePin = 3; // y-home pin. 

// States of the system. 
enum State {
  SEARCH,
  FIND
};

// We start with the SEARCH state. 
// We do an initial homing in setup.
State currentState = SEARCH;

unsigned long trackTime; 
long resetHomeTime = 60000; // 60 seconds = 1 minutes 

// Initialize X & Y stepper motors. 
AccelStepper xStepper( AccelStepper::DRIVER, 10, 11); // Pulse, Direction
AccelStepper yStepper( AccelStepper::DRIVER, 5, 6); // Pulse, Direction

void setup() {
  Serial.begin(9600);

  // Set output pins. 
  pinMode(yFeedbackPin, INPUT_PULLUP);
  pinMode(xHomePin, INPUT_PULLUP);
  pinMode(yHomePin, INPUT_PULLUP);
  
  // Home the system as soon as we start. 
  homeSystem();
}

void loop() {
    // Print switches only for debugging purposes. 
    // printSwitches(); 

    // Track time, so
    unsigned long currentTime = millis() - trackTime;
    if (currentTime > resetHomeTime) {
      homeSystem();
    }
    
    if (currentState == SEARCH) {
        // X-stepper search speed. 
        xStepper.setMaxSpeed(2000.0);
        xStepper.setAcceleration(2000.0);
        
        // Calculate a random position, distance to go, and numSteps to get to that position.
        int randomIdx = random(arraySize);
        float distance = positions[randomIdx] - currentXPosition; // It could be +ve/-ve and that'll determine direction to go to.
        long numSteps = (distance * xStepsPerRevolution) / distancePerRevolution;
      
        // Print the current position. 
        printPos(randomIdx, positions[randomIdx], numSteps);
        
        // Move to that random position. 
        xStepper.move(numSteps);
        xStepper.runToPosition();
      
        // Update my current position.
        currentXPosition = positions[randomIdx]; 

        // Update state. 
        currentState = FIND; 
    }
  
    if (currentState == FIND) {
       // Configure y-stepper speed for FIND state. 
       yStepper.setMaxSpeed(1000.0);
       yStepper.setAcceleration(2000.0);
       
       int ySteps = 0; 
       // Move y actuator into the slot.Make sure we don't exceed maxYSteps. 
       while (digitalRead(yFeedbackPin) == HIGH && yStepper.currentPosition() < maxYSteps) {
          yStepper.moveTo(ySteps);
          yStepper.run();
          ySteps++;
       }

       // Either received feedback or detected something. 
       // Stop the motor. 
       yStepper.stop();
       delay(2000);

       // If we haven't reached the maxYSteps, then we have detected something. 
       // So, push forward a little bit. Else, home back. 
       if (yStepper.currentPosition() < maxYSteps) {
            // Set speed for pushing mechanism.
            yStepper.setMaxSpeed(500.0);
            yStepper.setAcceleration(1000.0);
            
            yStepper.move(yStepsPerRevolution/2);
            yStepper.runToPosition();

            // After the push, stop the motor and wait for a bit. 
            yStepper.stop();
            delay(2000);
       }

       // Home the stepper.  
       homeYStepper();

       // Update state.
       currentState = SEARCH;
    }

    delay(500);
}

void homeXStepper() {
  xStepper.setCurrentPosition(0);

  // Set homing speed.  
  xStepper.setMaxSpeed(1000.0);
  xStepper.setAcceleration(1000.0);
  
  // Move in the positive direction.
  int xSteps = 0; 
  // This is PULLUP so it's HIGH if it's not pressed. 
  while (digitalRead(xHomePin) == HIGH) {  // Make the stepper move counterclockwise until the homeSwitch is activated.
    xStepper.move(xSteps);  // Set the position to move to
    xStepper.run();
    xSteps--;  // Move all the way to the left for calibration. 
  }

  // Stop the motor and wait for 2 seconds. 
  xStepper.stop();
  delay(2000);

  xStepper.setCurrentPosition(0);
  xSteps = 0; 

  // Since switch is depressed, we wait till the switch become
  // high again. Till we move forward. 
  while (digitalRead(xHomePin) == LOW) { // Make the Stepper move clockwise until the homeSwitch is deactivated
    xStepper.move(xSteps);
    xStepper.run();
    xSteps++; // increase by one for next move if needed
  }

  // We have found our 0 position. BOOM. 
  xStepper.setCurrentPosition(0);
}

void homeYStepper() { 
  yStepper.setCurrentPosition(0);

  // Set homing speed for y. 
  // Have a higher acceleration since it needs that to start. 
  yStepper.setMaxSpeed(1000.0);
  yStepper.setAcceleration(2000.0);
  
  // Steps to take in the y direction.
  int ySteps = 0; 
  // This is PULLUP so it's HIGH if it's not pressed. 
  while (digitalRead(yHomePin) == HIGH) {  // Make the stepper move backward until the homeSwitch is activated.
    yStepper.move(ySteps);  // Set the position to move to
    yStepper.run();
    ySteps--;  // Keep going backwards
  }

  // Wait for some time before moving back. 
  yStepper.stop();
  delay(2000);

  yStepper.setCurrentPosition(0);
  ySteps = 0;

  // Since switch is depressed, we wait till the switch become
  // high again. Till we move forward. 
  while (digitalRead(yHomePin) == LOW) { // Make the Stepper move forward until the homeSwitch is deactivated
    yStepper.move(ySteps);
    yStepper.run();
    ySteps++; // increase by one for next move if needed
  }

  // We have found our 0 position. Boom. 
  yStepper.setCurrentPosition(0); 
}

// Home the system. 
void homeSystem() {
  Serial.println("Starting homing.");
  homeYStepper();
  homeXStepper();
  currentState = SEARCH;
  currentXPosition = 0;
  trackTime = millis(); // Reset time to wait till next homing begins. 
}

void printSwitches() {
    boolean xHomeState = digitalRead(xHomePin);
    boolean yHomeState = digitalRead(yHomePin);
    boolean yFeedbackState = digitalRead(yFeedbackPin); 

    Serial.print("xHome, yHome, yFeedback, yFowardPin: "); 
    Serial.print(xHomeState);
    Serial.print(", ");
    Serial.print(yHomeState);
    Serial.print(", ");
    Serial.println(yFeedbackState); 
}

void printPos(int idx, float distance, float numSteps) {
  Serial.print("Destination: Position, Distance, NumSteps ");
  Serial.print(idx);
  Serial.print(", ");
  Serial.print(distance);
  Serial.print(", ");
  Serial.println(numSteps);
}

//// Testing Homing. 
//void moveYForward() {
//   float ySteps = 0; 
//   // Move the motor into the slot.  
//   while (digitalRead(yFeedbackPin) == HIGH) {
//      yStepper.moveTo(ySteps);
//      yStepper.run();
//      ySteps++;
//   }
//}
//
//// Testing Homing. 
//void moveXForward() {
//  int xSteps = 20000; 
//  xStepper.move(xSteps);
//  xStepper.runToPosition();
//}
