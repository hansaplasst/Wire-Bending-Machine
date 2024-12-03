#include <AccelStepper.h>
#include <Arduino.h>
#include <Servo.h>

#include "ioDebug.h"

#define limitSwitch 11  // Arduino Pin #
#define starSwitch 12   // Arduino Pin #
#define benderPin 2     // Arduino Pin #
StreamEx mySerial = Serial;  // Declare mySerial to enable ioDebug mySerial.printf

// Define the stepper motors and the pins the will use
AccelStepper feederStepper(1, 5, 6);  // (Type:driver, STEP, DIR)
AccelStepper zAxisStepper(1, 7, 8);
AccelStepper benderStepper(1, 9, 10);

Servo benderPinServo;
String dataIn = "";
String manualStatus = "";
int count = 0;
int dist;

// Declare function prototypes
void manual();
void star();
void cube();
void stand();
void blink(uint16_t count = 1, uint16_t ms = 100);

void setup() {
  Serial.begin(115200);
  DPRINTF(1, "Initializing...\n");
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(limitSwitch, INPUT_PULLUP);
  pinMode(starSwitch, INPUT_PULLUP);
  benderPinServo.attach(benderPin);

  benderPinServo.write(40);  // Initial position, bending pin up
  
  // Stepper motors max speed
  feederStepper.setMaxSpeed(2000);
  zAxisStepper.setMaxSpeed(2000);
  benderStepper.setMaxSpeed(2000);

  // Homing
  while (digitalRead(limitSwitch) != 0) {
    DPRINTF(1, "Homing: %i\n", digitalRead(limitSwitch));
    blink();
    benderStepper.setSpeed(1200);
    benderStepper.runSpeed();
    benderStepper.setCurrentPosition(0);  // When limit switch pressed set position to 0 steps
  }
  DPRINTF(1, "Done..\n");
  delay(40);

  // Move 1400 steps from the limit switch to starting position
  DPRINTF(1, "Moving to starting position..\n");
  while (benderStepper.currentPosition() != -1400) {
    benderStepper.setSpeed(-1200);  // if negative rotates anti-clockwise
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  blink(2);
  DPRINTF(1, "Done.\nWaiting for input...\n\n");
}

void loop() {
  static uint16_t cnt;
  cnt++;
  String mode = Serial.readString();
  if (mode.startsWith("manual")) {
    blink(1);
    DPRINTF(1, "Entering manual mode\n");
    manual();
  }
  if (digitalRead(starSwitch) == 0) {
    blink(2);
    DPRINTF(1, "*** BUILDING A NEW STAR ***\n");
    star();
    DPRINTF(1, "*** A NEW STAR IS BORN ***\n\nWaiting for input...\n\n");
    blink(10, 0);
  }
  if (mode.startsWith("cube")) {
    blink(3);
    DPRINTF(1, "Executing CUBE\n");
    cube();
  }
  if (mode.startsWith("stand")) {
    blink(4);
    DPRINTF(1, "Executing STAND\n");
    stand();
  }
}

void blink(uint16_t count, uint16_t ms) {
  DPRINTF(0, "Blink %i times, ms = %i\n", count, ms);
  while (count > 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    count--;
  }
}

void star() {
  while (count != 5) {
    DPRINTF(1, "\nRUN(%i)..\n", count + 1);

    int feed = 38;  //  mm
    DPRINTF(1, "Set feeder length to: %imm\n", feed);

    int feedDistance = feed * 48;  // 48- constats that map the mm value to number of steps the stepper show move

    DPRINTF(1, "Run Feeder Stepper until it reaches feeder lenght: %imm\n", feed);
    while (feederStepper.currentPosition() != feedDistance) {  // run until it reaches the distance value
      feederStepper.setSpeed(1200);
      feederStepper.run();
    }
    feederStepper.setCurrentPosition(0);  // reset the current position to 0
    DPRINTF(1, "Feeder Stepper Reset to current position\n");

    DPRINTF(1, "Bender pin UP\n");
    benderPinServo.write(40);  // Set the bender pin up
    delay(200);
    int angleConst = 18;  // angle constant

    // Bend the wire 52 degrees
    DPRINTF(1, "Bend wire 52°\n");
    while (benderStepper.currentPosition() != -52 * angleConst) {
      benderStepper.setSpeed(-700);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);

    // Go back 52 degrees to initial position
    DPRINTF(1, "Go back to initial position: -52°\n");
    while (benderStepper.currentPosition() != 52 * angleConst) {
      benderStepper.setSpeed(1200);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);

    // Feed the same distance again
    DPRINTF(1, "Run Feeder Stepper again until it reaches feeder lenght: %imm\n", feed);
    while (feederStepper.currentPosition() != feedDistance) {
      feederStepper.setSpeed(1200);
      feederStepper.run();
    }
    feederStepper.setCurrentPosition(0);
    delay(100);

    DPRINTF(1, "Bender pin DOWN\n");
    benderPinServo.write(130);  // Set the bender pin down
    delay(200);

    // Set bender to new initial position, for bending in the other direction
    DPRINTF(1, "Move Bender Stepper to new position: -42°\n");
    while (benderStepper.currentPosition() != -42 * angleConst) {
      benderStepper.setSpeed(-1200);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(200);

    DPRINTF(1, "Bender pin UP\n");
    benderPinServo.write(40);  // Bender pin up
    delay(200);

    DPRINTF(1, "Bend wire 105°\n");
    while (benderStepper.currentPosition() != 105 * angleConst) {
      benderStepper.setSpeed(700);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(50);

    // Set bender to new initial position, for bending in the other direction
    DPRINTF(1, "Move Bender Stepper to new position: -63°\n");
    while (benderStepper.currentPosition() != -63 * angleConst) {
      benderStepper.setSpeed(-1200);
      benderStepper.run();
    }
    delay(100);

    DPRINTF(1, "Bender pin DOWN\n");
    benderPinServo.write(130);
    benderStepper.setCurrentPosition(0);
    count++;
  }
  count = 0;
}

void cube() {
  int feed = 40;  //  mm
  int feedDistance = feed * 48;
  int angleConst = 16;
  // Step 1
  while (count != 3) {
    while (feederStepper.currentPosition() != feedDistance) {
      feederStepper.setSpeed(1200);
      feederStepper.run();
    }
    feederStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != -90 * angleConst) {
      benderStepper.setSpeed(-700);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != 90 * angleConst) {
      benderStepper.setSpeed(1200);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    count++;
  }
  count = 0;
  // Step 2
  while (zAxisStepper.currentPosition() != 88 * angleConst) {
    zAxisStepper.setSpeed(500);
    zAxisStepper.run();
  }
  zAxisStepper.setCurrentPosition(0);
  delay(100);
  // Step 3
  while (count != 2) {
    while (feederStepper.currentPosition() != feedDistance) {
      feederStepper.setSpeed(1200);
      feederStepper.run();
    }
    feederStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != -90 * angleConst) {
      benderStepper.setSpeed(-700);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != 90 * angleConst) {
      benderStepper.setSpeed(1200);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    count++;
  }
  count = 0;
  // Step 4
  while (zAxisStepper.currentPosition() != 85 * angleConst) {
    zAxisStepper.setSpeed(500);
    zAxisStepper.run();
  }
  zAxisStepper.setCurrentPosition(0);
  delay(100);
  // Step 5
  benderPinServo.write(130);
  delay(200);
  while (benderStepper.currentPosition() != -42 * angleConst) {
    benderStepper.setSpeed(-1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  while (count != 3) {
    delay(100);
    benderPinServo.write(40);
    delay(200);
    // Step 6
    while (feederStepper.currentPosition() != feedDistance) {
      feederStepper.setSpeed(1200);
      feederStepper.run();
    }
    feederStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != 90 * angleConst) {
      benderStepper.setSpeed(700);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != -90 * angleConst) {
      benderStepper.setSpeed(-1200);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    count++;
  }
  count = 0;
}

void stand() {
  int feed = 20;  // mm
  int feedDistance = feed * 48;
  int angleConst = 16;
  // Step 1
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != -90 * angleConst) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 90 * angleConst) {
    benderStepper.setSpeed(1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);

  // Step 2
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != -70 * angleConst) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 70 * angleConst) {
    benderStepper.setSpeed(1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);

  // Step 3
  feed = 80;  // mm
  feedDistance = feed * 48;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  delay(100);
  // Step 4
  benderPinServo.write(130);
  delay(200);
  while (benderStepper.currentPosition() != -42 * angleConst) {
    benderStepper.setSpeed(-1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  benderPinServo.write(40);
  delay(200);
  while (benderStepper.currentPosition() != 108 * angleConst) {
    benderStepper.setSpeed(700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != -66 * angleConst) {
    benderStepper.setSpeed(-1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);

  // Step 5
  benderPinServo.write(130);
  delay(200);
  // Step 6
  feed = 80;  // mm
  feedDistance = feed * 48;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  benderPinServo.write(40);
  delay(200);
  // Step 7
  while (zAxisStepper.currentPosition() != -90 * angleConst) {
    zAxisStepper.setSpeed(-500);
    zAxisStepper.run();
  }
  zAxisStepper.setCurrentPosition(0);
  delay(100);

  // Step 8
  while (benderStepper.currentPosition() != -90 * angleConst) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 90 * angleConst) {
    benderStepper.setSpeed(1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  // Step 6
  feed = 45;  // mm
  feedDistance = feed * 48;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  // Step 10
  while (benderStepper.currentPosition() != -90 * angleConst) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 48 * angleConst) {
    benderStepper.setSpeed(1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);

  // Step 11
  while (zAxisStepper.currentPosition() != 90 * angleConst) {
    zAxisStepper.setSpeed(500);
    zAxisStepper.run();
  }
  zAxisStepper.setCurrentPosition(0);
  delay(100);
  feed = 80;  // mm
  feedDistance = feed * 48;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);

  // Step 12
  while (benderStepper.currentPosition() != 110 * angleConst) {
    benderStepper.setSpeed(700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != -68 * angleConst) {
    benderStepper.setSpeed(-1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  // Step 13
  benderPinServo.write(130);
  delay(200);
  feed = 80;  // mm
  feedDistance = feed * 48;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  benderPinServo.write(40);
  delay(200);

  // Step 14
  while (benderStepper.currentPosition() != -70 * angleConst) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 70 * angleConst) {
    benderStepper.setSpeed(1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);

  // Step 15
  feed = 25;  // mm
  feedDistance = feed * 48;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  delay(100);
  // Step 16
  while (benderStepper.currentPosition() != -90 * angleConst) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 90 * angleConst) {
    benderStepper.setSpeed(1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);

  // Step 17
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
}

void manual() {
  int sign;
  String dataInS;
  int angle;
  int angleConst;
  Serial.println("  // MANUAL MODE //");
  while (!dataIn.startsWith("end")) {
    benderPinServo.write(130);
    delay(200);
    dataIn = Serial.readString();
    if (dataIn.startsWith("f")) {
      dataInS = dataIn.substring(1, dataIn.length());  // reads the feed value
      dist = dataInS.toInt();
      Serial.print("Feed ");
      Serial.print(dist);
      Serial.println("mm wire.");
      dist = dist * 48;
      while (feederStepper.currentPosition() != dist) {
        feederStepper.setSpeed(1200);
        feederStepper.run();
      }
      feederStepper.setCurrentPosition(0);
      delay(100);
    }
    if (dataIn.startsWith("b")) {
      if (dataIn.charAt(1) == '-') {
        dataInS = dataIn.substring(2, dataIn.length());  /// reads the angle value
        angle = dataInS.toInt();
        Serial.print("Bend -");
        Serial.print(angle);
        Serial.println(" degrees.");
        angleConst = 16;
        // Set "negative" bending initial position
        while (benderStepper.currentPosition() != -43 * angleConst) {
          benderStepper.setSpeed(-1200);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
        benderPinServo.write(40);
        delay(200);
        // Bend the wire
        while (benderStepper.currentPosition() != angle * angleConst) {
          benderStepper.setSpeed(700);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
        while (benderStepper.currentPosition() != (-1) * angle * angleConst) {
          benderStepper.setSpeed(-1200);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
        benderPinServo.write(130);
        delay(200);
        // Get back to original "positive" bending initial poistion
        while (benderStepper.currentPosition() != 43 * angleConst) {
          benderStepper.setSpeed(1200);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
      } else {
        dataInS = dataIn.substring(1, dataIn.length());
        angle = dataInS.toInt();
        Serial.print("Bend ");
        Serial.print(angle);
        Serial.println(" degrees.");
        angleConst = 16;
        benderPinServo.write(40);
        delay(200);
        while (benderStepper.currentPosition() != (-1) * angle * angleConst) {
          benderStepper.setSpeed(-700);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
        while (benderStepper.currentPosition() != angle * angleConst) {
          benderStepper.setSpeed(1200);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
      }
      dataInS = dataIn.substring(2, dataIn.length());
      angle = dataInS.toInt();
      angleConst = 16;
      while (benderStepper.currentPosition() != sign * angle * angleConst) {
        benderStepper.setSpeed(-700);
        benderStepper.run();
      }
      benderStepper.setCurrentPosition(0);
      delay(100);
      while (benderStepper.currentPosition() != sign * angle * angleConst) {
        benderStepper.setSpeed(1200);
        benderStepper.run();
      }
      benderStepper.setCurrentPosition(0);
      delay(100);
    }
    // Z-Axis Control
    if (dataIn.startsWith("z")) {
      if (dataIn.charAt(1) == '-') {
        dataInS = dataIn.substring(2, dataIn.length());
        angle = dataInS.toInt();
        Serial.print("Move Z-Axis -");
        Serial.print(angle);
        Serial.println(" degrees.");
        angleConst = 16;
        while (zAxisStepper.currentPosition() != angle * angleConst) {
          zAxisStepper.setSpeed(500);
          zAxisStepper.run();
        }
        zAxisStepper.setCurrentPosition(0);
        delay(100);
      } else {
        dataInS = dataIn.substring(1, dataIn.length());
        angle = dataInS.toInt();
        Serial.print("Move Z-Axis ");
        Serial.print(angle);
        Serial.println(" degrees.");
        angleConst = 16;
        while (zAxisStepper.currentPosition() != (-1) * angle * angleConst) {
          zAxisStepper.setSpeed(-500);
          zAxisStepper.run();
        }
        zAxisStepper.setCurrentPosition(0);
        delay(100);
      }
    }
    manualStatus = dataIn;
  }
}
