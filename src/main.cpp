#include <AccelStepper.h>
#include <Arduino.h>
#include <Servo.h>
#include <dprintf.h>

#define benderPinA 2          // Arduino Pin #
#define limitSwitch 11        // Arduino Pin #
#define benderPinUpPos 45     // Up position in degrees
#define benderPinDownPos 135  // Down position in degrees
#define starSwitch A0         // Arduino Pin # Use Analog pin as digital input since it's in the front
#define starSwitchGND A2      // Arduino StarSwitch GND

long stepsPerKeyPress = 100;  // Aantal stappen per manuele toetsaanslag

float benderStartPos = -365;   // Start position of the bender pin
float feederPrecision = 12;    // constant that maps the mm value to number of steps the stepper has to move
float anglePrecision = 5.769;  // constant that maps the angle degree to the number ot steps the stepper has to move
bool UP = true;
bool DOWN = false;

// Define DPRINTF macro
StreamEx mySerial = Serial;  // Declare mySerial to enable ioDebug mySerial.printf

// Define the stepper motors and the pins the motor will use
AccelStepper feederStepper(AccelStepper::DRIVER, 5, 6);    // Feeder: STEP on pin 5, DIR on pin 6
AccelStepper rotationStepper(AccelStepper::DRIVER, 7, 8);  // Bender: STEP on pin 7, DIR on pin 8
AccelStepper benderStepper(AccelStepper::DRIVER, 9, 10);   // Rotation: STEP on pin 9, DIR on pin 10

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
void rotate(AccelStepper &stepper, float steps, float speed = 1200, unsigned long rotateDelay = 100, bool limitDetection = true);
void benderPin(bool up, int msDelay = 400);
void (*resetFunc)(void) = 0;  // create a standard reset function

void setup() {
  Serial.begin(BAUDRATE);
  DPRINTF(1, "Initializing...\n");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(starSwitchGND, OUTPUT);     // Set Analog input to digital zero
  pinMode(starSwitch, INPUT_PULLUP);  // Set Analog input to digital

  benderPinServo.attach(benderPinA);  // Init Bender pin
  benderPin(DOWN);                    // Set Bender pin down

  // Stepper motors max speed
  feederStepper.setMaxSpeed(2000);
  rotationStepper.setMaxSpeed(2000);
  benderStepper.setMaxSpeed(2000);

  // Homing
  DPRINTF(1, "Homing: %i\n", digitalRead(limitSwitch));
  rotate(benderStepper, 700, 500);

  // Move bendigStepper to initial position
  rotate(benderStepper, benderStartPos, -1200, 100, false);  // disable limit switch (0)

  benderPin(UP);  // Bending pin UP

  DPRINTF(1, "Done..\n");
  delay(40);

  DPRINTF(1, "\nGebruik pijltjes-, pgUP- en pgDOWN-toetsen voor bediening:");
  DPRINTF(1, "\nRechts/Links: Feeder | Omhoog/Omlaag: Bender | pgUP/pgDOWN: Rotation\n");
  delay(40);
}

void loop() {
  int starBtnPressed;
  starBtnPressed = (digitalRead(starSwitch) == 0);

  if (Serial.available() || starBtnPressed) {  // Controleer of er data beschikbaar is op de seriële poort of dat de sterknop is ingedrukt
    if (starBtnPressed)
      star();
    else {
      char input = Serial.read();  // Lees het ingevoerde teken
      switch (input) {
        case '6':  // Pijltje rechts (Feeder rechtsom)
          rotate(feederStepper, stepsPerKeyPress);
          break;
        case '4':  // Pijltje links (Feeder linksom)
          rotate(feederStepper, -stepsPerKeyPress);
          break;
        case '8':  // Pijltje omhoog (Bender rechtsom)
          rotate(benderStepper, stepsPerKeyPress, 1200, 100, false);
          break;
        case '2':  // Pijltje omlaag (Bender linksom)
          rotate(benderStepper, -stepsPerKeyPress, -1200, 100, false);
          break;
        case '9':  // Toets 'PgUp' (Rotation linksom)
          rotate(rotationStepper, stepsPerKeyPress);
          break;
        case '3':  // Toets 'pgDn' (Rotation rechtsom)
          rotate(rotationStepper, -stepsPerKeyPress);
          break;
        case '+':  // Toets '+' (RESET)
          DPRINTF(1, "\n *** RESET ***");
          resetFunc();
          break;
        case '-':  // Toets '-' (Rotation rechtsom)
          DPRINTF(1, "\n *** NOT IMPLEMENTED ***");
          break;
        case '*':  // Toets '*' (Ster maken)
          star();
          break;
        default:
          DPRINTF(1, "\nOngeldige invoer, gebruik de nummers:\n 6(Feeder >), 4(Feeder <), 8(Bender UP), 2(Bender DOWN), 9(Rotate UP), 3(Rotate DOWN), *(STER), +(RESET)");
          break;
      }
    }
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

void benderPin(bool up, int msDelay) {
  DPRINTF(1, "Bender pin %s\n", up ? "UP" : "DOWN");
  benderPinServo.write(up ? benderPinUpPos : benderPinDownPos);  // Bender pin up
  delay(msDelay);
}

void rotate(AccelStepper &stepper, float steps, float speed, unsigned long rotateDelay, bool limitDetection) {
  DPRINTF(1, " rotate %s, %.2f steps\n", steps > 0 ? "rechtsom" : "linksom", steps);

  stepper.move(steps);
  if (steps < 0 && speed > 0) {
    DPRINTF(2, "WARNING: steps is negative (%.2f). Speed (%.2f) should also be negative!\nSetting speed to negative: ", steps, speed);
    speed = -speed;
    DPRINTF(2, "%.2f\n", speed);
  }

  if (steps > 0 && speed < 0) {
    DPRINTF(2, "WARNING: steps is positive (%.2f). Speed (%.2f) should also be positive!\nSetting speed to positive: ", steps, speed);
    speed = speed * -1;
    DPRINTF(2, "%.2f\n", speed);
  }

  while (stepper.distanceToGo() != 0) {  // run until it reaches the distance value
    // DPRINTF(1, "CurrentPos = %ld, steps = %ld\n", stepper.distanceToGo(), steps);
    if (digitalRead(limitSwitch) == 0) {
      DPRINTF(2, "WARNING: Bender limit switch active!\n");
      if (limitDetection) {
        DPRINTF(2, "Limit detection is ON. Exit rotate...\n");
        break;
      }
    }
    stepper.setSpeed(speed);
    stepper.run();  // This will move the motor one step towards the target
  }
  stepper.setCurrentPosition(0);
  delay(rotateDelay);
}

void star() {
  DPRINTF(1, "\n Create a STAR..\n");
  while (count != 1) {
    DPRINTF(1, "RUN(%i)..\n", count + 1);

    int feed = 90;  //  mm
    DPRINTF(1, "Set feeder length to: %imm\n", feed);
    int feedDistance = feed * feederPrecision;

    DPRINTF(1, "Run Feeder Stepper until it reaches feeder lenght: %imm\n", feed);
    rotate(feederStepper, feedDistance);

    DPRINTF(1, "Bender pin UP\n");
    benderPin(UP);  // Set the bender pin up

    // Bend the wire 52 degrees
    DPRINTF(1, "Bend wire 52°\n");
    rotate(benderStepper, -55 * anglePrecision, -700, 0);

    // Go back 52 degrees to initial position
    DPRINTF(1, "Go back to initial position: -52°\n");
    rotate(benderStepper, 55 * anglePrecision);

    // Feed the same distance again
    DPRINTF(1, "Run Feeder Stepper again until it reaches feeder lenght: %imm\n", feed);
    rotate(feederStepper, feedDistance);

    DPRINTF(1, "Bender pin DOWN\n");
    benderPin(DOWN);  // Set the bender pin down

    // Set bender to new initial position, for bending in the other direction
    DPRINTF(1, "Move Bender Stepper to new position: -42°\n");
    rotate(benderStepper, -42 * anglePrecision);

    benderPin(UP);  // Bender pin up

    DPRINTF(1, "Bend wire 105°\n");
    rotate(benderStepper, 105 * anglePrecision, 700);

    // Set bender to new initial position, for bending in the other direction
    DPRINTF(1, "Move Bender to initial start position\n");
    rotate(benderStepper, benderStartPos, 1200, 0, false);
    benderPin(DOWN);
    DPRINTF(1, "RUN(%i) FINISHED.\n", count + 1);
    count++;
  }
  count = 0;
}

void cube() {
  int feed = 40;  //  mm
  int feedDistance = feed * feederPrecision;
  // Step 1
  while (count != 3) {
    while (feederStepper.currentPosition() != feedDistance) {
      feederStepper.setSpeed(1200);
      feederStepper.run();
    }
    feederStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != -90 * anglePrecision) {
      benderStepper.setSpeed(-700);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != 90 * anglePrecision) {
      benderStepper.setSpeed(1200);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    count++;
  }
  count = 0;
  // Step 2
  while (rotationStepper.currentPosition() != 88 * anglePrecision) {
    rotationStepper.setSpeed(500);
    rotationStepper.run();
  }
  rotationStepper.setCurrentPosition(0);
  delay(100);
  // Step 3
  while (count != 2) {
    while (feederStepper.currentPosition() != feedDistance) {
      feederStepper.setSpeed(1200);
      feederStepper.run();
    }
    feederStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != -90 * anglePrecision) {
      benderStepper.setSpeed(-700);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != 90 * anglePrecision) {
      benderStepper.setSpeed(1200);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    count++;
  }
  count = 0;
  // Step 4
  while (rotationStepper.currentPosition() != 85 * anglePrecision) {
    rotationStepper.setSpeed(500);
    rotationStepper.run();
  }
  rotationStepper.setCurrentPosition(0);
  delay(100);
  // Step 5
  benderPin(DOWN);
  while (benderStepper.currentPosition() != -42 * anglePrecision) {
    benderStepper.setSpeed(-1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  while (count != 3) {
    delay(100);
    benderPin(UP);
    // Step 6
    while (feederStepper.currentPosition() != feedDistance) {
      feederStepper.setSpeed(1200);
      feederStepper.run();
    }
    feederStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != 90 * anglePrecision) {
      benderStepper.setSpeed(700);
      benderStepper.run();
    }
    benderStepper.setCurrentPosition(0);
    delay(100);
    while (benderStepper.currentPosition() != -90 * anglePrecision) {
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
  int feedDistance = feed * feederPrecision;
  // Step 1
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != -90 * anglePrecision) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 90 * anglePrecision) {
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
  while (benderStepper.currentPosition() != -70 * anglePrecision) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 70 * anglePrecision) {
    benderStepper.setSpeed(1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);

  // Step 3
  feed = 80;  // mm
  feedDistance = feed * feederPrecision;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  delay(100);
  // Step 4
  benderPin(DOWN);
  while (benderStepper.currentPosition() != -42 * anglePrecision) {
    benderStepper.setSpeed(-1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  benderPin(UP);
  while (benderStepper.currentPosition() != 108 * anglePrecision) {
    benderStepper.setSpeed(700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != -66 * anglePrecision) {
    benderStepper.setSpeed(-1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);

  // Step 5
  benderPin(DOWN);
  // Step 6
  feed = 80;  // mm
  feedDistance = feed * feederPrecision;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  benderPin(UP);
  // Step 7
  while (rotationStepper.currentPosition() != -90 * anglePrecision) {
    rotationStepper.setSpeed(-500);
    rotationStepper.run();
  }
  rotationStepper.setCurrentPosition(0);
  delay(100);

  // Step 8
  while (benderStepper.currentPosition() != -90 * anglePrecision) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 90 * anglePrecision) {
    benderStepper.setSpeed(1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  // Step 6
  feed = 45;  // mm
  feedDistance = feed * feederPrecision;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  // Step 10
  while (benderStepper.currentPosition() != -90 * anglePrecision) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 48 * anglePrecision) {
    benderStepper.setSpeed(1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);

  // Step 11
  while (rotationStepper.currentPosition() != 90 * anglePrecision) {
    rotationStepper.setSpeed(500);
    rotationStepper.run();
  }
  rotationStepper.setCurrentPosition(0);
  delay(100);
  feed = 80;  // mm
  feedDistance = feed * feederPrecision;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);

  // Step 12
  while (benderStepper.currentPosition() != 110 * anglePrecision) {
    benderStepper.setSpeed(700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != -68 * anglePrecision) {
    benderStepper.setSpeed(-1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  // Step 13
  benderPin(DOWN);
  feed = 80;  // mm
  feedDistance = feed * feederPrecision;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  benderPin(UP);

  // Step 14
  while (benderStepper.currentPosition() != -70 * anglePrecision) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 70 * anglePrecision) {
    benderStepper.setSpeed(1200);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);

  // Step 15
  feed = 25;  // mm
  feedDistance = feed * feederPrecision;
  while (feederStepper.currentPosition() != feedDistance) {
    feederStepper.setSpeed(1200);
    feederStepper.run();
  }
  feederStepper.setCurrentPosition(0);
  delay(100);
  // Step 16
  while (benderStepper.currentPosition() != -90 * anglePrecision) {
    benderStepper.setSpeed(-700);
    benderStepper.run();
  }
  benderStepper.setCurrentPosition(0);
  delay(100);
  while (benderStepper.currentPosition() != 90 * anglePrecision) {
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
  Serial.println("  // MANUAL MODE //");
  while (!dataIn.startsWith("end")) {
    benderPin(DOWN);
    dataIn = Serial.readString();
    if (dataIn.startsWith("f")) {
      dataInS = dataIn.substring(1, dataIn.length());  // reads the feed value
      dist = dataInS.toInt();
      Serial.print("Feed ");
      Serial.print(dist);
      Serial.println("mm wire.");
      dist = dist * feederPrecision;
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
        // Set "negative" bending initial position
        while (benderStepper.currentPosition() != -43 * anglePrecision) {
          benderStepper.setSpeed(-1200);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
        benderPin(UP);
        // Bend the wire
        while (benderStepper.currentPosition() != angle * anglePrecision) {
          benderStepper.setSpeed(700);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
        while (benderStepper.currentPosition() != (-1) * angle * anglePrecision) {
          benderStepper.setSpeed(-1200);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
        benderPin(DOWN);
        // Get back to original "positive" bending initial poistion
        while (benderStepper.currentPosition() != 43 * anglePrecision) {
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
        benderPin(UP);
        while (benderStepper.currentPosition() != (-1) * angle * anglePrecision) {
          benderStepper.setSpeed(-700);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
        while (benderStepper.currentPosition() != angle * anglePrecision) {
          benderStepper.setSpeed(1200);
          benderStepper.run();
        }
        benderStepper.setCurrentPosition(0);
        delay(100);
      }
      dataInS = dataIn.substring(2, dataIn.length());
      angle = dataInS.toInt();
      while (benderStepper.currentPosition() != sign * angle * anglePrecision) {
        benderStepper.setSpeed(-700);
        benderStepper.run();
      }
      benderStepper.setCurrentPosition(0);
      delay(100);
      while (benderStepper.currentPosition() != sign * angle * anglePrecision) {
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
        while (rotationStepper.currentPosition() != angle * anglePrecision) {
          rotationStepper.setSpeed(500);
          rotationStepper.run();
        }
        rotationStepper.setCurrentPosition(0);
        delay(100);
      } else {
        dataInS = dataIn.substring(1, dataIn.length());
        angle = dataInS.toInt();
        Serial.print("Move Z-Axis ");
        Serial.print(angle);
        Serial.println(" degrees.");
        while (rotationStepper.currentPosition() != (-1) * angle * anglePrecision) {
          rotationStepper.setSpeed(-500);
          rotationStepper.run();
        }
        rotationStepper.setCurrentPosition(0);
        delay(100);
      }
    }
    manualStatus = dataIn;
  }
}
