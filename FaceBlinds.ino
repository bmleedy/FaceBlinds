// Loop Speed
#define LOOP_DELAY_MS 50

// motors
#define MOTOR_ENA 9  // Yellow: 5 line bundle
#define MOTOR_IN1 7  // White: 5 line bundle
#define MOTOR_IN2 6  // Blue: 5 line bundle

// Analog Sensors
#define UP_HALL_SENSOR_PIN   A2
#define UP_HALL_SENSOR_UPPER_BOUND   535
#define UP_HALL_SENSOR_LOWER_BOUND   0

#define DOWN_HALL_SENSOR_PIN A3
#define DOWN_HALL_SENSOR_UPPER_BOUND   10000
#define DOWN_HALL_SENSOR_LOWER_BOUND   510

#define LIGHT_SENSOR_PIN     A5
#define LIGHT_SENSOR_UPPER_BOUND   10000
#define LIGHT_SENSOR_LOWER_BOUND   0

// control buttons (digital input)
#define DROP_BUTTON_PIN  11
#define LIFT_BUTTON_PIN  12
#define STOP_BUTTON_PIN  8


class AnalogSensor {
 private:
  int16_t upper_bound;
  int16_t lower_bound;
  int16_t pin;

 public:
  AnalogSensor(int16_t upper_bound, int16_t lower_bound, int16_t pin) {
    pinMode(pin, INPUT);
    this->upper_bound = upper_bound;
    this->lower_bound = lower_bound;
    this->pin = pin;
  }

  bool is_tripped() {
    return is_tripped_high() || is_tripped_low();
  }

  bool is_tripped_high() {
    int16_t val = this->value();
    return (val > this->upper_bound);
  }

  bool is_tripped_low() {
    int16_t val = this->value();
    return (val < this->lower_bound);
  }

  int16_t value() {
    return analogRead(pin);
  }
};


class InputIO {
 public:
  AnalogSensor * upSensor;
  AnalogSensor * downSensor;
  AnalogSensor * lightSensor;
  int16_t upButtonValue;
  int16_t downButtonValue;
  int16_t stopButtonValue;

  InputIO() {
    // Analog input pins
    upSensor = new AnalogSensor(UP_HALL_SENSOR_UPPER_BOUND,
                                UP_HALL_SENSOR_LOWER_BOUND,
                                UP_HALL_SENSOR_PIN);
    downSensor = new AnalogSensor(DOWN_HALL_SENSOR_UPPER_BOUND,
                                  DOWN_HALL_SENSOR_LOWER_BOUND,
                                  DOWN_HALL_SENSOR_PIN);
    lightSensor = new AnalogSensor(LIGHT_SENSOR_UPPER_BOUND,
                                   LIGHT_SENSOR_LOWER_BOUND,
                                   LIGHT_SENSOR_PIN);

    // Digital input pins
    pinMode(LIFT_BUTTON_PIN, INPUT_PULLUP);
    pinMode(DROP_BUTTON_PIN, INPUT_PULLUP);
    pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);

    this->update_values();
  }

  void update_values() {
    // digital input
    upButtonValue = digitalRead(LIFT_BUTTON_PIN);
    downButtonValue = digitalRead(DROP_BUTTON_PIN);
    stopButtonValue = digitalRead(STOP_BUTTON_PIN);
  }

  void dump() {
    Serial.print(F("upSensor: "));
    Serial.print(upSensor->value());
    Serial.print(F(" downSensorValue: "));
    Serial.print(downSensor->value());
    Serial.print(F(" lightSensor: "));
    Serial.print(lightSensor->value());
    Serial.print(F(" upButton: "));
    Serial.print(upButtonValue);
    Serial.print(F(" downButton: "));
    Serial.print(downButtonValue);
    Serial.print(F(" stopButton: "));
    Serial.print(stopButtonValue);
    Serial.println("");
  }
};

InputIO * io_pins;


enum state {
  // Non moving states
  TOP,
  BOTTOM,
  STOPPED_MIDDLE,

  // Moving states
  LIFTING,
  DROPPING
};

enum inputCommand {
  LIFT,
  DROP,
  STOP,
  NONE  // Default
};

inputCommand getLatestInput(InputIO * input_io) {
  // switch to getLatestInputWIFI() when ready
  return getLatestInputButton(input_io);
}

inputCommand getLatestInputButton(InputIO * input_io) {
//  Serial.println("signal");
  if (input_io->stopButtonValue == LOW) {
//      Serial.println("button stop");
      return STOP;
  } else if (input_io->downButtonValue == LOW) {
//      Serial.println("button drop");
      return DROP;
  } else if (input_io->upButtonValue == LOW) {
//      Serial.println("button lift");
      return LIFT;
  } else {
    return NONE;
  }
}

inputCommand getLatestInputWIFI() {
  return NONE;
}


void outputBasedOnState(state currentState) {
  if (currentState == LIFTING) {
    Serial.println("lifting");
    setMotorToLift();
  } else if (currentState == DROPPING) {
    Serial.println("dropping");
    setMotorToDrop();
  } else {
    Serial.println("stopped");
    stopMotor();
  }
}

state getNewState(state currentState,
                  InputIO * input_io,
                  inputCommand latestInput) {
  switch (currentState) {
    case TOP:
      if (latestInput == DROP) {
        return DROPPING;
      } else {
        return TOP;
      }
      break;
    case BOTTOM:
      if (latestInput == LIFT) {
        return LIFTING;
      } else {
        return BOTTOM;
      }
      break;
    case LIFTING:
      if (input_io->upSensor->is_tripped()) {
        return TOP;
      } else if (latestInput == STOP) {
        return STOPPED_MIDDLE;
      } else if (latestInput == DROP) {
        return DROPPING;
      } else {
        return LIFTING;
      }
      break;
    case DROPPING:
      if (input_io->downSensor->is_tripped()) {
        return BOTTOM;
      } else if (latestInput == STOP) {
        return STOPPED_MIDDLE;
      } else if (latestInput == LIFT) {
        return LIFTING;
      } else {
        return DROPPING;
      }
      break;
    case STOPPED_MIDDLE:
      if (latestInput == LIFT) {
        return LIFTING;
      } else if (latestInput == DROP) {
        return DROPPING;
      } else {
        return STOPPED_MIDDLE;
      }
      break;
  }
}

// control motor
void stopMotor() {
//  Serial.println("stop");
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
}

void setMotorToLift() {
//  Serial.println("lift");
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  // Setting the power level at 125 out of 255
  analogWrite(MOTOR_ENA, 255);
}

void setMotorToDrop() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  // Setting the power level (out of 255)
  analogWrite(MOTOR_ENA, 255);
}

// setup

state currentState = STOPPED_MIDDLE;

void setup_motor_controller() {
  // we have to set PWM pin as output
  pinMode(MOTOR_ENA, OUTPUT);

  // Logic pins are also set as output
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  // CLOCKWISE is DROPPING
  // COUNTER-CLOCKWISE is LIFTING
  stopMotor();
}

void setup() {
  Serial.begin(115200);      // setup serial comms (debugging)

  setup_motor_controller();  // setup outputs

  io_pins = new InputIO();   // setup inputs
}

void loop() {
  outputBasedOnState(currentState);

  io_pins->update_values();
  io_pins->dump();

  inputCommand latestInput = getLatestInput(io_pins);  // wifi or button

  currentState = getNewState(currentState, io_pins, latestInput);

  delay(LOOP_DELAY_MS);
}
