#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoMotors.h>
#include <ZumoReflectanceSensorArray.h>
#include <LinkedList.h>

#define NUM_SENSORS 6
unsigned int sensor_values[NUM_SENSORS];

Pushbutton button(ZUMO_BUTTON);
ZumoMotors motors;
ZumoReflectanceSensorArray sensors;

const char *TEST_STRING = "LINUSJONTE";

struct HalNine;

struct MoveDesc {
  uint32_t (*func)(struct HalNine*);
};

struct HalNine {
  float power;
  float turn;

  uint32_t leftInMove;
  uint32_t lastUpdated;

  LinkedList<MoveDesc> moveQueue;
};

/**** SET TIMEOUT ****/

struct SetTimeoutItem {
  uint32_t pit;
  void (*func)(void *);
  void *userData;
};

LinkedList<SetTimeoutItem> _setTimeoutQueue;

void setTimeout (void (*func)(void *), uint32_t delay, void *userData) {
  SetTimeoutItem theItem;

  theItem.pit = millis() + delay;
  theItem.func = func;
  theItem.userData = userData;
  
  _setTimeoutQueue.add(theItem);
}

void setTimeout (void (*func)(void *), uint32_t delay) {
  setTimeout(func, delay, NULL);
}

void triggerSetTimeout () {
  int theTime = millis();
  int theSize = _setTimeoutQueue.size();

  for (int i = 0; i < theSize; i++) {
    Serial.println("SET TIMEOUT LOOP");
    SetTimeoutItem theItem = _setTimeoutQueue.get(i);

    if (theItem.pit <= theTime) {
      Serial.println("SET TIMEOUT TRIGGER");
      theItem.func(theItem.userData);
      _setTimeoutQueue.remove(i);
      theSize--;
      i--;
    }
  }
}







uint32_t rnd (uint32_t _max) {
  return (millis() % _max);
}









/**** CONSOLE LOG ****/
void consoleLog(void *userData) {
  char *output = (char *) userData;

  Serial.print("> ");
  Serial.println(output);
}





uint32_t move_turn90left (struct HalNine *robot) {
  robot->power = 1;
  robot->turn = -0.5;

  return 350 + rnd(200);
}

uint32_t move_turn90right (struct HalNine *robot) {
  robot->power = 1;
  robot->turn = 0.5;

  return 350 + rnd(200);
}

uint32_t move_aBitForward (struct HalNine *robot) {
  robot->power = 1;
  robot->turn = 0;

  return 350 + rnd(200);
}

uint32_t move_FullReverse (struct HalNine *robot) {
  robot->power = -1;
  robot->turn = 0;

  return 250;
}

uint32_t move_SpinReverseLeft90 (struct HalNine *robot) {
  robot->power = -1;
  robot->turn = -1;

  return 320;
}

uint32_t move_SpinForwardRight90 (struct HalNine *robot) {
  robot->power = 1;
  robot->turn = 1;

  return 320;
}







// Forward declaration
void HalNine_enqueueMove (struct HalNine *robot, uint32_t (*func)(struct HalNine*));


void doA_SBendLeft (struct HalNine *robot) {
  HalNine_enqueueMove(robot, move_turn90left);
  HalNine_enqueueMove(robot, move_turn90right);
}

void doA_SBendRight (struct HalNine *robot) {
  HalNine_enqueueMove(robot, move_turn90right);
  HalNine_enqueueMove(robot, move_turn90left);
}

void doA_TurnAroundLeft (struct HalNine *robot) {
  HalNine_enqueueMove(robot, move_turn90left);
  HalNine_enqueueMove(robot, move_turn90left);
}

void doA_TurnAroundRight (struct HalNine *robot) {
  HalNine_enqueueMove(robot, move_turn90right);
  HalNine_enqueueMove(robot, move_turn90right);
}

void doA_OneEighty (struct HalNine *robot) {
  HalNine_enqueueMove(robot, move_FullReverse);
  HalNine_enqueueMove(robot, move_SpinReverseLeft90);
  HalNine_enqueueMove(robot, move_SpinForwardRight90);
}

















/**** HAL9000 IMPLEMENTATION ****/

void HalNine_doRandom (struct HalNine *robot) {
  uint32_t daMove = (millis() % 4);

  switch (daMove) {
    case 0: doA_SBendLeft(robot); break;
    case 1: doA_SBendRight(robot); break;
    case 2: doA_TurnAroundLeft(robot); break;
    case 3: doA_TurnAroundRight(robot); break;
  }
}

void HalNine_step (struct HalNine *robot) {
  uint32_t now = millis();
  uint32_t delta = now - robot->lastUpdated;

  robot->lastUpdated = now;

  if (robot->leftInMove > delta) {
    robot->leftInMove -= delta;
  } else {
    if (robot->moveQueue.size() == 0) {
      HalNine_doRandom(robot);
    }
    
    MoveDesc nextMove = robot->moveQueue.shift();
    robot->leftInMove = nextMove.func(robot);
  }

  motors.setSpeeds(
    (1 + robot->turn) * robot->power * 400,
    (1 - robot->turn) * robot->power * 400
  );
}

void HalNine_haltAndCatchFire (struct HalNine *robot) {
  robot->power = 0;
  motors.setSpeeds(0, 0);
}

void HalNine_init (struct HalNine *robot) {
  
  robot->power = 0;
  robot->turn = 0;
  robot->leftInMove = 0;
  robot->lastUpdated = millis();

}

void HalNine_enqueueMove (struct HalNine *robot, uint32_t (*func)(struct HalNine*)) {
  MoveDesc desc;

  desc.func = func;
  
  robot->moveQueue.add(desc);
}

void HalNine_abortAndClear (struct HalNine *robot) {
  HalNine_haltAndCatchFire(robot);
  robot->leftInMove = 0;
  robot->moveQueue.clear();
}









/**** ARDUINO GLUE ****/
HalNine globalRobot;

bool edgeDebouncing = false;

void resetDebouncing (void *userData) {
  edgeDebouncing = false;
}

void onEdge () {
//  if (edgeDebouncing) return;
//  edgeDebouncing = true;
  
  HalNine_abortAndClear(&globalRobot);

  motors.setSpeeds(-200, -200);
  delay(150);
  motors.setSpeeds(400, -400);
  delay(200);
  motors.setSpeeds(0, 0);

  HalNine_doRandom(&globalRobot);
  
//  doA_OneEighty(&globalRobot);

//  setTimeout(resetDebouncing, 150);
}


void setup() {
  Serial.begin(9600);
  
  sensors.init();
  motors.setSpeeds(0, 0);

  // Wait for button press
  button.waitForButton();

  // Delay 5 seconds
  delay(5000);

  HalNine_init(&globalRobot);
}

void loop() {
  sensors.read(sensor_values);

  HalNine_step(&globalRobot);
  triggerSetTimeout();

  if (sensor_values[0] < 1200 || sensor_values[5] < 1200) {
    onEdge();
  }
}

