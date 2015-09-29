#ifndef FSM_H
#define FSM_H

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>


// Abstract base class for state
#include "State.h"

// State classes
#include "State.h"

using namespace std;

class StateManager{

public:

  StateManager();

  void UpdateBehaviour(bool* stimuli);

  void ExecuteBehaviour(float& trans, float& rot, bool& servoOpen);
 
  void PrintCurrentState();
  string GetCurrentStateName();

  float GetProportionalGain();

  void SetMagneticHeadingError(float value);
  void SetFormationHeadingError(float value);

  float GetMagneticHeadingError();
  float GetFormationHeadingError();

  void SetRotSpeed(float speed);
  void SetTransSpeed(int speed);
  
  void CloseServo();
  void OpenServo();

  bool MovingForward();

private:
  
  State * currentState;

  float trans_speed;
  float rot_speed;
  bool openServo;

  float magneticHeadingError;
  float formationHeadingError;

  float kp;
};
#endif
