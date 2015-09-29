#ifndef STATE_EV
#define STATE_EV

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "State.h"

class StateEvade: public State{

 public:

  StateEvade();

  void Enter();
  void Execute(StateManager * fsm);
  void Exit();
  
  State * Transition(bool* stimuli);

  void Print();
  
  std::string GetNameString();
 
  private:

  string name;

  float deltaT;
  time_t timeStamp;
  bool timerExpired;

  bool first;
};
#endif
