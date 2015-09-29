#ifndef STATE_AL
#define STATE_AL

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "State.h"

class StateAlign: public State{

 public:

  StateAlign();

  void Enter();
  void Execute(StateManager * fsm);
  void Exit();
  
  State * Transition(bool* stimuli);

  void Print();
  
  std::string GetNameString();
 
  private:

  string name;

};
#endif
