#ifndef STATE_IS
#define STATE_IS

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "State.h"

#include "FSM.h"

class StateImpulseSpeed: public State{

 public:

  StateImpulseSpeed();

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
