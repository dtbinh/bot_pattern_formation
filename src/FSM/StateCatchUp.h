#ifndef STATE_CU
#define STATE_CU

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "State.h"


class StateCatchUp: public State{

 public:

  StateCatchUp();

  void Enter();
  void Execute(StateManager * fsm);
  void Exit();
  
  State * Transition(bool* stimuli);

  void Print();
  
  std::string GetNameString();
 
 private:

  std::string name;

};
#endif
