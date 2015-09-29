#ifndef STATE_CR
#define STATE_CR

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "State.h"

class StateCruise: public State{

 public:

  StateCruise();

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
