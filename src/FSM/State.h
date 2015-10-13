#ifndef STATE_AB
#define STATE_AB

#include <stdio.h>
#include <stdlib.h>
#include <string>

//#include "FSM.h"

class StateManager;

using namespace std;

class State{

 public:

  State(){};
  ~State(){};

  virtual void Enter(){};
  
  virtual void Execute(StateManager* fsm ){};
  
  virtual void Execute(){};
  virtual void Exit(){};
  
  virtual State * Transition(bool* stimuli){};

  virtual void Print(){};

  virtual void DetermineHeading(){};

  virtual string GetNameString(){};
  
  void SetStimuli(bool * stimuli){
    frontProx = stimuli[0];
    rearProx = stimuli[1];
    friendLeft = stimuli[2];
    friendRight = stimuli[3];
    friendAhead = stimuli[4];
    friendBehind = stimuli[5];
    aligned = stimuli[6];
  };

 protected:

  string name;
  
  bool frontProx;
  bool rearProx;
  bool friendLeft;
  bool friendRight;
  bool friendAhead;
  bool friendBehind;
  bool aligned;

};
#endif
