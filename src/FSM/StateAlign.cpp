#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "StateImpulseSpeed.h"
#include "StateCruise.h"
#include "StateCatchUp.h"
#include "StateAlign.h"
#include "StateEvade.h"
#include "StateHalt.h"

#include "State.h"

#include "FSM.h"

StateAlign::StateAlign(){
name = "Align"; 

/* initialize random seed: */
 srand (time(NULL));
};

void StateAlign::Enter(){};

void StateAlign::Execute(StateManager * fsm){
  //printf("Executing behaviour %s...\n", name.c_str());

  fsm->SetTransSpeed(0);
  
  // This value should depend on magneticHeadingError
  float trackingError = fsm->GetMagneticHeadingError();
  float kp = fsm->GetProportionalGain();  
  
  fsm->SetRotSpeed(kp*trackingError);

};

void StateAlign::Exit(){};

State * StateAlign::Transition(bool* stimuli){

  SetStimuli(stimuli);

  if(not aligned){
    return NULL;
  }
  else if(friendAhead and (friendLeft or friendRight) and not friendBehind ){
    return new StateCruise();
  }
  else if(friendAhead and not friendBehind ){
    return new StateCatchUp();
  }
  else if( frontProx ){
    return new StateEvade();
  }
  else if( friendBehind and not friendAhead){
    return new StateHalt();
  }
  else if (friendAhead and friendBehind){
    return new StateImpulseSpeed();
  }
  else{
    return NULL;
  }

};

std::string StateAlign::GetNameString(){
  return name;
};

void StateAlign::Print(){
  printf("%s\n",name.c_str());
};
