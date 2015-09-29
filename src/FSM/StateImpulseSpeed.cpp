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

StateImpulseSpeed::StateImpulseSpeed(){
  name = "ImpulseSpeed"; 
  
  /* initialize random seed: */
  srand (time(NULL));
};

void StateImpulseSpeed::Enter(){};

void StateImpulseSpeed::Execute(StateManager * fsm){
  //printf("Executing behaviour %s...\n", name.c_str());

  // This speed is half the standard speed 
  fsm->SetTransSpeed(2);
  
  // This value should depend on formationHeadingError
  float trackingError = fsm->GetFormationHeadingError();
  float kp = fsm->GetProportionalGain();  
  
  fsm->SetRotSpeed(kp*trackingError);

};

void StateImpulseSpeed::Exit(){};

State * StateImpulseSpeed::Transition(bool* stimuli){

  SetStimuli(stimuli);

  if(friendAhead and (friendLeft or friendRight) ){
    return new StateCruise();
  }
  else if(friendAhead and not friendBehind ){
    return new StateCatchUp();
  }
  else if(not aligned ){
    return new StateAlign();
  }
  else if( frontProx ){
    return new StateEvade();
  }
  else if( friendBehind and not friendAhead){
    return new StateHalt();
  }
  else{
    return NULL;
  }

};

std::string StateImpulseSpeed::GetNameString(){
  return name;
};

void StateImpulseSpeed::Print(){
  printf("%s\n",name.c_str());
};
