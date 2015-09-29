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

StateCatchUp::StateCatchUp(){
  name = "CatchUp"; 
  
  /* initialize random seed: */
  srand (time(NULL));
};

void StateCatchUp::Enter(){};

void StateCatchUp::Execute(StateManager * fsm){
  //printf("Executing behaviour %s...\n", name.c_str());

  // This speed is half the standard speed 
  fsm->SetTransSpeed(2);
  
  // This value should depend on formationHeadingError
  float trackingError = fsm->GetFormationHeadingError();
  float kp = fsm->GetProportionalGain();  
  
  fsm->SetRotSpeed(kp*trackingError);

};

void StateCatchUp::Exit(){};

State * StateCatchUp::Transition(bool* stimuli){

  SetStimuli(stimuli);

  if(friendAhead and (friendLeft or friendRight) ){
    return new StateCruise();
  }
  else if(friendBehind and friendAhead ){
    return new StateImpulseSpeed();
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

std::string StateCatchUp::GetNameString(){
  return name;
};

void StateCatchUp::Print(){
  printf("%s\n",name.c_str());
};
