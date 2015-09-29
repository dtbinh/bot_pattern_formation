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

StateCruise::StateCruise(){
  name = "Cruise"; 
  
  /* initialize random seed: */
  srand (time(NULL));
};

void StateCruise::Enter(){};

void StateCruise::Execute(StateManager * fsm){
  //printf("Executing behaviour %s...\n", name.c_str());

  // Full speed ahead!
  fsm->SetTransSpeed(1);
  
  // This value should depend on formationHeadingError
  fsm->SetRotSpeed(0.);

};

void StateCruise::Exit(){};

State * StateCruise::Transition(bool* stimuli){

  SetStimuli(stimuli);

  if(friendAhead and friendBehind ){
    return new StateImpulseSpeed();
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

std::string StateCruise::GetNameString(){
  return name;
};

void StateCruise::Print(){
  printf("%s\n",name.c_str());
};
