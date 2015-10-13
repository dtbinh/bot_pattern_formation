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

StateHalt::StateHalt(){
name = "Halt"; 

/* initialize random seed: */
 srand (time(NULL));
};

void StateHalt::Enter(){};

void StateHalt::Execute(StateManager * fsm){
  //printf("Executing behaviour %s...\n", name.c_str());
  
  fsm->SetTransSpeed(0);
  fsm->SetRotSpeed(0.);

};

void StateHalt::Exit(){};

State * StateHalt::Transition(bool* stimuli){

  SetStimuli(stimuli);

  if(not aligned)
    return new StateAlign();
  else if(not friendBehind and not friendAhead)
    return new StateCruise();
  else if(friendAhead and friendBehind)
    return new StateImpulseSpeed();
  else
      return NULL; 
};

std::string StateHalt::GetNameString(){
  return name;
};

void StateHalt::Print(){
  printf("%s\n",name.c_str());
};
