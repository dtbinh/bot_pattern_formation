#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>

#include "StateImpulseSpeed.h"
#include "StateCruise.h"
#include "StateCatchUp.h"
#include "StateAlign.h"
#include "StateEvade.h"
#include "StateHalt.h"

#include "State.h"

#include "FSM.h"

StateEvade::StateEvade(){
  name = "Evade"; 
  
  /* initialize random seed: */
  srand (time(NULL));

  deltaT = 5.;

  time(&timeStamp);
  timerExpired = false;

  first = true;
};

void StateEvade::Enter(){};

void StateEvade::Execute(StateManager * fsm){
  //printf("Executing behaviour %s...\n", name.c_str());

  // This speed is half the standard speed 
  fsm->SetTransSpeed(-1);

  if(first){
    float r = (M_PI/2.)*float(rand()/RAND_MAX);
    fsm->SetRotSpeed(r);
    first = false;
  }
};

void StateEvade::Exit(){};

State * StateEvade::Transition(bool* stimuli){

  SetStimuli(stimuli);
   
  time_t currentTime;
  time(&currentTime);

  // Check if manvouver has finished yet
  if(difftime(currentTime, timeStamp) > deltaT){
    timerExpired = true;
  }

  if(not timerExpired)
    return NULL;
  else
    return  new StateAlign();

};

std::string StateEvade::GetNameString(){
  return name;
};

void StateEvade::Print(){
  printf("%s\n",name.c_str());
};
