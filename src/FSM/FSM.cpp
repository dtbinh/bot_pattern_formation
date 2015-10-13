#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "FSM.h"
#include "State.h"
#include "StateAlign.h"
#include "blobClass.h"

using namespace std;

StateManager::StateManager(){

  //currentState = new StateRandomWalk();
  //currentState = new StateCatchUp();
  currentState = new StateAlign();

  trans_speed = 5.;
  rot_speed = 0.;
  openServo = true;
  
  kp = 4.0;

  magneticHeadingError = 0.;
  formationHeadingError = 0.;    
};

void StateManager::UpdateBehaviour(bool* stimuli){
  State * newState;
  
  newState = currentState->Transition(stimuli);
  
  // Transition returns NULL if no transition occurs
  // (i.e when state remains the same);
  if(newState != NULL){
    delete currentState;
    currentState = newState;
  }
  
};

void StateManager::ExecuteBehaviour(float& trans, float& rot, bool& servoOpen){
  
  currentState->Execute(this);
  
  trans = trans_speed;
  rot = rot_speed;
  servoOpen = openServo;
};

void StateManager::PrintCurrentState(){
  currentState->Print();
};

string StateManager::GetCurrentStateName(){
  return currentState->GetNameString();
}

void StateManager::SetMagneticHeadingError(float value){
  magneticHeadingError = value;
};

void StateManager::SetFormationHeadingError(float value){
  formationHeadingError = value;
};

float StateManager::GetMagneticHeadingError(){
  return magneticHeadingError;
};

float StateManager::GetFormationHeadingError(){
  return formationHeadingError;
};

float StateManager::GetProportionalGain(){
  return kp;
};

void StateManager::SetRotSpeed(float speed){
  rot_speed = speed;
};

void StateManager::SetTransSpeed(int speed){
  if(speed == 1){
    trans_speed = 5.;
  }
  else if(speed == -1){
    trans_speed = -2.5;
  }
  else if(speed == 2){
    trans_speed = 2.5;
  }
  else{
    trans_speed = 0.;
  }
};
bool StateManager::MovingForward(){
  if(trans_speed > 0.)
    return true;
  else
    return false;
};

void StateManager::UpdateBlobData(vector<blobClass*> aVectorOfBlobs){
  
  for(int i = 0; i < aVectorOfBlobs.size(); i++){
    blobVector.push_back(aVectorOfBlobs[i]);
  }

};

void StateManager::CloseServo(){
  openServo = false;
};

void StateManager::OpenServo(){
  openServo = true;
};
