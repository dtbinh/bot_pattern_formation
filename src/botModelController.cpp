#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf/transform_datatypes.h"

// Include for V-REP
#include "../include/v_repConst.h"
// Used data structures:
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/VisionSensorData.h"

// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosAuxiliaryConsolePrint.h"
#include "vrep_common/simRosAddStatusbarMessage.h"
#include "vrep_common/simRosAuxiliaryConsoleOpen.h"
#include "vrep_common/simRosAuxiliaryConsoleShow.h"

// Finite State Machine used to control Behaviour
#include "FSM/FSM.h"
#include "FSM/blobClass.h"

using namespace std;

// Create a node for communicating with ROS.
//ros::NodeHandle node("~");

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
float simulationTime=0.0f;

// Visual Servoing Data
vector<blobClass*> frontViewBlobVector;
vector<blobClass*> leftViewBlobVector;
vector<blobClass*> rightViewBlobVector;
vector<blobClass*> rearViewBlobVector;

vector<blobClass*> fullBlobVector;

float magneticHeadingError = 0.;
float formationHeadingError = 0.;

// Sensor booleans
bool frontProxSensor = false;
bool rearProxSensor = false;
bool friendLeft = false;
bool friendRight = false;
bool friendAhead = false;
bool friendBehind = false;
bool aligned = false;


//===========================================================================
// Function Prototypes
//===========================================================================
void sendMsg2Console(ros::NodeHandle node, int outputHandle, string msg);

//===========================================================================
// Topic subscriber callbacks:
//===========================================================================
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info){
  simulationTime=info->simulationTime.data;
  simulationRunning=(info->simulatorState.data&1)!=0;
}

void frontSensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens){
  printf("Front sensor.\n");
  
  frontProxSensor = true;
}

void rearSensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens){
  printf("Rear sensor.\n"); 
  
  rearProxSensor = true;
}

void cameraBlueCallback(const vrep_common::VisionSensorData::ConstPtr& sens){
}

void cameraRedCallback(const vrep_common::VisionSensorData::ConstPtr& sens){
}

void omniFrontCallback(const vrep_common::VisionSensorData::ConstPtr& sens){
  
  // one empty packet plus the number of blobs detected.
  int nPackets = sens->packetSizes.data.size();
  int numberOfBlobs =  sens->packetData.data[0];
  int datumPerBlob =  sens->packetData.data[1];
  // printf("========\nFront # of Blobs= %i\n", numberOfBlobs);
  if(nPackets < 1){
    printf("No packets sent!\n");
    return;
  }

  // If a blob is detected then a team mate is in front.
  if(numberOfBlobs > 0){
    friendAhead = true;
  }
  else{
    friendAhead = false;
  }

  // Record the blobs detected 
  frontViewBlobVector.clear();

  // Loop over the detected blobs and add them to the frontViewBlobVector 
  for(int i = 0; i < numberOfBlobs; i++){
    blobClass * newBlob = new  blobClass();
    
    float blobLocalX =  sens->packetData.data[i*datumPerBlob + 5 ];
    float blobLocalY =  sens->packetData.data[i*datumPerBlob + 6 ];
    
    float blobLocalBearing = atan2( blobLocalY, blobLocalX);

    // TODO: Need to make sure magneticHeadingError's time Stamp matches or is
    // at least local to the current time.
    newBlob->blobBearing =  blobLocalBearing - magneticHeadingError;

    float blobWidth =  sens->packetData.data[i * datumPerBlob + 7 ];
    float blobHeight =  sens->packetData.data[i * datumPerBlob + 8 ]; 
    newBlob->blobArea =  blobWidth*blobHeight;
    // printf("%f %f %f \n",blobLocalX, blobLocalY, newBlob->blobBearing);
    frontViewBlobVector.push_back(newBlob);
  }
 
  formationHeadingError = 0.;
  
  /*
    printf("Front # of Blobs = %i\n", numberOfBlobs); 
    printf("# of Packets = %d\n", nPackets);
    for(int i = 0; i < numberOfBlobs; i++){
    printf("========= BLOB # %i ==========\n",i+1);
    printf("Blob Size = %f\n", sens->packetData.data[i*datumPerBlob]);
    printf("Blob Orientation  = %f\n", sens->packetData.data[i*datumPerBlob+1]);
    printf("Blob X = %f\n", sens->packetData.data[i*datumPerBlob+2]);
    printf("Blob Y = %f\n", sens->packetData.data[i*datumPerBlob+3]);
    printf("Blob width  = %f\n", sens->packetData.data[i*datumPerBlob+4]);
    printf("Blob height = %f\n\n", sens->packetData.data[i*datumPerBlob+5]);
    }
  */
}

void omniBackCallback(const vrep_common::VisionSensorData::ConstPtr& sens){
 
  // one empty packet plus the number of blobs detected.
  int nPackets = sens->packetSizes.data.size();
  int numberOfBlobs =  sens->packetData.data[0];
  int datumPerBlob =  sens->packetData.data[1];

  if(nPackets < 1){
    printf("No packets sent!\n");
    return;
  }
  
  //printf("Back # of Blobs= %i\n", numberOfBlobs); 

  if(numberOfBlobs > 0){
    friendBehind = true;
  }
  else{
    friendBehind = false;
  }
  // Record the blobs detected 
  rearViewBlobVector.clear();

  // Loop over the detected blobs and add them to the frontViewBlobVector 
  for(int i = 0; i < numberOfBlobs; i++){
    blobClass * newBlob = new  blobClass();
    
    float blobLocalX =  sens->packetData.data[i*datumPerBlob + 5 ];
    float blobLocalY =  sens->packetData.data[i*datumPerBlob + 6 ];
    
    float blobLocalBearing = atan2( blobLocalY, blobLocalX);

    // TODO: Need to make sure magneticHeadingError's time Stamp matches or is
    // at least local to the current time.
    newBlob->blobBearing =  blobLocalBearing - magneticHeadingError;
    if(newBlob->blobBearing > 0.)
      newBlob->blobBearing -= M_PI;
    else
      newBlob->blobBearing += M_PI;

    float blobWidth =  sens->packetData.data[i * datumPerBlob + 7 ];
    float blobHeight =  sens->packetData.data[i * datumPerBlob + 8 ]; 
    newBlob->blobArea =  blobWidth*blobHeight;
    //printf("%f %f %f \n",blobLocalX, blobLocalY, newBlob->blobBearing);
    rearViewBlobVector.push_back(newBlob);
  }
  return;
}

void omniRightCallback(const vrep_common::VisionSensorData::ConstPtr& sens){

  
  // one empty packet plus the number of blobs detected.
  int nPackets = sens->packetSizes.data.size();
  int numberOfBlobs =  sens->packetData.data[0];
  int datumPerBlob =  sens->packetData.data[1];

  if(nPackets < 1){
    printf("No packets sent!\n");
    return;
  }
  // printf("Right # of Blobs= %i\n", numberOfBlobs); 
  
  if(numberOfBlobs > 0){
    friendRight = true;
  }
  else{
    friendRight = false;
  } 

  // Record the blobs detected 
  rightViewBlobVector.clear();

  // Loop over the detected blobs and add them to the frontViewBlobVector 
  for(int i = 0; i < numberOfBlobs; i++){
    blobClass * newBlob = new  blobClass();
    
    float blobLocalX =  sens->packetData.data[i*datumPerBlob + 5 ];
    float blobLocalY =  sens->packetData.data[i*datumPerBlob + 6 ];
    
    float blobLocalBearing = atan2( blobLocalY, blobLocalX);

    // TODO: Need to make sure magneticHeadingError's time Stamp matches or is
    // at least local to the current time.
    newBlob->blobBearing =  blobLocalBearing - magneticHeadingError+M_PI/2.;

    float blobWidth =  sens->packetData.data[i * datumPerBlob + 7 ];
    float blobHeight =  sens->packetData.data[i * datumPerBlob + 8 ]; 
    newBlob->blobArea =  blobWidth*blobHeight;
    // printf("%f %f %f \n",blobLocalX, blobLocalY, newBlob->blobBearing);
    rightViewBlobVector.push_back(newBlob);
  }
  return;
}

void omniLeftCallback(const vrep_common::VisionSensorData::ConstPtr& sens){
  
  // one empty packet plus the number of blobs detected.
  int nPackets = sens->packetSizes.data.size();
  int numberOfBlobs =  sens->packetData.data[0];
  int datumPerBlob =  sens->packetData.data[1];
 
  if(nPackets < 1){
    printf("No packets sent!\n");
    return;
  }
  
  // printf("Left # of Blobs = %i\n", numberOfBlobs); 
  
  if(numberOfBlobs > 0){
    friendLeft = true;
  }
  else{
    friendLeft = false;
  }
  
  // Record the blobs detected 
  leftViewBlobVector.clear();
  
  for(int i = 0; i < numberOfBlobs; i++){
    blobClass * newBlob = new  blobClass();
    
    float blobLocalX =  sens->packetData.data[i*datumPerBlob + 5 ];
    float blobLocalY =  sens->packetData.data[i*datumPerBlob + 6 ];
    
    float blobLocalBearing = atan2( blobLocalY, blobLocalX);

    // TODO: Need to make sure magneticHeadingError's time Stamp matches or is
    // at least local to the current time.
    newBlob->blobBearing =  blobLocalBearing - magneticHeadingError-M_PI/2.;

    float blobWidth =  sens->packetData.data[i * datumPerBlob + 7 ];
    float blobHeight =  sens->packetData.data[i * datumPerBlob + 8 ]; 
    newBlob->blobArea =  blobWidth*blobHeight;
    //printf("%f %f %f \n",blobLocalX, blobLocalY, newBlob->blobBearing);
    leftViewBlobVector.push_back(newBlob);
  }
  return;
}

void bodyOrientationCallback(const geometry_msgs::PoseStamped& pose){

  double orientation = tf::getYaw(pose.pose.orientation);
  
  magneticHeadingError = orientation + M_PI/2.;

  //printf("magneticError = %f\n",magneticHeadingError);

  if(magneticHeadingError > 0.05)
    aligned = false;
  else
    aligned = true;
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

void OpenServo(int servoMotorHandle, ros::Publisher servoPublisher){
 
  vrep_common::JointSetStateData servoPosition;
  float servoPos = 0.;
  
  servoPosition.handles.data.push_back(servoMotorHandle);
  servoPosition.setModes.data.push_back(1);
  servoPosition.values.data.push_back(servoPos);
  
  servoPublisher.publish(servoPosition);
}

void CloseServo(int servoMotorHandle, ros::Publisher servoPublisher){
  
  vrep_common::JointSetStateData servoPosition;
  float servoPos = M_PI;
  
  servoPosition.handles.data.push_back(servoMotorHandle);
  servoPosition.setModes.data.push_back(1);
  servoPosition.values.data.push_back(servoPos);
  
  servoPublisher.publish(servoPosition);
}

//===========================================================================
// Helper Functions
//===========================================================================
vector<blobClass*> AppendFirst2Second(vector<blobClass*> v1, vector<blobClass*> v2){
 
  for(int i = 0; i < v1.size(); i++)
    v2.push_back(v1[i]);
  
  return v2;
};

void sendMsg2Console(ros::NodeHandle node, int outputHandle, string msg){
   
  ros::ServiceClient consoleClient =
    node.serviceClient<vrep_common::simRosAuxiliaryConsolePrint>("/vrep/simRosAuxiliaryConsolePrint");
  vrep_common::simRosAuxiliaryConsolePrint consoleMsg;
 
  consoleMsg.request.consoleHandle=outputHandle;
  consoleMsg.request.text = msg;

  consoleClient.call(consoleMsg);

  return;
}

void RequestPublisher(ros::NodeHandle node, string topicName, 
		      int queueSize, int streamCmd, int objectHandle){

  ros::ServiceClient enablePublisherClient=
    node.serviceClient<vrep_common::simRosEnablePublisher>
    ("/vrep/simRosEnablePublisher");
  
  vrep_common::simRosEnablePublisher publisherRequest;
  
  publisherRequest.request.topicName = topicName;
  publisherRequest.request.queueSize = queueSize; 
  publisherRequest.request.streamCmd = streamCmd; 
  publisherRequest.request.auxInt1 = objectHandle; 
  publisherRequest.request.auxInt2 = -1; 
  publisherRequest.request.auxString = "";
  enablePublisherClient.call(publisherRequest);  
}

void RequestSubscriber(ros::NodeHandle node, string topicName, 
		       int queueSize, int streamCmd){
  
  ros::ServiceClient enableSubscriberClient =
    node.serviceClient<vrep_common::simRosEnableSubscriber>
    ("/vrep/simRosEnableSubscriber");
  
  
  vrep_common::simRosEnableSubscriber subscriberRequest;
  
  subscriberRequest.request.topicName = topicName; 
  subscriberRequest.request.queueSize = 1; 
  subscriberRequest.request.streamCmd = streamCmd;
  
  enableSubscriberClient.call(subscriberRequest);

}


//===========================================================================
// Main Function
//===========================================================================
int main(int argc,char* argv[]){  

  StateManager * fsm = new StateManager();

  // Parse the arguments passed to the node
  //===========================================================================
  int leftMotorHandle, rightMotorHandle;
  int servoMotorHandle;

  int frontSensorHandle, rearSensorHandle;
  int cameraRedHandle, cameraBlueHandle;
  
  int omniFrontHandle, omniBackHandle, omniRightHandle, omniLeftHandle;

  int outputHandle;   
  int bodyHandle;

  if (argc>=13){
    leftMotorHandle=atoi(argv[1]);
    rightMotorHandle=atoi(argv[2]);
    servoMotorHandle=atoi(argv[3]);
    frontSensorHandle=atoi(argv[4]);
    rearSensorHandle=atoi(argv[5]);
    cameraRedHandle=atoi(argv[6]);
    cameraBlueHandle=atoi(argv[7]);
   
    outputHandle=atoi(argv[8]);
    bodyHandle=atoi(argv[9]);

    omniFrontHandle = atoi(argv[10]);
    omniBackHandle = atoi(argv[11]);
    omniRightHandle = atoi(argv[12]);
    omniLeftHandle = atoi(argv[13]);
  }
  else{
    printf("Failed to acquire all object handles");
    sleep(5000);
    return 0;
  }
  //===========================================================================


  // Create a ROS node. The name has a random component: 
  //===========================================================================
  int _argc = 0;
  char** _argv = NULL;
  struct timeval tv;
  unsigned int timeVal=0;
  if (gettimeofday(&tv,NULL)==0)
    timeVal=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
  std::string nodeName("botModelController");
  std::string randId(boost::lexical_cast<std::string>(timeVal+int(999999.0f*(rand()/(float)RAND_MAX))));
  nodeName+=randId;		
  ros::init(_argc,_argv,nodeName.c_str());
  //===========================================================================


  if(!ros::master::check()){
    printf("ROS check failure...exiting\n");
    return(0);
  }
  // Create a node for communicating with ROS.
  //===========================================================================
  ros::NodeHandle node("~");
  //===========================================================================


  // Subscribe to the vrep info topic to know when the simulation ends
  //===========================================================================
   ros::Subscriber vrepInfoSub = 
    node.subscribe("/vrep/info/",1,infoCallback);
  //===========================================================================

 
  // Get V-Rep to publish sensor data to topics
  //===========================================================================
  
   // Front and Rear prox sensors.
   RequestPublisher(node, "frontSensorData"+randId, 1, 
		    simros_strmcmd_read_proximity_sensor, frontSensorHandle);
   RequestPublisher(node, "rearSensorData"+randId, 1, 
		    simros_strmcmd_read_proximity_sensor, rearSensorHandle);
   
   // Forward facing PUCK and GOAL camera
   RequestPublisher(node, "frontCameraRedData"+randId, 1, 
		    simros_strmcmd_read_vision_sensor , cameraRedHandle);
   RequestPublisher(node, "frontCameraBlueData"+randId, 1, 
		    simros_strmcmd_read_vision_sensor, cameraBlueHandle);

   // Omni-directional camera
   RequestPublisher(node, "omniFrontData"+randId, 1, 
		    simros_strmcmd_read_vision_sensor, omniFrontHandle);
   RequestPublisher(node, "omniBackData"+randId, 1, 
		    simros_strmcmd_read_vision_sensor, omniBackHandle);
   RequestPublisher(node, "omniRightData"+randId, 1, 
		    simros_strmcmd_read_vision_sensor, omniRightHandle);
   RequestPublisher(node, "omniLeftData"+randId, 1, 
   simros_strmcmd_read_vision_sensor, omniLeftHandle);
   
   // Compass 
   RequestPublisher(node, "bodyOrientationData"+randId, 1,
		    simros_strmcmd_get_object_pose, bodyHandle);
  
  //===========================================================================

  // Now subscribe to the sensor topics
  //===========================================================================
  string frontSensorTopicName("/vrep/frontSensorData");
  frontSensorTopicName += randId;
  ros::Subscriber frontSensorSub = 
    node.subscribe(frontSensorTopicName.c_str(),1,frontSensorCallback);

  string rearSensorTopicName("/vrep/rearSensorData");
  rearSensorTopicName += randId;
  ros::Subscriber rearSensorSub = 
    node.subscribe(rearSensorTopicName.c_str(),1,rearSensorCallback);
  
  string frontCameraRedTopicName("/vrep/frontCameraRedData");
  frontCameraRedTopicName += randId; 
  ros::Subscriber frontCameraRedSub = 
    node.subscribe(frontCameraRedTopicName.c_str(),1,cameraRedCallback);

  string frontCameraBlueTopicName("/vrep/frontCameraBlueData");
  frontCameraBlueTopicName += randId; 
  ros::Subscriber frontCameraBlueSub = 
    node.subscribe(frontCameraBlueTopicName.c_str(),1,cameraBlueCallback);

 
  string omniFrontTopicName("/vrep/omniFrontData");
  omniFrontTopicName += randId; 
  ros::Subscriber omniFrontSub = 
    node.subscribe(omniFrontTopicName.c_str(),1,omniFrontCallback);
 
  string omniBackTopicName("/vrep/omniBackData");
  omniBackTopicName += randId; 
  ros::Subscriber omniBackSub = 
    node.subscribe(omniBackTopicName.c_str(),1,omniBackCallback);   
  
  string omniRightTopicName("/vrep/omniRightData");
  omniRightTopicName += randId; 
  ros::Subscriber omniRightSub = 
    node.subscribe(omniRightTopicName.c_str(),1,omniRightCallback);
  
  string omniLeftTopicName("/vrep/omniLeftData");
  omniLeftTopicName += randId; 
  ros::Subscriber omniLeftSub = 
  node.subscribe(omniLeftTopicName.c_str(),1,omniLeftCallback);     
  
  string bodyOrientationTopicName("/vrep/bodyOrientationData");
  bodyOrientationTopicName += randId; 
  ros::Subscriber bodyOrientationSub = 
    node.subscribe(bodyOrientationTopicName.c_str(),1,bodyOrientationCallback);


  //===========================================================================


  // Now setup a publisher to control the WHEEL MOTORS and get V-REP to subscribe
  //===========================================================================
  
  ros::Publisher wheelSpeedPublisher = 
    node.advertise<vrep_common::JointSetStateData>("wheels",1);
 
  RequestSubscriber(node, "/"+nodeName+"/wheels", 1, simros_strmcmd_set_joint_state);

  //===========================================================================
  
  // Now setup a publisher to control the SERVO  and get V-REP to subscribe
  //===========================================================================
  
  ros::Publisher servoPublisher = 
    node.advertise<vrep_common::JointSetStateData>("servo",1);

  RequestSubscriber(node, "/"+nodeName+"/servo", 1, simros_strmcmd_set_joint_state);
 
  //===========================================================================================================================================================================================

  // The start of the control loop
  printf("botModelController started...\n");
  
  // These values are passed to the FSM
  float trans_speed = 5.;
  float rot_speed = 0.;
  bool openServo = true;
 
  float desiredLeftMotorSpeed = 10.;
  float desiredRightMotorSpeed = 10.;
  
  while (ros::ok() and simulationRunning){

    fullBlobVector.clear();
    fullBlobVector = AppendFirst2Second( frontViewBlobVector, fullBlobVector);
    fullBlobVector = AppendFirst2Second( leftViewBlobVector, fullBlobVector);
    fullBlobVector = AppendFirst2Second( rightViewBlobVector, fullBlobVector);
    fullBlobVector = AppendFirst2Second( rearViewBlobVector, fullBlobVector);
    int size = fullBlobVector.size();

    /*
    printf("=============\n# of Blobs = %i \n",  size);
    for(int i = 0; i < fullBlobVector.size(); i++){
      printf("blob # %i bearing %f\n",i,fullBlobVector[i]->blobBearing);
    }
    */

    bool stimuli[7];
    stimuli[0] = frontProxSensor;
    stimuli[1] = rearProxSensor;
    stimuli[2] = friendLeft;
    stimuli[3] = friendRight;
    stimuli[4] = friendAhead;
    stimuli[5] = friendBehind;
    stimuli[6] = aligned;

    // Send stimuli data
    fsm->UpdateBehaviour(stimuli);
    fsm->UpdateBlobData(fullBlobVector);
    // Send visual servo data
    fsm->SetMagneticHeadingError(magneticHeadingError);
    fsm->SetFormationHeadingError(formationHeadingError);

    // ExecuteBehaviour will return a translational speed, rotational speed, and a boolean
    // to determine weather to open or close the servo.
    fsm->ExecuteBehaviour(trans_speed, rot_speed, openServo);
    
    // Depending on what behaviour dictates open/close servo for puck lock
    if(openServo)
      OpenServo(servoMotorHandle, servoPublisher);
    else
      CloseServo(servoMotorHandle, servoPublisher);

    // Given the translation and rotation speeds dictated by the behaviour
    // state the desired left and right wheel motors speeds are calculated.
    desiredLeftMotorSpeed = (2.*trans_speed + rot_speed) / 2.;
    desiredRightMotorSpeed = (2.*trans_speed - rot_speed) / 2.;
 
    // Now that we know what speeds we need the wheels
    // to rotate at we can send that info to V-REP 
    vrep_common::JointSetStateData motorSpeeds;
   
    motorSpeeds.handles.data.push_back(leftMotorHandle);
    motorSpeeds.handles.data.push_back(rightMotorHandle);
    motorSpeeds.setModes.data.push_back(2); // 2 is the speed mode
    motorSpeeds.setModes.data.push_back(2);
    motorSpeeds.values.data.push_back(desiredLeftMotorSpeed);
    motorSpeeds.values.data.push_back(desiredRightMotorSpeed);
         
    wheelSpeedPublisher.publish(motorSpeeds);
 
    // A message to publish to the console in V-REP for debugging.
    std::string behaviour = fsm->GetCurrentStateName();
    std::ostringstream ss;
    ss << "behaviour = " << behaviour  <<"\n"
       << "frontProxSensor = " << frontProxSensor <<"\n"
       << "rearProxSensor = " << rearProxSensor <<"\n"
       << "friendLeft = " << friendLeft <<"\n"
       << "friendRight = " << friendRight <<"\n"
       << "friendAhead = " << friendAhead <<"\n"
       << "friendBehind = " << friendBehind <<"\n"
       << "aligned = " << aligned <<"\n"
       << "transSpeed = " << trans_speed<<"\n"
       << "rotSpeed = " << rot_speed<<"\n";
    
    std::string msg(ss.str());
    sendMsg2Console(node, outputHandle, msg);

    // TODO: find a better way to reset the proximity sensors
    // Reset Prox sensors
    frontProxSensor = false;
    rearProxSensor = false;

    // handle ROS messages:
    ros::spinOnce();
  }

  // Close down the node.
  ros::shutdown();
  printf("...botModelController stopped\n");
  return(0);
}

