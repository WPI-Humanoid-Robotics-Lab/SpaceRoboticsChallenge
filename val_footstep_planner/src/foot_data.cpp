#include "../include/foot_data.h"

//Constructor
FootLocation::FootLocation(Leg currentfoot, float xpos, float ypos, float zpos, float r, float p, float y){
	currentleg=currentfoot;
	pos.xPos=xpos;
	pos.yPos=ypos;
	pos.zPos=zpos;
	
	pos.roll=r;
	pos.pitch=p;
	pos.yaw=y;
}

//Default Constructor
FootLocation::FootLocation(){
}




//Get Current position
void FootLocation::GetPosition(Position& currentPos){
	currentPos.xPos=pos.xPos;
	currentPos.yPos=pos.yPos;
	currentPos.zPos=pos.zPos;
	
	currentPos.roll=pos.roll;
	currentPos.pitch=pos.pitch;
	currentPos.yaw=pos.yaw;
}

//Get feasibility
bool FootLocation::GetFeasibility(){
	return feasibility;
}

//Get the current leg
Leg FootLocation::GetLeg(){
	return currentleg; 
}

//Set the current position
void FootLocation::SetPosition(float xpos, float ypos, float zpos, float r, float p, float y){
	pos.xPos=xpos;
	pos.yPos=ypos;
	pos.zPos=zpos;
	
	pos.roll=r;
	pos.pitch=p;
	pos.yaw=y;
}


//Set the feasibility
void FootLocation::SetFeasibility(bool fbility){
	feasibility=fbility;
}

//Set the current leg
void FootLocation::SetLeg(Leg curleg){
	currentleg=curleg;
}
