#include "../include/foot_data.h"

FootLocation::FootLocation(Foot currentfoot, float xpos, float ypos, float zpos, float r, float p, float y){
	foot=currentfoot;
	pos.xPos=xpos;
	pos.yPos=ypos;
	pos.zPos=zpos;
	
	pos.roll=r;
	pos.pitch=p;
	pos.yaw=y;
	
	currentHold=GetHold();
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


//TODO: Write this function
Hold FootLocation::GetHold(){
	//Load the hold data
	Hold currentHold;
	//Load the file
	
	//Fix the values based on current location

	return currentHold;
}




