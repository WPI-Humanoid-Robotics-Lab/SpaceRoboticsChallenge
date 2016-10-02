#include "../include/foot_data.h"

FootLocation::FootLocation(Foot currentfoot, float xpos, float ypos, float zpos, float theta){
	foot=currentfoot;
	pos.xPos=xpos;
	pos.yPos=ypos;
	pos.zPos=zpos;
	theta=theta;
	
	//currentHold=GetHold();
}

//TODO: Write this function
/*Hold FootLocation::GetHold(){
	//Load the hold data
	Hold currentHold;
	//Load the file
	
	//Fix the values based on current location

	return currentHold;
}*/

//Current position
void FootLocation::GetPosition(Position& currentPos){
	currentPos.xPos=pos.xPos;
	currentPos.yPos=pos.yPos;
	currentPos.zPos=pos.zPos;
}
