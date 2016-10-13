#ifndef _FOOT_DATA_H_
#define _FOOT_DATA_H_

#include <iostream>
#include "Planning/StanceCSpace.h"

//Enum for foot
enum Leg {RIGHT=0, RIGHT_BACK=1, LEFT_BACK=2, LEFT=3};

struct Position{
	float xPos;
	float yPos;
	float zPos;
	
	float roll;
	float pitch;
	float yaw;
};

//Foot location in space may be stored here
class FootLocation{
	public:
		FootLocation(Leg currentfoot, float xpos, float ypos, float zpos, float r, float p, float y);
		FootLocation();
		
		//Getters
		void GetPosition(Position& currentposition);
		bool GetFeasibility();
		Leg GetLeg();

		//Setters
		void SetPosition(float xpos, float ypos, float zpos, float r, float p, float y);
		void SetFeasibility(bool fbility);
		void SetLeg(Leg curleg);
	private:
		Leg currentleg;
	
		//Position of the leg
		Position pos;
	
		//Feasibility
		bool feasibility;
};

#endif

