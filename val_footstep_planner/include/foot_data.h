#ifndef _FOOT_DATA_H_
#define _FOOT_DATA_H_

#include <iostream>
#include "Planning/StanceCSpace.h"

//Enum for foot
enum Foot {lf_foot, rf_foot, lb_foot, rb_foot};

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
		FootLocation(Foot currentfoot, float xpos, float ypos, float zpos, float r, float p, float y);
		//Getters
		void GetPosition(Position& currentposition);
		bool GetFeasibility();

		//Setters
		void SetPosition(float xpos, float ypos, float zpos, float r, float p, float y);
		void SetFeasibility(bool fbility);
		
	private:
		//Describes which foot
		Foot foot;
	
		//Position of the leg
		Position pos;
	
		//Feasibility
		bool feasibility;
		
		//Hold information for planner		
		Hold currentHold;
		
		//Get the hold position
		Hold GetHold();
};

#endif

