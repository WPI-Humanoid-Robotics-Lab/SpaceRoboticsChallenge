#include <iostream>
//#include "Planning/StanceCSpace.h"

//Enum for foot
enum Foot {l_foot, r_foot};

struct Position{
	float xPos;
	float yPos;
	float zPos;
};

//Foot location in space may be stored here
class FootLocation{
	public:
		FootLocation(Foot currentfoot, float xpos, float ypos, float zpos, float theta);
		void GetPosition(Position& currentposition);

	private:
		//Describes which foot
		Foot foot;
	
		//Position of the leg
		Position pos;
		float theta;	
		
		//Hold information for planner		
		//Hold currentHold;
		
		//Get the hold position
		//Hold GetHold();
};


