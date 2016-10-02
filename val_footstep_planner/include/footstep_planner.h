#ifndef _FOOTSTEP_PLANNER_H_
#define _FOOTSTEP_PLANNER_H_

#include <iostream>
#include <cstdlib>
#include "foot_data.h"
#include <map>

enum RobotWalkType{ biped, quadruped };

//Footstep sequence which will be passed to the low level planner
class FootstepSequence{
	public:
		//Map Holding sequence of footsteps
		std::map<int, FootLocation> sequence;
};

#endif
