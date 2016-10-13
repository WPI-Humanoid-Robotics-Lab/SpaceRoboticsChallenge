#ifndef _MIQQP_H_
#define _MIQQP_H_

#include <iostream>
#include "gurobi_c++.h"
#include "../include/footstep_planner.h"
#define pi 3.14159

class MiqQP{
	public: 
		MiqQP(RobotWalkType type, Leg start);
		FootstepSequence PlanFootSteps();

	private:
		RobotWalkType walktype;
		Leg startleg;
		int numSteps=100;
		vector<Leg> footrhythm;	
		FootstepSequence PlanBlankFootSteps(int numsteps);
		void SetFootStepRhythm();
};
#endif
