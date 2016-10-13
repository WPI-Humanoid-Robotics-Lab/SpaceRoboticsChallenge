#include "../include/footstep_planner.h"
#include "../include/miqqp.h"
#include <iostream>

enum PlannerType {A, ANA, ARA, MIP};


int main(){
	//Class holding all the footsteps
	FootstepSequence footSequence;
	
	//Options for the footstep planner
	RobotWalkType walktype= BIPED;
	PlannerType planningAlgorithm=MIP;
	Leg startleg=RIGHT;

	//TODO:Load a terrain	
	//TODO: Get safe regions in the terrain
	
	if(planningAlgorithm==MIP){
		MiqQP MipPlanner(BIPED, startleg);
		footSequence=MipPlanner.PlanFootSteps();
	}
	
	return 0;
}
