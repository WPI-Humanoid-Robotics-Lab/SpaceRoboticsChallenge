#include "../include/footstep_planner.h"
#include "../include/miqqp.h"
#include <iostream>

enum PlannerType {AStar, ANAStar, ARAStar, MIP};


int main(){
	//Class holding all the footsteps
	FootstepSequence footSequence;
	
	//Options for the footstep planner
	RobotWalkType walktype=	biped;
	Foot startfoot= lf_foot;
	PlannerType planningAlgorithm=MIP;
	
	//TODO:Load a terrain

	
	if(planningAlgorithm==MIP){
		MiqQP MipPlanner(biped, startfoot);
		footSequence=MipPlanner.PlanFootSteps();
	}
	
	return 0;
}
