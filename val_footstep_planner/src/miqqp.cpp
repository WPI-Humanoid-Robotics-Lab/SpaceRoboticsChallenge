#include "../include/miqqp.h"

MiqQP::MiqQP(RobotWalkType type, Leg start){
	startleg=start;
	walktype=type;
	SetFootStepRhythm();	
}

void MiqQP::SetFootStepRhythm(){
	if(walktype==QUADRUPED){
		for(int i=0; i<4; i++){
			footrhythm.push_back(static_cast<Leg>((static_cast<int>(startleg)+i)%4));
		}
	}
	else if(walktype==BIPED){
		for(int i=0; i<2; i++){
			footrhythm.push_back(static_cast<Leg>((static_cast<int>(startleg)+i)%2));
			
		}
	}
}		

//Sets default values to each footstep
FootstepSequence MiqQP::PlanBlankFootSteps(int numsteps){
	FootstepSequence footsequence;
	return footsequence;
}


FootstepSequence MiqQP::PlanFootSteps(){
	FootstepSequence footsequence;

	//TODO: Come up with an approximation on number of footsteps. This can be a research topic

	
	//Setup the MIP problem
	GRBEnv env = GRBEnv();
	GRBModel model = GRBModel(env);

	//Variables


	//Update the model
	model.update();	
	//Create the optimization function



	//Constraints on the region (Obstacle free regions)
	//TODO: Implement this. At the moment, footsteps are not constrained in any way

	//Constraints on reachability

	//Trim number of footsteps
	
	model.optimize();
	//Results
	return footsequence;
}
