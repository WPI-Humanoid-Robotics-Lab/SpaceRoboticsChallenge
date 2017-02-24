#include "planner.h"

Planner* Planner::m_pInstance = NULL;

//Planner constructor
Planner* Planner::getplanner(RobotWorld& _world, int _robot, const char* xmlfile){
	if(!m_pInstance)
		m_pInstance = new Planner(_world, _robot, xmlfile);
	return m_pInstance;
}

//Load the simulation file
int Planner::loadsimulationfile(const char* xmlfile){
	if(!backend->LoadAndInitSim(xmlfile)){
		cerr<<"Error loading simulation from command line\n";
		return 1;
	}
	return 0;
}

//Initialize the default space
void Planner::initializecspace(){
	settings.InitializeDefault(*world);
}

bool Planner::plantopoint(CSpace* space, Stance curstance, Config qstart, Config qgoal, HaltingCondition& cond, string& plannerSettings, MultiPath& mpath){
	 MotionPlannerFactory factory;
    if(!plannerSettings.empty())
      factory.LoadJSON(plannerSettings);
    MotionPlannerInterface* planner = factory.Create(space,qstart,qgoal);
    MilestonePath milestonePath;
    string res = planner->Plan(milestonePath,cond);
    cout<<"Planner terminated with condition "<<res<<endl;
    PropertyMap stats;
    planner->GetStats(stats);
    cout<<"Planner stats: ";
    stats.Print(cout);
    cout<<endl;
    delete planner;
    if (res.compare("foundSolution")==0){
      mpath.sections.resize(mpath.sections.size()+1);
      mpath.sections.back().milestones.resize(milestonePath.NumMilestones());
      for(int j=0;j<milestonePath.NumMilestones();j++)
       mpath.sections.back().milestones[j] = milestonePath.GetMilestone(j);
       mpath.SetStance(curstance, mpath.sections.size()-1);
      return 1;
    }
    if (milestonePath.edges.size()!=0){
      mpath.sections.resize(mpath.sections.size()+1);
      mpath.sections.back().milestones.resize(milestonePath.NumMilestones());
      for(int j=0;j<milestonePath.NumMilestones();j++)
       mpath.sections.back().milestones[j] = milestonePath.GetMilestone(j);
       mpath.SetStance(curstance, mpath.sections.size()-1);
      return 1;
    }
    return 0;
}


void Planner::updateworld(Config currentconfig){
	world->GetRobot(world->robots[robot]->name)->q=currentconfig;
	world->GetRobot(world->robots[robot]->name)->UpdateConfig(currentconfig);
	world->GetRobot(world->robots[robot]->name)->UpdateGeometry();
}


Config Planner::movecom(vector<int> activevalues, Config startconfig, float comx, float comy, float comz){
	StanceCSpace stretchspace(*world, robot, &settings);
	bool sample_checked=false;
	int limbsize=rlimbsState.lState.size();
	stretchspace.SetHold(rlimbsState.lState[0].flatHold);
	stretchspace.SetHold(rlimbsState.lState[1].flatHold);
	updateworld(startconfig);
	Vector3 curcom=world->GetRobot(world->robots[robot]->name)->GetCOM();
	Vector2 goalcom;
	goalcom[0]=curcom[0]-comx;
	goalcom[1]=curcom[1]-comy;

		printf("goalComX: %f currComx: %f comp: %f\n", goalcom[0],curcom[0],comx );
		printf("goalComy: %f currComy: %f comp: %f\n", goalcom[1],curcom[1],comy );

for(int i=0; i<activevalues.size(); i++){
		stretchspace.fixedDofs.push_back(activevalues[i]);
		stretchspace.fixedValues.push_back(startconfig[activevalues[i]]);
	}

	if(!stretchspace.SolveContact(goalcom))
	{
		//while we aren't in the desired COM, sample
		while (!sample_checked){
			//sample until it's feasible
			while(!stretchspace.IsFeasible(stretchspace.robot.q))
			{
				stretchspace.Sample(stretchspace.robot.q);
			}
			//check if it's what we want

			if(!(sample_checked = check_com(stretchspace.robot.q,goalcom[0],goalcom[1],comz,0.1,0.1,0.1))){//.1, .1, .1
				//clear, because it wasn't.
				stretchspace.robot.q=clearworld(stretchspace.robot.q);
			}
		}
	}

	return stretchspace.robot.q;
}

Config Planner::movecomlasttest(vector<int> activevalues, Config startconfig, float comx, float comy){
	StanceCSpace stretchspace(*world, robot, &settings);
	int limbsize=rlimbsState.lState.size();
	stretchspace.SetHold(rlimbsState.lState[0].flatHold);
	stretchspace.SetHold(rlimbsState.lState[1].flatHold);

	//stretchspace.fixedDofs.push_back(1);
	//stretchspace.fixedValues.push_back(startconfig[1]+0.01);
	updateworld(startconfig);
	Vector3 curcom=world->GetRobot(world->robots[robot]->name)->GetCOM();
	Vector2 goalcom;

	goalcom[0]=curcom[0]-comx;
	goalcom[1]=curcom[1]-comy;

	for(int i=0; i<activevalues.size(); i++){
			stretchspace.fixedDofs.push_back(activevalues[i]);
			stretchspace.fixedValues.push_back(startconfig[activevalues[i]]);
		}


	if(!stretchspace.SolveContact(goalcom))
	{
		cout<<"Unable to solve contacts\n";
	}
		//while we aren't in the desired COM, sample
		// while(!stretchspace.IsFeasible(stretchspace.robot.q))
		// {
		// 	stretchspace.Sample(stretchspace.robot.q);
		// }
		cout<<stretchspace.robot.q<<"\n";
	return stretchspace.robot.q;
}

//returns true if the config is inside the desired COM
bool Planner::check_com(Config sampleconfig,float x, float y, float z, float xTolerance, float yTolerance,float zTolerance){
	bool x_check=false;
	bool y_check=false;
	bool z_check=false;
		if((abs(sampleconfig[0]-x)<xTolerance || x == 999))
		{
			x_check = true;
		}else{	cout<<" X";}
		if(abs(sampleconfig[1]-y)<yTolerance || y == 999)
		{
			y_check = true;
		}else{	cout<<" Y";}
		if(sampleconfig[2] < (z+(zTolerance*.5)) && (sampleconfig[2] > z-zTolerance ) || z == 999)
		{
			z_check = true;
		}else{	cout<<" Z";}
		if(x_check == true && y_check == true && z_check == true)
		{
			cout<<"sample success\n";
			return true;
		}
		cout<<"\n";
	return false;
}

float Planner::l2Norm(Config sampleconfig,Config currentconfig)
{
	float accum = 0.0;
	for (int i = 0; i < 83; ++i) {
			accum += (sampleconfig(i)-currentconfig(i))*(sampleconfig(i)-currentconfig(i));
	}

	return sqrt(accum);
}


Config Planner::clearworld(Config currentconfig){
	for(int i=6; i<38; i++){
		if(i<3 || i>5)
			currentconfig[i]=0;
		//currentconfig[i]=rand();
	}
	return currentconfig;
}


Config Planner::setstartconfig(){
	StanceCSpace startspace(*world, robot, &settings);
	vector<int> activevalues, excludelimbs;

	int limbsize=2;

	Config startconfig=world->GetRobot(world->robots[robot]->name)->q;
	startspace.SetHold(rlimbsState.lState[0].flatHold);
	startspace.SetHold(rlimbsState.lState[1].flatHold);
	while(!startspace.IsFeasible(startspace.robot.q))
		startspace.Sample(startspace.robot.q);
	return startspace.robot.q;
}



void Planner::addmilestonepath(int leg, Config startConfig,Config goalConfig, MultiPath& mpath){
  Stance currentStance;
  StanceCSpace moveCSpace(*world, robot, &settings);
  for(int i=0; i<rlimbsState.lState.size();i++){
    if(i!=leg){
      	currentStance.insert(rlimbsState.lState[i].flatHold);
    }
  }
  moveCSpace.SetStance(currentStance);
  mpath.sections.resize(mpath.sections.size()+1);
  mpath.sections.back().milestones.resize(2);
  mpath.sections.back().milestones[0] = startConfig;
  mpath.sections.back().milestones[1] = goalConfig;
  mpath.SetStance(currentStance, mpath.sections.size()-1);
}



Config Planner::straightpath(vector<int> activevalues, int leg, Config startconfig){
	StanceCSpace stretchspace(*world, robot, &settings);
	int limbsize=rlimbsState.lState.size();
	bool sample_checked = false;
	stretchspace.SetHold(rlimbsState.lState[0].flatHold);
	stretchspace.SetHold(rlimbsState.lState[1].flatHold);
	Vector3 curcom=world->GetRobot(world->robots[robot]->name)->GetCOM();

	Config best_sample;
	float currL2Norm =0;
	float l2Norm_min =50;
	int numsamples = 200;

	for(int i=0; i<activevalues.size(); i++){
			stretchspace.fixedDofs.push_back(activevalues[i]);
			stretchspace.fixedValues.push_back(startconfig[activevalues[i]]);
	}
	cout<<"solving for step\n";
	if(!stretchspace.SolveContact())
	{
		cout<<"Unable to solve STEP  contact!!\n";
		//while we aren't in the desired COM, sample
		while (!sample_checked){
		  //sample until it's feasible
		  while(!stretchspace.IsFeasible(stretchspace.robot.q))
		  {
		    stretchspace.Sample(stretchspace.robot.q);
		  }
			sample_checked = check_com(stretchspace.robot.q,curcom[0],curcom[1],curcom[2],.02,.02,.4);//,.02,.02,.4
			if(!sample_checked){
				//clear, because it wasn't.
				stretchspace.robot.q=clearworld(stretchspace.robot.q);
			}
		}

	// 	for(int i=0;i<numsamples;i++)
	// 	{
	// 	  while(!stretchspace.IsFeasible(stretchspace.robot.q))
	// 	  {
	// 	    stretchspace.Sample(stretchspace.robot.q);
	// 	  }
	// 		currL2Norm = l2Norm(stretchspace.robot.q,startconfig);
	// 	  if(currL2Norm < l2Norm_min)
	// 	  {
	// 	    l2Norm_min = currL2Norm;
	// 			cout<<currL2Norm<<" ";
	// 	    best_sample = stretchspace.robot.q;
	// 			sample_checked = true;
	// 	  }
	// 	  stretchspace.robot.q=clearworld(stretchspace.robot.q);
	// 	}
	// }
	// if(sample_checked)
	// {
	// 	stretchspace.robot.q = best_sample;
	}

		return stretchspace.robot.q;
}



void Planner::planmilestonepath(int leg, Config startconfig, Config goalconfig, MultiPath& mpath){
	Stance currentstance;
	StanceCSpace movecspace(*world, robot, &settings);
	for(int i=0; i<rlimbsState.lState.size(); i++){
		if(i!=leg)
		{
				currentstance.insert(rlimbsState.lState[i].flatHold);
		}
	}
	string plannerSettings="{type:\"rrt\", perturbationRadius:0.9, bidirectional:1, shortcut:0, collisionEpsilon:5.5}";
	movecspace.SetStance(currentstance);
	HaltingCondition cond;
	cond.maxIters=500;
	cond.timeLimit=100;
	cond.foundSolution=true;
	plantopoint(&movecspace, currentstance, startconfig, goalconfig,cond, plannerSettings, mpath);
}
