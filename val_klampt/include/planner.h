#ifndef PLANNER_H
#define PLANNER_H

#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include "Planning/StanceCSpace.h"
#include <Interface/SimulationGUI.h>
#include "Contact/Stance.h"
#include "Contact/ContactFeature.h"
#include "Contact/ContactFeatureMapping.h"
#include "robotlimbsstate.h"
#include <algorithm>
#include <cmath>
#include <stdlib.h>
enum HoldType { HookHold, FlatHold, FlatclimbHold};
struct ConfigRank{
	Config currentconfig;
	float rank;
};

struct by_rank {
    bool operator()(ConfigRank const &a, ConfigRank const &b) {
        return a.rank < b.rank;
    }
};

class Planner{
	public:
		static Planner* getplanner(RobotWorld& _world, int _robot, const char* xmlfile);
		int loadsimulationfile(const char* xmlfile);
		void initializecspace();
		void loadstartpositions();
		Config straightpath(vector<HoldType> htype, int leg, Config startconfig);
		RobotLimbsState rlimbsState;
		Config movecom(vector<int> activevalues, Config curconfig, float comx, float comy, float comz);
		Config movecomlasttest(vector<int> activevalues, Config curconfig, float composition, float compositiony);

		bool check_com(Config sampleconfig,float x, float y, float z,float xTolerance, float yTolerance,float zTolerance);
		float l2Norm(Config,Config);
		Config setstartconfig();
		Config straightpath(vector<int> activevalues, int leg, Config startconfig);
		void addmilestonepath(int leg, Config startConfig,Config goalConfig, MultiPath& mpath);
		void planmilestonepath(int leg, Config startconfig, Config goalconfig, MultiPath& mpath);
	private:

		Planner(RobotWorld& _world, int _robot, const char* xmlfile){
			world = &_world;
			world->InitCollisions();
			robot=_robot;
			backend=new SimGUIBackend(&_world);
			sim=backend->sim;
			loadsimulationfile(xmlfile);
			initializecspace();
		}
		bool plantopoint(CSpace* space, Stance curstance, Config qstart, Config qgoal, HaltingCondition& cond, string& plannerSettings, MultiPath& mpath);
		void fixjointsforhold(StanceCSpace& curstance, vector<int>& activevalues, Config currentConfig, int numholds, vector<HoldType> htype, vector<int> excludelimb);
		vector<int> fixposition();
		float calculateheuristic(Config curconfig, Config sampleconfig);
		Config clearworld(Config currentconfig);
		RobotWorld* world;
		WorldPlannerSettings settings;
		SimGUIBackend* backend;
		WorldSimulation sim;
		int robot;
		void updateworld(Config currentconfig);
		vector<int> fixfeetindices();
		Planner(const Planner& );
		float calculatecost(Config currentconfig, int leg);
		Planner& operator=(Planner const&);
		static Planner* m_pInstance;

};

#endif
