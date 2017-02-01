#include "timeassignment.h"
#include <Modeling/Interpolate.h>

//This create smooth paths between 2 milestones
void InterpolateInverseCosine(Robot& robot, MultiPath mpath, vector<Config>& fullPath, vector<double>& timeStep){
  double globalTime=0.0;
  fullPath.clear();
  timeStep.clear();
  vector<Config>  intermediatePath;
  for(int i=0; i<mpath.sections.size(); i++){
    vector<IKGoal> ikGoals;

    //Get IK goals from the holds for the section
    for(int j=0; j<mpath.sections[i].holds.size(); j++){
      ikGoals.push_back(mpath.sections[i].holds[j].ikConstraint);
    }
    double sectionSpeed = 3;

    double sectionDuration = 0;
    for(int j=0; j<mpath.sections[i].milestones.size()-1; j++){
      double GoalValue=Distance(robot, mpath.sections[i].milestones[j],mpath.sections[i].milestones[j+1], 1.0/0.0);
      double scale=Sqrt(GoalValue)*sectionSpeed;
      sectionDuration += scale;

      if(InterpolateConstrainedPath(robot,mpath.sections[i].milestones[j],mpath.sections[i].milestones[j+1], ikGoals,intermediatePath)){

        fullPath.insert(fullPath.end(), intermediatePath.begin(), intermediatePath.end());

        double tFinal=1;
        double stepSize=tFinal/(intermediatePath.size());
        double tTime=tFinal, timeValue=0.0;

        for(int asd=0; asd<intermediatePath.size(); asd++){
          timeValue=globalTime+(scale*acos(2*tTime-1)/3.14);
          tTime-=stepSize;
          timeStep.push_back(timeValue);
        }
        globalTime+=(timeValue-globalTime);
        intermediatePath.clear();
      }
    }
    printf("Section %d time %g\n",i,sectionDuration);
  }
}
