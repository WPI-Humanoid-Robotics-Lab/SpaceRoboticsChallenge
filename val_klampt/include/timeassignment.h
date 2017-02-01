#include <string>
#include <cstring>
#include <iostream>
#include <cmath>
#include "Modeling/Paths.h"
#include "Modeling/MultiPath.h"
#include "Planning/RobotTimeScaling.h"
using namespace std;
//Class which plans for intermediate stance configurations
void InterpolateInverseCosine(Robot& robot, MultiPath mpath, vector<Config>& fullPath, vector<double>& timeStep);
