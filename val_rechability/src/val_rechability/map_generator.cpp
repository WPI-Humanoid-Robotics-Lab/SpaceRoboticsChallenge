#include <val_rechability/val_rechability.h>

int main(int argc, char **argv)
{
    ros::init (argc, argv, "reachability_map");
    ros::NodeHandle(nh);

    // create object for valkyire
    rechabilityMap valRechability(nh);

    valRechability.fkMapGenerator("leftArm");

    ros::spin();

    return 0;
}
