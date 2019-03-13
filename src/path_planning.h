
#include <stdio.h>
#include <vector>

using namespace std;

#define MAX_SPEED (49.0)
class path_planning
{

public:

path_planning();
~path_planning();

int get_current_lane(){return lane;}
double get_ref_velocity(){return ref_vel;}

void estimate_ref_velocity(std::vector<std::vector<double>> sensor_fusion,double car_s,int prev_size);

private:

int lane;
double ref_vel;
bool vehicle_in_front;
};