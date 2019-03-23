
#include <stdio.h>
#include <vector>

using namespace std;

#define MAX_SPEED (49.0)
#define VELOCITY_INC (0.7)
#define VELOCITY_DEC_SLOW (0.05)
#define VELOCITY_DEC (0.15)
#define VELOCITY_DEC_EMERGENCY (1.25)

#define MIN_FRONT_DISTANCE (40.0)

enum car_actions_e {DRIVING_AHEAD,DEACCELERATING,TURNING_LEFT,TURNING_RIGHT};

typedef struct
{
    bool too_close;
    double distance_front_vehicle;
    double vehicle_in_front_speed;
    int acceleration;

}vehicle_in_front_t;

typedef struct
{
    bool free_left;
    bool free_right;
    bool free_left_far;
    bool free_center_far;
    bool free_right_far;

    bool free_forward_lanes;

}lateral_lanes_t;

class path_planning
{

public:

path_planning();
~path_planning();

int get_current_lane(){return lane;}
double get_ref_velocity(){return ref_vel;}
double get_acceleration(){return acceleration;}
void estimate_ref_velocity(std::vector<std::vector<double>> sensor_fusion,double car_s,double car_d,double car_yaw,int prev_size,double ego_speed);

private:
vehicle_in_front_t detect_vehicle_in_front(std::vector<std::vector<double>> sensor_fusion, double car_s);
lateral_lanes_t check_free_lanes(std::vector<std::vector<double>> sensor_fusion,double ego_speed, double car_s,double prev_size);
int lane;
double ref_vel;
bool vehicle_in_front;
double prev_yaw;
car_actions_e prev_car_action;
car_actions_e car_action;
double acceleration;
bool reduce_speed;
};