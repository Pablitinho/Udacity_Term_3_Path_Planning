#include "path_planning.h"
#include "math.h"
//----------------------------------------------------------------
path_planning::path_planning()
{
    lane = 1;
    ref_vel=49.5;
}
//----------------------------------------------------------------
path_planning::~path_planning()
{
    
}
//----------------------------------------------------------------
void path_planning::estimate_ref_velocity(std::vector<std::vector<double>> sensor_fusion,double car_s, int prev_size)
{
    bool vehicle_in_front=false;
    for ( int i = 0; i < sensor_fusion.size(); i++ ) 
    {
        float d = sensor_fusion[i][6];
        if (d<(2+4*lane+2) && d>(2+4*lane-2))
        {
            // Find car speed.
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            //printf("Car In front of us before s: %lf\n",check_car_s);
            check_car_s+=((double)prev_size*0.02*check_speed);
            //printf("Car In front of us after s: %lf\n",check_car_s);
            if ((check_car_s>car_s) && ((check_car_s-car_s)<40))
            {
                if (check_speed<MAX_SPEED)
                {
                    ref_vel=ref_vel -0.5;//check_speed;
                    if (ref_vel<check_speed)
                    {
                        ref_vel = check_speed;
                    }
                }
                else
                {
                    ref_vel=MAX_SPEED;
                }
                vehicle_in_front=true;
                printf("Car In front of us with speed: %lf\n",ref_vel);
                break;
            }
        }
    }

    if (!vehicle_in_front)
    {
        ref_vel+=1.5;
        if (ref_vel>MAX_SPEED)
        {
            ref_vel=MAX_SPEED;
        }
        
    }

}