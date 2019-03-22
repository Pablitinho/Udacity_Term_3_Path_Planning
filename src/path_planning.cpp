#include "path_planning.h"
#include "math.h"
#include "float.h"
typedef enum car_type {CAR_IN_FRONT,CAR_LEFT,CAR_RIGHT};

//----------------------------------------------------------------
path_planning::path_planning()
{
    lane = 1;
    ref_vel=0.0;
    vehicle_in_front=false;
}
//----------------------------------------------------------------
path_planning::~path_planning()
{
    
}
//----------------------------------------------------------------
int get_car_line(float d)
{
    int car_lane=-1;

    if (d>0 && d<4)
    {
        car_lane =0;
    }
    else if (d>4 && d<8)
    {
        car_lane =1;
    }
    else if (d>8 && d<12)
    {
        car_lane =2;
    }

    return car_lane;
}
//----------------------------------------------------------------
void path_planning::estimate_ref_velocity(std::vector<std::vector<double>> sensor_fusion,double car_s,double car_d,double car_yaw, int prev_size,double ego_speed)
{

    bool vehicle_in_front=false;
    bool vehicle_in_left=false;
    bool vehicle_in_right=false;

    float speed_reduction=0.25;
    double gap = 30;
    double MAX_TTC = 15.0;
    double distance_front_vehicle = 100000000000000.0;
    double vehicle_in_front_speed =0;

    //printf("Car d: %lf \n",car_d);

    for ( int i = 0; i < sensor_fusion.size(); i++ ) 
    {
        float d = sensor_fusion[i][6];

        int car_lane= get_car_line(d);

        //if (d<(2+4*lane+2) && d>(2+4*lane-2))
        // Car in the same lane
        //if (car_lane==lane)
        {
            // Find car speed.
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            //printf("Car In front of us before s: %lf\n",check_car_s);
            //check_car_s+=((double)prev_size*0.02*check_speed);
            //printf("Car In front of us after s: %lf\n",check_car_s);
            double diff_s = check_car_s-car_s;

            double ttc = fabs(diff_s)/(((ego_speed*0.447)-check_speed*0.447));

            if (car_lane==lane && (check_car_s>car_s) && (diff_s<50))
            {
                vehicle_in_front=true;
                distance_front_vehicle = diff_s;
                vehicle_in_front_speed=ref_vel;
            }
            else if ((car_lane - lane==-1) && fabs(diff_s)<20)//&&(check_car_s>(car_s-gap) && (check_car_s< (car_s+gap))))
            {
                vehicle_in_left=true;

            }
            else if ((car_lane - lane==1) && fabs(diff_s)<20)//&&(check_car_s>(car_s-gap) && (check_car_s< (car_s+gap))))
            {
                vehicle_in_right=true;
            }

            if (car_lane==lane && (check_car_s>car_s) && (diff_s<40))
            {
               printf("Car in front lane: %i diffs: %lf ",car_lane,fabs(diff_s));
            }
            else if ((car_lane - lane==-1)  && fabs(diff_s)<20)//&&(check_car_s>(car_s-gap) && (check_car_s< (car_s+gap))))
            {
                printf("Car Left lane: %i diffs: %lf ",car_lane,fabs(diff_s));

            }
            else if ((car_lane - lane==1) && fabs(diff_s)<20)//&&(check_car_s>(car_s-gap) && (check_car_s< (car_s+gap))))
             {
                printf("Car Right lane: %i diffs: %lf ",car_lane,fabs(diff_s));
            }
            // Check if the car is in front of us
            /*if ((check_car_s>car_s) && (diff_s<40)) 
            {
                if (lane>0)
                {
                    lane=0;
                }
                if (diff_s<distance_front_vehicle)
                {

                    distance_front_vehicle = diff_s;
                    vehicle_in_front_speed=ref_vel;
                }
                vehicle_in_front=true;
                printf("Car In front of us with speed: %lf s: %lf\n",vehicle_in_front_speed,distance_front_vehicle);
                break;
            }*/
        }
    }
    printf("\n");
    if (!vehicle_in_front)
    {
        ref_vel+=3*speed_reduction;
        if (ref_vel>MAX_SPEED)
        {
            ref_vel=MAX_SPEED;
        }


    }
    else
    {
      // We can not change the lane  
      if (!vehicle_in_left && lane>0 && ego_speed>30 && fabs(prev_yaw - car_yaw)<1)
      {
        lane--;
      }
      else if (!vehicle_in_right && lane!=2 && ego_speed>30 && fabs(prev_yaw - car_yaw)<1)
      {
        lane++;
      }
      else
      {
        if (distance_front_vehicle<20)
        {
            ref_vel-= 5*speed_reduction;
        }
        else
        {
            ref_vel-= speed_reduction;
        }
      }
    }

    prev_yaw = car_yaw;
}