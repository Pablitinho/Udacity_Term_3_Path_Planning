#include "path_planning.h"
#include "math.h"
#include "float.h"


//----------------------------------------------------------------
path_planning::path_planning()
{
    lane = 1;
    ref_vel=0.0;
    vehicle_in_front=false;
    prev_car_action= DRIVING_AHEAD;

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
bool car_is_behind(double check_car_s, double car_s)
{
     double diff_s = car_s - check_car_s;
     if (diff_s>20)
     {
         printf("Car behind: diff: %lf car_s: %lf other_car: %lf\n",diff_s,car_s,check_car_s);
         return true;
     }
     else 
     {
         return false;
     }
}
//----------------------------------------------------------------
bool car_slow(double ego_speed, double car_speed)
{
     if (ego_speed>(car_speed+5))
     {
         return true;
     }
     else 
     {
         return false;
     }
}
//----------------------------------------------------------------
bool inside_lane(double car_d,int lane)
{
    //int lane_d = get_car_line(car_d);

    float diff_d = (car_d-(float)(lane*4+2));

    if (fabs(diff_d)>1)
    {
        return false;
    }
    else 
    {
        return true;
    }
}

vehicle_in_front_t path_planning::detect_vehicle_in_front(std::vector<std::vector<double>> sensor_fusion, double car_s)
{
    vehicle_in_front_t vehicle_in_front;
    vehicle_in_front.distance_front_vehicle=-1.0;
    vehicle_in_front.too_close=false;
    vehicle_in_front.vehicle_in_front_speed=-1.0;
    vehicle_in_front.acceleration = 1;

    vector<vector<double>> cars_in_front;
	for (int id_sen_f = 0; id_sen_f < sensor_fusion.size(); id_sen_f++)
    {
		double s_val = sensor_fusion[id_sen_f][5];
		double d_val = sensor_fusion[id_sen_f][6];

        if (get_car_line(d_val)==this->lane && s_val>car_s)
        {
			cars_in_front.push_back(sensor_fusion[id_sen_f]);	
		}
	}  

	// get closest car in front of us
	vector<double> lead_car;
	if (cars_in_front.size() > 0)
    {
		lead_car = cars_in_front[0];
		for (int i = 0; i < cars_in_front.size(); i++)
        {
			double current_s = cars_in_front[i][5];
			if (current_s < lead_car[5])
            {
				lead_car = cars_in_front[i];
		    }
	    }

		// check if lead car is close enough to follow
        double lead_s = lead_car[5];
        if (lead_s - car_s < MIN_FRONT_DISTANCE)
        {
                double lvx = lead_car[3];
                double lvy = lead_car[4];
                double lead_v = sqrt(lvx*lvx + lvy*lvy) *2.24;
                // change 'a' to decelerate, change ref_v to lead_v
                vehicle_in_front.acceleration = -1;
                vehicle_in_front.vehicle_in_front_speed = lead_v;
                vehicle_in_front.too_close = true;
                vehicle_in_front.distance_front_vehicle = lead_s - car_s;
        }
	}  

    return vehicle_in_front;
}
//----------------------------------------------------------------
lateral_lanes_t path_planning::check_free_lanes(std::vector<std::vector<double>> sensor_fusion,double ego_speed, double car_s,double prev_size)
{
    lateral_lanes_t lateral_lanes;
    lateral_lanes.free_left=true;
    lateral_lanes.free_right=true;
    lateral_lanes.free_forward_lanes = false;

    lateral_lanes.free_left_far = true;
    lateral_lanes.free_center_far = true;
    lateral_lanes.free_right_far = true;
    

    for ( int id = 0; id < sensor_fusion.size(); id++ ) 
    {
        float d = sensor_fusion[id][6];

        int car_lane= get_car_line(d);

        // Find car speed.
        double vx = sensor_fusion[id][3];
        double vy = sensor_fusion[id][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[id][5];
        
       // check_car_s+=((double)prev_size*0.02*check_speed);
    
        double diff_s = check_car_s-car_s;

        // Check if the car is in front of us 
        bool car_forward = diff_s>0 && diff_s<50 ? true : false;

        // Near cars
        if ((car_lane - this->lane==-1) && fabs(diff_s)<30)
        {
            lateral_lanes.free_left = false;
        }
        else if ((car_lane - this->lane==1) && fabs(diff_s)<30)
        {
            lateral_lanes.free_right = false;
        }

        // Far away cars
        if ((car_lane==0) && car_forward)
        {
            lateral_lanes.free_left_far = false;
        }
        else if ((car_lane==1)  && car_forward)
        {
            lateral_lanes.free_center_far = false;
        }
        else if ((car_lane==2) && car_forward)
        {
            lateral_lanes.free_right_far = false;
        }
        
    }
    // Check if there is far away free lanes
    if (lateral_lanes.free_left_far || lateral_lanes.free_center_far || lateral_lanes.free_right_far)
    {
        lateral_lanes.free_forward_lanes =true;
    }
    else
    {
        lateral_lanes.free_forward_lanes =false;
    }

    return lateral_lanes;
}
//----------------------------------------------------------------
void path_planning::estimate_ref_velocity(std::vector<std::vector<double>> sensor_fusion,double car_s,double car_d,double car_yaw, int prev_size,double ego_speed)
{

    vehicle_in_front_t vehicle_in_front = detect_vehicle_in_front(sensor_fusion, car_s);
    lateral_lanes_t lateral_lanes = check_free_lanes(sensor_fusion,ego_speed, car_s, prev_size);

    printf("In Front Car: %s, ",vehicle_in_front.too_close== true ? "True": "False");
    printf("Free Left: %s, ",lateral_lanes.free_left== true ? "True": "False");
    printf("Free Right: %s, ",lateral_lanes.free_right== true ? "True": "False");
    printf("Free Left Far: %s, ",lateral_lanes.free_left_far== true ? "True": "False");
    printf("Free Center Far: %s, ",lateral_lanes.free_center_far== true ? "True": "False");
    printf("Free Right Far: %s, ",lateral_lanes.free_right_far== true ? "True": "False");
    //printf("Free Lanes: %s ",lateral_lanes.free_forward_lanes== true ? "True": "False");

    printf("\n");

    if (vehicle_in_front.too_close)
    {
        if (lateral_lanes.free_left && lane>0 && inside_lane(car_d, lane) && lateral_lanes.free_forward_lanes)
        {
            if ((lateral_lanes.free_left_far && lane ==1) || (lateral_lanes.free_center_far && lane ==2))
            {
                lane--;
            }
        }
        else if (lateral_lanes.free_right && lane!=2 && inside_lane(car_d, lane) && lateral_lanes.free_forward_lanes)
        {
            if ((lateral_lanes.free_right_far && lane ==1) || (lateral_lanes.free_center_far && lane ==0))
            {
                lane++;
            }
        }
        else
        {
            if (vehicle_in_front.distance_front_vehicle<MIN_FRONT_DISTANCE/4.0)
            {
                ref_vel-= VELOCITY_DEC;
            }
            else if (vehicle_in_front.distance_front_vehicle<MIN_FRONT_DISTANCE/2.0)
            {
                if (ego_speed>vehicle_in_front.vehicle_in_front_speed)  
                {
                    ref_vel-= VELOCITY_DEC;
                }  
                else
                {
                    ref_vel = vehicle_in_front.vehicle_in_front_speed;
                }
            }
            else
            {
                ref_vel-= VELOCITY_DEC;
            }
        }

    }
    else
    {
        ref_vel+=VELOCITY_INC;
        if (ref_vel>MAX_SPEED)
        {
            ref_vel=MAX_SPEED;
        }
        
    }
    //printf("Car d: %lf \n",car_d);

    /*for ( int i = 0; i < sensor_fusion.size(); i++ ) 
    {
        float d = sensor_fusion[i][6];

        int car_lane= get_car_line(d);
        {
            // Find car speed.
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            
            check_car_s+=((double)prev_size*0.02*check_speed);
       
            double diff_s = check_car_s-car_s;

            if (car_lane==lane && (check_car_s>car_s) && (diff_s<40))
            {
                too_close = true;

                distance_front_vehicle = diff_s;
                vehicle_in_front_speed=ref_vel;
            }
            else if ((car_lane - lane==-1) && fabs(diff_s)<20)//!car_is_behind(check_car_s,car_s) && !car_slow(ego_speed,check_speed))////&&(check_car_s>(car_s-gap) && (check_car_s< (car_s+gap))))
            {
                free_left = false;
            }
            else if ((car_lane - lane==1) && fabs(diff_s)<20)// !car_is_behind(check_car_s,car_s) && !car_slow(ego_speed,check_speed))//&& fabs(diff_s)<20)//&&(check_car_s>(car_s-gap) && (check_car_s< (car_s+gap))))
            {
                free_right = false;
            }
        }
    }*/
    //printf("\n");

                /*if (car_lane==lane && (check_car_s>car_s) && (diff_s<60))
            {
               printf("Car in front lane: %i diffs: %lf ",car_lane,fabs(diff_s));
            }
            else if ((car_lane - lane==-1) && !car_is_behind(check_car_s,car_s) && !car_slow(ego_speed,check_speed))
            {
                printf("Car Left lane: %i diffs: %lf ",car_lane,fabs(diff_s));

            }
            else if ((car_lane - lane==1) && !car_is_behind(check_car_s,car_s) && !car_slow(ego_speed,check_speed))
            {
                printf("Car Right lane: %i diffs: %lf ",car_lane,fabs(diff_s));
            }*/
    
    /*if (too_close)
    {
      // Turn to the left
      if (free_left && lane>0 && ego_speed>30 && inside_lane(car_d, lane) 
           && (prev_car_action!=TURNING_LEFT || prev_car_action!=TURNING_RIGHT))
      {
        lane--;
        car_action=TURNING_LEFT;
        printf("TURNING_LEFT\n");

      }// Turn to the right
      else if (free_right && lane!=2 && ego_speed>30 && inside_lane(car_d, lane) 
               && (prev_car_action!=TURNING_LEFT || prev_car_action!=TURNING_RIGHT))
      {
        lane++;
        car_action=TURNING_RIGHT;
        printf("TURNING_RIGHT\n");
      }
      else
      {
        if (!car_slow(ref_vel,vehicle_in_front_speed))
        {
            if (distance_front_vehicle<5)
            {
                ref_vel=0;
                printf("ref_vel(Level 0): %lf , vel.other: %lf\n",ref_vel,vehicle_in_front_speed);
            }
            if (distance_front_vehicle<10)
            {
                ref_vel-=VELOCITY_DEC_EMERGENCY;
                printf("ref_vel(Level 1): %lf , vel.other: %lf\n",ref_vel,vehicle_in_front_speed);
            }
            else if (distance_front_vehicle<30)
            {
                ref_vel-= VELOCITY_DEC;
            printf("ref_vel(Level 2): %lf , vel.other: %lf\n",ref_vel,vehicle_in_front_speed);
            }
            else
            {
                printf("ref_vel(Level 3): %lf , vel.other: %lf\n",ref_vel,vehicle_in_front_speed);
                ref_vel-= VELOCITY_DEC_SLOW;
            }
        }
        else
        {
            ref_vel+=VELOCITY_INC;
            if (ref_vel>MAX_SPEED)
            {
                ref_vel=MAX_SPEED;
            }
            printf("ref_vel(Level 4): %lf , vel.other: %lf\n",ref_vel,vehicle_in_front_speed);
        }
        car_action = DEACCELERATING;
        printf("DEACCELERATION\n");

      }
    }
    else 
    {
        ref_vel+=VELOCITY_INC;
        if (ref_vel>MAX_SPEED)
        {
            ref_vel=MAX_SPEED;
        }
        car_action=DRIVING_AHEAD;
    }*/

    prev_car_action = car_action;
    prev_yaw = car_yaw;
}