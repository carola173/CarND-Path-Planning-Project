# CarND-Path-Planning-Project

Self-Driving Car Engineer Nanodegree Program

#### Goal

In this project we have to generate a reference trajectory for our car to drive in the virtual environment given in the simulator. Details about the car's localization and sensor fusion data is given and there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. Collision should be avoided at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

Steps that I followed to achieve the goal:
1. Car was made tp drive below the speed limit of 50mph.
2. Took the current lane as drivable until there was not car in front of it that has slower speed.
3. If car in front is slow in speed and lane change is needed then after smooth transition the car changes the lane and also avoids the collision.
4. In case the lane change is not possible then the speed of the car slows down.

# Rubic Criteria:

#### 1. The code compiles correctly.

The code complies when the cmake and make is used.

#### 2. The car is able to drive at least 4.32 miles without incident.

yes the car drives at least 4.32miles without any collision.

#### 3.The car drives according to the speed limit.

Yes the car doesn't cross the maximum speed limit i.e. 50MPH

#### 4. Max Acceleration and Jerk are not Exceeded.

Yes the car doesn't cross the max accleration and jerk limit.

#### 5. Car does not have collisions.

During by test run, the car doesn't collides any car or avoid any traffic rules.

#### 6. The car stays in its lane, except for the time between changing lanes.

During my test run the stayed on the lane apart from the lane changing time.

#### 7. The car is able to change lanes

Car changes lane when car in front of it is slower and a relative smooth lane change can take place

# Model Documentation

We are using the spline library to generate the future trajectory waypoints for the car to follow.

The following code is used to detect if we need to change the lane or not based on the details given by the sensor fusion part. All the sensor fusion details are given by the simulator as an input to the C++ code
```	
	for (int i=0;i<sensor_fusion.size();i++){
				// The "d" value would specify the lane position of the other cars, that we get from the 6 position
				float d= sensor_fusion[i][6];
				int car_lane_dir=-1;
				if(d > 0 && d< 4){
					car_lane_dir=0;
				}
				else if (d> 4 && d< 8)
					car_lane_dir=1;
				else if (d> 8 && d< 12)
					car_lane_dir=2;
				if(car_lane_dir < 0)
					continue;
				// Getting the car speed details from the sensor fusion readings
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_car_speed = sqrt(vx*vx + vy*vy);
                double check_car_s_val = sensor_fusion[i][5];
				check_car_s_val += (double)(previous_path_size*0.02*check_car_speed);
				// Checking our car lane with respect to the other cars on the road so as to take the proper action
				cout<<"Main Car lane position  : "<<lane << endl;
				if(car_lane_dir - lane ==0)
				{	
					if(check_car_s_val > car_s && (check_car_s_val - car_s) < 30)
						front_car = true; // The current car is in front of out car
					
				}
				// setting the value for front_car variable as true only when current vehicle is ahead and the difference between the two vehicle is sufficient large
				else if (car_lane_dir - lane == -1)
				{
					if( (car_s - 30) < check_car_s_val && (car_s + 30) > check_car_s_val)
						left_car = true; // Move to the left lane
				}
				else if (car_lane_dir - lane == 1)
				{
					if((car_s - 30) < check_car_s_val && (car_s + 30) > check_car_s_val)	
						right_car =true;
				}
			}
```

The following code snippet shows how we use the spine library to get the next wavepoints
```
            vector<double> next_waypoint_0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_waypoint_1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_waypoint_2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
```
Also put these points into a vector as

```
            ptsx.push_back(next_waypoint_0[0]);
            ptsx.push_back(next_waypoint_1[0]);
            ptsx.push_back(next_waypoint_2[0]);

            ptsy.push_back(next_waypoint_0[1]);
            ptsy.push_back(next_waypoint_1[1]);
            ptsy.push_back(next_waypoint_2[1]);
```
To fill the paths points using spline, we do the following:
```
	for( int i = 1; i < 50 - previous_path_x.size(); i++ ) {
              ref_velocity += speed_difference;
              if ( ref_velocity > MAX_SPEED_LIMIT ) {
                ref_velocity = MAX_SPEED_LIMIT;
              } else if ( ref_velocity < MAX_ACC_LIMIT ) {
                ref_velocity = MAX_ACC_LIMIT;
              }
              double N = target_dist/(0.02*ref_velocity/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
```











