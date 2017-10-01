# CarND-Path-Planning-Project

## Goals
> In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also, the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Implementation


The code is mainly based on the code shown in [Path Planning Project Walkthrough and Q&A](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d). In particular the behavior planning was improved the make lane changes more reliable. All relevant source code can be found in `src\main.cpp`

## Behaviour Planner

The implemented behavior planner offers basic functions, such as avoiding collision with the car in front of the car, and changing lanes to the left or to the right, if the car ahead is too slow. The behavior planner determines what lane the car should be in. The outcome of this is then to feed to the path planner by setting a target lane.

First of all other cars are detected based on the relative position of the car.
The current lane in which the car is driving is determined by the variable `lane`.
Each lane measures 4 meters wide.

From the sensor fusion module we get a list of all detected cars. By looping through the list and comparing the distances and current speed, we can get the current distance and position of the next car in each lane.

```c++
double center_car_velocity = max_velocity;
double left_car_velocity   = max_velocity;
double right_car_velocity  = max_velocity;
double center_car_dist = 100.0;
double left_car_dist   = 100.0;
double right_car_dist  = 100.0;

for( int i = 0; i < sensor_fusion.size(); i++ )
{
    float d = sensor_fusion[i][6];
    double other_vx    = sensor_fusion[i][3];
    double other_vy    = sensor_fusion[i][4];
    double other_v     = sqrt( other_vx*other_vx + other_vy*other_vy );
    double other_check = sensor_fusion[i][5]; // s-value of other car

    // Predict the distance at the end of previous planning assuming constant velocity
    double other_dist = (double)other_check + (double)prev_size*time_step*other_v;

    if( d < (2+4*lane+2) && d > (2+4*lane-2) ) {
        // check s-value greater than mine
        if ((other_dist > car_s - offset_s) && ((other_dist - car_s) < center_car_dist) ) {
            center_car_dist     = other_dist - car_s;
            center_car_velocity = other_v;
        }
    }
    if( d < (2+4*(lane-1)+2) && d > (2+4*(lane-1)-2) && lane > 0 ) {
        if ((other_dist > car_s - offset_s) && (other_dist - car_s) < left_car_dist) {
            left_car_dist      = other_dist - car_s;
            left_car_velocity  = other_v;
        }
    }
    if( d < (2+4*(lane+1)+2) && d > (2+4*(lane+1)-2) && lane < 2 ) {
        if ((other_dist > car_s - offset_s) && (other_dist - car_s) < right_car_dist) {
            right_car_dist     = other_dist - car_s;
            right_car_velocity = other_v;
        }
    }
}
```

Based on this information a decision is made on which lane to drive.
The path planners strategy is calculated by using cost functions in order to find the best path.

Three strategies are considered: Changing lanes to the left, changing lanes to the right and staying in the same lane. For each of the three strategies, a velocity cost and a distance cost is evaluated. Velocity cost determines how slow the vehicles on another lane is going and the distance weight determines how far away the vehicle is relative to our own position.
The two different costs are weighted by `velocity_weight ` and `distance_weight `

```c++
// Calculate the cost of each lane
double center_cost = velocity_weight / center_car_velocity + distance_weight / abs(center_car_dist);
double left_cost   = velocity_weight / left_car_velocity   + distance_weight / abs(left_car_dist);
double right_cost  = velocity_weight / right_car_velocity  + distance_weight / abs(right_car_dist);
if( lane == 0 ) left_cost  = 1.0;
if( lane == 2 ) right_cost = 1.0;

// Change lane if the next lane has lower cost and the vehicle is not already changing lanes
double changing_lane = true;
if( car_d < (2+4*lane+2) && car_d > (2+4*lane-2) ) {
    changing_lane = false;
}

```

Based on the cost a new lane is then chosen.

```c++
// change lanes
if( min( left_cost, right_cost) < center_cost && !changing_lane)
{
    // decide to which lane to change
    if( right_cost < left_cost && lane < 2 )
        lane += 1;
    else if( lane > 0 )
        lane -= 1;
}
```


In addition, the reference speed of the vehicle is controlled by the behavior planner. A simple control algorithm is implemented, which accelerates the car by `max_acceleration` as long as the current velocity set point `ref_velocity` is below the maximum velocity `max_velocity`. In case of a very slow vehicle ahead, an emergency brake is implemented doubling the value of (negative) `max_acceleration`.

```c++
// Slow down and speed down based on signal in constant acceleration
if( center_car_dist < (ref_velocity/max_velocity)*50 )
{
    // emergency break
    if( ref_velocity > center_car_velocity + 10 )
        ref_velocity -= 2 * max_acceleration;

    if( ref_velocity > center_car_velocity )
        ref_velocity -= max_acceleration;
}
else
{
    if( ref_velocity < max_velocity )
        ref_velocity += max_acceleration;
}
```

## Path Planner

The path planner calculates waypoints based on the speed setpoint `ref_velocity` and the lane to drive in, given by `lane`. In order to get smooth trajectories, splines are generated based on three sparse anker waypoints. The implementation is mainly based on the walkthrough implementation.

```c++
// use Freenet coordinates that are evely spaced with a distance of 30
vector<double> next_wp0 = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);

for (int i = 0; i < ptsx.size(); i++)
{
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
}
```

Using frenet coordinates (coordinates that project the street on a straight line) simplifies the math for calculating the waypoint world coordinates, which can simply be done by calculating:

```
    x = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw)
    y = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw)
```

Finally, a smooth list of waypoints is generated and then send to the simulator. I used [spline tool](http://kluge.in-chemnitz.de/opensource/spline/) for C++ for spline calculation.

### Race mode

I implemented a race mode that will result in a much faster driving. It can be enabled by setting:

```c++
bool race_mode = true;
```

## Summary

This implementation fulfills the given requirement of driving around the track without incidents for at least 4.3 miles:

![finish](./img/finish.png)

For the given track, this implementation seems to be sufficient.
For different scenarios, it might be important to check more frequently for other vehicles and their predicted state. Especially when the car is going slow it might collide with faster vehicles while changing lanes. In this case, possible collisions must be detected and the behavior should re-planed.
