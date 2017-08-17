//
// Created by Syed Aziz Enam on 8/11/17.
//
#include "PathPlanner.h"
#include <cmath>
#include "spline.h"
#include <iostream>
using namespace std;

const double speed_limit = 50.0;

PathPlanner::PathPlanner(WAYPOINT_MAP &map) {

    this->ref_vel = 0.0;
    this->lane = 1;
    this->fsm_state = FSM_STATE::STRAIGHT;
    this->map = map;
}


vector<vector<double>> PathPlanner::generateTrajectory(CAR_STATE &carState,
                                          vector<vector<double>> previous_path,
                                          vector<vector<double>> sensor_fusion) {

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    gcarState = carState;
    gprevious_path = previous_path;
    gsensor_fusion = sensor_fusion;

    PathPlanner::BehaviorPlanner();

    PathPlanner::pushTrajectoryValues(next_x_vals, next_y_vals);

    return {next_x_vals, next_y_vals};
};

void PathPlanner::BehaviorPlanner(){

    if( this->fsm_state == FSM_STATE::STRAIGHT) {
        PathPlanner::stateStraight();
    }
    else if( this->fsm_state == FSM_STATE::MATCH_SPEED){
        PathPlanner::stateMatchSpeed();
    }
    else if( this->fsm_state == FSM_STATE::CHECK_LANES){
        PathPlanner::stateCheckLanes();
    }
    else if( this->fsm_state == FSM_STATE::CHANGE_LANE){
        PathPlanner::stateChangeLane();
    }
};

void PathPlanner::stateStraight() {

    for(int i=0; i<gsensor_fusion.size(); i++) {
        vector<double> traffic_car = gsensor_fusion[i];
        double traffic_car_d = traffic_car[6];
        double traffic_car_s = traffic_car[5];

        if(abs(traffic_car_d - gcarState.d) < 2.0
           && (traffic_car_s - gcarState.s) > 0
           && (traffic_car_s - gcarState.s) < 30) {

            this->match_car_id = traffic_car[0];
            this->fsm_state = FSM_STATE::MATCH_SPEED;
            cout << "CHANGED STATE TO MATCH_SPEED"<< endl;
        }
    }

    if(this->ref_vel < (speed_limit-0.5)) {
        this->ref_vel += 0.25;
    }
    else if (this->ref_vel >= speed_limit){
        this->ref_vel -= 0.25;
    }
};

void PathPlanner::stateMatchSpeed() {

    vector<double> front_car;
    vector<double> left_car = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    vector<double> right_car = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};;

    for( int i=0; i<gsensor_fusion.size(); i++) {
        vector<double> sensorfusion_car = gsensor_fusion[i];
        if (this->match_car_id == sensorfusion_car[0]){
            front_car = sensorfusion_car;
        }
    }

    if(abs(front_car[6] - gcarState.d) > 2.0 ||
            (front_car[5] - gcarState.s) > 30) {
        this->fsm_state = FSM_STATE::STRAIGHT;
        cout << "CHANGED STATE TO STRAIGHT"<< endl;
    }

    double match_vel = sqrt((front_car[3]*front_car[3]) + (front_car[4]*front_car[4]));

    double delta_vel = (match_vel - this->ref_vel)/50;
    this->ref_vel += delta_vel;

    if (abs(match_vel - this->ref_vel)/(match_vel) < 0.75) {
        this->fsm_state = FSM_STATE::CHECK_LANES;
        cout << "CHANGED STATE TO CHECk_LANES"<< endl;
    }

};

void PathPlanner::stateCheckLanes() {

    vector<double> front_car;
    vector<vector<double>> left_cars;
    vector<vector<double>> right_cars;

    double left_lane = this->gcarState.d - 4.0;
    double right_lane = this->gcarState.d + 4.0;

    while (left_lane < 0.0) { left_lane += 12.0;}
    while (right_lane > 12.0) { right_lane -= 12.0;}


    for (int i = 0; i < gsensor_fusion.size(); i++) {
        vector<double> sensorfusion_car = gsensor_fusion[i];
        if (this->match_car_id == sensorfusion_car[0]) {
            front_car = sensorfusion_car;
        }

        if (abs(this->gcarState.s - sensorfusion_car[5]) < 30.0) {
            if (abs(sensorfusion_car[6] - left_lane) < 2.0) {
                left_cars.push_back(sensorfusion_car);
            }
            else if (abs(sensorfusion_car[6] - right_lane) < 2.0) {
                right_cars.push_back(sensorfusion_car);
            }
        }
    }

    if (left_cars.empty() && this->lane != 0) {

        this->lane -= 1;
        this->fsm_state = FSM_STATE::CHANGE_LANE;
        cout << "CHANGED STATE TO CHANGE_LANE"<< endl;
    }
    else if (right_cars.empty() && this->lane != 2) {

        this->lane += 1;
        this->fsm_state = FSM_STATE::CHANGE_LANE;
        cout << "CHANGED STATE TO CHANGE_LANE"<< endl;
    }
    else {
        this->fsm_state = FSM_STATE::STRAIGHT;
        cout << "CHANGED STATE TO STRAIGHT"<< endl;
    }
};

void PathPlanner::stateChangeLane() {

    if(this->ref_vel < (speed_limit-0.5)) {
        this->ref_vel += 0.25;
    }

    if (abs(this->gcarState.d - (2+4*this->lane)) < 0.5){
        this->fsm_state = FSM_STATE::STRAIGHT;
        cout << "CHANGED STATE TO STRAIGHT"<< endl;
    }
}

void PathPlanner::pushTrajectoryValues(vector<double> &next_x_vals, vector<double> &next_y_vals){

    double pos_x;
    double pos_x2;
    double pos_y;
    double pos_y2;
    double angle;
    int path_size = gprevious_path[0].size();

    vector<double> previous_path_x = gprevious_path[0];
    vector<double> previous_path_y = gprevious_path[1];

    vector<double> spline_ptsx;
    vector<double> spline_ptsy;

    for(int i = 0; i < path_size; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    if(path_size < 2)
    {
        pos_x = gcarState.x;
        pos_y = gcarState.y;
        angle = gcarState.yaw_r;

        pos_x2 = pos_x - cos(angle);
        pos_y2 = pos_y - sin(angle);
    }
    else
    {
        pos_x = previous_path_x[path_size-1];
        pos_y = previous_path_y[path_size-1];

        pos_x2 = previous_path_x[path_size-2];
        pos_y2 = previous_path_y[path_size-2];
        angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    }

    vector<double> way_point0 = getXY(gcarState.s+30, (2+4*this->lane), this->map.s, this->map.x, this->map.y);
    vector<double> way_point1 = getXY(gcarState.s+60, (2+4*this->lane), this->map.s, this->map.x, this->map.y);
    vector<double> way_point2 = getXY(gcarState.s+90, (2+4*this->lane), this->map.s, this->map.x, this->map.y);

    spline_ptsx.push_back(pos_x2);
    spline_ptsx.push_back(pos_x);
    spline_ptsx.push_back(way_point0[0]);
    spline_ptsx.push_back(way_point1[0]);
    spline_ptsx.push_back(way_point2[0]);

    spline_ptsy.push_back(pos_y2);
    spline_ptsy.push_back(pos_y);
    spline_ptsy.push_back(way_point0[1]);
    spline_ptsy.push_back(way_point1[1]);
    spline_ptsy.push_back(way_point2[1]);

    for (int i=0; i< spline_ptsx.size(); i++) {

        double shift_x = spline_ptsx[i] - pos_x;
        double shift_y = spline_ptsy[i] - pos_y;

        spline_ptsx[i] = shift_x*cos(-angle) - shift_y*sin(-angle);
        spline_ptsy[i] = shift_x*sin(-angle) + shift_y*cos(-angle);
    }

    tk::spline spline;
    spline.set_points(spline_ptsx, spline_ptsy);

    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_dist = distance(0.0, 0.0, target_x, target_y);
    double x_point = 0.0;
    double y_point;

    double N = (target_dist/(0.02*this->ref_vel/2.24));

    for(int i = 0; i < 50-path_size; i++) {

        x_point += (target_x)/N;
        y_point = spline(x_point);

        double shift_x = x_point*cos(angle) - y_point*sin(angle);
        double shift_y = x_point*sin(angle) + y_point*cos(angle);

        shift_x += pos_x;
        shift_y += pos_y;

        next_x_vals.push_back(shift_x);
        next_y_vals.push_back(shift_y);
    }
};