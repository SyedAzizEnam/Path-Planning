//
// Created by Syed Aziz Enam on 8/11/17.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "utils.h"


class PathPlanner {

public:

    WAYPOINT_MAP map;

    double ref_vel;

    double match_car_id;

    int lane;

    enum FSM_STATE{
        STRAIGHT,
        MATCH_SPEED,
        CHECK_LANES,
        CHANGE_LANE
    } fsm_state;

    explicit PathPlanner(WAYPOINT_MAP &map);

    virtual ~PathPlanner() {}

    std::vector<vector<double>> generateTrajectory(CAR_STATE &carState, std::vector<std::vector<double>> previous_path, std::vector<std::vector<double>> sensor_fusion);

private:

    CAR_STATE gcarState;
    std::vector<vector<double>> gprevious_path;
    std::vector<vector<double>> gsensor_fusion;

    void BehaviorPlanner();

    void stateStraight();

    void stateMatchSpeed();

    void stateCheckLanes();

    void stateChangeLane();

    void pushTrajectoryValues(std::vector<double> &next_x_vals, std::vector<double> &next_y_vals);
};

#endif //PATH_PLANNING_PATHPLANNER_H


