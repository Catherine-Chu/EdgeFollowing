//
// Created by 褚文杰 on 2020/3/15.
//

#ifndef EDGEFOLLOWING_AGENT_H
#define EDGEFOLLOWING_AGENT_H

#include <vector>
#include <random>
#include <algorithm>
#include <iostream>

extern const int width;
extern const int height;
extern const double PI;
const double DISTANCE_MAX = 0x7f7f7f7f;
//const double DISIRED_DISTANCE = sqrt(2);
const double DISIRED_DISTANCE = 1;
const int GRADIENT_MAX = 0x7f7f7f7f;
const double GRADIENT_DIS_G = sqrt(2);
const int RAND_ID_MAX = 10000000;
enum RState {start=0, wait_to_move=1, move_while_outside=2, move_while_inside=3, joined_shape=4};
extern std::vector<RState> agent_running_states;
extern std::vector<std::vector<int>> agent_maps;
extern std::vector<std::vector<int>> agent_poses;
extern std::vector<int> agent_gradient_values;
extern std::vector<int> agent_local_ids;
extern std::vector<std::vector<int>> grids;

extern std::vector<std::vector<int>> agent_prev_poses;
extern std::vector<std::vector<int>> agent_prev_nei_ids; // n*3

class Agent {
public:
    int id;
    int local_id;
    int pos_x;
    int pos_y;
    int sense_radius;
    int gradient;
    bool gradient_seed;
    bool is_seed_robot;
    bool id_generated;
    RState state;
    Agent();
    Agent(int id);
//    Agent(int x, int y, int r=2*sqrt(2), bool g_seed=false);
    void set_config(int x, int y, int r=2*sqrt(2), int gradient=GRADIENT_MAX, bool g_seed=false, bool r_seed=false, bool is_reset=true);
    void set_id(int id);
    void generate_locally_unique_id();
    void gradient_formation();
    bool edge_following();
    bool shape_self_assembly();
private:
    double prev; //prev will store previously measured distance to nearest neighbor
    double yeild_distance;
    int prev_x, prev_y;
    //supplement functions for edge_following
    std::vector<int> get_neighbors();
    std::vector<int> get_stationary_neighbors();
    bool has_moving_neighbors(int r=2);
    int get_moving_agent_ahead(int f_nei=-1);
    double measure_distance_t_neighbor(int n_id);
    bool move_forward(int f_nei);
    bool move_forward_counterclockwise(int f_nei);
    bool move_forward_clockwise(int f_nei);
    void turn_clockwise(int dx, int dy, int& n_pos_x, int& n_pos_y, double angle=PI/2);
    void turn_counter_clockwise(int dx, int dy, int & n_pos_x, int & n_pos_y, double angle=PI/2);
    std::vector<int> stay_to_clockwise(int f_nei);
    std::vector<int> stay_to_counter_clockwise(int f_nei);
    void stay_to_move(int& n_pos_x, int& n_pos_y);
    bool stay_to_move();
    bool try_find_action(int & n_pos_x, int & n_pos_y);
    void rand_move(int & n_pos_x, int & n_pos_y, int f_nei);
};


#endif //EDGEFOLLOWING_AGENT_H
