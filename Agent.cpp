//
// Created by 褚文杰 on 2020/3/15.
//

#include "Agent.h"

using namespace std;

Agent::Agent() {
    id = -1;
    pos_x = -1;
    prev_x = pos_x;
    pos_y = -1;
    prev_y = pos_y;
    sense_radius = 0;
    prev = DISTANCE_MAX;
    yeild_distance = sqrt(2);
    gradient = GRADIENT_MAX;
    gradient_seed = false;
    is_seed_robot = false;
    id_generated = false;
    state = start;
    local_id = -1;
}

Agent::Agent(int id) {
    this->id = id;
    pos_x = -1;
    prev_x = pos_x;
    pos_y = -1;
    prev_y = pos_y;
    sense_radius = 0;
    prev = DISTANCE_MAX;
    yeild_distance = sqrt(2);
    gradient = GRADIENT_MAX;
    gradient_seed = false;
    is_seed_robot = false;
    id_generated = false;
    state = start;
    local_id = -1;
}

//Agent::Agent(int x, int y, int r, bool g_seed){
//    id = -1;
//    pos_x = x;
//    pos_y = y;
//    prev_x = pos_x;
//    prev_y = pos_y;
//    sense_radius = r;
//    prev = DISTANCE_MAX;
//    yeild_distance = sqrt(2);
//    gradient = GRADIENT_MAX;
//    gradient_seed = g_seed;
//    is_seed_robot = false;
//    id_generated = false;
//    state = start;
//}

void Agent::set_id(int id) {
    this->id = id;
}

void Agent::set_config(int x, int y, int r, int gradient, bool g_seed, bool r_seed, bool is_reset) {
    //默认必须在有id的情况下才能调用set_config,可以加一个assert(id>-1)
    //只能在初始化之后调用set_config,调用即默认agent本身状态重置,prev_x与prev_y需与pos_x与pos_y统一
    if (is_reset) {
        prev = DISTANCE_MAX;
        id_generated = false;
        state = start;
        local_id = -1;
        agent_running_states[id] = state;
        agent_prev_nei_ids[id].clear();
    }
    pos_x = x;
    pos_y = y;
    prev_x = pos_x;
    prev_y = pos_y;
    this->gradient = gradient;
    sense_radius = r;
    gradient_seed = g_seed;
    is_seed_robot = r_seed;
    agent_poses[id] = {x, y};
    agent_maps[x][y] = id;
    agent_gradient_values[id] = gradient;
    agent_prev_poses[id] = {x, y};
}

void Agent::generate_locally_unique_id() {
    if (not id_generated) {
        srand(time(NULL));
        //time(NULL)精确到秒,所以如果程序运行非常快,在一秒内生成的local_id都是一样的,导致前期第一次生成所有agent local id都一样
        //所以在他们第二次进入wait-to-move重新生成local-id时会进入while循环,导致前几个iteration耗时长,后面local-id都稳定后就快了
        local_id = rand() % RAND_ID_MAX;
        id_generated = true;
    } else {
        vector<int> neighbors = get_neighbors();
        for (int &nei:neighbors) {
//            while(local_id == agent_local_ids[nei]){
            while (local_id == agent_local_ids[nei]) {
                id_generated = false;
                generate_locally_unique_id();
            }
            //把if改成while基本可以确保生成unique local id,但是这个while的开销其实很高
            //if是无法保证可以得到unique id的,只能是把RAND_ID_MAX设置的尽量大,来减小概率
        }
    }
    agent_local_ids[id] = local_id;
}

void Agent::gradient_formation() {
    if (gradient_seed) {
        gradient = 0;
    } else {
        if (is_seed_robot) {
            gradient = 1;
        } else {
            gradient = GRADIENT_MAX;
            vector<int> neighbors = get_neighbors();
            for (int &nei:neighbors) {
                if (measure_distance_t_neighbor(nei) <= GRADIENT_DIS_G) {
                    if (agent_running_states[nei]!=move_while_outside && agent_running_states[nei]!=move_while_inside && agent_gradient_values[nei] < gradient) {
                        gradient = agent_gradient_values[nei];
                    }
                }
            }
            if (gradient < GRADIENT_MAX)
                gradient += 1;
        }
    }
    agent_gradient_values[id] = gradient;
}

bool Agent::edge_following() {
    bool following_state = true;
    double current = DISTANCE_MAX;
    vector<int> neighbors = get_stationary_neighbors();
    int f_nei = -1;
    for (int &nei:neighbors) {
        double m_dis = measure_distance_t_neighbor(nei);
        if (m_dis < current) {
            current = m_dis;
            f_nei = nei;
        }
    }

    //如果视野范围内没有其他不动的agent了,说明上一步有问题,需要退回
    if (f_nei == -1) {
        cout << "No visible stationary neighbor can be followed." << endl;
        int n_pos_x = prev_x;
        int n_pos_y = prev_y;
        if (n_pos_x != pos_x or n_pos_y != pos_y) {
            prev_x = pos_x;
            prev_y = pos_y;
            pos_x = n_pos_x;
            pos_y = n_pos_y;
            agent_poses[id] = {pos_x, pos_y};
            agent_prev_poses[id] = {prev_x, prev_y};
            agent_maps[pos_x][pos_y] = id;
            if (agent_maps[prev_x][prev_y] == id) {
                agent_maps[prev_x][prev_y] = -1;
            }
        }
        prev = current;
        following_state = false;
        return following_state;
    }

    //视野范围内有其他不动的agent,作为要被follow的点,则将其加入记录中
    agent_prev_nei_ids[id].push_back(f_nei);
    if (agent_prev_nei_ids[id].size() > 3) {
        agent_prev_nei_ids[id].erase(agent_prev_nei_ids[id].begin());
    }

//    if (id == 35) {
//        cout << "in edge following :" << f_nei << ", prev is: " << prev << ", current is: " << current << endl;
//    }

    //根据上一步中距上一个follow点的距离,和当前步中距当前follow点的距离,以及目标follow距离判定应该往哪里运动
    //首先根据prev判定之前是否已经开始following了,从不动的状态开始的话,没有forward的概念,需要特殊处理
    if (prev < DISTANCE_MAX) {
        //说明已经开始edge following了,有forward动作存在
        if (current < DISIRED_DISTANCE) {
            if (prev < current) {
                // getting closer to the desired edge-following distance,
                // in our experiment, this branch should never be triggered.
                following_state = move_forward(f_nei);
            } else {
                following_state = move_forward_counterclockwise(f_nei);
            }
        } else if (current > DISIRED_DISTANCE) {
            if (prev > current) {
                // getting closer to the desired edge-following distance
                following_state = move_forward(f_nei);
            } else {
                following_state = move_forward_clockwise(f_nei);
            }
        } else {
            if (prev == current) {
                // stay to the desired edge-following distance
                following_state = move_forward(f_nei);
            } else if (prev > current) {
//                following_state = move_forward_counterclockwise(f_nei);
                following_state = move_forward(f_nei);
            } else {
                following_state = move_forward_clockwise(f_nei);
            }
        }
        if (not following_state) {
            cout << "Don't move in following state." << endl;
            //这种情况下,虽然没有动,但可能是允许范围内的没有动,所以先返回true,使实验可以继续
            return true;
        }
    } else {
        //从不动到运动第一步
        following_state = stay_to_move();
        if (not following_state) {
            cout << "Experiment failed!" << endl;
            return following_state;
        }
    }

    //记录到follow的点的距离,供下一步判断
    prev = current;
    return following_state;
}

bool Agent::shape_self_assembly() {
    bool assembly_state = true;
    gradient_formation();
    switch (state) {
        case start: {
            if (gradient_seed || is_seed_robot) {
                state = joined_shape;
            } else {
                if (gradient < GRADIENT_MAX) {
                    state = wait_to_move;
                }
            }
            break;
        }
        case wait_to_move: {
            // decide when to start moving
            generate_locally_unique_id();
            if (not has_moving_neighbors(1)) {
                // find highest gradient value among neighbors
                int h = 0;
                vector<int> neighbors = get_neighbors();
                vector<int> same_highest_gradient;
                for (int &nei:neighbors) {
                    if (h < agent_gradient_values[nei]) {
                        h = agent_gradient_values[nei];
                        same_highest_gradient.clear();
                        same_highest_gradient.push_back(nei);
                    } else if (h == agent_gradient_values[nei]) {
                        same_highest_gradient.push_back(nei);
                    }
                }
//                if(pos_x==4){
//                    cout<<"pos_x=4"<<" ";
//                    cout<<id<<" "<<pos_x<<" "<<pos_y<<" "<<gradient<<" "<<h<<" "<<same_highest_gradient[0]<<endl;
//                }
//                if(id==239 or id==224){
//                    cout<<"id "<<id<<" "<<pos_x<<" "<<pos_y<<" "<<gradient<<" "<<h<<" "<<same_highest_gradient.size()<<endl;
//                    for(int j=pos_y+4; j>=pos_y-4; j--){
//                        for(int i=pos_x-4; i<=pos_x+4; i++){
//                            if(agent_maps[i][j]>=0){
//                                cout<<agent_gradient_values[agent_maps[i][j]]<<" ";
//                            }else{
//                                cout<<"INF"<<" ";
//                            }
//                        }
//                        cout<<endl;
//                    }
//                }
                if (gradient > h) {
                    state = move_while_outside;
                } else if (gradient == h) {
                    bool flag = true;
                    for (int &h_nei:same_highest_gradient) {
                        if (local_id > agent_local_ids[h_nei]) {
                            flag = flag && true;
                        } else {
                            flag = flag && false;
                            break;
                        }
                    }
                    if (flag) {
                        state = move_while_outside;
                    }
                }
            }
            break;
        }
        case move_while_outside: {
            // edge-follow while outside desired shape
            if (grids[pos_x][pos_y] == 1) {
                state = move_while_inside;
                break;
            }
            int a_nei = get_moving_agent_ahead();
//            if (id == 780) {
//                cout << a_nei;
//                if (a_nei >= 0) {
//                    cout << " x,y:" << agent_poses[a_nei][0] << " " << agent_poses[a_nei][1] << ". pos_x,pos_y:"
//                         << pos_x << " " << pos_y << endl;
//                } else {
//                    cout << endl;
//                }
//            }
            if (measure_distance_t_neighbor(a_nei) > yeild_distance) {
//                if (id == 780) {
//                    cout << "start edge following with pos: " << pos_x << "," << pos_y << endl;
//                }
                assembly_state = edge_following();
//                if (id == 780) {
//                    cout << "current pos:" << pos_x << "," << pos_y << endl;
//                    cout << assembly_state << endl;
//                }
            } else {
                // motion: stop
                break;
            }
//            assembly_state = edge_following();
            break;
        }
        case move_while_inside: {
            if (grids[pos_x][pos_y] == 0) {
                state = joined_shape;
                break;
            }
            vector<int> neighbors = get_neighbors();
            double m = DISTANCE_MAX;
            int n_nei;
            for (int &nei:neighbors) {
                double m_dis = measure_distance_t_neighbor(nei);
                if (m_dis < m) {
                    m = m_dis;
                    n_nei = nei;
                }
                if (agent_running_states[nei]==joined_shape && measure_distance_t_neighbor(nei) <= GRADIENT_DIS_G) {
                    if (gradient <= agent_gradient_values[nei]) {
                        state = joined_shape;
                    }
                }
            }
            if (state == joined_shape) {
                break;
            }

            int a_nei = get_moving_agent_ahead(n_nei);
            if (measure_distance_t_neighbor(a_nei) >= yeild_distance) {
                assembly_state = edge_following();
            } else {
                // motion: stop
                break;
            }
//            assembly_state = edge_following();
            break;
        }
        case joined_shape: {
            // motion: stop
            break;
        }
    }
    if (state != joined_shape)
        gradient_formation();
    agent_running_states[id] = state;
//    cout << "id: " << id << ", pos=(" << pos_x << "," << pos_y << "), local_id: " << local_id << ", gradient: "
//         << gradient << ", state: " << state << endl;

    return assembly_state;
}

vector<int> Agent::get_neighbors() {
    vector<int> res;
    for (int i = max(0, int(pos_x - sense_radius)); i <= min(int(pos_x + sense_radius), width); i++) {
        for (int j = max(0, int(pos_y - sense_radius)); j <= min(int(pos_y + sense_radius), height); j++) {
            if ((i != pos_x or j != pos_y) and sqrt(pow(i - pos_x, 2) + pow(j - pos_y, 2)) <= sense_radius) {
                int nei = agent_maps[i][j];
                if (nei >= 0) {
                    res.push_back(nei);
                }
            }
        }
    }
    return res;
}


vector<int> Agent::get_stationary_neighbors() {
    vector<int> res;
    for (int i = max(0, int(pos_x - sense_radius)); i <= min(int(pos_x + sense_radius), width); i++) {
        for (int j = max(0, int(pos_y - sense_radius)); j <= min(int(pos_y + sense_radius), height); j++) {
            if ((i != pos_x or j != pos_y) and sqrt(pow(i - pos_x, 2) + pow(j - pos_y, 2)) <= sense_radius) {
                int nei = agent_maps[i][j];
                if (nei >= 0) {
                    if ((agent_running_states[nei] != move_while_inside and
                         agent_running_states[nei] != move_while_outside)) {
                        res.push_back(nei);
                    }
                }
            }
        }
    }
    return res;
}

bool Agent::has_moving_neighbors(int r) {
    bool res = false;
    for (int i = max(0, pos_x - r); i <= min(pos_x + r, width); i++) {
        for (int j = max(0, pos_y - r); j <= min(pos_y + r, height); j++) {
            if ((i != pos_x or j != pos_y) and sqrt(pow(i - pos_x, 2) + pow(j - pos_y, 2)) <= r*sqrt(2)) {
                int nei = agent_maps[i][j];
                if (nei >= 0) {
                    if (agent_running_states[nei] == move_while_outside ||
                        agent_running_states[nei] == move_while_inside) {
                        res = true;
                        break;
                    }
                }
            }
        }
        if (res)
            break;
    }
    return res;
}

double Agent::measure_distance_t_neighbor(int n_id) {
    double res;
    if (n_id >= 0 and n_id < agent_poses.size())
        res = sqrt(pow(pos_x - agent_poses[n_id][0], 2) + pow(pos_y - agent_poses[n_id][1], 2));
    else
        res = DISTANCE_MAX;
    return res;
}

int Agent::get_moving_agent_ahead(int f_nei) {
    vector<int> neighbors = get_neighbors();
    if (f_nei == -1) {
        double m = DISTANCE_MAX;
        for (int &nei:neighbors) {
            if (agent_running_states[nei] != move_while_outside and agent_running_states[nei] != move_while_inside) {
                double m_dis = measure_distance_t_neighbor(nei);
                if (m_dis < m) {
                    m = m_dis;
                    f_nei = nei;
                }
            }
        }
    }
    double min_dis_ahead = DISTANCE_MAX;
    int ahead = -1;
    for (int &nei:neighbors) {
//        if(id == 780 && pos_x == 73 && pos_y == 36){
//            cout<<"780 agent prev pos: ("<<prev_x<<", "<<prev_y<<")"<<endl;
//            cout<<"nei id: "<<nei<<", nei pos: ("<<agent_poses[nei][0]<<", "<<agent_poses[nei][1]<<"), "
//            <<"agent state: "<<agent_running_states[nei]<<", prev pos: ("<<agent_prev_poses[nei][0]<<", "
//            <<agent_prev_poses[nei][1]<<")"<<endl;
//        }
        if (agent_running_states[nei] == move_while_inside || agent_running_states[nei] == move_while_outside) {
            int temp_ahead = -1;
//            for (int &nei_id : agent_prev_nei_ids[nei]) {
//                if (nei_id == f_nei) {
//                    int mv1[2]={agent_prev_poses[nei_id][0]-prev_x,agent_prev_poses[nei_id][1]-prev_y};
//                    int mv2[2]={prev_y-pos_y, pos_x-prev_x};
//                    if(mv1[0]*mv2[0]+mv1[1]*mv2[1]>=0){
//                        temp_ahead = nei;
//                        break;
//                    }
//                }
//            }
            int mv1[2]={agent_prev_poses[nei][0]-prev_x,agent_prev_poses[nei][1]-prev_y};
            int mv2[2]={pos_y-prev_y, prev_x-pos_x};
            int mv3[2]={pos_x-prev_x,pos_y-prev_y};
            int rev_mv1[2]={-mv1[0],-mv1[1]};
            int rev_mv2[2]={agent_poses[nei][1]-agent_prev_poses[nei][1],agent_prev_poses[nei][0]-agent_poses[nei][0]};
            int rev_mv3[2]={agent_poses[nei][0]-agent_prev_poses[nei][0],agent_poses[nei][1]-agent_prev_poses[nei][1]};
            int l_mv2[2]={prev_y-pos_y, pos_x-prev_x};
            int l_rev_mv2[2]={-rev_mv2[0],-rev_mv2[1]};
//            cout<<"mv values: "<<mv1[0]<<" "<<mv1[1]<<" "<<mv2[0]<<" "<<mv2[1]<<" "<<mv3[0]<<" "<<mv3[1]<<endl;
            if(mv1[0]*mv2[0]+mv1[1]*mv2[1]>=0 &&
            (mv1[0]*mv2[0]+mv1[1]*mv2[1])/sqrt((mv1[0]*mv1[0]+mv1[1]*mv1[1])*(mv2[0]*mv2[0]+mv2[1]*mv2[1]))<1
            && mv1[0]*mv3[0]+mv1[1]*mv3[1]>0){
                //neibor在右,可能为前驱
                int num1 = rev_mv1[0]*rev_mv2[0]+rev_mv1[1]*rev_mv2[1];
                double num2 = (rev_mv1[0]*rev_mv2[0]+rev_mv1[1]*rev_mv2[1])/sqrt((rev_mv1[0]*rev_mv1[0]+rev_mv1[1]*rev_mv1[1])*(rev_mv2[0]*rev_mv2[0]+rev_mv2[1]*rev_mv2[1]));
                int num3 = rev_mv1[0]*rev_mv3[0]+rev_mv1[1]*rev_mv3[1];
                //若互为之右,互不影响,不算作前驱
                if(not (num1>=0 && num2<1 && num3>0)){
                    //否则视为前驱
                    temp_ahead = nei;
                }
            }else if(mv1[0]*l_mv2[0]+mv1[1]*l_mv2[1]>=0 &&
                     (mv1[0]*l_mv2[0]+mv1[1]*l_mv2[1])/sqrt((mv1[0]*mv1[0]+mv1[1]*mv1[1])*(l_mv2[0]*l_mv2[0]+l_mv2[1]*l_mv2[1]))<1
                     && mv1[0]*mv3[0]+mv1[1]*mv3[1]>0){
                //neibor在左
                int num1 = rev_mv1[0]*l_rev_mv2[0]+rev_mv1[1]*l_rev_mv2[1];
                double num2 = (rev_mv1[0]*l_rev_mv2[0]+rev_mv1[1]*l_rev_mv2[1])/sqrt((rev_mv1[0]*rev_mv1[0]+rev_mv1[1]*rev_mv1[1])*(l_rev_mv2[0]*l_rev_mv2[0]+l_rev_mv2[1]*l_rev_mv2[1]));
                int num3 = rev_mv1[0]*rev_mv3[0]+rev_mv1[1]*rev_mv3[1];
                if(num1>=0 && num2<1 && num3>0){
                    //互为之左,nei也是有可能为前驱的,谁沿着原有方向继续走会被挡住,则谁为后
                    if(agent_maps[pos_x+pos_x-prev_x][pos_y+pos_y-prev_y]>=0) {
                        temp_ahead = nei;
                    }
                }
            }
            if (temp_ahead != -1) {
                double m_dis = measure_distance_t_neighbor(temp_ahead);
                if (m_dis < min_dis_ahead) {
                    min_dis_ahead = m_dis;
                    ahead = temp_ahead;
                }
            }
        }
    }
    return ahead;
}

bool Agent::stay_to_move() {
    bool success_move = true;
    int l_x = pos_x - 1;
    int r_x = pos_x + 1;
    int u_y = pos_y + 1;
    int d_y = pos_y - 1;
    int n_pos_x = pos_x, n_pos_y = pos_y;
    //穷举所有的边缘情况
    if (agent_maps[pos_x][d_y] >= 0 and agent_maps[pos_x][u_y] == -1) {
        //说明在上部边缘,下有连通,向上运动要么直接脱离连通,要么和另一个在连通图内的agent顶点相连
        if (agent_maps[l_x][pos_y] >= 0 and agent_maps[r_x][pos_y] == -1) {
            //如果左侧有agent存在,右侧没有,说明位于右上角,此时应该向右运动(同时说明在右部边缘)
            n_pos_x = r_x;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] == -1) {
            //如果右侧有agent存在,左侧没有,说明位于左上角,此时应该向上运动(同时说明在左部边缘)
            n_pos_y = u_y;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] >= 0) {
            //左右均有agent存在,应该向上运动
            n_pos_y = u_y;
        } else {
            //左右均没有agent存在,应该向右运动(顺时针)
            n_pos_x = r_x;
        }
    } else if (agent_maps[pos_x][d_y] == -1 and agent_maps[pos_x][u_y] >= 0) {
        //说明在下部边缘,上有连通,向下运动要么直接脱离连通,要么和另一个在连通图内的agent顶点相连
        if (agent_maps[l_x][pos_y] >= 0 and agent_maps[r_x][pos_y] == -1) {
            //如果左侧有agent存在,右侧没有,说明位于右下角,此时应该向下运动(同时说明在右部边缘)
            n_pos_y = d_y;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] == -1) {
            //如果右侧有agent存在,左侧没有,说明位于左下角,此时应该向左运动(同时说明在左部边缘)
            n_pos_x = l_x;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] >= 0) {
            //左右均有agent存在,应该向下运动
            n_pos_y = d_y;
        } else {
            //左右均没有agent存在,应该向左运动(顺时针)
            n_pos_x = l_x;
        }
    } else if (agent_maps[pos_x][d_y] == -1 and agent_maps[pos_x][u_y] == -1) {
        //上下均没有agent,那么左右必然至少有一边有agent
        if (agent_maps[l_x][pos_y] >= 0 and agent_maps[r_x][pos_y] == -1) {
            //如果左侧有agent存在,右侧没有,此时应该向下运动(同时说明在右部边缘)
            n_pos_y = d_y;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] == -1) {
            //如果右侧有agent存在,左侧没有,此时应该向上运动(同时说明在左部边缘)
            n_pos_y = u_y;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] >= 0) {
            //左右均有agent存在,这种情况形状应该是成环的,向上或向下均可打破环,但要看是从环内部运动还是外部运动
            //seed在下,向下运动则为环内部运动,而目标在环外部,所以必须保证在环外部运动,不然会在环内转一圈回到原点再转出去
            //在初始化足够合理的情况下不应该进到这里,万一进到这里要给一个提醒,暂时处理成随机向上或者向下
            cout << "Circle exists in the initialization!" << endl;
            srand(time(NULL));
            if (double (rand())/RAND_MAX > 0.5){
                n_pos_y = u_y;
            }else{
                n_pos_y = d_y;
            }
        } else {
            //左右均没有agent存在
//            cout << "Unvalid initialization!" << endl;
            if (agent_maps[l_x][d_y] >= 0) {
                //左下角有
                if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] == -1) {
                    //仅左下角有,向下
                    n_pos_y = pos_y - 1;
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] == -1) {
                    //左下+右下,向右
                    n_pos_x = pos_x + 1;
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] >= 0) {
                    //左下+左上,向下
                    n_pos_y = pos_y - 1;
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] == -1) {
                    //左下+右上,上下左右随机
                    srand(time(NULL));
                    double prb = double(rand() % RAND_MAX) / (RAND_MAX - 1);
                    if (prb <= 0.25) {
                        n_pos_y = pos_y + 1;
                    } else if (prb <= 0.5) {
                        n_pos_y = pos_y - 1;
                    } else if (prb <= 0.75) {
                        n_pos_x = pos_x - 1;
                    } else {
                        n_pos_x = pos_x + 1;
                    }
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] == -1) {
                    //只有左上没有,向上
                    n_pos_y = pos_y + 1;
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] >= 0) {
                    //只有右上没有,向右
                    n_pos_x = pos_x + 1;
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] >= 0) {
                    //只有右下没有,向下
                    n_pos_y = pos_y - 1;
                } else {
                    //都有,上下左右随机
                    srand(time(NULL));
                    double prb = double(rand() % RAND_MAX) / (RAND_MAX - 1);
                    if (prb <= 0.25) {
                        n_pos_y = pos_y + 1;
                    } else if (prb <= 0.5) {
                        n_pos_y = pos_y - 1;
                    } else if (prb <= 0.75) {
                        n_pos_x = pos_x - 1;
                    } else {
                        n_pos_x = pos_x + 1;
                    }
                }
            } else {
                //左下角没有
                if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] == -1) {
                    //都没有
                    //出现初始状态即孤立的点,则为不合法的初始状态
                    cout << "Unvalid initialization!" << endl;
                    success_move = false;
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] == -1) {
                    //仅右下,向右
                    n_pos_x = pos_x + 1;
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] >= 0) {
                    //仅左上,向左
                    n_pos_x = pos_x - 1;
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] == -1) {
                    //仅右上,向上
                    n_pos_y = pos_y + 1;
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] == -1) {
                    //左下+左上没有,向上
                    n_pos_y = pos_y + 1;
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] >= 0) {
                    //左下+右上没有,上下左右随机
                    srand(time(NULL));
                    double prb = double(rand() % RAND_MAX) / (RAND_MAX - 1);
                    if (prb <= 0.25) {
                        n_pos_y = pos_y + 1;
                    } else if (prb <= 0.5) {
                        n_pos_y = pos_y - 1;
                    } else if (prb <= 0.75) {
                        n_pos_x = pos_x - 1;
                    } else {
                        n_pos_x = pos_x + 1;
                    }
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] >= 0) {
                    //左下+右下没有,向左
                    n_pos_x = pos_x - 1;
                } else {
                    //仅左下没有,向左
                    n_pos_x = pos_x - 1;
                }
            }
        }
    } else {
        //上下均有agent,则不可能是左右两边均有agent,否则不可能可以动,进到这个函数就不合理
        if (agent_maps[l_x][pos_y] >= 0 and agent_maps[r_x][pos_y] == -1) {
            //如果左侧有agent存在,右侧没有,此时应该向右运动(同时说明在右部边缘)
            n_pos_x = r_x;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] == -1) {
            //如果右侧有agent存在,左侧没有,此时应该向左运动(同时说明在左部边缘)
            n_pos_x = l_x;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] >= 0) {
            //左右均有agent存在,不可移动,进到这个函数就不合理
//            cout<<pos_x<<" "<<pos_y<<" "<<id<<" "<<state<<endl;
            cout << "Unvalid selection, possible gradient error, agent can't move!" << endl;
            success_move = false;
        } else {
            //左右均无agent存在,这种情况形状应该是成环的,向左或向右均可打破环,但要看是从环内部运动还是外部运动
            //seed在右,向右运动则为环内部运动,而目标在环外部,所以必须保证在环外部运动,不然会在环内转一圈回到原点再转出去
            //在初始化足够合理的情况下不应该进到这里,万一进到这里要给一个提醒,暂时处理成随机向左或者向右
            cout << "Circle exists in the initialization!" << endl;
            srand(time(NULL));
            if (double(rand() % RAND_MAX) / (RAND_MAX - 1) > 0.5) {
                n_pos_x = r_x;
            } else {
                n_pos_x = l_x;
            }

        }
    }

    if (success_move) {
        if (n_pos_x != pos_x or n_pos_y != pos_y) {
            prev_x = pos_x;
            prev_y = pos_y;
            pos_x = n_pos_x;
            pos_y = n_pos_y;
            agent_poses[id] = {pos_x, pos_y};
            agent_prev_poses[id] = {prev_x, prev_y};
            agent_maps[pos_x][pos_y] = id;
            if (agent_maps[prev_x][prev_y] == id) {
                agent_maps[prev_x][prev_y] = -1;
            }
        }
    }

    return success_move;
}

void Agent::rand_move(int &n_pos_x, int &n_pos_y, int f_nei) {
    vector<vector<int>> option_positions;
    int x = agent_poses[f_nei][0];
    int y = agent_poses[f_nei][1];
    for (int i = max(0, x - 1); i <= min(width, x + 1); i++) {
        for (int j = max(0, y - 1); j < min(height, y + 1); j++) {
            if (agent_maps[i][j] < 0 and fabs(i - pos_x) <= 1 and fabs(j - pos_y) <= 1) {
                option_positions.push_back({i, j});
            }
        }
    }
    if (option_positions.size() > 0) {
        srand(time(NULL));
        int rand_p = rand() % option_positions.size();
        n_pos_x = option_positions[rand_p][0];
        n_pos_y = option_positions[rand_p][1];
    }
}

bool Agent::try_find_action(int &n_pos_x, int &n_pos_y) {
    bool success_move = true;
    int l_x = pos_x - 1;
    int r_x = pos_x + 1;
    int u_y = pos_y + 1;
    int d_y = pos_y - 1;
    n_pos_x = pos_x;
    n_pos_y = pos_y;
    //穷举所有的边缘情况
    if (agent_maps[pos_x][d_y] >= 0 and agent_maps[pos_x][u_y] == -1) {
        //说明在上部边缘,下有连通,向上运动要么直接脱离连通,要么和另一个在连通图内的agent顶点相连
        if (agent_maps[l_x][pos_y] >= 0 and agent_maps[r_x][pos_y] == -1) {
            //如果左侧有agent存在,右侧没有,说明位于右上角,此时应该向右运动(同时说明在右部边缘)
            n_pos_x = r_x;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] == -1) {
            //如果右侧有agent存在,左侧没有,说明位于左上角,此时应该向上运动(同时说明在左部边缘)
            n_pos_y = u_y;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] >= 0) {
            //左右均有agent存在,应该向上运动
            n_pos_y = u_y;
        } else {
            //左右均没有agent存在,应该向右运动(顺时针)
            n_pos_x = r_x;
        }
    } else if (agent_maps[pos_x][d_y] == -1 and agent_maps[pos_x][u_y] >= 0) {
        //说明在下部边缘,上有连通,向下运动要么直接脱离连通,要么和另一个在连通图内的agent顶点相连
        if (agent_maps[l_x][pos_y] >= 0 and agent_maps[r_x][pos_y] == -1) {
            //如果左侧有agent存在,右侧没有,说明位于右下角,此时应该向下运动(同时说明在右部边缘)
            n_pos_y = d_y;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] == -1) {
            //如果右侧有agent存在,左侧没有,说明位于左下角,此时应该向左运动(同时说明在左部边缘)
            n_pos_x = l_x;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] >= 0) {
            //左右均有agent存在,应该向下运动
            n_pos_y = d_y;
        } else {
            //左右均没有agent存在,应该向左运动(顺时针)
            n_pos_x = l_x;
        }
    } else if (agent_maps[pos_x][d_y] == -1 and agent_maps[pos_x][u_y] == -1) {
        //上下均没有agent,那么左右必然至少有一边有agent
        if (agent_maps[l_x][pos_y] >= 0 and agent_maps[r_x][pos_y] == -1) {
            //如果左侧有agent存在,右侧没有,此时应该向下运动(同时说明在右部边缘)
            n_pos_y = d_y;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] == -1) {
            //如果右侧有agent存在,左侧没有,此时应该向上运动(同时说明在左部边缘)
            n_pos_y = u_y;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] >= 0) {
            //左右均有agent存在,这种情况形状应该是成环的,向上或向下均可打破环,但要看是从环内部运动还是外部运动
            //seed在下,向下运动则为环内部运动,而目标在环外部,所以必须保证在环外部运动,不然会在环内转一圈回到原点再转出去
            //在初始化足够合理的情况下不应该进到这里,万一进到这里要给一个提醒,暂时处理成随机向上或者向下
            cout << "Circle exists in the situation!" << endl;
            srand(time(NULL));
            if (double (rand())/RAND_MAX > 0.5){
                n_pos_y = u_y;
            }else{
                n_pos_y = d_y;
            }
        } else {
            //左右均没有agent存在
//            cout << "Unvalid situation!" << endl;
            if (agent_maps[l_x][d_y] >= 0) {
                //左下角有
                if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] == -1) {
                    //仅左下角有,向下
                    n_pos_y = pos_y - 1;
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] == -1) {
                    //左下+右下,向右
                    n_pos_x = pos_x + 1;
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] >= 0) {
                    //左下+左上,向下
                    n_pos_y = pos_y - 1;
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] == -1) {
                    //左下+右上,上下左右随机
                    srand(time(NULL));
                    double prb = double(rand() % RAND_MAX) / (RAND_MAX - 1);
                    if (prb <= 0.25) {
                        n_pos_y = pos_y + 1;
                    } else if (prb <= 0.5) {
                        n_pos_y = pos_y - 1;
                    } else if (prb <= 0.75) {
                        n_pos_x = pos_x - 1;
                    } else {
                        n_pos_x = pos_x + 1;
                    }
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] == -1) {
                    //只有左上没有,向上
                    n_pos_y = pos_y + 1;
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] >= 0) {
                    //只有右上没有,向右
                    n_pos_x = pos_x + 1;
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] >= 0) {
                    //只有右下没有,向下
                    n_pos_y = pos_y - 1;
                } else {
                    //都有,上下左右随机
                    srand(time(NULL));
                    double prb = double(rand() % RAND_MAX) / (RAND_MAX - 1);
                    if (prb <= 0.25) {
                        n_pos_y = pos_y + 1;
                    } else if (prb <= 0.5) {
                        n_pos_y = pos_y - 1;
                    } else if (prb <= 0.75) {
                        n_pos_x = pos_x - 1;
                    } else {
                        n_pos_x = pos_x + 1;
                    }
                }
            } else {
                //左下角没有
                if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] == -1) {
                    //都没有
                    //出现初始状态即孤立的点,则为不合法的初始状态
                    cout << "Unvalid situation!" << endl;
                    success_move = false;
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] == -1) {
                    //仅右下,向右
                    n_pos_x = pos_x + 1;
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] >= 0) {
                    //仅左上,向左
                    n_pos_x = pos_x - 1;
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] == -1) {
                    //仅右上,向上
                    n_pos_y = pos_y + 1;
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] == -1) {
                    //左下+左上没有,向上
                    n_pos_y = pos_y + 1;
                } else if (agent_maps[r_x][d_y] >= 0 and agent_maps[r_x][u_y] == -1 and agent_maps[l_x][u_y] >= 0) {
                    //左下+右上没有,上下左右随机
                    srand(time(NULL));
                    double prb = double(rand() % RAND_MAX) / (RAND_MAX - 1);
                    if (prb <= 0.25) {
                        n_pos_y = pos_y + 1;
                    } else if (prb <= 0.5) {
                        n_pos_y = pos_y - 1;
                    } else if (prb <= 0.75) {
                        n_pos_x = pos_x - 1;
                    } else {
                        n_pos_x = pos_x + 1;
                    }
                } else if (agent_maps[r_x][d_y] == -1 and agent_maps[r_x][u_y] >= 0 and agent_maps[l_x][u_y] >= 0) {
                    //左下+右下没有,向左
                    n_pos_x = pos_x - 1;
                } else {
                    //仅左下没有,向左
                    n_pos_x = pos_x - 1;
                }
            }
        }
    } else {
        //上下均有agent,则不可能是左右两边均有agent,否则不可能可以动,进到这个函数就不合理
        if (agent_maps[l_x][pos_y] >= 0 and agent_maps[r_x][pos_y] == -1) {
            //如果左侧有agent存在,右侧没有,此时应该向右运动(同时说明在右部边缘)
            n_pos_x = r_x;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] == -1) {
            //如果右侧有agent存在,左侧没有,此时应该向左运动(同时说明在左部边缘)
            n_pos_x = l_x;
        } else if (agent_maps[r_x][pos_y] >= 0 and agent_maps[l_x][pos_y] >= 0) {
            //左右均有agent存在,不可移动,进到这个函数就不合理
            cout << "Unvalid situation, possible gradient error, agent can't move!" << endl;
            success_move = false;
        } else {
            //左右均无agent存在,这种情况形状应该是成环的,向左或向右均可打破环,但要看是从环内部运动还是外部运动
            //seed在右,向右运动则为环内部运动,而目标在环外部,所以必须保证在环外部运动,不然会在环内转一圈回到原点再转出去
            //在初始化足够合理的情况下不应该进到这里,万一进到这里要给一个提醒,暂时处理成随机向左或者向右
            cout << "Circle exists in the situation!" << endl;
            srand(time(NULL));
            if (double(rand() % RAND_MAX) / (RAND_MAX - 1) > 0.5) {
                n_pos_x = r_x;
            } else {
                n_pos_x = l_x;
            }

        }
    }

    return success_move;
}

bool Agent::move_forward(int f_nei) {
    int n_pos_x = pos_x + pos_x - prev_x;
    int n_pos_y = pos_y + pos_y - prev_y;


    if (agent_maps[n_pos_x][n_pos_y] >= 0) {
        // the target waypoint has been occupied;
        bool res = try_find_action(n_pos_x, n_pos_y);
        if (not res) {
            rand_move(n_pos_x, n_pos_y, f_nei);
        }
    }

    if (n_pos_x != pos_x or n_pos_y != pos_y) {
        prev_x = pos_x;
        prev_y = pos_y;
        pos_x = n_pos_x;
        pos_y = n_pos_y;
        agent_poses[id] = {pos_x, pos_y};
        agent_prev_poses[id] = {prev_x, prev_y};
        agent_maps[pos_x][pos_y] = id;
        if (agent_maps[prev_x][prev_y] == id) {
            agent_maps[prev_x][prev_y] = -1;
        }
        return true;
    } else {
        return false;
    }
}

void Agent::turn_clockwise(int dx, int dy, int &n_pos_x, int &n_pos_y, double angle) {
    if (dx == 0 and dy == 0) {
        cout << "No forward action, should restart again." << endl;
    } else {
        if (angle == PI / 4) {
            if (dx == 0 and dy == 1) {
                n_pos_x = pos_x + 1;
                n_pos_y = pos_y + 1;
            } else if (dx == 1 and dy == 1) {
                n_pos_x = pos_x + 1;
                n_pos_y = pos_y;
            } else if (dx == 1 and dy == 0) {
                n_pos_x = pos_x + 1;
                n_pos_y = pos_y - 1;
            } else if (dx == 1 and dy == -1) {
                n_pos_x = pos_x;
                n_pos_y = pos_y - 1;
            } else if (dx == 0 and dy == -1) {
                n_pos_x = pos_x - 1;
                n_pos_y = pos_y - 1;
            } else if (dx == -1 and dy == -1) {
                n_pos_x = pos_x - 1;
                n_pos_y = pos_y;
            } else if (dx == -1 and dy == 0) {
                n_pos_x = pos_x - 1;
                n_pos_y = pos_y + 1;
            } else if (dx == -1 and dy == 1) {
                n_pos_x = pos_x;
                n_pos_y = pos_y + 1;
            }
        } else {
            if ((dx == 0 and dy == 1) or (dx == 1 and dy == 1)) {
                n_pos_x = pos_x + 1;
                n_pos_y = pos_y;
            } else if ((dx == 1 and dy == 0) or (dx == 1 and dy == -1)) {
                n_pos_x = pos_x;
                n_pos_y = pos_y - 1;
            } else if ((dx == 0 and dy == -1) or (dx == -1 and dy == -1)) {
                n_pos_x = pos_x - 1;
                n_pos_y = pos_y;
            } else if ((dx == -1 and dy == 0) or (dx == -1 and dy == 1)) {
                n_pos_x = pos_x;
                n_pos_y = pos_y + 1;
            }
        }
    }
}

bool Agent::move_forward_clockwise(int f_nei) {
    int dx = pos_x - prev_x;
    int dy = pos_y - prev_y;
    int n_pos_x = pos_x, n_pos_y = pos_y;

    turn_clockwise(dx, dy, n_pos_x, n_pos_y, PI / 2);

    if (agent_maps[n_pos_x][n_pos_y] >= 0) {
        // the target waypoint has been occupied;
        bool res = try_find_action(n_pos_x, n_pos_y);
        if (not res) {
            rand_move(n_pos_x, n_pos_y, f_nei);
        }
    }

    if (n_pos_x != pos_x or n_pos_y != pos_y) {
        prev_x = pos_x;
        prev_y = pos_y;
        pos_x = n_pos_x;
        pos_y = n_pos_y;
        agent_poses[id] = {pos_x, pos_y};
        agent_prev_poses[id] = {prev_x, prev_y};
        agent_maps[pos_x][pos_y] = id;
        if (agent_maps[prev_x][prev_y] == id) {
            agent_maps[prev_x][prev_y] = -1;
        }
        return true;
    } else {
        return false;
    }
}

void Agent::turn_counter_clockwise(int dx, int dy, int &n_pos_x, int &n_pos_y, double angle) {
    if (dx == 0 and dy == 0) {
        cout << "No forward action, should restart again." << endl;
    } else {
        if (angle == PI / 4) {
            if (dx == 0 and dy == 1) {
                n_pos_x = pos_x - 1;
                n_pos_y = pos_y + 1;
            } else if (dx == 1 and dy == 1) {
                n_pos_x = pos_x;
                n_pos_y = pos_y + 1;
            } else if (dx == 1 and dy == 0) {
                n_pos_x = pos_x + 1;
                n_pos_y = pos_y + 1;
            } else if (dx == 1 and dy == -1) {
                n_pos_x = pos_x + 1;
                n_pos_y = pos_y;
            } else if (dx == 0 and dy == -1) {
                n_pos_x = pos_x + 1;
                n_pos_y = pos_y - 1;
            } else if (dx == -1 and dy == -1) {
                n_pos_x = pos_x;
                n_pos_y = pos_y - 1;
            } else if (dx == -1 and dy == 0) {
                n_pos_x = pos_x - 1;
                n_pos_y = pos_y - 1;
            } else if (dx == -1 and dy == 1) {
                n_pos_x = pos_x - 1;
                n_pos_y = pos_y;
            }
        } else {
            if ((dx == 0 and dy == 1) or (dx == -1 and dy == 1)) {
                n_pos_x = pos_x - 1;
                n_pos_y = pos_y;
            } else if ((dx == -1 and dy == 0) or (dx == -1 and dy == -1)) {
                n_pos_x = pos_x;
                n_pos_y = pos_y - 1;
            } else if ((dx == 0 and dy == -1) or (dx == 1 and dy == -1)) {
                n_pos_x = pos_x + 1;
                n_pos_y = pos_y;
            } else if ((dx == 1 and dy == 0) or (dx == 1 and dy == 1)) {
                n_pos_x = pos_x;
                n_pos_y = pos_y + 1;
            }
        }

    }
}

bool Agent::move_forward_counterclockwise(int f_nei) {
    int dx = pos_x - prev_x;
    int dy = pos_y - prev_y;
    int n_pos_x = pos_x, n_pos_y = pos_y;

    turn_counter_clockwise(dx, dy, n_pos_x, n_pos_y, PI / 2);

    if (agent_maps[n_pos_x][n_pos_y] >= 0) {
        // the target waypoint has been occupied;
        bool res = try_find_action(n_pos_x, n_pos_y);
        if (not res) {
            rand_move(n_pos_x, n_pos_y, f_nei);
        }
    }

    if (n_pos_x != pos_x or n_pos_y != pos_y) {
        prev_x = pos_x;
        prev_y = pos_y;
        pos_x = n_pos_x;
        pos_y = n_pos_y;
        agent_poses[id] = {pos_x, pos_y};
        agent_prev_poses[id] = {prev_x, prev_y};
        agent_maps[pos_x][pos_y] = id;
        if (agent_maps[prev_x][prev_y] == id) {
            agent_maps[prev_x][prev_y] = -1;
        }
        return true;
    } else {
        return false;
    }
}


/*Possible 1*/
//void Agent::edge_following() {
//    double current = DISTANCE_MAX;
//    vector<int> neighbors = get_stationary_neighbors();
//    int f_nei = -1;
//    for(int & nei:neighbors){
//        double m_dis = measure_distance_t_neighbor(nei);
//        if(m_dis<current) {
//            current = m_dis;
//            f_nei = nei;
//        }
//    }
//    //如果视野范围内没有其他不动的agent了,说明上一步有问题,需要退回
////    if(f_nei == -1){
////        int n_pos_x = prev_x;
////        int n_pos_y = prev_y;
////        if(n_pos_x != pos_x or n_pos_y != pos_y) {
////            prev_x = pos_x;
////            prev_y = pos_y;
////            pos_x = n_pos_x;
////            pos_y = n_pos_y;
////            agent_poses[id]={pos_x,pos_y};
////            agent_prev_poses[id] = {prev_x,prev_y};
////            agent_maps[pos_x][pos_y]=id;
////            if(agent_maps[prev_x][prev_y]== id){
////                agent_maps[prev_x][prev_y] = -1;
////            }
////        }
////        prev = current;
////        return;
////    }
//    agent_prev_nei_ids[id].push_back(f_nei);
//    if(agent_prev_nei_ids[id].size()>3){
//        agent_prev_nei_ids[id].erase(agent_prev_nei_ids[id].begin());
//    }
//    if(current<DISIRED_DISTANCE){
//        if(prev<current){
//            // getting closer to the desired edge-following distance,
//            // in our experiment, this branch should never be triggered.
//            move_forward(f_nei);
//        }else{
//            move_forward_counterclockwise(f_nei);
//        }
//    }else{
//        if(prev>current){
//            // getting closer to the desired edge-following distance
//            move_forward(f_nei);
//        }else{
//            move_forward_clockwise(f_nei);
//        }
//    }
//    prev = current;
//}
//
//void Agent::move_forward(int f_nei) {
//    int n_pos_x = pos_x + pos_x - prev_x;
//    int n_pos_y = pos_y + pos_y - prev_y;
//
////    if(pos_x == prev_x and pos_y == prev_y) {
////        // begin with staying still, no concept of forward, just clockwise
////        vector<int> res = stay_to_clockwise(f_nei);
////        n_pos_x = res[0];
////        n_pos_y = res[1];
////    }else {
////        vector<vector<int>> option_positions;
////        if (agent_maps[n_pos_x][n_pos_y] >= 0) {
////            // the target waypoint has been occupied;
////            int x = agent_poses[f_nei][0];
////            int y = agent_poses[f_nei][1];
////            for (int i = max(0, x - 1); i <= min(width, x + 1); i++) {
////                for (int j = max(0, y - 1); j < min(height, y + 1); j++) {
////                    if (agent_maps[i][j] < 0 and fabs(i - pos_x) <= 1 and fabs(j - pos_y) <= 1) {
////                        option_positions.push_back({i, j});
////                    }
////                }
////            }
////        }
////
////        if (option_positions.size() > 0) {
////            srand(time(NULL));
////            int rand_p = rand() % option_positions.size();
////            n_pos_x = option_positions[rand_p][0];
////            n_pos_y = option_positions[rand_p][1];
////        }
////    }
//    if(pos_x == prev_x and pos_y == prev_y) {
//        // begin with staying still, no concept of forward, just away from nei
//        n_pos_x = pos_x + pos_x - agent_poses[f_nei][0];
//        n_pos_y = pos_y + pos_y - agent_poses[f_nei][1];
//    }
//
//    vector<vector<int>> option_positions;
//    if (agent_maps[n_pos_x][n_pos_y] >= 0) {
//        // the target waypoint has been occupied;
//        int x = agent_poses[f_nei][0];
//        int y = agent_poses[f_nei][1];
//        for (int i = max(0, x - 1); i <= min(width, x + 1); i++) {
//            for (int j = max(0, y - 1); j < min(height, y + 1); j++) {
//                if (agent_maps[i][j] < 0 and fabs(i - pos_x) <= 1 and fabs(j - pos_y) <= 1) {
//                    option_positions.push_back({i, j});
//                }
//            }
//        }
//    }
//
//    if (option_positions.size() > 0) {
//        srand(time(NULL));
//        int rand_p = rand() % option_positions.size();
//        n_pos_x = option_positions[rand_p][0];
//        n_pos_y = option_positions[rand_p][1];
//    }
//
//
//    if(n_pos_x != pos_x or n_pos_y != pos_y) {
//        prev_x = pos_x;
//        prev_y = pos_y;
//        pos_x = n_pos_x;
//        pos_y = n_pos_y;
//        agent_poses[id]={pos_x,pos_y};
//        agent_prev_poses[id] = {prev_x,prev_y};
//        agent_maps[pos_x][pos_y]=id;
//        if(agent_maps[prev_x][prev_y]== id){
//            agent_maps[prev_x][prev_y] = -1;
//        }
//    }
//}
//
//void Agent::move_forward_clockwise(int f_nei) {
//    int dx = pos_x - prev_x;
//    int dy = pos_y - prev_y;
//    int n_pos_x, n_pos_y;
//    if(dx == 0 and dy == 0){
//        // start with stay still, no concept of forward, so just away
//        n_pos_x = pos_x + pos_x - agent_poses[f_nei][0];
//        n_pos_y = pos_y + pos_y - agent_poses[f_nei][1];
//    }else{
//        if(dx==0 and dy==1){
//            n_pos_x = pos_x + 1;
//            n_pos_y = pos_y + 1;
//        }else if (dx==1 and dy==1){
//            n_pos_x = pos_x + 1;
//            n_pos_y = pos_y;
//        }else if (dx == 1 and dy==0){
//            n_pos_x = pos_x + 1;
//            n_pos_y = pos_y -1;
//        }else if(dx == 1 and dy==-1){
//            n_pos_x = pos_x;
//            n_pos_y = pos_y -1;
//        }else if(dx == 0 and dy == -1){
//            n_pos_x = pos_x -1 ;
//            n_pos_y = pos_y -1;
//        }else if(dx == -1 and dy == -1){
//            n_pos_x = pos_x -1;
//            n_pos_y = pos_y;
//        }else if(dx == -1 and dy == 0){
//            n_pos_x = pos_x -1;
//            n_pos_y = pos_y+1;
//        }else if(dx == -1 and dy == 1){
//            n_pos_x = pos_x;
//            n_pos_y = pos_y +1;
//        }
//    }
//    vector<vector<int>> option_positions;
//    if (agent_maps[n_pos_x][n_pos_y] >= 0) {
//        // the target waypoint has been occupied;
//        int x = agent_poses[f_nei][0];
//        int y = agent_poses[f_nei][1];
//        for (int i = max(0, x - 1); i <= min(width, x + 1); i++) {
//            for (int j = max(0, y - 1); j < min(height, y + 1); j++) {
//                if (agent_maps[i][j] < 0 and fabs(i - pos_x) <= 1 and fabs(j - pos_y) <= 1) {
//                    option_positions.push_back({i, j});
//                }
//            }
//        }
//    }
//
//    if (option_positions.size() > 0) {
//        srand(time(NULL));
//        int rand_p = rand() % option_positions.size();
//        n_pos_x = option_positions[rand_p][0];
//        n_pos_y = option_positions[rand_p][1];
//    }
////    if(dx == 0 and dy == 0){
////        // start with stay still, no concept of forward, so just clockwise
////        vector<int> res = stay_to_clockwise(f_nei);
////        n_pos_x = res[0];
////        n_pos_y = res[1];
////    }else{
////        if(dx==0 and dy==1){
////            n_pos_x = pos_x + 1;
////            n_pos_y = pos_y + 1;
////        }else if (dx==1 and dy==1){
////            n_pos_x = pos_x + 1;
////            n_pos_y = pos_y;
////        }else if (dx == 1 and dy==0){
////            n_pos_x = pos_x + 1;
////            n_pos_y = pos_y -1;
////        }else if(dx == 1 and dy==-1){
////            n_pos_x = pos_x;
////            n_pos_y = pos_y -1;
////        }else if(dx == 0 and dy == -1){
////            n_pos_x = pos_x -1 ;
////            n_pos_y = pos_y -1;
////        }else if(dx == -1 and dy == -1){
////            n_pos_x = pos_x -1;
////            n_pos_y = pos_y;
////        }else if(dx == -1 and dy == 0){
////            n_pos_x = pos_x -1;
////            n_pos_y = pos_y+1;
////        }else if(dx == -1 and dy == 1){
////            n_pos_x = pos_x;
////            n_pos_y = pos_y +1;
////        }
////        vector<vector<int>> option_positions;
////        if (agent_maps[n_pos_x][n_pos_y] >= 0) {
////            // the target waypoint has been occupied;
////            int x = agent_poses[f_nei][0];
////            int y = agent_poses[f_nei][1];
////            for (int i = max(0, x - 1); i <= min(width, x + 1); i++) {
////                for (int j = max(0, y - 1); j < min(height, y + 1); j++) {
////                    if (agent_maps[i][j] < 0 and fabs(i - pos_x) <= 1 and fabs(j - pos_y) <= 1) {
////                        option_positions.push_back({i, j});
////                    }
////                }
////            }
////        }
////
////        if (option_positions.size() > 0) {
////            srand(time(NULL));
////            int rand_p = rand() % option_positions.size();
////            n_pos_x = option_positions[rand_p][0];
////            n_pos_y = option_positions[rand_p][1];
////        }
////    }
//
//    if(n_pos_x != pos_x or n_pos_y != pos_y) {
//        prev_x = pos_x;
//        prev_y = pos_y;
//        pos_x = n_pos_x;
//        pos_y = n_pos_y;
//        agent_poses[id]={pos_x,pos_y};
//        agent_prev_poses[id] = {prev_x,prev_y};
//        agent_maps[pos_x][pos_y]=id;
//        if(agent_maps[prev_x][prev_y]== id){
//            agent_maps[prev_x][prev_y] = -1;
//        }
//    }
//}
//
//void Agent::move_forward_counterclockwise(int f_nei) {
//    int dx = pos_x - prev_x;
//    int dy = pos_y - prev_y;
//    int n_pos_x, n_pos_y;
////    if(dx == 0 and dy == 0){
////        // start with stay still, no concept of forward, so just counnterclockwise
////        vector<int> res = stay_to_counter_clockwise(f_nei);
////        n_pos_x = res[0];
////        n_pos_y = res[1];
////    }else{
////        if(dx==0 and dy==1){
////            n_pos_x = pos_x - 1;
////            n_pos_y = pos_y + 1;
////        }else if (dx==1 and dy==1){
////            n_pos_x = pos_x;
////            n_pos_y = pos_y + 1;
////        }else if (dx == 1 and dy==0){
////            n_pos_x = pos_x + 1;
////            n_pos_y = pos_y + 1;
////        }else if(dx == 1 and dy==-1){
////            n_pos_x = pos_x + 1;
////            n_pos_y = pos_y;
////        }else if(dx == 0 and dy == -1){
////            n_pos_x = pos_x +1 ;
////            n_pos_y = pos_y -1;
////        }else if(dx == -1 and dy == -1){
////            n_pos_x = pos_x;
////            n_pos_y = pos_y - 1;
////        }else if(dx == -1 and dy == 0){
////            n_pos_x = pos_x -1;
////            n_pos_y = pos_y - 1;
////        }else if(dx == -1 and dy == 1){
////            n_pos_x = pos_x - 1;
////            n_pos_y = pos_y;
////        }
////        vector<vector<int>> option_positions;
////        if (agent_maps[n_pos_x][n_pos_y] >= 0) {
////            // the target waypoint has been occupied;
////            int x = agent_poses[f_nei][0];
////            int y = agent_poses[f_nei][1];
////            for (int i = max(0, x - 1); i <= min(width, x + 1); i++) {
////                for (int j = max(0, y - 1); j < min(height, y + 1); j++) {
////                    if (agent_maps[i][j] < 0 and fabs(i - pos_x) <= 1 and fabs(j - pos_y) <= 1) {
////                        option_positions.push_back({i, j});
////                    }
////                }
////            }
////        }
////
////        if (option_positions.size() > 0) {
////            srand(time(NULL));
////            int rand_p = rand() % option_positions.size();
////            n_pos_x = option_positions[rand_p][0];
////            n_pos_y = option_positions[rand_p][1];
////        }
////    }
//    if(dx == 0 and dy == 0){
//        // start with stay still, no concept of forward, so just away
//        n_pos_x = pos_x + pos_x - agent_poses[f_nei][0];
//        n_pos_y = pos_y + pos_y - agent_poses[f_nei][1];
//    }else{
//        if(dx==0 and dy==1){
//            n_pos_x = pos_x - 1;
//            n_pos_y = pos_y + 1;
//        }else if (dx==1 and dy==1){
//            n_pos_x = pos_x;
//            n_pos_y = pos_y + 1;
//        }else if (dx == 1 and dy==0){
//            n_pos_x = pos_x + 1;
//            n_pos_y = pos_y + 1;
//        }else if(dx == 1 and dy==-1){
//            n_pos_x = pos_x + 1;
//            n_pos_y = pos_y;
//        }else if(dx == 0 and dy == -1){
//            n_pos_x = pos_x +1 ;
//            n_pos_y = pos_y -1;
//        }else if(dx == -1 and dy == -1){
//            n_pos_x = pos_x;
//            n_pos_y = pos_y - 1;
//        }else if(dx == -1 and dy == 0){
//            n_pos_x = pos_x -1;
//            n_pos_y = pos_y - 1;
//        }else if(dx == -1 and dy == 1){
//            n_pos_x = pos_x - 1;
//            n_pos_y = pos_y;
//        }
//    }
//
//    vector<vector<int>> option_positions;
//    if (agent_maps[n_pos_x][n_pos_y] >= 0) {
//        // the target waypoint has been occupied;
//        int x = agent_poses[f_nei][0];
//        int y = agent_poses[f_nei][1];
//        for (int i = max(0, x - 1); i <= min(width, x + 1); i++) {
//            for (int j = max(0, y - 1); j < min(height, y + 1); j++) {
//                if (agent_maps[i][j] < 0 and fabs(i - pos_x) <= 1 and fabs(j - pos_y) <= 1) {
//                    option_positions.push_back({i, j});
//                }
//            }
//        }
//    }
//
//    if (option_positions.size() > 0) {
//        srand(time(NULL));
//        int rand_p = rand() % option_positions.size();
//        n_pos_x = option_positions[rand_p][0];
//        n_pos_y = option_positions[rand_p][1];
//    }
//
//    if(n_pos_x != pos_x or n_pos_y != pos_y) {
//        prev_x = pos_x;
//        prev_y = pos_y;
//        pos_x = n_pos_x;
//        pos_y = n_pos_y;
//        agent_poses[id]={pos_x,pos_y};
//        agent_prev_poses[id] = {prev_x,prev_y};
//        agent_maps[pos_x][pos_y]=id;
//        if(agent_maps[prev_x][prev_y]== id){
//            agent_maps[prev_x][prev_y] = -1;
//        }
//    }
//}
//
//vector<int> Agent::stay_to_clockwise(int f_nei) {
//    int n_pos_x, n_pos_y;
//    int dx = pos_x - agent_poses[f_nei][0];
//    int dy = pos_y - agent_poses[f_nei][1];
//    if (dx == 0) {
//        if (dy > 0) {
//            n_pos_x = pos_x + 1;
//            n_pos_y = pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_y = pos_y -1;
//                if(agent_maps[n_pos_x][n_pos_y]>=0){
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y;
//                }
//            }
//        }else{
//            n_pos_x = pos_x - 1;
//            n_pos_y = pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_y = pos_y + 1;
//                if(agent_maps[n_pos_x][n_pos_y]>=0){
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y;
//                }
//            }
//        }
//    }else if (dx > 0){
//        if(dy == 1){
//            n_pos_x = pos_x;
//            n_pos_y = pos_y - 1;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x;
//                n_pos_y = pos_y;
//            }
//        }else if(dy == 0){
//            n_pos_x = 0;
//            n_pos_y = pos_y -1;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x - 1;
//                if(agent_maps[n_pos_x][n_pos_y]>=0){
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y;
//                }
//            }
//        }else{
//            n_pos_x = pos_x - 1;
//            n_pos_y = pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x;
//                n_pos_y = pos_y;
//            }
//        }
//    }else{
//        if(dy == -1){
//            n_pos_x = pos_x;
//            n_pos_y = pos_y + 1;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x;
//                n_pos_y = pos_y;
//            }
//        }else if(dy == 0){
//            n_pos_x = 0;
//            n_pos_y = pos_y +1;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x + 1;
//                if(agent_maps[n_pos_x][n_pos_y]>=0){
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y;
//                }
//            }
//        }else{
//            n_pos_x = pos_x + 1;
//            n_pos_y = pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x;
//                n_pos_y = pos_y;
//            }
//        }
//    }
//    vector<int> res = {n_pos_x,n_pos_y};
//    return res;
//}
//
//vector<int> Agent::stay_to_counter_clockwise(int f_nei) {
//    cout<<id<<"follows"<<f_nei<<endl;
//    int n_pos_x, n_pos_y;
//    int dx = pos_x - agent_poses[f_nei][0];
//    int dy = pos_y - agent_poses[f_nei][1];
//    cout<<"my poses:"<<pos_x<<" "<<pos_y<<endl;
//    cout<<"follow poses:"<<agent_poses[f_nei][0]<<" "<<agent_poses[f_nei][1]<<endl;
//    for(int j=pos_y+2; j>=pos_y-2; j--){
//        for(int i=pos_x-2; i<=pos_x+2; i++){
//            cout<<agent_maps[i][j]<<" ";
//        }
//        cout<<endl;
//    }
//
//    if (dx == 0) {
//        cout<<"dx is 0."<<endl;
//        if (dy > 0) {
//            n_pos_x = pos_x - 1;
//            n_pos_y = pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_y = pos_y -1;
//                if(agent_maps[n_pos_x][n_pos_y]>=0){
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y;
//                }
//            }
//        }else{
//            n_pos_x = pos_x + 1;
//            n_pos_y = pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_y = pos_y + 1;
//                if(agent_maps[n_pos_x][n_pos_y]>=0){
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y;
//                }
//            }
//        }
//    }else if (dx > 0){
//        cout<<"dx > 0."<<endl;
//        if(dy == 1){
//            n_pos_x = pos_x - 1;
//            n_pos_y = pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x;
//                n_pos_y = pos_y;
//            }
//        }else if(dy == 0){
//            n_pos_x = 0;
//            n_pos_y = pos_y +1;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x - 1;
//                if(agent_maps[n_pos_x][n_pos_y]>=0){
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y;
//                }
//            }
//        }else{
//            n_pos_x = pos_x;
//            n_pos_y = pos_y-1;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x;
//                n_pos_y = pos_y;
//            }
//        }
//    }else{
//        cout<<"dx < 0."<<endl;
//        if(dy == -1){
//            n_pos_x = pos_x + 1;
//            n_pos_y = pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x;
//                n_pos_y = pos_y;
//            }
//        }else if(dy == 0){
//            n_pos_x = 0;
//            n_pos_y = pos_y -1;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x + 1;
//                if(agent_maps[n_pos_x][n_pos_y]>=0){
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y;
//                }
//            }
//        }else{
//            n_pos_x = pos_x;
//            n_pos_y = pos_y - 1;
//            if(agent_maps[n_pos_x][n_pos_y]>=0){
//                n_pos_x = pos_x;
//                n_pos_y = pos_y;
//            }
//        }
//    }
//    cout<<"new position"<<n_pos_x<<" "<<n_pos_y<<endl;
//    vector<int> res = {n_pos_x,n_pos_y};
//    return res;
//}

/*Possible 2*/
//void Agent::edge_following() {
//    double current = DISTANCE_MAX;
//    vector<int> neighbors = get_stationary_neighbors();
//    int f_nei = -1;
//    for(int & nei:neighbors){
//        double m_dis = measure_distance_t_neighbor(nei);
//        if(m_dis<current) {
//            current = m_dis;
//            f_nei = nei;
//        }
//    }
//
//    agent_prev_nei_ids[id].push_back(f_nei);
//    if(agent_prev_nei_ids[id].size()>3){
//        agent_prev_nei_ids[id].erase(agent_prev_nei_ids[id].begin());
//    }
//
//    int n_pos_x, n_pos_y;
//    if(current<DISIRED_DISTANCE){
//        if(prev<current){
//            // getting closer to the desired edge-following distance,
//            // in our experiment, this branch should never be triggered.
//            n_pos_x = pos_x - prev_x + pos_x;
//            n_pos_y = pos_y - prev_y + pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>0) {
//                if ((pos_x - prev_x == 1 and pos_y - prev_y == 0) or (pos_x - prev_x == 1 and pos_y - prev_y == 1)) {
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y + 1;
//                } else if ((pos_x - prev_x == -1 and pos_y - prev_y == 0) or
//                           (pos_x - prev_x == -1 and pos_y - prev_y == -1)) {
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y - 1;
//                } else if ((pos_x - prev_x == 0 and pos_y - prev_y == 1) or
//                           (pos_x - prev_x == -1 and pos_y - prev_y == 1)) {
//                    n_pos_x = pos_x - 1;
//                    n_pos_y = pos_y;
//                } else if ((pos_x - prev_x == 0 and pos_y - prev_y == -1) or
//                           (pos_x - prev_x == 1 and pos_y - prev_y == -1)) {
//                    n_pos_x = pos_x + 1;
//                    n_pos_y = pos_y;
//                }
//            }
//        }else{
//            if ((pos_x - prev_x == 1 and pos_y - prev_y == 0) or (pos_x - prev_x == 1 and pos_y - prev_y == 1)) {
//                n_pos_x = pos_x;
//                n_pos_y = pos_y + 1;
//            } else if ((pos_x - prev_x == -1 and pos_y - prev_y == 0) or
//                       (pos_x - prev_x == -1 and pos_y - prev_y == -1)) {
//                n_pos_x = pos_x;
//                n_pos_y = pos_y - 1;
//            } else if ((pos_x - prev_x == 0 and pos_y - prev_y == 1) or
//                       (pos_x - prev_x == -1 and pos_y - prev_y == 1)) {
//                n_pos_x = pos_x - 1;
//                n_pos_y = pos_y;
//            } else if ((pos_x - prev_x == 0 and pos_y - prev_y == -1) or
//                       (pos_x - prev_x == 1 and pos_y - prev_y == -1)) {
//                n_pos_x = pos_x + 1;
//                n_pos_y = pos_y;
//            }else{
//                //刚开始没动的时候
//                n_pos_x = pos_x;
//                n_pos_y = pos_y;
//                stay_to_move(n_pos_x, n_pos_y);
//            }
//        }
//    }else if(current == DISIRED_DISTANCE){
//        if(prev>=current){
//            n_pos_x = pos_x - prev_x + pos_x;
//            n_pos_y = pos_y - prev_y + pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>0){
//                if((pos_x-prev_x == 1 and pos_y-prev_y==0) or (pos_x-prev_x==1 and pos_y-prev_y==1)){
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y+1;
//                }else if((pos_x-prev_x == -1 and pos_y-prev_y==0) or (pos_x-prev_x==-1 and pos_y-prev_y==-1)){
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y-1;
//                }else if((pos_x-prev_x == 0 and pos_y-prev_y==1) or (pos_x-prev_x==-1 and pos_y-prev_y==1)){
//                    n_pos_x = pos_x -1;
//                    n_pos_y = pos_y;
//                }else if((pos_x-prev_x == 0 and pos_y-prev_y ==-1) or (pos_x-prev_x==1 and pos_y-prev_y==-1) ){
//                    n_pos_x = pos_x+1;
//                    n_pos_y = pos_y;
//                }else{
//                    //刚开始没动的时候
//                    stay_to_move(n_pos_x, n_pos_y);
//                }
//            }
//        }else{
//            n_pos_x = pos_x;
//            n_pos_y = pos_y;
//            stay_to_move(n_pos_x,n_pos_y);
//        }
//    }else{
//        if(prev>current){
//            // getting closer to the desired edge-following distance
//            n_pos_x = pos_x - prev_x + pos_x;
//            n_pos_y = pos_y - prev_y + pos_y;
//            if(agent_maps[n_pos_x][n_pos_y]>0) {
//                if ((pos_x - prev_x == 1 and pos_y - prev_y == 0) or (pos_x - prev_x == 1 and pos_y - prev_y == 1)) {
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y + 1;
//                } else if ((pos_x - prev_x == -1 and pos_y - prev_y == 0) or
//                           (pos_x - prev_x == -1 and pos_y - prev_y == -1)) {
//                    n_pos_x = pos_x;
//                    n_pos_y = pos_y - 1;
//                } else if ((pos_x - prev_x == 0 and pos_y - prev_y == 1) or
//                           (pos_x - prev_x == -1 and pos_y - prev_y == 1)) {
//                    n_pos_x = pos_x - 1;
//                    n_pos_y = pos_y;
//                } else if ((pos_x - prev_x == 0 and pos_y - prev_y == -1) or
//                           (pos_x - prev_x == 1 and pos_y - prev_y == -1)) {
//                    n_pos_x = pos_x + 1;
//                    n_pos_y = pos_y;
//                }
//            }
//        }else{
//            if ((pos_x - prev_x == 1 and pos_y - prev_y == 0) or (pos_x - prev_x == 1 and pos_y - prev_y == 1)) {
//                n_pos_x = pos_x;
//                n_pos_y = pos_y + 1;
//            } else if ((pos_x - prev_x == -1 and pos_y - prev_y == 0) or
//                       (pos_x - prev_x == -1 and pos_y - prev_y == -1)) {
//                n_pos_x = pos_x;
//                n_pos_y = pos_y - 1;
//            } else if ((pos_x - prev_x == 0 and pos_y - prev_y == 1) or
//                       (pos_x - prev_x == -1 and pos_y - prev_y == 1)) {
//                n_pos_x = pos_x - 1;
//                n_pos_y = pos_y;
//            } else if ((pos_x - prev_x == 0 and pos_y - prev_y == -1) or
//                       (pos_x - prev_x == 1 and pos_y - prev_y == -1)) {
//                n_pos_x = pos_x + 1;
//                n_pos_y = pos_y;
//            }else{
//                //刚开始没动的时候
//                n_pos_x = pos_x;
//                n_pos_y = pos_y;
//                stay_to_move(n_pos_x, n_pos_y);
//            }
//        }
//    }
//
////    vector<vector<int>> option_positions;
////    if (agent_maps[n_pos_x][n_pos_y] >= 0) {
////        // the target waypoint has been occupied;
////        int x = agent_poses[f_nei][0];
////        int y = agent_poses[f_nei][1];
////        for (int i = max(0, x - 1); i <= min(width, x + 1); i++) {
////            for (int j = max(0, y - 1); j < min(height, y + 1); j++) {
////                if (agent_maps[i][j] < 0 and fabs(i - pos_x) <= 1 and fabs(j - pos_y) <= 1) {
////                    option_positions.push_back({i, j});
////                }
////            }
////        }
////    }
////
////    if (option_positions.size() > 0) {
////        srand(time(NULL));
////        int rand_p = rand() % option_positions.size();
////        n_pos_x = option_positions[rand_p][0];
////        n_pos_y = option_positions[rand_p][1];
////    }
//
//
//    if(n_pos_x != pos_x or n_pos_y != pos_y) {
//        prev_x = pos_x;
//        prev_y = pos_y;
//        pos_x = n_pos_x;
//        pos_y = n_pos_y;
//        agent_poses[id]={pos_x,pos_y};
//        agent_prev_poses[id] = {prev_x,prev_y};
//        agent_maps[pos_x][pos_y]=id;
//        if(agent_maps[prev_x][prev_y]== id){
//            agent_maps[prev_x][prev_y] = -1;
//        }
//    }
//
//    prev = current;
//}
//
//void Agent::stay_to_move(int & n_pos_x, int & n_pos_y){
//    if(agent_maps[pos_x+1][pos_y]>=0 and agent_maps[pos_x][pos_y+1]==-1
//       and agent_maps[pos_x-1][pos_y]>=0 and agent_maps[pos_x][pos_y-1]==-1){
//        int n1 = agent_maps[pos_x-1][pos_y+1]>=0?1:0;
//        int n2 = agent_maps[pos_x+1][pos_y+1]>=0?1:0;
//        int n3 = agent_maps[pos_x-1][pos_y-1]>=0?1:0;
//        int n4= agent_maps[pos_x+1][pos_y-1]>=0?1:0;
//        if(n1+n2<n3+n4){
//            n_pos_x=pos_x;
//            n_pos_y=pos_y+1;
//        }else{
//            n_pos_x=pos_x;
//            n_pos_y=pos_y-1;
//        }
//    }else if(agent_maps[pos_x+1][pos_y]>=0 and agent_maps[pos_x][pos_y+1]==-1){
//        n_pos_x=pos_x;
//        n_pos_y=pos_y+1;
//    }else if(agent_maps[pos_x-1][pos_y]>=0 and agent_maps[pos_x][pos_y-1]==-1){
//        n_pos_x=pos_x;
//        n_pos_y=pos_y-1;
//    }
//
//    if(agent_maps[pos_x][pos_y+1]>=0 and agent_maps[pos_x-1][pos_y]==-1 and
//       agent_maps[pos_x][pos_y-1]>=0 and agent_maps[pos_x+1][pos_y]==-1){
//        int n1 = agent_maps[pos_x-1][pos_y+1]>=0?1:0;
//        int n2 = agent_maps[pos_x+1][pos_y+1]>=0?1:0;
//        int n3 = agent_maps[pos_x-1][pos_y-1]>=0?1:0;
//        int n4= agent_maps[pos_x+1][pos_y-1]>=0?1:0;
//        if(n1+n3<n2+n4){
//            n_pos_x = pos_x-1;
//            n_pos_y = pos_y;
//        }else{
//            n_pos_x=pos_x+1;
//            n_pos_y=pos_y;
//        }
//    }else if(agent_maps[pos_x][pos_y+1]>=0 and agent_maps[pos_x-1][pos_y]==-1){
//        n_pos_x = pos_x-1;
//        n_pos_y = pos_y;
//    }else if(agent_maps[pos_x][pos_y-1]>=0 and agent_maps[pos_x+1][pos_y]==-1){
//        n_pos_x = pos_x +1;
//        n_pos_y = pos_y;
//    }
//}
