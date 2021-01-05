#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <thread>
#include <future>
#include <dirent.h>
#include <queue>
#include <type_traits>
#include <string>
#include <iterator>
#include <limits>
#include <fstream>
#include <math.h>
#include "Agent.h"

using namespace std;

vector<vector<vector<int>>> agent_init_pos_in_shapes; //7*agent_num*2
vector<vector<int>> agent_maps;
vector<vector<int>> agent_poses;
vector<int> agent_gradient_values;
vector<int> agent_local_ids;
vector<RState> agent_running_states;
vector<vector<int>> grids;
vector<vector<int>> agent_prev_poses;
vector<vector<int>> agent_prev_nei_ids; // n*3
const int width = 80;
const int height = 80;
const double PI = atan(1.0) * 4;

//const string RUN_ENV = "MAC";
const string RUN_ENV = "LINUX";
const int TEMP_THREAD_NUM = 1;
vector<Agent> swarm;
enum Agent_num_mode {
    less_a = -1, equal_a = 0, more_a = 1
};
Agent_num_mode a_mode = more_a;

void parallel_swarms(promise<bool> &is_success, int t_id, int s_id, int e_id) {
    bool tmp = true;
    for (int i = s_id; i <= e_id; i++) {
        bool res = swarm[i].shape_self_assembly();
        tmp = tmp and res;
    }
    is_success.set_value(tmp);
}

double getMold(const vector<vector<int>> &vec) {   //求向量的模长
    int n = vec.size();
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        int m = vec[i].size();
        for (int j = 0; j < m; ++j) {
            sum += vec[i][j] * vec[i][j];
        }
    }
    return sqrt(sum);
}

double getSimilarity(const vector<vector<int>> &lhs, const vector<vector<int>> &rhs) {
    int n = lhs.size();
    if (n == rhs.size()) {
        double tmp = 0.0;  //内积
        for (int i = 0; i < n; ++i) {
            int m = lhs[i].size();
            if (m == rhs[i].size()) {
                for (int j = 0; j < m; ++j) {
                    tmp += lhs[i][j] * rhs[i][j];
                }
            } else {
                return -1;
            }
        }
        return tmp / (getMold(lhs) * getMold(rhs));
    } else {
        return -1;
    }
}

bool initialize_no_seed_agent_positions(int f, int min_i, int min_j, int shape_agent_num) {
    int cnt = 4;
    bool is_enough = false;
    if (min_j - 3 < 4 or min_i < 4 or min_i >= width - 4) {
        cout << "Too narrow space for edge following." << endl;
        return is_enough;
    }

    for (int j = min_j - 3; j >= 4; j--) {
//        swarm[cnt].set_config(min_i,j,2*sqrt(2));
        agent_init_pos_in_shapes[f].push_back({min_i, j});
        cnt++;
        if (cnt == shape_agent_num) {
            is_enough = true;
            break;
        }
    }
//    int max_height = int(0.6 * height);
    int max_height = height-3;
    vector<vector<int>> incre_options;
    if (not is_enough) {
        for (int i = min_i + 1; i < width - 4; i++) {
            for (int j = 4; j <= max_height; j++) {
                if (j > min_j - 3) {
                    bool is_valid = true;
                    for (int px = i - 2; px <= i + 2 && px<width-1; px++) {
                        for (int py = j - 2; py <= j + 2 && py<height-1; py++) {
//                            cout<<px<<" "<<py<<endl;
                            if (grids[px][py] == 0) {
                                is_valid = is_valid && true;
                            } else {
                                is_valid = is_valid && false;
                                break;
                            }
                        }
                        if (not is_valid) {
                            break;
                        }
                    }
                    if (is_valid) {
//                        swarm[cnt].set_config(i,j,2*sqrt(2));
                        agent_init_pos_in_shapes[f].push_back({i, j});
                        cnt++;
                    } else {
                        break;
                    }
                } else {
//                    swarm[cnt].set_config(i,j,2*sqrt(2));
                    agent_init_pos_in_shapes[f].push_back({i, j});
                    cnt++;
                }
                if (cnt == shape_agent_num) {
                    is_enough = true;
                    break;
                }
                if (j == max_height) {
                    incre_options.push_back({i, j});
                }
            }
            if (is_enough) {
                break;
            }
        }
    }
    if (not is_enough) {
        for (int i = min_i - 1; i >= 4; i--) {
            for (int j = 4; j <= max_height; j++) {
                if (j > min_j - 3) {
                    bool is_valid = true;
                    for (int px = i - 2; px <= i + 3 && px<width-1; px++) {
                        for (int py = j - 2; py <= j + 2 &&py<height-1; py++) {
//                            cout<<px<<" "<<py<<endl;
                            if (grids[px][py] == 0) {
                                is_valid = is_valid && true;
                            } else {
                                is_valid = is_valid && false;
                                break;
                            }
                        }
                        if (not is_valid) {
                            break;
                        }
                    }
                    if (is_valid) {
//                        swarm[cnt].set_config(i,j,2*sqrt(2));
                        agent_init_pos_in_shapes[f].push_back({i, j});
                        cnt++;
                    } else {
                        break;
                    }
                } else {
//                    swarm[cnt].set_config(i,j,2*sqrt(2));
                    agent_init_pos_in_shapes[f].push_back({i, j});
                    cnt++;
                }
                if (cnt == shape_agent_num) {
                    is_enough = true;
                    break;
                }
                if (j == max_height) {
                    incre_options.push_back({i, j});
                }
            }
            if (is_enough) {
                break;
            }
        }
    }
    if (not is_enough) {
        for (int o = 0; o < incre_options.size(); o++) {
            int i = incre_options[o][0];
            for (int j = incre_options[o][1] + 1; j < height-2; j++) {
                if (j > min_j - 3) {
                    bool is_valid = true;
                    for (int px = i - 2; px <= i + 2 && px<width-1; px++) {
                        for (int py = j - 2; py <= j + 2 && py<height-1; py++) {
//                            cout<<px<<" "<<py<<endl;
                            if (grids[px][py] == 0) {
                                is_valid = is_valid && true;
                            } else {
                                is_valid = is_valid && false;
                                break;
                            }
                        }
                        if (not is_valid) {
                            break;
                        }
                    }
                    if (is_valid) {
//                        swarm[cnt].set_config(i,j,2*sqrt(2));
                        agent_init_pos_in_shapes[f].push_back({i, j});
                        cnt++;
                    } else {
                        break;
                    }
                } else {
//                    swarm[cnt].set_config(i,j,2*sqrt(2));
                    agent_init_pos_in_shapes[f].push_back({i, j});
                    cnt++;
                }
                if (cnt == shape_agent_num) {
                    is_enough = true;
                    break;
                }
            }
            if (is_enough) {
                break;
            }
        }
    }

    if (not is_enough) {
        cout << "There isn't enough space for initialization." << endl;

    }
    return is_enough;
}

void split(const string &s, vector<string> &tokens, const string &delimiters = " ") {
    string::size_type lastPos = s.find_first_not_of(delimiters, 0);
    string::size_type pos = s.find_first_of(delimiters, lastPos);
    while (string::npos != pos || string::npos != lastPos) {
        tokens.emplace_back(s.substr(lastPos, pos - lastPos));
        lastPos = s.find_first_not_of(delimiters, pos);
        pos = s.find_first_of(delimiters, lastPos);
    }
}

int main(int argc, char **argv) {
//    string dictionary, out_dict;
//    if (RUN_ENV == "WIN") {
//        dictionary =
//                "D:\\projects\\CLionProjects\\InitSettingGenerator\\tmp_edge_display\\" + to_string(width) + '_' +
//                to_string(height);
//        out_dict = "D:\\projects\\CLionProjects\\EdgeFollowing\\exp";
//    } else if (RUN_ENV == "MAC") {
////        dictionary =
////                "/Users/chuwenjie/CLionProjects/InitSettingGenerator/tmp_edge_display/" + to_string(width) + '*' +
////                to_string(height);
//        dictionary = "/Users/chuwenjie/PycharmProjects/AssemblyShape/figures/cato/" + to_string(width) + '_' +
//                     to_string(height);
//        out_dict = "/Users/chuwenjie/CLionProjects/EdgeFollowing/exp";
//    } else {
//        dictionary = "./exp/" + to_string(width) + '*' + to_string(height);
//        out_dict = "./exp";
//    }

    string work_root = argv[1];
    string dictionary, out_dict;
    if (RUN_ENV == "WIN") {
        dictionary = work_root +
                     "\\inputs\\" + argv[2] + "\\" + to_string(width) + '_' + to_string(height) + "\\" + argv[4] ;
        out_dict = work_root + "\\outputs\\" + argv[3] + "\\" + to_string(width) + '_' + to_string(height) + "\\" + argv[4];
    } else {
        dictionary = work_root + "/inputs/" + argv[2] + "/" + to_string(width) + '_' + to_string(height) + '/' + argv[4];
        out_dict = work_root + "/outputs/" + argv[3] + "/" + to_string(width) + '_' + to_string(height) + '/' + argv[4];
    }
    DIR *dir;
    struct dirent *ptr;
    vector<string> name_posts;
    vector<int> shape_nums;
    vector<string> filelist;
    vector<int> a_num_s;
    const char *p = dictionary.c_str();
    if ((dir = opendir(p)) == NULL) {
        perror("Open dir error...");
        exit(1);
    }
    while ((ptr = readdir(dir)) != NULL) {
        if (string(ptr->d_name).compare(0, 4, "grid") == 0)    //file
        {
            string temp;
            if (RUN_ENV == "WIN") {
                temp = dictionary + '\\' + ptr->d_name;
            } else {
                temp = dictionary + '/' + ptr->d_name;
            }
            filelist.push_back(temp);
            string _post = ptr->d_name;
            //Add
            vector<string> split_list;
            split(_post, split_list, "_");
            string shape_num_str = split_list[split_list.size() - 2];
            string a_num_str = split_list[split_list.size() - 1];
            a_num_str = a_num_str.substr(0, a_num_str.find("."));
            int shape_num = std::stoi(shape_num_str);
            int a_num = std::stoi(a_num_str);
            //Add End
//            int shape_num = _post[5] - '0';
//            int a_num = atoi(_post.substr(7, _post.size() - 11).c_str());
            shape_nums.push_back(shape_num);
            a_num_s.push_back(a_num);
            _post = _post.substr(4, _post.size() - 4);
            name_posts.push_back(_post);
        }
    }
    closedir(dir);

    for (int x = 0; x < width; x++) {
        grids.emplace_back();
        for (int y = 0; y < height; y++) {
            grids[x].push_back(0);
        }
    }

    for (int f = 0; f < filelist.size(); f++) {
        vector<vector<int>> agent_init_pos_in_a_shape;
        agent_init_pos_in_shapes.push_back(agent_init_pos_in_a_shape);
        //read the grid environment
        fstream infile;
        infile.open(filelist[f], ios::in);
        if (!infile) {
            cout << "open failed" << endl;
            exit(1);
        }
        int i = 0;
        int j = height - 1;
        int min_i = width;
        int min_j = height;
        while (!infile.eof() and j >= 0) {
            i = 0;
            while (!infile.eof() and i < width) {
                infile >> grids[i][j];
                if (grids[i][j] == 1) {
                    if (j <= min_j) {
                        if (j < min_j) {
                            min_j = j;
                            min_i = i;
                        } else {
                            if (i < min_i) {
                                min_i = i;
                            }
                        }
                    }
                }
                i++;
            }
            j--;
        }
        infile.close();

        //初始化swarm,确定环境中在运行的agent数目
        int shape_agent_num;
        for (int k = 0; k < a_num_s[f]; k++) {
            swarm.push_back(Agent(k));
        }
        srand(time(NULL));
        //TODO
//        int rand_more = rand() % int(a_num_s[f] * 0.05);
        int rand_more = 0;
        if (a_mode == more_a) {
            //如果是多,则为N+3~N+0.05N+3
            for (int i = 0; i < rand_more + 3; i++) {
                int si = swarm.size();
                swarm.push_back(Agent(si));
            }
        } else if (a_mode == equal_a) {
            //如果是正好,由于seed robot中只有v3在目标内,所以至少为N+3
            for (int i = 0; i < 3; i++) {
                int si = swarm.size();
                swarm.push_back(Agent(si));
            }
        } else {
            rand_more = rand_more - 3;
            //如果是少,则为N+3-0.05N~N+3
            if (rand_more > 0) {
                for (int i = 0; i < rand_more; i++) {
                    swarm.erase(swarm.end());
                }
            } else if (rand_more < 0) {
                for (int i = 0; i < -rand_more; i++) {
                    int si = swarm.size();
                    swarm.push_back(Agent(si));
                }
            }
        }
        shape_agent_num = swarm.size();

        //initialize seed robot v0-v3, noting that only v3 is within the target shape,
        //so the total number of agent for n-n shape is at least n+3 agent.
        agent_init_pos_in_shapes[f].push_back({min_i, min_j - 1});
        agent_init_pos_in_shapes[f].push_back({min_i + 1, min_j - 1});
        agent_init_pos_in_shapes[f].push_back({min_i, min_j - 2});
        agent_init_pos_in_shapes[f].push_back({min_i, min_j});
        //初始化剩余agent位置
        bool is_enough = initialize_no_seed_agent_positions(f, min_i, min_j, shape_agent_num);
        if (not is_enough) {
            cout << "Experiment failed!" << endl;
            continue;
        }

        //实验loop相关的设置及评估量
        int exp_num = 1;
//        const int THREAD_NUM = TEMP_THREAD_NUM > a_num_s[f] ? a_num_s[f] : TEMP_THREAD_NUM; //实验运行线程数
        const int THREAD_NUM = TEMP_THREAD_NUM > shape_agent_num ? shape_agent_num : TEMP_THREAD_NUM; //实验运行线程数
        //实验中要记录的值
        int valid_exp = 0; //valid_exp指在有限步数内100%完成的实验数
        double exp_avg_iter = 0;
        double exp_avg_iter_t = 0;
        int minor_valid_exp = 0; //minor_valid_exp指在有限步数内,未完成目标数小于等于3的实验数,也就是包含所有的valid_exp
        double minor_exp_avg_iter = 0; //在所有包含于minor_valid_exp内的实验中,只剩最后3个target情况下平均花了多少个iteration
        // ->这个量没有太大意义,不如统计在valid_exp中,最后3步的平均iteration数，然后只是记录在多次实验中，100%完成的比例，最后查3个以内的比例,最后差很多的比例
        double minor_exp_avg_iter_t = 0; //在所有包含于minor_valid_exp内的实验中,平均每个iteration花的时间
        double exp_avg_similarity = 0;

        //设置同初始状态下的多轮实验，其设置参数及基本运行情况记录输出路径
        string out_arg;
        if (RUN_ENV == "WIN") {
            out_arg = out_dict + "\\args_" + to_string(width) + "_" +
                      to_string(height) + name_posts[f];
        } else {
            out_arg = out_dict + "/args_" + to_string(width) + "_" +
                      to_string(height) + name_posts[f];
        }
        ofstream outarg(out_arg, ios::app);

        //初始化所有相关的全局变量
        for (int i = 0; i < width; i++) {
            agent_maps.push_back(vector<int>());
            for (int j = 0; j < height; j++) {
                agent_maps[i].push_back(-1);
            }
        }
        for (int i = 0; i < swarm.size(); i++) {
            agent_poses.push_back({-1, -1});
            agent_gradient_values.push_back(GRADIENT_MAX);
            agent_local_ids.push_back(-1);
            agent_running_states.push_back(start);
            agent_prev_poses.push_back({-1, -1});
            agent_prev_nei_ids.push_back(vector<int>());
        }
        for (int e = 0; e < exp_num; e++) {
            //在每轮实验开始初始化所有agent的位置
            swarm[0].set_config(agent_init_pos_in_shapes[f][0][0], agent_init_pos_in_shapes[f][0][1], 2 * sqrt(2), 0,
                                true, true, true);
            swarm[1].set_config(agent_init_pos_in_shapes[f][1][0], agent_init_pos_in_shapes[f][1][1], 2 * sqrt(2), 1,
                                false, true, true);
            swarm[2].set_config(agent_init_pos_in_shapes[f][2][0], agent_init_pos_in_shapes[f][2][1], 2 * sqrt(2), 1,
                                false, true, true);
            swarm[3].set_config(agent_init_pos_in_shapes[f][3][0], agent_init_pos_in_shapes[f][3][1], 2 * sqrt(2), 1,
                                false, true, true);
            for (int s = 4; s < agent_init_pos_in_shapes[f].size(); s++) {
                swarm[s].set_config(agent_init_pos_in_shapes[f][s][0], agent_init_pos_in_shapes[f][s][1], 2 * sqrt(2),
                                    GRADIENT_MAX, false, false, true);
            }

            //设置每轮实验过程具体agent位置变化记录文件路径
            string out_name;
            if (RUN_ENV == "WIN") {
                out_name = out_dict + "\\poses_" + to_string(e) + "_" +
                           to_string(width) + "_" + to_string(height) +
                           name_posts[f];
            } else {
                out_name = out_dict + "/poses_" + to_string(e) + "_" +
                           to_string(width) + "_" + to_string(height) +
                           name_posts[f];
            }
            ofstream outfile(out_name, ios::app);

            //record the initialization
            outfile << "arguments: " << width << ' ' << height << ' ' << shape_agent_num << ' ' << THREAD_NUM
                    << endl;
            outfile << "agent positions:" << endl;
            for (int k = 0; k < shape_agent_num; k++) {
                if (k < shape_agent_num - 1) {
                    outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
                } else {
                    outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
                }
//                agent_maps[swarm[k].pos_x][swarm[k].pos_y] = swarm[k].id;
            }
            outfile << endl;

            vector<thread> threads;
            int terminal = 0;
            int minor_terminal = 0;
            clock_t startT, endT;
            vector<double> dec_times;

            int ac_tar = shape_agent_num - 4;
            int pre_ac_tar = ac_tar;
            int rep_cnt=0;
            int stop_tar = 0;
            int mv_tar = 0;
            vector<int> ac_tar_decay;
            vector<int> mv_tar_line;
            ac_tar_decay.push_back(ac_tar);
            mv_tar_line.push_back(mv_tar);

//            while (ac_tar > 0 && terminal < 1000 and minor_terminal < 500) {
            while (ac_tar > 0 and stop_tar<a_num_s[f]+3 and rep_cnt<=200) {
                if(pre_ac_tar == ac_tar){
                    rep_cnt ++;
                }else{
                    rep_cnt = 0;
                }
                cout << "ac_tar: " << ac_tar << ", mv_tar: " << mv_tar << endl;
                terminal += 1;
                startT = clock();

                //将agent的移动分配给不同的线程控制
                int left = int(shape_agent_num % THREAD_NUM);
                int alloc = 0;
                int s_ids[THREAD_NUM];
                int e_ids[THREAD_NUM];
                promise<bool> promise_s[THREAD_NUM];
                future<bool> res_s[THREAD_NUM];
                srand(time(NULL));
                for (int k = 0; k < THREAD_NUM; k++) {
                    s_ids[k] = -1, e_ids[k] = -1;
                    if (left > alloc) {
                        s_ids[k] = k * (int(shape_agent_num / THREAD_NUM) + 1);
                        e_ids[k] = s_ids[k] + int(shape_agent_num / THREAD_NUM);
                        alloc++;
                    } else {
                        s_ids[k] = alloc * (int(shape_agent_num / THREAD_NUM) + 1) +
                                   (k - alloc) * int(shape_agent_num / THREAD_NUM);
                        e_ids[k] = s_ids[k] + int(shape_agent_num / THREAD_NUM) - 1;
                    }
                }

                for (int k = 0; k < THREAD_NUM; k++) {
                    res_s[k] = promise_s[k].get_future();
                    threads.emplace_back(parallel_swarms, ref(promise_s[k]), k, s_ids[k], e_ids[k]);
                }

                bool success_iter = true;
                for (int k = 0; k < THREAD_NUM; k++) {
                    success_iter = success_iter and res_s[k].get();
                }

                // 等待其他线程join
                for (int k = 0; k < THREAD_NUM; k++) {
                    threads[k].join();
                }
                threads.clear();

                endT = clock();
                dec_times.push_back((double) (endT - startT));

                //记录群体状态
                pre_ac_tar = ac_tar;
                ac_tar = 0;
                mv_tar = 0;
                stop_tar = 0;
                for (int a = 0; a < swarm.size(); a++) {
//                    cout<<a<<" "<<swarm[a].state<<endl;
                    if (swarm[a].state == move_while_inside || swarm[a].state == move_while_outside) {
                        mv_tar++;
                    }
                    if (swarm[a].state != joined_shape) {
                        ac_tar++;
                    }else{
                        stop_tar++;
                    }
                }
                ac_tar_decay.push_back(ac_tar);
                mv_tar_line.push_back(mv_tar);

                //record new positions for all agents
                outfile << "agent positions:" << endl;
                for (int k = 0; k < shape_agent_num; k++) {
                    if (k < shape_agent_num - 1) {
                        outfile << swarm[k].pos_x << ',' << swarm[k].pos_y << ' ';
                    } else {
                        outfile << swarm[k].pos_x << ',' << swarm[k].pos_y;
                    }
                }
                outfile << endl;

                if (stop_tar >= a_num_s[f]) {
                    minor_terminal += 1;
                }

                if (not success_iter) {
                    outarg << "Experiment " << e << " failed." << endl;
                    break;
                }
            }
            outfile.flush();
            outfile.close();


            double avg_t = 0;
            for (int k = 0; k < dec_times.size(); k++) {
                avg_t += dec_times[k];
            }
            double avg_iteration = avg_t / (dec_times.size() * CLOCKS_PER_SEC);
            vector<vector<int>> formed_shape;
            for (int w = 0; w < width; w++) {
                formed_shape.push_back(vector<int>());
                for (int h = 0; h < height; h++) {
                    if (agent_maps[w][h] >= 0) {
                        formed_shape[w].push_back(1);
                    } else {
                        formed_shape[w].push_back(0);
                    }
                }
            }
            int temp_0 = grids[min_i][min_j-1];
            int temp_1 = grids[min_i+1][min_j-1];
            int temp_2 = grids[min_i][min_j-2];
            int temp_3 = grids[min_i][min_j];
            grids[min_i][min_j-1] = 1;
            grids[min_i+1][min_j-1] = 1;
            grids[min_i][min_j-2] = 1;
            grids[min_i][min_j] = 1;
            double mse_similarity = getSimilarity(grids, formed_shape);
            exp_avg_similarity = exp_avg_similarity + mse_similarity;
            grids[min_i][min_j-1] = temp_0;
            grids[min_i+1][min_j-1] = temp_1;
            grids[min_i][min_j-2] = temp_2;
            grids[min_i][min_j] = temp_3;

            outarg << "Experiment " << e << ":" << endl;
            outarg << "The average decision time for each iteration is: " << avg_iteration << "s." << endl;
            outarg << "Main: program exiting after " << terminal << " steps, and " << minor_terminal
                   << " steps for the last 3 positions, the similarity is " << mse_similarity << endl;
            outarg << "Decay line:";
            for (int k = 0; k < ac_tar_decay.size(); k++) {
                outarg << ' ' << ac_tar_decay[k];
            }
            outarg << endl;
            outarg << "The number of moving agent line:";
            for (int k = 0; k < mv_tar_line.size(); k++) {
                outarg << ' ' << mv_tar_line[k];
            }
            outarg << endl;
            ac_tar_decay.clear();
            mv_tar_line.clear();

            cout << "End an experiment! Clearing..." << endl;
            //重置全局变量内的值，使其恢复到初始状态
            vector<vector<int>> array(width, vector<int>(height, -1));
            agent_maps.swap(array);
            for (int i = 0; i < swarm.size(); i++) {
                vector<int> pos = {-1, -1};
                agent_poses[i].swap(pos);
                agent_gradient_values[i] = GRADIENT_MAX;
                agent_local_ids[i] = -1;
                agent_running_states[i] = start;
                vector<int> prev_pos = {-1, -1};
                agent_prev_poses[i].swap(prev_pos);
                agent_prev_nei_ids[i].clear();
            }

            if (stop_tar == a_num_s[f]+3) {
                valid_exp += 1;
                exp_avg_iter += terminal;
                exp_avg_iter_t += avg_iteration;
            }
            if (stop_tar >= a_num_s[f]) {
                minor_valid_exp += 1;
                minor_exp_avg_iter = minor_exp_avg_iter + (terminal - minor_terminal);
                minor_exp_avg_iter_t += avg_iteration;
            }
        }
        exp_avg_iter = exp_avg_iter / valid_exp;
        exp_avg_iter_t = exp_avg_iter_t / valid_exp;
        minor_exp_avg_iter = minor_exp_avg_iter / minor_valid_exp;
        minor_exp_avg_iter_t = minor_exp_avg_iter_t / minor_valid_exp;
        exp_avg_similarity = exp_avg_similarity / exp_num;
        cout << f << ": exp_avg_iter=" << exp_avg_iter << ", exp_avg_iter_time=" << exp_avg_iter_t << endl;
        outarg << endl << endl;
        outarg << "After " << exp_num << " experiments, for shape " << f << ", avg_similarity=" << exp_avg_similarity
               << ", " << valid_exp << " experiments success, exp_avg_iter=" << exp_avg_iter
               << ", exp_avg_iter_time=" << exp_avg_iter_t << ", minor_exp_avg_iter=" << minor_exp_avg_iter
               << ", minor_exp_avg_iter_time=" << minor_exp_avg_iter_t << endl;
        outarg.flush();
        outarg.close();
        //清理所有全局变量的值，方便下一个shape设置下开始实验
        //由于每次实验都是固定规模下,agent_maps的大小始终不变,如果不想每次清理,就在f循环之外初始push一遍,
        //这样在exp开始之前就不需要再push了,每次实验结束重置一遍就可以了
        vector<vector<int>> tmp_agent_maps;
        agent_maps.swap(tmp_agent_maps);
        //下面的是必须清理的,因为新的shape下agent数量不一样,都是重新push_back的
        swarm.clear();
        vector<vector<int>> tmp_agent_poses;
        agent_poses.swap(tmp_agent_poses);
        vector<int> tmp_agent_gradient_values;
        agent_gradient_values.swap(tmp_agent_gradient_values);
        vector<int> tmp_agent_local_ids;
        agent_local_ids.swap(tmp_agent_local_ids);
        vector<RState> tmp_agent_running_states;
        agent_running_states.swap(tmp_agent_running_states);
        vector<vector<int>> tmp_prev_agent_maps;
        agent_prev_poses.swap(tmp_prev_agent_maps);
        vector<vector<int>> tmp_agent_prev_nei_ids;
        agent_prev_nei_ids.swap(tmp_agent_prev_nei_ids);
    }

    return 0;
}
