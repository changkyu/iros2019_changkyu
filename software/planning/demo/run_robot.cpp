#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>
#define LOG BOOST_LOG_TRIVIAL(trace)

#include "planner.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

int main(int argc, char* argv[])
{
    string fp_plan;
    
    po::options_description desc("Example Usage");
    desc.add_options()
        ("help", "help")
        ("input,i", po::value<string>(&fp_plan)->default_value(""), "plan file")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if( vm.count("help") ) 
    {        
        cout << desc << endl;
        return 0;
    }

    fs::path path_plan(fp_plan);
    if( !fs::is_regular_file(path_plan) )
    {
        cerr << "[Error] Invalid file: " << fp_plan << endl;
        return 0;
    }


    YAML::Node node_res = YAML::LoadFile(fp_plan);

    int len_actions = node_res["actions"].size();
    int len_path = node_res["path"].size();
    int n_objs = (node_res["path"][0].size()-1) / 3;

    vector<RobotObjectSetup::Object> objects(n_objs);
    for( int o=0; o<n_objs; o++ )
    {
        objects[o].name = "dove_beauty_bar";
        objects[o].dims.resize(3);
        objects[o].dims[0] = 0.035;
        objects[o].dims[1] = 0.066;
        objects[o].dims[2] = 0.096;
        objects[o].radius  = 0.096;
        objects[o].q_offset.setEulerZYX(0, -89.9999/ 180.0 * M_PI, 0);
        objects[o].shape = new btBoxShape(btVector3(objects[o].dims[0]*0.5, 
                                                    objects[o].dims[1]*0.5, 
                                                    objects[o].dims[2]*0.5));
        objects[o].z_offset = 0.035 * 0.5;
        objects[o].mass = 0.135;
    }

    vector<Planner::Action> actions(len_actions);
    for( int a=0; a<len_actions; a++ )
    {
        actions[a].type        = (Planner::TYPE_ACTION)node_res["actions"][a][0].as<int>();
        actions[a].idx_target  = node_res["actions"][a][1].as<double>();
        actions[a].idx_target2 = node_res["actions"][a][2].as<double>();        
        actions[a].x           = node_res["actions"][a][3].as<double>();
        actions[a].y           = node_res["actions"][a][4].as<double>();
        actions[a].yaw         = node_res["actions"][a][5].as<double>();
    }

    KukaTableSetup env(objects);
    ompl::base::State* state = env.allocState();
    ompl::base::State* state_init = env.allocState();
    og::PathGeometric path(env.getAllForAllSpaceInformation());
    for( int p=0; p<len_path; p++ )
    {
        STATE_ROBOT(state) = node_res["path"][p][0].as<double>();
        for( int o=1; o<=n_objs; o++ )
        {
            ObjectState* state_obj = STATE_OBJECT(state,o);
            state_obj->setX(  node_res["path"][p][3*(o-1)+1].as<double>());
            state_obj->setY(  node_res["path"][p][3*(o-1)+2].as<double>());
            state_obj->setYaw(node_res["path"][p][3*(o-1)+3].as<double>());
        }
        path.append(state);
    }

    env.getAllForAllSpaceInformation()->copyState(state_init,path.getState(0));

    env.freeState(state_init);
    env.freeState(state);

    return 0;
} 