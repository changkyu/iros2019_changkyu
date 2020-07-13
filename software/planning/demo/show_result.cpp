#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include "planner.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace std;

int main(int argc, char* argv[])
{    
    bool action_based;
    bool do_sim;
    bool debug;
    bool show_goal;
    string fp_input;
    string fp_record;
    string env_name;
    po::options_description desc("Example Usage");
    desc.add_options()
        ("help", "help")
        ("input,i",  po::value<string>(&fp_input))        
        ("env,e",    po::value<string>(&env_name)->default_value("tabletop_kuka"))        
        ("sim,s",    po::value<bool>(&do_sim)->default_value(false))
        ("record,r", po::value<string>(&fp_record)->default_value("") )
        ("action,a", po::value<bool>(&action_based)->default_value(false) )
        ("goal,g",   po::value<bool>(&show_goal)->default_value(false) )
        ("debug,d",  po::value<bool>(&debug)->default_value(false) )
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if( vm.count("help") ) 
    {        
        cout << desc << endl;
        return 0;
    }

    fs::path path_input(fp_input);
    if( !fs::is_regular_file(path_input) )
    {
        cerr << "[Error] Invalid input: " << fp_input << endl;
        return 0;
    }

    YAML::Node node_res = YAML::LoadFile(fp_input);

    if( show_goal )
    {
        int n_objs = node_res["state"].size()/3;

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

        RobotObjectSetup* env = new BoxSetup(objects,2.00);
        Planner planner(*env);
        
        ompl::base::State* state = env->allocState();        
        for( int o=1; o<=n_objs; o++ )
        {
            STATE_OBJECT(state,o)->setX(  node_res["state"][(o-1)*3  ].as<double>());
            STATE_OBJECT(state,o)->setY(  node_res["state"][(o-1)*3+1].as<double>());
            STATE_OBJECT(state,o)->setYaw(node_res["state"][(o-1)*3+2].as<double>());
        }

        cv::Mat img = cv::Mat::zeros(1000,1000,CV_8UC3);
        planner.visualizeState(img,state);
        cv::imshow("goal",img);
        cv::waitKey();
        
        delete env;
        return 0;
    }


    int len_actions = node_res["actions"].size();
    int len_path = node_res["path"].size();
    int n_objs = (node_res["path"][0].size()-1) / 3;

cout << "len_path: " << len_path << endl;
cout << "n_objs: " << n_objs << endl;

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

    RobotObjectSetup* env;
    if( env_name.compare("openspace_sim")==0 )
    {
        env = new BoxSetup(objects,2.00);
    }
    else if( env_name.compare("tabletop_kuka")==0 )
    {
        env = new KukaTableSetup(objects);

    }
    else if( env_name.compare("redbox_kuka")==0 )
    {
        vector<double> workspace{0.43, 0.34, -0.095, 0.6123724, 0.6123724, -0.3535534, -0.3535534, 0.44, 0.22, 0.34};                
        env = new KukaBoxSetup(objects,workspace);
    }
    else if( env_name.compare("amazonbox_kuka")==0 )
    {
        vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.31, 0.12, 0.24};
        env = new KukaBoxSetup(objects,workspace);                
    }
    else if( env_name.compare("bluebox_kuka")==0 )
    {
        vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.47, 0.12, 0.27};
        env = new KukaBoxSetup(objects,workspace);                
    }
    else if( env_name.compare("rectbox_kuka")==0 )
    {
        vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 1.20, 0.12, 0.90};
        env = new KukaBoxSetup(objects,workspace);                
    }
    else
    {
        cerr << "[Error] Unknown Experiment: " << env_name << endl;
        return 0;
    }

    ompl::base::State* state = env->allocState();

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

    og::PathGeometric path(env->getAllForAllSpaceInformation());
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

    Planner planner(*env);
    if( do_sim )
    {
#if DEBUG
        if( action_based )
        {
            cout << "path.length(): " << path.getStateCount() << ", actions.size(): " << actions.size() << endl;
            ifstream ifs("/home/cs1080/tmp.save");
            planner.load_precomputed_planners(ifs);
            ifs.close();
            planner.simulate_gui(path,actions);
        }
        else
        {
            planner.simulate_gui(path, fp_record);    
        }
#endif
    }

    if( debug )
    {
        for( int p=0; p<len_path; p++ )
        {
            ompl::base::State* state = path.getState(p);
            cout << "p:" << p << endl;
            cv::Mat img = cv::Mat::zeros(1000,1000,CV_8UC3);
            env->visualizeSetup(img);
            planner.visualizeState(img,state);
            cv::imshow("path",img);
            int key = cv::waitKey();
            if( key==65361 )
            {
                if( p==0 ) p = -1;
                else p = p-2;
            }             
        }
    }

    cv::Mat img = cv::Mat::zeros(1000,1000,CV_8UC3);
    env->visualizeSetup(img);
    planner.visualizePath(img,path);
    cv::imshow("path",img);
    cv::waitKey();
    //planner.simulate(path);

    env->freeState(state);
    delete env;
}