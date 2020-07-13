#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>
#define LOG BOOST_LOG_TRIVIAL(trace)

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "planner.hpp"
#include "ros_gazebo.hpp"
#include "ros_markers.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace std;
using namespace pcl;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tabletop_kuka");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    string env_name;
    double ee_z = 0.417;
    double plane_z = -0.21;
    string fp_path;
    vector<float> workspace;
    int idx=0;

    nh.getParam("/changkyu/pkg_ijcai2019/sim_scene/env_name",env_name);
    nh.getParam("/changkyu/pkg_ijcai2019/sim_scene/path",fp_path);
    nh.getParam("/changkyu/pkg_ijcai2019/sim_scene/endeff_z",ee_z);
    nh.getParam("/changkyu/pkg_ijcai2019/sim_scene/crop_space",workspace);
    nh.getParam("/changkyu/pkg_ijcai2019/sim_scene/plane_z",plane_z);
    nh.getParam("/changkyu/pkg_ijcai2019/sim_scene/idx",idx);

    fs::path path_path(fp_path);
    if( !fs::is_regular_file(path_path) )
    {
        ROS_ERROR_STREAM("Invalid file: " << fp_path);
        return 0;
    }

    Markers markers(nh);
    tf::Quaternion q_dove;
    q_dove.setRPY(0, -89.9999/ 180.0 * M_PI, 0);
    markers.AddMesh( "dove_beauty_bar", q_dove, 0.035*0.5 + plane_z );

    YAML::Node node_path = YAML::LoadFile(fp_path);
    int n_objs = (node_path["path"][0].size() - 1) / 3;

    // Provide Object Information    
    vector<string> names_obj(n_objs);
    vector<RobotObjectSetup::Object> objects(n_objs);
    for( int o=0; o<n_objs; o++ )
    {
        names_obj[o] = "dove_beauty_bar";
        objects[o].name = "dove_beauty_bar";
        objects[o].dims.resize(3);
        objects[o].dims[0] = 0.035;
        objects[o].dims[1] = 0.066;
        objects[o].dims[2] = 0.096;
        objects[o].radius  = sqrt(0.096*0.096 + 0.066*0.066)*0.5;
        objects[o].q_offset.setEulerZYX(0, -89.9999/ 180.0 * M_PI, 0);
        objects[o].shape = new btBoxShape(btVector3(objects[o].dims[0]*0.5, 
                                                    objects[o].dims[1]*0.5, 
                                                    objects[o].dims[2]*0.5));
        objects[o].z_offset = 0.035 * 0.5;
        objects[o].mass = 0.135;
    }

    vector<double> conf_objs;
    for( int o=0; o<n_objs; o++ )
    {
        double x   = node_path["path"][idx][o*3+1].as<double>();
        double y   = node_path["path"][idx][o*3+2].as<double>();
        double yaw = node_path["path"][idx][o*3+3].as<double>();
        conf_objs.push_back(x);
        conf_objs.push_back(y);
        conf_objs.push_back(yaw);
    }

    if( env_name.compare("openspace_sim")==0 )
    {
        markers.PublishPlane();
    }
    else
    {
        markers.PublishWorkspace();
    }
    markers.PublishObjects(conf_objs,names_obj);

    int len_path = node_path["path"].size();    
    vector<vector<vector<double> > > states(len_path);
    for( int p=0; p<len_path; p++ )
    {
        states[p].resize(n_objs+1);
        states[p][0].resize(1);
        states[p][0][0] = node_path["path"][p][0].as<double>();
        for( int o=1; o<=n_objs; o++ )
        {
            states[p][o].resize(3);
            states[p][o][0] = node_path["path"][p][(o-1)*3+1].as<double>();
            states[p][o][1] = node_path["path"][p][(o-1)*3+2].as<double>();
            states[p][o][2] = node_path["path"][p][(o-1)*3+3].as<double>();
        }
    }
    
    vector<Planner::Action> actions;
    for( int p=0; p<node_path["actions"].size(); p++ )
    {
        Planner::Action action;
        action.type        = (Planner::TYPE_ACTION)node_path["actions"][p][0].as<int>();
        action.idx_target  = node_path["actions"][p][1].as<int>();
        action.idx_target2 = node_path["actions"][p][2].as<int>();
        action.x           = node_path["actions"][p][3].as<double>();
        action.y           = node_path["actions"][p][4].as<double>();
        action.yaw         = node_path["actions"][p][5].as<double>();

        actions.push_back(action);
    }    

    MyGazebo robot(nh);
    robot.GotoHome();
    robot.AddMesh( "dove_beauty_bar", q_dove, 0.035*0.5 - 0.210 );

    {
        vector<string> names_obj;
        vector<string> names_all;
        for( int o=1; o<=9; o++ )
        {
            stringstream ss;
            ss << "obj" << o;
            names_all.push_back(ss.str());
            names_obj.push_back("dove_beauty_bar");
        }
        robot.SetObjectNames(names_obj);
        robot.DeleteAllModels(names_all);        
    }

    for( int o=1; o<=n_objs; o++ )
    {
        stringstream ss;
        ss << "obj" << o;
        robot.SpawnModel( ss.str(), "dove_beauty_bar", 
                          states[0][o][0], states[0][o][1], states[0][o][2] );
    }

    cout << "actions.size(): " << actions.size() << ", states.size(): " << states.size() << endl;

    int p = 1;
    for( int a=0; a<actions.size(); a++ )
    {
        if( actions[a].type==Planner::ACTION_TRANSITION )
        {       
            cout << "ACTION_TRANSITION" << ": "
                 << actions[a].x    << ", "
                 << actions[a].y    << ", "
                 << actions[a].yaw  << endl;

            robot.Transition(actions[a].idx_target,actions[a].x,actions[a].y,actions[a].yaw);
        }
        else if( actions[a].type==Planner::ACTION_TRANSFER ||
                 actions[a].type==Planner::ACTION_PUSHING     )
        {
            bool is_pushing = actions[a].type==Planner::ACTION_PUSHING;
            vector<double> xs, ys, yaws;
            xs.push_back(actions[a-1].x);
            ys.push_back(actions[a-1].y);
            yaws.push_back(actions[a-1].yaw);
            int o = actions[a].idx_target;
            int o2 = actions[a].idx_target2;
            while( a<actions.size() && 
                   (actions[a].type==Planner::ACTION_TRANSFER ||
                   (actions[a].type==Planner::ACTION_PUSHING    ) ) )
            {
                cout << ((actions[a].type==Planner::ACTION_TRANSFER)?"ACTION_TRANSFER: ":"ACTION_PUSHING: ")
                     << actions[a].x    << ", "
                     << actions[a].y    << ", "
                     << actions[a].yaw  << endl;
                xs.push_back(actions[a].x);
                ys.push_back(actions[a].y);
                yaws.push_back(actions[a].yaw);
                a++;
            }

            for( int i=0; i<xs.size(); i++ )
            {
                robot.Transfer(o, o2, xs[i],ys[i],yaws[i]);

                if( xs[i]  ==states[p-1][o][0] &&
                    ys[i]  ==states[p-1][o][1] &&
                    yaws[i]==states[p-1][o][2]    )
                {

                }
                else
                {
                    robot.SetState(states[p]);
                    p++;                
                }                
            }

            a--;
        }
    }

    robot.GotoHome();
}
