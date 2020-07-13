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
    int id;
    string name_experiment;
    string name_planner;
    bool do_merge;
    bool vis;
    bool skip;
    vector<int> ns;
    vector<int> ks{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

    po::options_description desc("Example Usage");
    desc.add_options()
        ("help", "help")
        ("id,i",   po::value<int>(&id)->default_value(0), "id")
        ("idx,k",  po::value<vector<int>>(&ks)->multitoken(), "init #")
        ("vis,v",  po::value<bool>(&vis)->default_value(false), "vis")
        ("skip,s", po::value<bool>(&skip)->default_value(true), "skip")
        ("nobjs,n",po::value<vector<int> >(&ns)->multitoken(), "numbers of objects")
        //("merge,m",po::value<bool>(&do_merge)->default_value(true), "do_merge")
        ("expr,e", po::value<string>(&name_experiment)->default_value("tabletop_kuka"), 
                   "the name of experiment [openspace_sim, tabletop_kuka, redbox_kuka, amazonbox_kuka]")
        ("planner,p", po::value<string>(&name_planner)->default_value("ours"), 
                   "the name of planner [ours, plrs, mopl]")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if( vm.count("help") ) 
    {        
        cout << desc << endl;
        return 0;
    }

    string dp_root = "/home/cs1080/projects/ijcai2019/software/planning/";

    if( name_planner.compare("ours_selfish")==0 )
    {
        do_merge = false;
    }
    else if( name_planner.compare("ours_pushing")==0 )
    {
        do_merge = true;
    }

/*
    int i_max = 20;
    if( name_experiment.compare("bluebox_kuka")==0 )
    {
        i_max = 10;
    }
*/        
    //for( int i=1; i<=i_max; i++ )    
    for( int k=0;k<ks.size(); k++)
    {   
        int i=ks[k];
        //if( k!=-1 && i!=k ) continue;
        for( int j=0; j<ns.size(); j++ )
        {
            int n = ns[j];            
            string name = "ours";

            int n_objs = n;
            vector<RobotObjectSetup::Object> objects(n_objs);
            for( int o=0; o<n_objs; o++ )
            {
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
            
            RobotObjectSetup* env;
            if( name_experiment.compare("openspace_sim")==0 )
            {
                env = new BoxSetup(objects,2.10);
            }
            else if( name_experiment.compare("tabletop_kuka")==0 )
            {
                env = new KukaTableSetup(objects);
            }
            else if( name_experiment.compare("tabletop_video")==0 )
            {
                env = new KukaTableSetup(objects);
            }
            else if( name_experiment.compare("redbox_kuka")==0 )
            {
                vector<double> workspace{0.43, 0.34, -0.095, 0.6123724, 0.6123724, -0.3535534, -0.3535534, 0.44, 0.22, 0.34};                
                env = new KukaBoxSetup(objects,workspace);
            }
            else if( name_experiment.compare("bluebox_kuka")==0 )
            {
                //vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.47, 0.12, 0.27};                
                vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.90, 0.12, 0.60};
                env = new KukaBoxSetup(objects,workspace);
            }
            else if( name_experiment.compare("amazonbox_kuka")==0 )
            {
                vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.31, 0.12, 0.24};
                env = new KukaBoxSetup(objects,workspace);                
            }
            else if( name_experiment.compare("rectbox_kuka")==0 )
            {
                vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 1.20, 0.12, 0.90};
                env = new KukaBoxSetup(objects,workspace);                
            }
            else
            {
                cerr << "[Error] Unknown Experiment: " << name_experiment << endl;
                return 0;
            }

            char fp_init[256], fp_goal[256], fp_res[256];
            sprintf(fp_init,"%s/input/%s/%s.n%d.%03d.init",                 dp_root.c_str(), name_experiment.c_str(), "dove_beauty_bar", n_objs, i);
            sprintf(fp_goal,"%s/input/%s/%s.n%d.%03d.goal",                 dp_root.c_str(), name_experiment.c_str(), "dove_beauty_bar", n_objs, 1);
            sprintf(fp_res,"%s/result/%s/now/%s/%s.%s.n%d.%03d.id%03d.res", dp_root.c_str(), name_experiment.c_str(), name_planner.c_str(), name_planner.c_str(), "dove_beauty_bar", n_objs, i, id);

            fs::path path_res(fp_res);
            if( skip && fs::is_regular_file(path_res)==true )
            {
                cout << "[SKIP]" << fp_res << " already exists" << endl;
                continue;
            }

            cout << "[READ] " << fp_init << endl;
            cout << "[READ] " << fp_goal << endl;            


            ompl::base::State* state_init = env->allocState();
            ompl::base::State* state_goal = env->allocState();

            YAML::Node node_init = YAML::LoadFile(fp_init);
            YAML::Node node_goal = YAML::LoadFile(fp_goal);

            STATE_ROBOT(state_init) = -1;
            STATE_ROBOT(state_goal) = -1;
            for( int o=1; o<=n_objs; o++ )
            {
                STATE_OBJECT(state_init,o)->setX(  node_init["state"][0+(o-1)*3].as<double>());
                STATE_OBJECT(state_init,o)->setY(  node_init["state"][1+(o-1)*3].as<double>());
                STATE_OBJECT(state_init,o)->setYaw(node_init["state"][2+(o-1)*3].as<double>());

                STATE_OBJECT(state_goal,o)->setX(  node_goal["state"][0+(o-1)*3].as<double>());
                STATE_OBJECT(state_goal,o)->setY(  node_goal["state"][1+(o-1)*3].as<double>());
                STATE_OBJECT(state_goal,o)->setYaw(node_goal["state"][2+(o-1)*3].as<double>());
            }

            if( name_experiment.compare("bluebox_kuka")==0  ||
                name_experiment.compare("rectbox_kuka")==0    )
            {
#if 0
                vector<double> xs,ys,yaws;
                for( int o=1; o<=n_objs; o++ )
                {
                    xs.push_back(  STATE_OBJECT(state_init,o)->getX());
                    ys.push_back(  STATE_OBJECT(state_init,o)->getY());
                    yaws.push_back(STATE_OBJECT(state_init,o)->getYaw());
                }
                for( int o=1; o<=n_objs; o++ )
                {
                    double x   = STATE_OBJECT(state_goal,o)->getX();
                    double y   = STATE_OBJECT(state_goal,o)->getY();
                    double yaw = STATE_OBJECT(state_goal,o)->getYaw();
                
                    int i_min = -1;
                    double dist_min = INFINITY;
                    for( int i=0; i<n_objs; i++ )
                    {
                        double dist = sqrt((x-xs[i])*(x-xs[i])+(y-ys[i])*(y-ys[i]));
                        if( dist_min > dist )
                        {
                            dist_min = dist;
                            i_min = i;
                        }
                    }

                    STATE_OBJECT(state_init,o)->setX(xs[i_min]);
                    STATE_OBJECT(state_init,o)->setY(ys[i_min]);
                    STATE_OBJECT(state_init,o)->setYaw(yaws[i_min]);

                    xs[i_min] = INFINITY;
                    ys[i_min] = INFINITY;
                }
#else 
                vector<double> xs,ys,yaws;
                for( int o=1; o<=n_objs; o++ )
                {
                    xs.push_back(  STATE_OBJECT(state_init,o)->getX());
                    ys.push_back(  STATE_OBJECT(state_init,o)->getY());
                    yaws.push_back(STATE_OBJECT(state_init,o)->getYaw());
                }
                for( int o=1; o<=n_objs; o++ )
                {
                    /*
                    double x   = STATE_OBJECT(state_goal,o)->getX();
                    double y   = STATE_OBJECT(state_goal,o)->getY();
                    double yaw = STATE_OBJECT(state_goal,o)->getYaw();
                    */
                    double x,y;
                    if( o==1 )
                    {
                        x   = STATE_OBJECT(state_goal,o)->getX();
                        y   = STATE_OBJECT(state_goal,o)->getY();
                    }
                    else
                    {
                        double x_del = STATE_OBJECT(state_goal,o)->getX() - STATE_OBJECT(state_goal,1)->getX();
                        double y_del = STATE_OBJECT(state_goal,o)->getY() - STATE_OBJECT(state_goal,1)->getY();
                        x = STATE_OBJECT(state_init,1)->getX() + x_del;
                        y = STATE_OBJECT(state_init,1)->getY() + y_del;
                    }
                
                    int i_min = -1;
                    double dist_min = INFINITY;
                    for( int i=0; i<n_objs; i++ )
                    {
                        double dist = sqrt((x-xs[i])*(x-xs[i])+(y-ys[i])*(y-ys[i]));
                        if( dist_min > dist )
                        {
                            dist_min = dist;
                            i_min = i;
                        }
                    }

                    STATE_OBJECT(state_init,o)->setX(xs[i_min]);
                    STATE_OBJECT(state_init,o)->setY(ys[i_min]);
                    STATE_OBJECT(state_init,o)->setYaw(yaws[i_min]);

                    xs[i_min] = INFINITY;
                    ys[i_min] = INFINITY;
                }
#endif
            }

            og::PathGeometric path(env->getAllForAllSpaceInformation());
            vector<Planner::Action> actions;

            Planner planner(*env);

            clock_t begin, end;
            begin = clock();
            
            if( name_planner.compare(0,4,"ours")==0 )
            {
                ifstream ifs("/home/cs1080/tmp.save");
                planner.load_precomputed_planners(ifs);
                ifs.close();
                planner.plan(state_init, state_goal, path, actions, do_merge);
            }
            else if( name_planner.compare("plrs")==0 )
            {
                planner.plan_plRS(state_init, state_goal, path, actions);
            }
            else if( name_planner.compare("mopl")==0 )
            {

            }
            else if( name_planner.compare("kino")==0 )
            {
                planner.UseKino();
                planner.plan(state_init, state_goal, path, actions, true);
            }
            else                
            {
                cerr << "[Error] Unknown planner: " << name_planner << endl;
                return 0;
            }

            end = clock();
            double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;          
            planner.save_plan( fp_res, name, n_objs, time_spent, -1, state_init, state_goal, path, actions );

            if( vis )
            {
                cv::Mat img = cv::Mat::zeros(1000,1000,CV_8UC3);
                env->visualizeSetup(img);
                planner.visualizePath(img,path);
                cv::imshow("vis",img);
                cv::waitKey();
            }
            

            env->freeState(state_init);
            env->freeState(state_goal);
            delete env;
        }
    }
    return 0;
}
