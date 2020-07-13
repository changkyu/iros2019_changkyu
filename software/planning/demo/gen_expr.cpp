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
    string dp_output;    
    string name_experiment;
    bool vis;
    bool goal_only;
    vector<int> ns;

    po::options_description desc("Example Usage");
    desc.add_options()
        ("help", "help")
        ("outdir,o", po::value<string>(&dp_output)->default_value(""),      "output directory")
        ("expr,e",   po::value<string>(&name_experiment)->default_value(""), "the name of experiment [openspace_sim, tabletop_kuka, redbox_kuka, amazonbox_kuka]")
        ("vis,v",    po::value<bool>(&vis)->default_value(false), "visualize")
        ("goal,g",   po::value<bool>(&goal_only)->default_value(false), "goal_only")
        ("nobjs,n",  po::value<vector<int> >(&ns)->multitoken(), "numbers of objects")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if( vm.count("help") ) 
    {        
        cout << desc << endl;
        return 0;
    }
    if( name_experiment.compare("")==0 )
    {
        cout << desc << endl;
        cerr << "[Error] name is required" << endl;
        return 0;
    }
    else if( name_experiment.compare("openspace_sim")!=0 &&
             name_experiment.compare("tabletop_kuka")!=0 &&
             name_experiment.compare("redbox_kuka")  !=0 &&
             name_experiment.compare("bluebox_kuka")  !=0 &&
             name_experiment.compare("rectbox_kuka")  !=0 &&
             name_experiment.compare("bluebox_video")  !=0 &&
             name_experiment.compare("tabletop_video")  !=0 &&
             name_experiment.compare("amazonbox_kuka")  !=0    )
    {
        cerr << "[Error] Unknown experiment" << endl;
        return 0;
    }

    fs::path path_output(dp_output);
    if( !fs::is_directory(path_output) )
    {
        cout << desc << endl;
        cerr << "[Error] Invalid output directory: " << dp_output << endl;
        return 0;   
    }

    fs::path path_output_name(dp_output + "/" + name_experiment);
    if( !fs::is_directory(path_output_name) )
    {
        boost::filesystem::create_directory(path_output_name);
    }

    double times[] = {10};
    for( int i=1; i<=20; i++ )
    {
        for( int j=0; j<ns.size(); j++ )
        {
            int n_objs = ns[j];
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
                env = new BoxSetup(objects,2.00);
            }
            else if( name_experiment.compare("tabletop_kuka")==0 )
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
                vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.47, 0.12, 0.27};
                env = new KukaBoxSetup(objects,workspace);
            }
            else if( name_experiment.compare("rectbox_kuka")==0 )
            {
                vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.50, 0.12, 0.30};
                env = new KukaBoxSetup(objects,workspace);
            }
            else if( name_experiment.compare("tabletop_video")==0 )
            {
                env = new KukaTableSetup(objects);
            }
            else if( name_experiment.compare("bluebox_video")==0 )
            {
                vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.47, 0.12, 0.27};
                env = new KukaBoxSetup(objects,workspace);
            }
            else if( name_experiment.compare("amazonbox_kuka")==0 )
            {
                vector<double> workspace{0.43, -0.36, -0.140, 0.615, 0.615, 0.348, 0.348, 0.31, 0.12, 0.24};
                env = new KukaBoxSetup(objects,workspace);
            }
            else
            {
                cerr << "Unknown Experiment: " << name_experiment << endl;
                return 0;
            }
            
            char fp_init[256], fp_goal[256];
            sprintf(fp_init,"%s/%s/%s.n%d.%03d.init", dp_output.c_str(), name_experiment.c_str(), "dove_beauty_bar", n_objs, i);
            sprintf(fp_goal,"%s/%s/%s.n%d.%03d.goal", dp_output.c_str(), name_experiment.c_str(), "dove_beauty_bar", n_objs, 1);

            Planner planner(*env);
            ompl::base::StateSamplerPtr sampler
             = env->getSingleForAllSpaceInformation()->allocStateSampler();
            ompl::base::State* sample
             = env->getAllForAllSpaceInformation()->allocState();
                        
            // goal
            if( goal_only==true )            
            {
                for( int o=1; o<=n_objs; o++ )
                {
                    ObjectState* sample_obj = STATE_OBJECT(sample,o);
                    double x,y,yaw;
                    env->getGoalState(o, &x, &y, &yaw);
                    sample_obj->setX(x);
                    sample_obj->setY(y);
                    sample_obj->setYaw(yaw);
                }

                ofstream ofs(fp_goal);
                ofs << "state: [";
                for( int o=1; o<=n_objs; o++ )
                {
                    ObjectState* sample_obj = STATE_OBJECT(sample,o);
                    ofs << sample_obj->getX()   << ",";
                    ofs << sample_obj->getY()   << ",";
                    ofs << sample_obj->getYaw() << ",";
                }
                ofs << "]" << endl;

                if( vis )
                {
                    cv::Mat img2 = cv::Mat::zeros(1000,1000,CV_8UC3);
                    env->visualizeSetup(img2);
                    planner.visualizeState(img2,sample);
                    cv::imshow("goal",img2);
                    cv::waitKey();
                }            
            }            

            // init
            if( goal_only==false )
            {
                vector<int> idxes_done;
                for( int o=1; o<=n_objs; o++ )
                {
                    env->setParamSingleForAll(o,idxes_done,sample);

                    ObjectState* sample_obj = STATE_OBJECT(sample,o);
                    sampler->sampleUniform(sample_obj);

                    idxes_done.push_back(o);
                }
                {
                    ofstream ofs(fp_init);
                    ofs << "state: [";
                    for( int o=1; o<=n_objs; o++ )
                    {
                        ObjectState* sample_obj = STATE_OBJECT(sample,o);
                        ofs << sample_obj->getX()   << ",";
                        ofs << sample_obj->getY()   << ",";
                        ofs << sample_obj->getYaw() << ",";
                    }
                    ofs << "]" << endl;
                }            
                if( vis )
                {
                    cv::Mat img = cv::Mat::zeros(1000,1000,CV_8UC3);
                    env->visualizeSetup(img);
                    planner.visualizeState(img,sample);
                    cv::imshow("init",img);
                    cv::waitKey();                    
                }
            }            
                
            env->getAllForAllSpaceInformation()->freeState(sample);

            delete env;
            for( int o=0; o<n_objs; o++ )
            {
                delete objects[o].shape;
            }
        }
    }
    return 0;
}