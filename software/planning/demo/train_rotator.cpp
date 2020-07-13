#include <fstream>

#define DEBUG 0

#include "mdp_planner.hpp"

using namespace std;

int main()
{
    int n_objs = 2;
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

    double yaw = 10 * M_PI / 180.0;


    SoapRotators planner(objects,yaw);    
    ofstream ofs("/home/cs1080/tmp.save");
    planner.store(ofs);
    ofs.close();    
/*
    ifstream ifs("/home/cs1080/tmp.save");
    SoapRotators planner(ifs);
    ifs.close();
*/
    ompl::geometric::PathGeometric path(planner.getQuerySpaceInformation());
    ompl::base::State* state_init = planner.getQuerySpaceInformation()->allocState();
    
    STATE_OBJECT(state_init,1)->setX(10);
    STATE_OBJECT(state_init,1)->setY(20);
    STATE_OBJECT(state_init,1)->setYaw(0);
    STATE_OBJECT(state_init,2)->setX(15);
    STATE_OBJECT(state_init,2)->setY(15);
    STATE_OBJECT(state_init,2)->setYaw(0);

    std::vector<SoapRotators::DockingPoint> possible_dockings;
    planner.getDockingPoints( state_init, 0, 0, possible_dockings, true );

    cout << "state docking: ";
    cout << STATE_OBJECT(possible_dockings[0].state,1)->getX()   << ", ";
    cout << STATE_OBJECT(possible_dockings[0].state,1)->getY()   << ", ";
    cout << STATE_OBJECT(possible_dockings[0].state,1)->getYaw() << ", ";
    cout << STATE_OBJECT(possible_dockings[0].state,2)->getX()   << ", ";
    cout << STATE_OBJECT(possible_dockings[0].state,2)->getY()   << ", ";
    cout << STATE_OBJECT(possible_dockings[0].state,2)->getYaw() << endl;

    vector<MDP::action_t> actions;
    planner.plan(possible_dockings[0], actions, path);

    for( int i=0; i<actions.size(); i++ )
    {
        cout << "action: " << actions[i].x   << ", "
                           << actions[i].y   << ", "
                           << actions[i].yaw << endl;
    }

    for( int i=0; i<path.getStateCount(); i++ )
    {
        cout << "state: ";
        cout << STATE_OBJECT(path.getState(i),1)->getX()   << ", ";
        cout << STATE_OBJECT(path.getState(i),1)->getY()   << ", ";
        cout << STATE_OBJECT(path.getState(i),1)->getYaw() << ", ";
        cout << STATE_OBJECT(path.getState(i),2)->getX()   << ", ";
        cout << STATE_OBJECT(path.getState(i),2)->getY()   << ", ";
        cout << STATE_OBJECT(path.getState(i),2)->getYaw() << endl;
    }

#if 0
    double x_offset = -0.096;
    double yaw = 10 * M_PI / 180.0;
    
#if 0
    MDP_Rotator rotator(yaw, objects, x_offset);
    rotator.setup();
    rotator.constructGraph();
    rotator.valueIterations();
    cout << "saving... ";
    ofstream ofs("/home/cs1080/tmp.save");    
    ofs << rotator;
    ofs.close();
    cout << "[done]" << endl; 
#else
    MDP_Rotator rotator;
    cout << "loading... " << flush;
    ifstream ifs("/home/cs1080/tmp.save");
    ifs >> rotator;
    ifs.close();
    cout << "[done]" << endl;        
#endif

    std::vector<MDP::action_t> actions;
    ompl::base::State* state     = rotator.si_->allocState();
    ompl::base::State* state_res = rotator.si_->allocState();

    STATE_OBJREL_0(state)->value = 0;
    STATE_OBJREL_1(state)->setX(-0.096);    
    STATE_OBJREL_1(state)->setY(0);
    STATE_OBJREL_1(state)->setYaw(0);

    rotator.policies(state,actions,state_res);
    for( int i=0; i<actions.size(); i++ )
    {
        cout << "action: " << actions[i].x << "," 
                           << actions[i].y << "," 
                           << actions[i].yaw  << endl;
    }
    rotator.si_->printState(state_res);
#endif
    return 0;
}