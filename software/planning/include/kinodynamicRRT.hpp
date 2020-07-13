#ifndef KINODYNAMICRRT__HPP
#define KINODYNAMICRRT__HPP

#include "ompl/geometric/planners/rrt/RRT.h"
//#include "planning/omplRRT.hpp"
#include <ompl/base/spaces/SE2StateSpace.h>
#include "BulletCollision/CollisionShapes/btConvexShape.h"

class KinodynamicRRT : public ompl::geometric::RRT
{
public:

    enum TYPE_ACTION
    {
        ACTION_TRANSITION=-1,
        ACTION_PUSH=0,
        ACTION_ROTATE,
        NUM_ACTION_TO_SAMPLE
    };

    struct Action
    {        
        TYPE_ACTION type;
        int idx_target;
        double x;
        double y;
        double yaw;
    };

    struct SimShape
    {
        btConvexShape* shape;
        double z_offset;
        btQuaternion q_offset;
        double mass;
    };

    KinodynamicRRT( const ompl::base::SpaceInformationPtr &si, 
                    int n_sample_actions_,
                    int n_objects,
                    int o_target,
                    int o_pusher,
                    bool star = false                           )
     : ompl::geometric::RRT(si)
    {
        n_sample_actions = n_sample_actions_;
        n_objs = n_objects;
        is_star = star;

        space_endeff = si_->getStateSpace()
                       ->as<ompl::base::CompoundStateSpace>()
                       ->as<ompl::base::SE2StateSpace>(0);
/*
        for( int o=0; o<n_objects; o++ )
        {
            idxes_remain.push_back(o);
        }
*/
        idx_target_ = o_target-1;
        idx_pusher_ = o_pusher-1;
    }

    ~KinodynamicRRT()
    {
        
    }

    void setTargetGoalXY( double x, double y )
    {
        x_goal_ = x;
        y_goal_ = y;
    }

    void AddShapes( const std::vector<SimShape> &shapes )
    {
        simshapes.insert(simshapes.end(), shapes.begin(), shapes.end());
    }

    void AddShape(btConvexShape* shape, double z, btQuaternion &q, double mass)
    {
        SimShape simshape;
        simshape.shape = shape;        
        simshape.z_offset = z;
        simshape.q_offset = q;
        simshape.mass = mass;

        simshapes.push_back(simshape);
    }

    void PrintAction(const Action &action)
    {
        if( action.type == ACTION_TRANSITION )
        {
            std::cout << "ACTION [TRS] [" << action.idx_target << "] " << action.x << "," << action.y << std::endl;
        }
        else if( action.type == ACTION_PUSH )
        {
            std::cout << "ACTION [PSH] [" << action.idx_target << "] " << action.x << "," << action.y << std::endl;
        }
        else if( action.type == ACTION_ROTATE )
        {
            std::cout << "ACTION [ROT] [" << action.idx_target << "] " << action.yaw / M_PI * 180 << std::endl;
        }
        else
        {
            std::cout << "ACTION [UNKNOWN] ??? " << action.type 
            << " [" << action.idx_target << "] " << action.x << "," << action.y << "," << action.yaw << std::endl;
        }
    }

    double GetCost(ompl::base::State* s1, int idx_obj, ompl::base::State* s2);

    ompl::base::PlannerStatus 
        solve(const ompl::base::PlannerTerminationCondition &ptc);

private:

    void SampleAction(Action *action);
    double ContstrainedProp(const Motion &parent, const Action &action, Motion* result);

    void DebugMotion(const ompl::base::State* state);

    int n_sample_actions;
    int n_objs;
    bool is_star;

    ompl::base::SE2StateSpace* space_endeff;
    
    std::vector<SimShape> simshapes;

/*
    const ompl::base::State* goal_;
    std::vector<int> idxes_done;
    std::vector<int> idxes_remain;
*/

    int idx_target_;
    int idx_pusher_;
    double x_goal_;
    double y_goal_;
};

#endif