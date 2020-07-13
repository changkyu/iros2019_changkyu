#include "kinodynamicRRT.hpp"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <boost/math/constants/constants.hpp>

#include "bullet_simulation/bullet_simulation.hpp"

using namespace std;
using namespace ompl;

void KinodynamicRRT::SampleAction(Action* action)
{
    action->type = (TYPE_ACTION)(rand() % NUM_ACTION_TO_SAMPLE);

//    int rnd_type = rand() % 4;
//    if( rnd_type==0 ) action->type = ACTION_ROTATE;
//    else              action->type = ACTION_PUSH;

    /*
    double p = (((double)rand())/(double)RAND_MAX);
    if( p >= 0.9 || idx_target == -1 )
    {
        action->idx_target = idxes_remain[rand() % idxes_remain.size()];        
        idx_target = action->idx_target;
    }
    else
    {
        action->idx_target = idx_target;
    }
    */
    action->idx_target = idx_pusher_;   

    if( action->type == ACTION_PUSH )
    {        
        double theta = (((double)rand())/(double)RAND_MAX)*0.90 * 2 * M_PI; //[0~0.9*2pi]
        double len = (((double)rand())/(double)RAND_MAX) * 0.09 + 0.1; //[0.01~0.10]
        
        action->x = cos(theta) * len;
        action->y = sin(theta) * len;
        action->yaw = NAN;
    }
    else if( action->type == ACTION_ROTATE )
    {
        double theta = ( 0.05 + (((double)rand())/(double)RAND_MAX)*0.1 ) * M_PI; //[0.05 ~ 0.95*pi]
        //double theta = (((double)rand())/(double)RAND_MAX) * 10 * M_PI / 180.0;
        if( rand()%2==0 ) theta = -theta;

        action->x = NAN;
        action->y = NAN;
        action->yaw = theta;
    }
    else
    {
        OMPL_ERROR("Invalid action type");
    }
}

double KinodynamicRRT::GetCost(ompl::base::State* s1, int idx_obj, ompl::base::State* s2)
{
    ompl::base::SE2StateSpace::StateType* endeff_init
     = s1->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateSpace::StateType>(0);
    ompl::base::SE2StateSpace::StateType* endeff_pre
     = s1->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateSpace::StateType>(idx_obj+1);
    ompl::base::SE2StateSpace::StateType* endeff_res
     = s2->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateSpace::StateType>(0);

    return   space_endeff->distance(endeff_init, endeff_pre)
           + space_endeff->distance(endeff_pre,  endeff_res);
}

#define DEBUG 0

double KinodynamicRRT::ContstrainedProp( const Motion &parent, 
                                         const Action &action, Motion* result)
{
    /*
    cout << "action.type: " << action.type 
                  << ", target: " << action.idx_target
                  << ", x:" << action.x 
                  << ", y:" << action.y 
                  << ", yaw:" << action.yaw
                  << endl;
    */
    
    int k = action.idx_target;
    base::SE2StateSpace::StateType* object_states[n_objs];
    for( int o=0; o<n_objs; o++ )
    {
        object_states[o]
         = (*parent.state->as<base::CompoundState>())[o+1]->as<base::SE2StateSpace::StateType>();
    }
    
    vector<int> idxes(n_objs);
    vector<btVector3> positions(n_objs);
    vector<btQuaternion> rotations(n_objs);
    for( int o=0; o<n_objs; o++ )
    {        
        positions[o][0] = object_states[o]->getX();
        positions[o][1] = object_states[o]->getY();
        positions[o][2] = simshapes[o].z_offset;
        
        rotations[o].setEulerZYX(object_states[o]->getYaw(),0,0);
        rotations[o] = rotations[o] * simshapes[o].q_offset;
    }

#if DEBUG
    BulletSimulationGui sim;
#else
    BulletSimulation sim;
#endif

    // set objects
    sim.SetGravity(btVector3(0,0,-0.1));
    sim.AddPlaneShape(btVector4(0,0,1,0));
    for( int o=0; o<n_objs; o++ )
    {
        float mass = k==o?0:simshapes[o].mass;
        idxes[o] = sim.AddCollisionShape( simshapes[o].shape, positions[o],rotations[o], 
                                          mass, 1.0, false                 );
    }

#if DEBUG
    sim.SpinInit();
    //sim.ResetCamera( 0.3,-90,45, 0,0,0 );
    //sim.ResetCamera( 0.3,0,80, 0.3,-0.3,0 );
    sim.ResetCamera( 1.0,0,80, 1.0,0.0,0 );    
    //sim.Spin(0.1);    
#endif

    const double delta_time = 0.01;
    for( float time=0; time < 1; time += delta_time )
    {
        sim.StepSimulation(delta_time);
    }

    if( action.type == ACTION_PUSH )
    {
        btVector3 lin_vec(action.x,action.y,0);
        sim.MoveRotateObject(idxes[k], lin_vec, 0);
    }
    else if( action.type == ACTION_ROTATE )
    {
        sim.MoveRotateObject(idxes[k], btVector3(0,0,0), action.yaw);
    }
    
    for( float time=0; time < 1; time += delta_time )
    {
        sim.StepSimulation(delta_time);
    }

    // result state
    base::SE2StateSpace::StateType* object_result[n_objs];
    for( int o=0; o<n_objs; o++ )
    {
        object_result[o]
         = (*result->state->as<base::CompoundState>())[o+1]->as<base::SE2StateSpace::StateType>();
    }    
    
    for( int o=0; o<n_objs; o++ )
    {
        btVector3 position_res;
        btQuaternion rotation_res;
        sim.GetObjectPose(idxes[o], rotation_res, position_res);

        btScalar yaw, pitch, roll;
        rotation_res.getEulerZYX( yaw, pitch, roll);
        object_result[o]->setX(position_res[0]);
        object_result[o]->setY(position_res[1]);
        object_result[o]->setYaw(yaw);
    }

    result->state->as<base::CompoundStateSpace::StateType>()
                 ->as<base::RealVectorStateSpace::StateType>(0)->values[0] = k+1;
    
    // result cost    
    //result->cost = parent.cost + GetCost(parent.state,k,result->state);
    //result->cost = parent.cost + si_->distance(parent.state,result->state);
    
#if DEBUG    
    //sim.Spin(0.1);
    sim.SpinExit();
#endif

    return 0;
}

void KinodynamicRRT::DebugMotion( const ompl::base::State* state )
{
    const base::SE2StateSpace::StateType* object_states[n_objs];
    for( int o=0; o<n_objs; o++ )
    {
        object_states[o] = state->as<base::CompoundStateSpace::StateType>()
                                ->as<base::SE2StateSpace::StateType>(o+1);
    }

    vector<int> idxes(n_objs);
    vector<btVector3> positions(n_objs);
    vector<btQuaternion> rotations(n_objs);
    for( int o=0; o<n_objs; o++ )
    {        
        positions[o][0] = object_states[o]->getX();
        positions[o][1] = object_states[o]->getY();
        positions[o][2] = simshapes[o].z_offset;
        
        rotations[o].setEulerZYX(object_states[o]->getYaw(),0,0);
        rotations[o] = rotations[o] * simshapes[o].q_offset;
    }

    BulletSimulationGui sim;

    // set objects
    sim.SetGravity(btVector3(0,0,-0.1));
    sim.AddPlaneShape(btVector4(0,0,1,0));                    
    for( int o=0; o<n_objs; o++ )
    {
        idxes[o] = sim.AddCollisionShape( simshapes[o].shape, positions[o],rotations[o], 
                                          0, 1.0, false                 );
    }

    sim.SpinInit();
    sim.ResetCamera( 0.3,0,80, 0.0,0.0,0 );
    sim.Spin(0.1);    
    sim.SpinExit();
}

base::PlannerStatus 
KinodynamicRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                 *goal   = pdef_->getGoal().get();
//    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        /*
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
        */

        // [changkyu]
        int n_objs = si_->getStateSpace()->as<base::CompoundStateSpace>()->getSubspaceCount()-1;
        for( int o=1; o<=n_objs; o++ )
        {
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, st);
            motion->state->as<base::CompoundStateSpace::StateType>()
                         ->as<base::RealVectorStateSpace::StateType>(0)->values[0] = o;
            //motion->cost = 0;
            nn_->add(motion);
        }
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("Starting with %u states", nn_->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion *rmotion   = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    std::vector<Motion *> nbh;
    double cost_min = std::numeric_limits<double>::infinity();
    double k_rrg = boost::math::constants::e<double>() + (boost::math::constants::e<double>()/(double)si_->getStateSpace()->getDimension());
    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
//        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
//            goal_s->sampleGoal(rstate);
//        else
        {
            sampler_->sampleUniform(rstate);
        }

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        /* find state to add */
        int i_min = -1;
        double d_min = INFINITY;
        Motion* imotions[n_sample_actions];
        Action ractions[n_sample_actions];
        for( int i=0; i<n_sample_actions; i++ )
        {
            imotions[i] = new Motion(si_);
            SampleAction(&ractions[i]);
            ContstrainedProp(*nmotion, ractions[i], imotions[i]);
            
            //double d = distanceFunctionState(imotions[i]->state, rstate);
            double d = si_->distance(imotions[i]->state, rstate);
            if( d < d_min )
            {
                d_min = d;
                i_min = i;
            }
        }

        if ( si_->checkMotion(nmotion->state, imotions[i_min]->state)==false ||
             i_min == -1 )
        {            
            continue;
        }

        /* create a motion */
        Motion *motion = new Motion(si_);
        /*
        if( is_star )
        {
            Motion* motion_parent = nmotion;
            Motion* motion_new = imotions[i_min];
            Action action = ractions[i_min];

            std::vector<Motion*> nbh;
            unsigned int k = std::ceil(k_rrg * log((double)nn_->size()+1));
            nn_->nearestK(motion_new, k, nbh);

            Motion* jmotions[n_sample_actions * nbh.size()];
            Action  jactions[n_sample_actions * nbh.size()];
            for(int n=0; n<nbh.size(); n++)
            {
                for( int j=0; j<n_sample_actions; j++ )
                {
                    int idx = n*n_sample_actions + j;
                    jmotions[idx] = new Motion(si_);
                    SampleAction(&jactions[idx]);
                    ContstrainedProp(*nbh[n], jactions[idx], jmotions[idx]);

                    double d = distanceFunctionState(jmotions[idx]->state, rstate);
                    if( ( d-d_min < 0.01*n_objs ) &&
                        //( motion_new->cost > jmotions[idx]->cost ) &&
                        ( si_->checkMotion(nbh[n]->state, jmotions[idx]->state))    )
                    {
                        //cout << "[UPDATE]" << endl;
                        motion_parent = nbh[n];
                        motion_new = jmotions[idx];
                        action = jactions[idx];
                    }
                }
            }

            si_->copyState(motion->state, motion_new->state);
            motion->parent = motion_parent;
            //motion->cost = motion_new->cost;

            for(int idx=0; idx<n_sample_actions*nbh.size(); idx++) 
                delete jmotions[idx];
        }    
        else
        */
        {
            si_->copyState(motion->state, imotions[i_min]->state);
            motion->parent = nmotion;
            //motion->cost = imotions[i_min]->cost;
        }

        for( int i=0; i<n_sample_actions; i++ ) delete imotions[i];

        nn_->add(motion);

        double dist = 0.0;
        //bool sat = goal->isSatisfied(motion->state, &dist);

        bool sat;
        {
            double x   = motion->state->as<ompl::base::CompoundStateSpace::StateType>()
                                      ->as<ompl::base::SE2StateSpace::StateType>(idx_target_+1)->getX();
            double y   = motion->state->as<ompl::base::CompoundStateSpace::StateType>()
                                      ->as<ompl::base::SE2StateSpace::StateType>(idx_target_+1)->getY();
           
            cout << "goal: " << x_goal_ << ", " << y_goal_ << " : " << x << ", " << y << endl;
 
            double dist = sqrt( (x_goal_ - x)*(x_goal_ - x) + 
                                (y_goal_ - y)*(y_goal_ - y)   );

            sat = dist < 0.01;            
        }
        
        if (sat)
        {
            approxdif = dist;
            solution = motion;
            break;
            /*
            if( cost_min > motion->cost )
            {
                cost_min = motion->cost;
                approxdif = dist;
                solution = motion;
            }
            */
        }
        if (dist < approxdif)
        {
            approxdif = dist;
            approxsol = motion;
        }

#if 0
        for( int o=0; o<n_objs; o++ )
        {
            const ompl::base::SE2StateSpace::StateType* state_gbj
             = goal_->as<ompl::base::CompoundStateSpace::StateType>()
                    ->as<ompl::base::SE2StateSpace::StateType>(o+1);

            const ompl::base::SE2StateSpace::StateType* state_obj
             = motion->state->as<ompl::base::CompoundStateSpace::StateType>()
                            ->as<ompl::base::SE2StateSpace::StateType>(o+1);

            int o_done = -1;
            int o_remain = -1;
            for( int oo=0; oo<idxes_done.size(); oo++ )
            {
                if( idxes_done[oo] == o )
                {
                    o_done = oo;                    
                }
            }
            for( int oo=0; oo<idxes_remain.size(); oo++ )
            {
                if( idxes_remain[oo] == o )
                {
                    o_remain = oo;
                }
            }

            if( si_->getStateSpace()->as<ompl::base::CompoundStateSpace>()
                   ->getSubspace(o+1)->distance(state_obj,state_gbj) < 0.005 )
            {
                if( o_done==-1 )
                {
                    idxes_done.push_back(o);
                    int tmp = idxes_remain[o_remain];
                    idxes_remain[o_remain] = idxes_remain.back();
                    idxes_remain.pop_back();
                }
            }
            else
            {
                if( o_done!=-1 )
                {
                    idxes_remain.push_back(o);                    
                    int tmp = idxes_done[o_done];
                    idxes_done[o_done] = idxes_done.back();
                    idxes_done.pop_back();
                }
            }
        }

        if( idxes_remain.size()==0 ) break;
#endif
    }

    bool solved = false;
    bool approximate = false;
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        geometric::PathGeometric *path = new geometric::PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
        {
            path->append(mpath[i]->state);
        }
            
        pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("Created %u states", nn_->size());

    OMPL_INFORM("approxdif: %f", approxdif);

    return base::PlannerStatus(solved, approximate);
}
