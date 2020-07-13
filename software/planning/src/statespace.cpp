#include "statespace.hpp"
#include "bullet_simulation/collision_simulation.hpp"

namespace ob = ompl::base;
using namespace std;

double 
RobotObjectStateSpace::distance( const ob::State *state1, 
                                 const ob::State *state2 ) const
{    
    int o_1 = STATE_ROBOT(state1);
    int o_2 = STATE_ROBOT(state2);

    if( o_1<=0 || o_2<=0 )
    {
        return ompl::base::CompoundStateSpace::distance(state1, state2);
    }    
    else if( o_1 == o_2 )
    {
        const ObjectStateSpace::StateType *obj_1 = STATE_OBJECT(state1,o_1);
        const ObjectStateSpace::StateType *obj_2 = STATE_OBJECT(state2,o_2);
        return components_[o_1]->distance(obj_1, obj_2);
    }
    else
    {
        // move end effector to target object
        const ob::SE2StateSpace::StateType *end_1   = STATE_OBJECT(state1,o_1);
        const ObjectStateSpace::StateType *obj_prev = STATE_OBJECT(state1,o_2);
        const ObjectStateSpace::StateType *obj_post = STATE_OBJECT(state2,o_2);

        double dist = COST_SWITCH_OBJECT;

//cout << "dist::";
//cout << dist << " -> ";
        dist += components_[o_1]->distance(end_1, obj_prev);
//cout << dist << " -> ";        
        dist += components_[o_2]->distance(obj_prev, obj_post);
//cout << dist << endl;        
        return dist;
    }
}

bool 
RobotObjectSetup::isCollide( 
  double x_i, double y_i, double yaw_i, const Object &obj_i,
  double x_j, double y_j, double yaw_j, const Object &obj_j  ) const
{
    CollisionSimulation sim;
    btVector3 dims_i( obj_i.dims[0]*0.5,
                      obj_i.dims[1]*0.5,
                      obj_i.dims[2]*0.5  );
    btVector3 dims_j( obj_j.dims[0]*0.5,
                      obj_j.dims[1]*0.5,
                      obj_j.dims[2]*0.5  );

    btQuaternion q_i;
    q_i.setEulerZYX(yaw_i,0,0);
    q_i = q_i * obj_i.q_offset;
    btQuaternion q_j;
    q_j.setEulerZYX(yaw_j,0,0);
    q_j = q_j * obj_j.q_offset;

    sim.AddBox(btVector3(x_i,y_i,0),q_i,dims_i);
    sim.AddBox(btVector3(x_j,y_j,0),q_j,dims_j);
    return sim.Test();
}

bool 
RobotObjectSetup
::SingleForAllStateValidityChecker
::isValid( const ob::State *state ) const
{
/*
cout << "idx_target=" << idx_target_ << endl;
cout << "idx_obs=[";
for( int j=0; j<idxes_obs_.size(); j++ )
{
    cout << idxes_obs_[j] << ",";
}
cout << "]" << endl;
cout << "state_all=";
ross_.getAllForAllSpaceInformation()->printState(state_all_);
cout << endl;
*/
    double x   = state->as<ObjectStateSpace::StateType>()->getX();
    double y   = state->as<ObjectStateSpace::StateType>()->getY();
    double yaw = state->as<ObjectStateSpace::StateType>()->getYaw();
    const RobotObjectSetup::Object &obj = objects_[idx_target_-1];
/*
cout << "target obj pose: " << x << ", " << y << ", " << yaw << endl;
*/
    if( ross_.isValid(x,y,yaw)==false ) return false;

    for( int i=1; i<=n_objs_; i++ )
    {
        if( i != idx_target_ )
        {
            bool is_obs = false;
            for( int j=0; j<idxes_obs_.size(); j++ )
            {
                if( i == idxes_obs_[j] )
                {
                    is_obs = true;
                    break;
                }
            }
            if( is_obs==false ) continue;

            double x_i =   state_all_->as<ObjectStateSpace::StateType>(i)->getX();
            double y_i =   state_all_->as<ObjectStateSpace::StateType>(i)->getY();
            double yaw_i = state_all_->as<ObjectStateSpace::StateType>(i)->getYaw();
            const RobotObjectSetup::Object &obj_i = objects_[i-1];

            double d = sqrt((x-x_i)*(x-x_i)+(y-y_i)*(y-y_i));
            if( d < ( objects_[i-1          ].radius + 
                      objects_[idx_target_-1].radius   ) )
            {
                if( ross_.isCollide( x,   y,   yaw,   obj,
                                     x_i, y_i, yaw_i, obj_i ) ) return false;
            }
        }
    }
    return true;
}

RobotObjectSetup
::AllForAllStateValidityChecker
::AllForAllStateValidityChecker( ob::SpaceInformation *si,
                                 RobotObjectSetup &ross )
 : ob::StateValidityChecker(si),
   ross_(ross),
   objects_(ross.objects_)
{        
    n_objs = objects_.size();
}

bool 
RobotObjectSetup
::AllForAllStateValidityChecker
::isValid(const ob::State *state) const
{
    const ob::CompoundState *cs
     = state->as<ob::CompoundStateSpace::StateType>();

    for( int i=1; i<=n_objs; i++ )
    {
        double x_i = cs->as<ob::SE2StateSpace::StateType>(i)->getX();
        double y_i = cs->as<ob::SE2StateSpace::StateType>(i)->getY();
        double yaw_i = cs->as<ob::SE2StateSpace::StateType>(i)->getYaw();

        if( ross_.isValid(x_i,y_i,yaw_i)==false ) return false;
    }

    for( int i=1; i<n_objs; i++ )
    {
        double x_i = cs->as<ob::SE2StateSpace::StateType>(i)->getX();
        double y_i = cs->as<ob::SE2StateSpace::StateType>(i)->getY();
        double yaw_i = cs->as<ob::SE2StateSpace::StateType>(i)->getYaw();

        for( int j=i+1; j<=n_objs; j++ )
        {
            double x_j = cs->as<ob::SE2StateSpace::StateType>(j)->getX();
            double y_j = cs->as<ob::SE2StateSpace::StateType>(j)->getY();
            double yaw_j = cs->as<ob::SE2StateSpace::StateType>(j)->getYaw();

            double d = sqrt((x_i-x_j)*(x_i-x_j)+(y_i-y_j)*(y_i-y_j));
            if( d < (objects_[i-1].radius + 
                     objects_[j-1].radius   ) )
            {
                if( ross_.isCollide( x_i,y_i,yaw_i, objects_[i-1],
                                     x_j,y_j,yaw_j, objects_[j-1] )) return false;
            }
        }
    }
    return true;
}

RobotObjectSetup
::AllForAllStateSampler
::AllForAllStateSampler( ob::SpaceInformation *si,
                         RobotObjectSetup &ross )
 : ob::StateSampler(&*si->getStateSpace()),
   ross_(ross),
   objects_(ross.objects_)
{        
    n_objs = objects_.size();
    samplers_.resize(n_objs);        
    for( int o=1; o<=n_objs; o++ )
    {
        samplers_[o-1] = si->getStateSpace()
                           ->as<ob::CompoundStateSpace>()
                           ->as<ob::SE2StateSpace>(o)->allocDefaultStateSampler();
    }
}

void
RobotObjectSetup
::AllForAllStateSampler
::sampleUniform(ob::State* state)
{
    bool valid = true;
    STATE_ROBOT(state) = -1;        

    for( int i=1; i<=n_objs; i++ )
    {            
        do
        {
            ob::SE2StateSpace::StateType* sample_i = STATE_OBJECT(state,i);
            samplers_[i-1]->sampleUniform(sample_i);

            double x_i   = sample_i->getX();
            double y_i   = sample_i->getY();
            double yaw_i = sample_i->getYaw();

            valid = ross_.isValid(x_i,y_i,yaw_i);
            if( valid==false ) continue;
            
            for( int j=1; j<i; j++ )
            {
                ob::SE2StateSpace::StateType* sample_j = STATE_OBJECT(state,j);

                double x_j   = sample_j->getX();
                double y_j   = sample_j->getY();
                double yaw_j = sample_j->getYaw();

                double d = sqrt((x_i-x_j)*(x_i-x_j)+(y_i-y_j)*(y_i-y_j));
                if( d <= (objects_[i-1].radius + 
                          objects_[j-1].radius   ))
                {
                    if( ross_.isCollide(x_i,y_i,yaw_i,objects_[i-1],
                                        x_j,y_j,yaw_j,objects_[j-1] ) )
                    {
                        valid = false;
                        break;
                    }
                }
            }
        } while( valid == false );
    }
}

bool RobotObjectSetup::validateSingleForAll( const ObjectState* state,
                                             int idx_target, const vector<int> &idxes_obs,
                                             ObjectState* state_res )
{
    ParamSingleForAll param_org = getParamSingleForAll();

    ParamSingleForAll param = param_org;
    param.idx_target = idx_target;
    param.idxes_obs  = idxes_obs;
    setParamSingleForAll(param);

    ObjectState* state_tmp = si_single4all_->allocState()->as<ObjectState>();
    si_single4all_->copyState(state_tmp,state);

    bool res = false;
    double x   = state->getX();
    double y   = state->getY();
    double yaw = state->getYaw();
    for( int i=0; i<states_around_zero_.size(); i++ )
    {
        state_tmp->setX(   x   + states_around_zero_[i]->getX()   );
        state_tmp->setY(   y   + states_around_zero_[i]->getY()   );
        state_tmp->setYaw( yaw + states_around_zero_[i]->getYaw() );

        if( si_single4all_->isValid(state_tmp) )
        {
            si_single4all_->copyState(state_res, state_tmp);
            res = true;
            break;
        }
    }

    si_single4all_->freeState(state_tmp);
    setParamSingleForAll(param_org);

    return res;
}

std::ostream& operator<< (std::ostream& os, const RobotObjectSetup::Object &object)
{
    int len_name = object.name.length();
    os.write(reinterpret_cast<const char*>(&len_name),sizeof(len_name));
    os.write(reinterpret_cast<const char*>(object.name.c_str()),sizeof(len_name));

    int n_dims = object.dims.size();
    os.write(reinterpret_cast<const char*>(&n_dims),sizeof(n_dims));
    for( int i=0; i<n_dims; i++ )
    {
        double val = object.dims[i];
        os.write(reinterpret_cast<const char*>(&val),sizeof(val));
    }

    os.write(reinterpret_cast<const char*>(&object.radius),sizeof(object.radius));

    os.write(reinterpret_cast<const char*>(&object.z_offset),sizeof(object.z_offset));

    double qx = object.q_offset.x();
    double qy = object.q_offset.y();
    double qz = object.q_offset.z();
    double qw = object.q_offset.w();

    os.write(reinterpret_cast<const char*>(&qx),sizeof(qx));
    os.write(reinterpret_cast<const char*>(&qy),sizeof(qy));
    os.write(reinterpret_cast<const char*>(&qz),sizeof(qz));
    os.write(reinterpret_cast<const char*>(&qw),sizeof(qw));

    os.write(reinterpret_cast<const char*>(&object.mass),sizeof(object.mass));
}

std::istream& operator>> (std::istream& is, RobotObjectSetup::Object &object)
{
    int len_name;
    char name[256];
    is.read(reinterpret_cast<char*>(&len_name),sizeof(len_name));    
    is.read(reinterpret_cast<char*>(name),sizeof(len_name));
    object.name = name;

    int n_dims;
    is.read(reinterpret_cast<char*>(&n_dims),sizeof(n_dims));
    object.dims.resize(n_dims);
    for( int i=0; i<n_dims; i++ )
    {
        double val;
        is.read(reinterpret_cast<char*>(&val),sizeof(val));
        object.dims[i] = val;
    }

    is.read(reinterpret_cast<char*>(&object.radius),sizeof(object.radius));

    is.read(reinterpret_cast<char*>(&object.z_offset),sizeof(object.z_offset));

    double qx, qy, qz, qw;
    is.read(reinterpret_cast<char*>(&qx),sizeof(qx));
    is.read(reinterpret_cast<char*>(&qy),sizeof(qy));
    is.read(reinterpret_cast<char*>(&qz),sizeof(qz));
    is.read(reinterpret_cast<char*>(&qw),sizeof(qw));
    object.q_offset = btQuaternion(qx,qy,qz,qw);

    is.read(reinterpret_cast<char*>(&object.mass),sizeof(object.mass));

    object.shape = new btBoxShape(btVector3(object.dims[0]*0.5, 
                                            object.dims[1]*0.5, 
                                            object.dims[2]*0.5));
}
