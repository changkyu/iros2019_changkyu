#ifndef CHANGKYU_STATESPACE__HPP
#define CHANGKYU_STATESPACE__HPP

#include <Eigen/Dense>

#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/geometric/PathGeometric.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <btBulletCollisionCommon.h>

#define STATE_ROBOT(x) ((x)->as<ompl::base::CompoundStateSpace::StateType>()\
                           ->as<RobotStateSpace::StateType>(0)\
                           ->values[0])

#define STATE_OBJECT(x,o) ((x)->as<ompl::base::CompoundStateSpace::StateType>()\
                              ->as<ObjectStateSpace::StateType>(o))

#define STATE_OBJREL_0(x) ((x)->as<ompl::base::CompoundStateSpace::StateType>()\
                               ->as<ompl::base::SO2StateSpace::StateType>(0))

#define STATE_OBJREL_1(x) ((x)->as<ompl::base::CompoundStateSpace::StateType>()\
                               ->as<ObjectStateSpace::StateType>(1))

inline double distance_angle(double b, double a)
{
    double alpha = a * 180.0 / M_PI;
    double beta  = b * 180.0 / M_PI;
    double d = (int)(beta - alpha) % 360;
    double res;
    if( d > 180 ) res = 360 - d;
    else if( d < -180 ) res = 360 + d;
    else res = d;

    return res * M_PI / 180.0;    
}

class RobotStateSpace : public ompl::base::RealVectorStateSpace
{
public:
    RobotStateSpace(int n_objs) : ompl::base::RealVectorStateSpace(1)
    {
        setBounds(-1,n_objs);
    }
    ~RobotStateSpace(){}
};

typedef RobotStateSpace::StateType RobotState;

class ObjectStateSpace : public ompl::base::SE2StateSpace
{
public:
    ObjectStateSpace() : ompl::base::SE2StateSpace()
    {

    }
    ObjectStateSpace(const ompl::base::RealVectorBounds &bounds)
     : ompl::base::SE2StateSpace()
    {
        setBounds(bounds);
    }

    ~ObjectStateSpace(){}
};

typedef ObjectStateSpace::StateType ObjectState;

class ObjectRelativeStateSpace : public ompl::base::CompoundStateSpace
{
public:
    ObjectRelativeStateSpace(const ompl::base::RealVectorBounds &bounds)
     : ompl::base::CompoundStateSpace()
    {
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace),1.0);
        addSubspace(ompl::base::StateSpacePtr(new ObjectStateSpace(bounds)),1.0);
    }

    ~ObjectRelativeStateSpace(){}
};

typedef ObjectRelativeStateSpace::StateType ObjectRelativeState;

class RobotObjectStateSpace : public ompl::base::CompoundStateSpace
{
public:
    RobotObjectStateSpace(int n)
     : ompl::base::CompoundStateSpace(),
       COST_SWITCH_OBJECT(1)
    {
        addSubspace(ompl::base::StateSpacePtr(new RobotStateSpace(n)), 0.0);
        for( int o=1; o<=n; o++ )
        {
            addSubspace(ompl::base::StateSpacePtr(new ObjectStateSpace), 1.0);
        }
    }

    RobotObjectStateSpace(int n, const ompl::base::RealVectorBounds &bounds)
     : ompl::base::CompoundStateSpace(),
       COST_SWITCH_OBJECT(1)
    {   
        addSubspace(ompl::base::StateSpacePtr(new RobotStateSpace(n)), 0.0);
        for( int o=1; o<=n; o++ )
        {
            addSubspace(ompl::base::StateSpacePtr(new ObjectStateSpace(bounds)), 1.0);
        }
    }

    RobotObjectStateSpace(int n, const std::vector<ompl::base::RealVectorBounds> &bounds)
     : ompl::base::CompoundStateSpace(),
       COST_SWITCH_OBJECT(1)
    {   
        addSubspace(ompl::base::StateSpacePtr(new RobotStateSpace(n)), 0.0);
        for( int o=1; o<=n; o++ )
        {
            addSubspace(ompl::base::StateSpacePtr(new ObjectStateSpace(bounds[o-1])), 1.0);
        }
    }

    ~RobotObjectStateSpace()
    {

    }

    virtual  double distance( const ompl::base::State *state1,
                              const ompl::base::State *state2 ) const;

private:
    const double COST_SWITCH_OBJECT;    
};

inline bool CompareStateZero(ObjectState* a, ObjectState* b, const ObjectState* s0, ompl::base::StateSpacePtr ss)
{
    double da = ss->distance(s0,a);
    double db = ss->distance(s0,b);
    return da < db;
}

typedef RobotObjectStateSpace::StateType RobotObjectState;

class RobotObjectSetup
{
public:
    struct Object
    {
        std::string name;
        btConvexShape* shape;        
        std::vector<double> dims;
        double radius;
        double z_offset;
        btQuaternion q_offset;
        double mass;
    };

    struct ParamSingleForAll
    {
        int idx_target;
        std::vector<int> idxes_obs;
        RobotObjectStateSpace::StateType* state_all;
    };

    struct ParamSingleForClear
    {        
        int idx_target;
        std::vector<std::pair<int,ObjectState*> > obstacles;
    };

    RobotObjectSetup( const std::vector<Object> &objects )
     : bounds_(2),
       n_objs_(objects.size())
    {
        objects_ = objects;
    }

    RobotObjectSetup( const std::vector<Object> &objects, 
                      double x_low, double x_high, 
                      double y_low, double y_high   )
     : bounds_(2),
       n_objs_(objects.size())
    {
        objects_ = objects;

        bounds_.setLow( 0,x_low );
        bounds_.setHigh(0,x_high);
        bounds_.setLow( 1,y_low );
        bounds_.setHigh(1,y_high);

        si_single_  = ompl::base::SpaceInformationPtr(allocSingleSpaceInformation());
        si_single4all_ = ompl::base::SpaceInformationPtr(allocSingleForAllSpaceInformation());
        si_single4clear_ = ompl::base::SpaceInformationPtr(allocSingleForClearSpaceInformation());
        si_all4all_ = ompl::base::SpaceInformationPtr(allocAllForAllSpaceInformation());
    }
    ~RobotObjectSetup()
    {
        for( int i=0; i<param_single4clear_.obstacles.size(); i++ )
        {
            si_single4clear_->freeState(param_single4clear_.obstacles[i].second);
        }
        param_single4clear_.obstacles.clear();

        for( int i=0; i<states_around_zero_.size(); i++ )
        {
            si_single_->freeState(states_around_zero_[i]);
        }
    }

    void setBounds( double x_low, double x_high, 
                    double y_low, double y_high  )
    {
        bounds_.setLow( 0,x_low );
        bounds_.setHigh(0,x_high);
        bounds_.setLow( 1,y_low );
        bounds_.setHigh(1,y_high);

        si_single_  = ompl::base::SpaceInformationPtr(allocSingleSpaceInformation());
        si_single4all_ = ompl::base::SpaceInformationPtr(allocSingleForAllSpaceInformation());
        si_single4clear_ = ompl::base::SpaceInformationPtr(allocSingleForClearSpaceInformation());
        si_all4all_ = ompl::base::SpaceInformationPtr(allocAllForAllSpaceInformation());
    }

    void setup()
    {
        ompl::base::StateSpacePtr ss(new ompl::base::SE2StateSpace);

        ompl::base::RealVectorBounds bounds(2);
        bounds.setLow( 0,-0.025);
        bounds.setHigh(0, 0.025);
        bounds.setLow( 1,-0.025);
        bounds.setHigh(1, 0.025);
        ss->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

        ompl::base::StateSamplerPtr ss_single = ss->allocStateSampler();
        ObjectState* state_zero = ss->allocState()->as<ObjectState>();
        state_zero->setX(0);
        state_zero->setY(0);
        state_zero->setYaw(0);

        for( int i=0; i<10000; i++ )
        {
            ObjectState* state = ss->allocState()->as<ObjectState>();
            ss_single->sampleUniform(state);
            states_around_zero_.push_back(state);
        }
        std::sort( states_around_zero_.begin(), 
                   states_around_zero_.end(), 
                   std::bind( CompareStateZero, 
                              std::placeholders::_1, std::placeholders::_2, 
                              state_zero, ss ));

        ss->freeState(state_zero);
    }

    virtual bool isValid(double x, double y, double yaw) = 0;
    virtual void visualizeSetup(cv::Mat &img) = 0;
    virtual void getGoalState(int o, double* x, double* y, double* yaw) = 0;

    int getObjectCount(){ return n_objs_; }

    std::vector<Object>& getObjects(){ return objects_; }

    ompl::base::State* allocState()
    {
        return si_all4all_->allocState();
    }

    void freeState(ompl::base::State* state)
    {
        si_all4all_->freeState(state);
    }

    ompl::base::SpaceInformationPtr getSingleSpaceInformation()
    {
        return si_single_;
    }

    ompl::base::SpaceInformationPtr getSingleForAllSpaceInformation()
    {
        return si_single4all_;
    }

    ompl::base::SpaceInformationPtr getSingleForClearSpaceInformation()
    {
        return si_single4clear_;
    }

    ompl::base::SpaceInformationPtr getAllForAllSpaceInformation()
    {
        return si_all4all_;
    }

    ParamSingleForAll getParamSingleForAll()
    { 
        return param_single4all_; 
    }

    ParamSingleForClear getParamSingleForClear()
    { 
        return param_single4clear_; 
    }

    void setParamSingleForAll( const ParamSingleForAll &param )
    {
        param_single4all_ = param;
    }

    void setParamSingleForAll( int idx_target, 
                               std::vector<int> &idxes_obs, 
                               ompl::base::State *state_all )
    {
        ParamSingleForAll param;        
        param.idx_target = idx_target;
        param.idxes_obs  = idxes_obs;
        param.state_all  = state_all->as<RobotObjectStateSpace::StateType>();
        setParamSingleForAll(param);
    }

    void setParamSingleForClear( const ParamSingleForClear &param )
    {
        param_single4clear_ = param;
    }

    void setParamSingleForClear( int idx_target,
                                 int idx_obs,
                                 const ompl::geometric::PathGeometric &path_obs,
                                 const std::vector<std::pair<int,ObjectState*> > &obstacles )
    {        
        for( int i=0; i<param_single4clear_.obstacles.size(); i++ )
        {
            si_single4clear_->freeState(param_single4clear_.obstacles[i].second);            
        }
        param_single4clear_.obstacles.clear();

        ParamSingleForClear param;
        param.idx_target = idx_target;
        for( int i=0; i<obstacles.size(); i++ )
        {
            ObjectState* state = si_single4clear_->allocState()->as<ObjectState>();
            si_single4clear_->copyState(state,obstacles[i].second);
            param.obstacles.push_back(std::make_pair(obstacles[i].first,state));
        }

        ObjectState* state_path0 = si_single4clear_->allocState()->as<ObjectState>();
        si_single4clear_->copyState(state_path0,path_obs.getState(0));
        param.obstacles.push_back(std::make_pair(idx_obs,state_path0));
        for( int p=1; p<path_obs.getStateCount(); p++ )
        {
            const ObjectState* state_prev = path_obs.getState(p-1)->as<ObjectState>();
            ObjectState* state_curr = si_single4clear_->allocState()->as<ObjectState>();
            si_single4clear_->copyState(state_curr,path_obs.getState(p));

            double dist = sqrt( (state_curr->getX()-state_prev->getX())*
                                (state_curr->getX()-state_prev->getX())+
                                (state_curr->getY()-state_prev->getY())*
                                (state_curr->getY()-state_prev->getY())  );

            int n_middle = ((int)(dist / 0.05)) + 1;
            for( int m=1; m<n_middle; m++ )
            {
                ObjectState* state_midd = si_single4clear_->allocState()->as<ObjectState>();
                si_single4clear_->getStateSpace()->interpolate( 
                        state_prev,state_curr, m/(double)n_middle, state_midd);
                param.obstacles.push_back(std::make_pair(idx_obs,state_midd));
            }
            param.obstacles.push_back(std::make_pair(idx_obs,state_curr));
        }

        setParamSingleForClear(param);
    }

    bool validateSingleForAll( const ObjectState* state,
                               int idx_target, const std::vector<int> &idxes_obs,
                               ObjectState* state_res );

    std::vector<double> &getWorkspace(){ return workspace_; }

protected:

    ompl::base::StateSamplerPtr 
    allocSingleStateSampler(const ompl::base::StateSpace* space, 
                                  ompl::base::SpaceInformation *si2 )
    {
        return ompl::base::StateSamplerPtr(new SingleStateSampler(si2,*this));
    }

    ompl::base::StateSamplerPtr 
    allocSingleForAllStateSampler(const ompl::base::StateSpace* space, 
                                        ompl::base::SpaceInformation *si2 )
    {
        return ompl::base::StateSamplerPtr(new SingleForAllStateSampler(si2));
    }

    ompl::base::StateSamplerPtr 
    allocSingleForClearStateSampler(const ompl::base::StateSpace* space, 
                                          ompl::base::SpaceInformation *si2 )
    {
        return ompl::base::StateSamplerPtr(new SingleForClearStateSampler(si2));
    }

    ompl::base::StateSamplerPtr 
    allocAllForAllStateSampler(const ompl::base::StateSpace* space, 
                                     ompl::base::SpaceInformation *si2 )
    {
        return ompl::base::StateSamplerPtr(new AllForAllStateSampler(si2,*this));
    }

    ompl::base::SpaceInformation* allocSingleSpaceInformation()
    {
        ompl::base::StateSpacePtr space(new ObjectStateSpace(bounds_));
        space->setValidSegmentCountFactor(10);

        ompl::base::SpaceInformation* si
         = new ompl::base::SpaceInformation(space);
        
        ompl::base::StateValidityCheckerPtr validitychecker(
            new SingleStateValidityChecker(si,*this));
        si->setStateValidityChecker(validitychecker);

        ompl::base::MotionValidatorPtr motionchecker(
            new ompl::base::DiscreteMotionValidator(si));
        si->setMotionValidator(motionchecker);

        ompl::base::StateSamplerAllocator fn
         = std::bind(&RobotObjectSetup::allocSingleStateSampler, this, std::placeholders::_1, si);        
        space->setStateSamplerAllocator(fn);

        return si;
    }

    ompl::base::SpaceInformation* allocSingleForAllSpaceInformation()
    {
        ompl::base::StateSpacePtr space(new ObjectStateSpace(bounds_));
        space->setValidSegmentCountFactor(10);

        ompl::base::SpaceInformation* si
         = new ompl::base::SpaceInformation(space);
        
        ompl::base::StateValidityCheckerPtr validitychecker(
            new SingleForAllStateValidityChecker(si,*this));
        si->setStateValidityChecker(validitychecker);

        ompl::base::MotionValidatorPtr motionchecker(
            new ompl::base::DiscreteMotionValidator(si));
        si->setMotionValidator(motionchecker);

        ompl::base::StateSamplerAllocator fn
         = std::bind(&RobotObjectSetup::allocSingleForAllStateSampler, this, std::placeholders::_1, si);
        space->setStateSamplerAllocator(fn);

        return si;
    }

    ompl::base::SpaceInformation* allocSingleForClearSpaceInformation()
    {
        ompl::base::StateSpacePtr space(new ObjectStateSpace(bounds_));
        space->setValidSegmentCountFactor(10);

        ompl::base::SpaceInformation* si
         = new ompl::base::SpaceInformation(space);
        
        ompl::base::StateValidityCheckerPtr validitychecker(
            new SingleForClearStateValidityChecker(si,*this));
        si->setStateValidityChecker(validitychecker);

        ompl::base::MotionValidatorPtr motionchecker(
            new ompl::base::DiscreteMotionValidator(si));
        si->setMotionValidator(motionchecker);

        ompl::base::StateSamplerAllocator fn
         = std::bind(&RobotObjectSetup::allocSingleForClearStateSampler, this, std::placeholders::_1, si);
        space->setStateSamplerAllocator(fn);

        return si;
    }

    ompl::base::SpaceInformation* allocAllForAllSpaceInformation()
    {
        ompl::base::StateSpacePtr space(new RobotObjectStateSpace(objects_.size(),bounds_));
        space->setValidSegmentCountFactor(10);

        ompl::base::SpaceInformation* si
         = new ompl::base::SpaceInformation(space);
        
        ompl::base::StateValidityCheckerPtr validitychecker(
            new AllForAllStateValidityChecker(si,*this));
        si->setStateValidityChecker(validitychecker);

        ompl::base::MotionValidatorPtr motionchecker(
            new ompl::base::DiscreteMotionValidator(si));
        si->setMotionValidator(motionchecker);

        ompl::base::StateSamplerAllocator fn            
         = std::bind(&RobotObjectSetup::allocAllForAllStateSampler, this, std::placeholders::_1, si);
        space->setStateSamplerAllocator(fn);

        return si;
    }

    class SingleStateValidityChecker : public ompl::base::StateValidityChecker
    {
    public:
        SingleStateValidityChecker( ompl::base::SpaceInformation *si,
                                    RobotObjectSetup &ross )
         : ompl::base::StateValidityChecker(si),
           ross_(ross)
        {

        }        
        ~SingleStateValidityChecker(){}

        virtual bool isValid(const ompl::base::State *state) const
        {
            double x   = state->as<ObjectStateSpace::StateType>()->getX();
            double y   = state->as<ObjectStateSpace::StateType>()->getY();
            double yaw = state->as<ObjectStateSpace::StateType>()->getYaw();
            return ross_.isValid(x,y,yaw);
        }

    private:
        RobotObjectSetup &ross_;        
    };

    class SingleStateSampler : public ompl::base::StateSampler
    {
    public:
        SingleStateSampler( ompl::base::SpaceInformation* si,                                  
                            RobotObjectSetup &ross       )
         : ompl::base::StateSampler(&*si->getStateSpace())
        {
            sampler_ = si->getStateSpace()->allocDefaultStateSampler();
            svc_ = si->getStateValidityChecker();
        }
        ~SingleStateSampler(){}

        virtual void sampleUniform( ompl::base::State* state)
        {
            do
            {
                sampler_->sampleUniform(state);
            } while ( svc_->isValid(state)==false );
        }
        virtual void sampleUniformNear( ompl::base::State* state, 
                                        const ompl::base::State* near, 
                                        double distance )
        {
            do
            {
                sampler_->sampleUniformNear(state, near, distance);
            } while ( svc_->isValid(state)==false );
        }
        virtual void sampleGaussian( ompl::base::State* state, 
                                         const ompl::base::State* near, 
                                         double distance )
        {
            throw ompl::Exception("SingleStateSampler::sampleGaussian", "not implemented");
        }

    private:
        ompl::base::StateValidityCheckerPtr svc_;
        ompl::base::StateSamplerPtr sampler_;
    };

    class SingleForAllStateValidityChecker : public ompl::base::StateValidityChecker
    {
    public:
        SingleForAllStateValidityChecker( ompl::base::SpaceInformation *si,
                                          RobotObjectSetup &ross )
         : ompl::base::StateValidityChecker(si),
           ross_(ross),
           objects_(ross.objects_),
           idx_target_(ross.param_single4all_.idx_target),
           idxes_obs_(ross.param_single4all_.idxes_obs),
           state_all_(ross.param_single4all_.state_all)
        {
            n_objs_ = objects_.size();            
        }

        ~SingleForAllStateValidityChecker(){}

        virtual bool isValid(const ompl::base::State *state) const;

    private:
        int n_objs_;
        int &idx_target_;
        std::vector<int> &idxes_obs_;
        RobotObjectStateSpace::StateType* &state_all_;
        RobotObjectSetup &ross_;
        std::vector<Object> &objects_;
    };

    class SingleForAllStateSampler : public ompl::base::StateSampler
    {
    public:
        SingleForAllStateSampler( ompl::base::SpaceInformation* si )
         : ompl::base::StateSampler(&*si->getStateSpace())
        {
            sampler_ = si->getStateSpace()->allocDefaultStateSampler();
            svc_ = si->getStateValidityChecker();
        }
        ~SingleForAllStateSampler(){}

        virtual void sampleUniform( ompl::base::State* state)
        {
            do
            {
                sampler_->sampleUniform(state);
            } while ( svc_->isValid(state)==false );
        }
        virtual void sampleUniformNear( ompl::base::State* state, 
                                        const ompl::base::State* near, 
                                        double distance )
        {
            do
            {
                sampler_->sampleUniformNear(state, near, distance);
            } while ( svc_->isValid(state)==false );
            //throw ompl::Exception("SingleForAllStateSampler::sampleUniformNear", "not implemented");
        }
        virtual void sampleGaussian( ompl::base::State* state, 
                                     const ompl::base::State* near, 
                                     double distance )
        {
            throw ompl::Exception("SingleForAllStateSampler::sampleGaussian", "not implemented");
        }

    private:
        ompl::base::StateValidityCheckerPtr svc_;
        ompl::base::StateSamplerPtr sampler_;
    };

    class SingleForClearStateValidityChecker : public ompl::base::StateValidityChecker
    {
    public:
        SingleForClearStateValidityChecker( ompl::base::SpaceInformation *si,
                                            RobotObjectSetup &ross )
         : ompl::base::StateValidityChecker(si),
           ross_(ross),
           objects_(ross.objects_),
           idx_target_(ross.param_single4clear_.idx_target),
           obstacles_(ross.param_single4clear_.obstacles)
        {
            n_objs_ = objects_.size();            
        }

        ~SingleForClearStateValidityChecker(){}

        virtual bool isValid(const ompl::base::State *state) const
        {
            double x   = state->as<ObjectState>()->getX();
            double y   = state->as<ObjectState>()->getY();
            double yaw = state->as<ObjectState>()->getYaw();
            const RobotObjectSetup::Object &obj = objects_[idx_target_-1];

            if( ross_.isValid(x,y,yaw)==false ) return false;

            for( int o=0; o<obstacles_.size(); o++ )
            {
                int i = obstacles_[o].first;
                if( i==idx_target_ ) continue;

                ObjectState* state_i = obstacles_[o].second;

                double x_i =   state_i->getX();
                double y_i =   state_i->getY();
                double yaw_i = state_i->getYaw();
                const RobotObjectSetup::Object &obj_i = objects_[i-1];

                double d = sqrt((x-x_i)*(x-x_i)+(y-y_i)*(y-y_i));
                if( d < ( objects_[i-1          ].radius + 
                          objects_[idx_target_-1].radius   ) )
                {
                    if( ross_.isCollide( x,   y,   yaw,   obj,
                                         x_i, y_i, yaw_i, obj_i ) ) return false;
                }
            }
            return true;
        }

    private:
        int n_objs_;
        int &idx_target_;
        std::vector<std::pair<int,ObjectState*> > &obstacles_;
        RobotObjectSetup &ross_;
        std::vector<Object> &objects_;
    };

    class SingleForClearStateSampler : public ompl::base::StateSampler
    {
    public:
        SingleForClearStateSampler( ompl::base::SpaceInformation* si )
         : ompl::base::StateSampler(&*si->getStateSpace())
        {
            sampler_ = si->getStateSpace()->allocDefaultStateSampler();
            svc_ = si->getStateValidityChecker();
        }
        ~SingleForClearStateSampler(){}

        virtual void sampleUniform( ompl::base::State* state)
        {
            do
            {
                sampler_->sampleUniform(state);
            } while ( svc_->isValid(state)==false );
        }
        virtual void sampleUniformNear( ompl::base::State* state, 
                                        const ompl::base::State* near, 
                                        double distance )
        {
            do
            {
                sampler_->sampleUniformNear(state, near, distance);
            } while ( svc_->isValid(state)==false );
        }
        virtual void sampleGaussian( ompl::base::State* state, 
                                     const ompl::base::State* near, 
                                     double distance )
        {
            throw ompl::Exception("SingleForClearStateSampler::sampleGaussian", "not implemented");
        }

    private:
        ompl::base::StateValidityCheckerPtr svc_;
        ompl::base::StateSamplerPtr sampler_;
    };

    class AllForAllStateValidityChecker : public ompl::base::StateValidityChecker
    {
    public:
        AllForAllStateValidityChecker( ompl::base::SpaceInformation *si,
                                       RobotObjectSetup &ross );
        virtual bool isValid(const ompl::base::State *state) const;

    private:
        int n_objs;
        RobotObjectSetup &ross_;
        std::vector<Object> &objects_;
    };

    class AllForAllStateSampler : public ompl::base::StateSampler
    {
    public:
        AllForAllStateSampler( ompl::base::SpaceInformation *si,
                               RobotObjectSetup &ross      );

        virtual void sampleUniform( ompl::base::State* state);
        virtual void sampleUniformNear( ompl::base::State* state, 
                                        const ompl::base::State* near, 
                                        double distance )
        {            
            throw ompl::Exception("AllForAllStateSampler::sampleUniformNear", "not implemented");
        }
        virtual void sampleGaussian( ompl::base::State* state, 
                                     const ompl::base::State* near, 
                                     double distance )
        {            
            throw ompl::Exception("AllForAllStateSampler::sampleGaussian", "not implemented");
        }

        ~AllForAllStateSampler(){}

    private:
        int n_objs;
        RobotObjectSetup &ross_;
        std::vector<ompl::base::StateSamplerPtr> samplers_;
        std::vector<Object> &objects_;
    };

    bool isCollide( double x_i, double y_i, double yaw_i, const Object &obj_i,
                    double x_j, double y_j, double yaw_j, const Object &obj_j  ) const;

    int n_objs_;
    std::vector<Object> objects_;
    ompl::base::RealVectorBounds bounds_;
    std::vector<double> workspace_;

private:

    ompl::base::SpaceInformationPtr si_single_;
    ompl::base::SpaceInformationPtr si_single4all_;
    ompl::base::SpaceInformationPtr si_single4clear_;
    ompl::base::SpaceInformationPtr si_all4all_;

    ParamSingleForAll   param_single4all_;
    ParamSingleForClear param_single4clear_;

    std::vector<ObjectState*> states_around_zero_;
};

class BoxSetup : public RobotObjectSetup
{
public:        
    BoxSetup(const std::vector<Object> &objects, double size)
     : RobotObjectSetup( objects, -size*0.5, size*0.5, -size*0.5, size*0.5 ),
       size_(size)
    {}
    ~BoxSetup(){}

    bool isValid(double x, double y, double yaw)
    {
        return ((-size_*0.5) <= x) && (x <= (size_*0.5)) &&
               ((-size_*0.5) <= y) && (y <= (size_*0.5));    
    }

    void getGoalState(int o, double* x, double* y, double* yaw)
    {
        int n_cols = size_ / 0.10;
        int r = (o-1) / n_cols;
        int c = (o-1) % n_cols;

        *x = -size_*0.5 + 0.13*(r + 0.5);
        *y = -size_*0.5 + 0.10*(c + 0.5);        
        *yaw = 0;


#if 0 // center
        int idx = o-1;
        if( o==1 )
        {
            *x=0; *y=0; *yaw=0; 
            return;
        }
        else
        {
            o--;
        }

        int lv = 0;
        while( o > 0 )
        {
            lv++;
            o -= lv*2*4;
        }

        int i = lv*2*4 + o -1;
        int g = i / (lv*2);

        if( g==0 )
        {
            int j = i - g*(lv*2);
            *x = objects_[idx].dims[2] * lv;
            *y = objects_[idx].dims[1] * (j-lv+1);
            *yaw = 0;
        }
        else if( g==1 )
        {
            int j = i - g*(lv*2);
            *x = objects_[idx].dims[2] *-(j-lv+1);
            *y = objects_[idx].dims[1] * lv;
            *yaw = 0;
        }
        else if( g==2 )
        {
            int j = i - g*(lv*2);
            *x = objects_[idx].dims[2] *-lv;
            *y = objects_[idx].dims[1] *-(j-lv+1);
            *yaw = 0;
        }
        else
        {
            int j = i - g*(lv*2);
            *x = objects_[idx].dims[2] * (j-lv+1);
            *y = objects_[idx].dims[1] *-lv;
            *yaw = 0;
        }
#endif
    }

    void visualizeSetup(cv::Mat &img)
    {
        cv::line(img,cv::Point(500-500*-size_*0.5,500-500*-size_*0.5),
                     cv::Point(500-500*-size_*0.5,500-500* size_*0.5), cv::Scalar(0,255,0) );
        cv::line(img,cv::Point(500-500*-size_*0.5,500-500* size_*0.5),
                     cv::Point(500-500* size_*0.5,500-500* size_*0.5), cv::Scalar(0,255,0) );
        cv::line(img,cv::Point(500-500* size_*0.5,500-500* size_*0.5),
                     cv::Point(500-500* size_*0.5,500-500*-size_*0.5), cv::Scalar(0,255,0) );
        cv::line(img,cv::Point(500-500* size_*0.5,500-500*-size_*0.5),
                     cv::Point(500-500*-size_*0.5,500-500*-size_*0.5), cv::Scalar(0,255,0) );
    }

private:    
    double size_;
};

class KukaTableSetup : public RobotObjectSetup
{
public:        
    KukaTableSetup(const std::vector<Object> &objects)
     : RobotObjectSetup( objects, 0, 0.70, -0.70, 0.70 )
    {
        std::vector<double> workspace_{0.43, 0.34, -0.095, 0.6123724, 0.6123724, -0.3535534, -0.3535534, 0.44, 0.22, 0.34};

        Eigen::Matrix3d tf_ws_rot
         = Eigen::Quaterniond(workspace_[3],
                              workspace_[4],
                              workspace_[5],
                              workspace_[6]).toRotationMatrix();
        
        Eigen::Vector3d angles = tf_ws_rot.eulerAngles(2,1,0);
        yaw_goal = angles[0];// - 0.5*M_PI;

        Eigen::Matrix4d tf_ws;
        tf_ws << tf_ws_rot(0,0),tf_ws_rot(0,1),tf_ws_rot(0,2),workspace_[0],
                 tf_ws_rot(1,0),tf_ws_rot(1,1),tf_ws_rot(1,2),workspace_[1],
                 tf_ws_rot(2,0),tf_ws_rot(2,1),tf_ws_rot(2,2),workspace_[2],
                              0,             0,             0,           1;

        double x_min = -workspace_[7]*0.5;
        double x_max =  workspace_[7]*0.5;
        double y_min = -workspace_[8]*0.5;
        double y_max =  workspace_[8]*0.5;
        double z_min = -workspace_[9]*0.5;
        double z_max =  workspace_[9]*0.5;

        mx_goals(0,0) = x_max-0.005-0.100*2.5; mx_goals(1,0) = 0; mx_goals(2,0) = -0.070; mx_goals(3,0) = 1;
        mx_goals(0,1) = x_max-0.005-0.100*2.5; mx_goals(1,1) = 0; mx_goals(2,1) =  0.000; mx_goals(3,1) = 1;
        mx_goals(0,2) = x_max-0.005-0.100*2.5; mx_goals(1,2) = 0; mx_goals(2,2) =  0.070; mx_goals(3,2) = 1;
        mx_goals(0,3) = x_max-0.005-0.100*1.5; mx_goals(1,3) = 0; mx_goals(2,3) = -0.070; mx_goals(3,3) = 1;
        mx_goals(0,4) = x_max-0.005-0.100*1.5; mx_goals(1,4) = 0; mx_goals(2,4) =  0.000; mx_goals(3,4) = 1;
        mx_goals(0,5) = x_max-0.005-0.100*1.5; mx_goals(1,5) = 0; mx_goals(2,5) =  0.070; mx_goals(3,5) = 1;
        mx_goals(0,6) = x_max-0.005-0.100*0.5; mx_goals(1,6) = 0; mx_goals(2,6) = -0.070; mx_goals(3,6) = 1;
        mx_goals(0,7) = x_max-0.005-0.100*0.5; mx_goals(1,7) = 0; mx_goals(2,7) =  0.000; mx_goals(3,7) = 1;
        mx_goals(0,8) = x_max-0.005-0.100*0.5; mx_goals(1,8) = 0; mx_goals(2,8) =  0.070; mx_goals(3,8) = 1;

        mx_goals(0,9) = x_max-0.005-0.100*-0.5; mx_goals(1,9) = 0; mx_goals(2,9) =  0.000; mx_goals(3,9) = 1;

        mx_goals = tf_ws * mx_goals;

    }
    ~KukaTableSetup(){}

    bool isValid(double x, double y, double yaw)
    {        
        double dist;
        dist = sqrt(x*x + y*y);
//        if( dist < 0.40 || 0.70 < dist ) return false;
        if( dist < 0.43 || 0.70 < dist ) return false;
        if( x < 0.15 ) return false;

        double dist1;
        dist1 = sqrt( (x-0.43)*(x-0.43) + (y-( 0.34))*(y-( 0.34)) );
        
        double dist2;
        dist2 = sqrt( (x-0.43)*(x-0.43) + (y-(-0.34))*(y-(-0.34)) );
        
        if( dist1 > 0.4 && dist2 > 0.4 ) return false;
        
        return true;
    }

    void getGoalState(int o, double* x, double* y, double* yaw)
    {
        *x = mx_goals(0,o-1);
        *y = mx_goals(1,o-1);
        *yaw = yaw_goal;
    }

    void visualizeSetup(cv::Mat &img)
    {
        cv::circle(img, cv::Point(500,500), 500*0.40, cv::Scalar(0,255,0));
        cv::circle(img, cv::Point(500,500), 500*0.70, cv::Scalar(0,255,0));
    }

private:
    
    double yaw_goal;
    Eigen::Matrix<double,4,10> mx_goals;

};

class KukaBoxSetup : public RobotObjectSetup
{
public:        
    KukaBoxSetup( const std::vector<Object> &objects,
                  const std::vector<double> &workspace )
     : RobotObjectSetup( objects )
    {
        workspace_ = workspace;
//        workspace_[7] += 0.01;
//        workspace_[8] += 0.01;
//        workspace_[9] += 0.01;

        Eigen::Matrix3d tf_ws_rot
         = Eigen::Quaterniond(workspace_[3],
                              workspace_[4],
                              workspace_[5],
                              workspace_[6]).toRotationMatrix();
        
        tf_ws << tf_ws_rot(0,0),tf_ws_rot(0,1),tf_ws_rot(0,2),workspace_[0],
                 tf_ws_rot(1,0),tf_ws_rot(1,1),tf_ws_rot(1,2),workspace_[1],
                 tf_ws_rot(2,0),tf_ws_rot(2,1),tf_ws_rot(2,2),workspace_[2],
                              0,             0,             0,           1;

        Eigen::Vector3d angles = tf_ws_rot.eulerAngles(2,1,0);
        yaw_goal = angles[0];// - 0.5*M_PI;

        tf_ws_inv = tf_ws.inverse();

        x_min = -workspace_[7]*0.5;
        x_max =  workspace_[7]*0.5;
        y_min = -workspace_[8]*0.5;
        y_max =  workspace_[8]*0.5;
        z_min = -workspace_[9]*0.5;
        z_max =  workspace_[9]*0.5;

        pts.resize(4);
        pts[0].x = tf_ws(0,0)*x_min + tf_ws(0,2)*z_min + tf_ws(0,3);
        pts[1].x = tf_ws(0,0)*x_min + tf_ws(0,2)*z_max + tf_ws(0,3);
        pts[2].x = tf_ws(0,0)*x_max + tf_ws(0,2)*z_max + tf_ws(0,3);
        pts[3].x = tf_ws(0,0)*x_max + tf_ws(0,2)*z_min + tf_ws(0,3);
        pts[0].y = tf_ws(1,0)*x_min + tf_ws(1,2)*z_min + tf_ws(1,3);
        pts[1].y = tf_ws(1,0)*x_min + tf_ws(1,2)*z_max + tf_ws(1,3);
        pts[2].y = tf_ws(1,0)*x_max + tf_ws(1,2)*z_max + tf_ws(1,3);
        pts[3].y = tf_ws(1,0)*x_max + tf_ws(1,2)*z_min + tf_ws(1,3);

        double xx_min =  INFINITY;
        double yy_min =  INFINITY;
        double xx_max = -INFINITY;
        double yy_max = -INFINITY;
        for( int p=0; p<pts.size(); p++ )
        {
            if( xx_min > pts[p].x ) xx_min = pts[p].x;
            if( yy_min > pts[p].y ) yy_min = pts[p].y;
            if( xx_max < pts[p].x ) xx_max = pts[p].x;
            if( yy_max < pts[p].y ) yy_max = pts[p].y;
        }        
        setBounds(xx_min,xx_max,yy_min,yy_max);
        
        /*
        mx_goals(0,0) = x_max+0.20-0.100*0.5; mx_goals(1,0) = 0; mx_goals(2,0) = -0.070; mx_goals(3,0) = 1;
        mx_goals(0,1) = x_max+0.20-0.100*0.5; mx_goals(1,1) = 0; mx_goals(2,1) =  0.000; mx_goals(3,1) = 1;
        mx_goals(0,2) = x_max+0.20-0.100*0.5; mx_goals(1,2) = 0; mx_goals(2,2) =  0.070; mx_goals(3,2) = 1;
        mx_goals(0,3) = x_max+0.20-0.100*1.5; mx_goals(1,3) = 0; mx_goals(2,3) = -0.070; mx_goals(3,3) = 1;
        mx_goals(0,4) = x_max+0.20-0.100*1.5; mx_goals(1,4) = 0; mx_goals(2,4) =  0.000; mx_goals(3,4) = 1;
        mx_goals(0,5) = x_max+0.20-0.100*1.5; mx_goals(1,5) = 0; mx_goals(2,5) =  0.070; mx_goals(3,5) = 1;
        mx_goals(0,6) = x_max+0.20-0.100*2.5; mx_goals(1,6) = 0; mx_goals(2,6) = -0.070; mx_goals(3,6) = 1;
        mx_goals(0,7) = x_max+0.20-0.100*2.5; mx_goals(1,7) = 0; mx_goals(2,7) =  0.000; mx_goals(3,7) = 1;
        mx_goals(0,8) = x_max+0.20-0.100*2.5; mx_goals(1,8) = 0; mx_goals(2,8) =  0.070; mx_goals(3,8) = 1;
        mx_goals(0,9) = x_max+0.20-0.100*3.5; mx_goals(1,9) = 0; mx_goals(2,9) = -0.070; mx_goals(3,9) = 1;
        mx_goals(0,10)= x_max+0.20-0.100*3.5; mx_goals(1,10) = 0; mx_goals(2,10) =  0.000; mx_goals(3,10) = 1;
        mx_goals(0,11)= x_max+0.20-0.100*3.5; mx_goals(1,11) = 0; mx_goals(2,11) =  0.070; mx_goals(3,11) = 1;
        */

        mx_goals(0,0) = x_max-0.100*0.5; mx_goals(1,0)  = 0; mx_goals(2,0)  = z_min+0.07*0.5 ; mx_goals(3,0) = 1;
        mx_goals(0,1) = x_max-0.100*0.5; mx_goals(1,1)  = 0; mx_goals(2,1)  = z_min+0.07*1.5 ; mx_goals(3,1) = 1;
        mx_goals(0,2) = x_max-0.100*0.5; mx_goals(1,2)  = 0; mx_goals(2,2)  = z_min+0.07*2.5 ; mx_goals(3,2) = 1;
        mx_goals(0,3) = x_max-0.100*1.5; mx_goals(1,3)  = 0; mx_goals(2,3)  = z_min+0.07*0.5 ; mx_goals(3,3) = 1;
        mx_goals(0,4) = x_max-0.100*1.5; mx_goals(1,4)  = 0; mx_goals(2,4)  = z_min+0.07*1.5 ; mx_goals(3,4) = 1;
        mx_goals(0,5) = x_max-0.100*1.5; mx_goals(1,5)  = 0; mx_goals(2,5)  = z_min+0.07*2.5 ; mx_goals(3,5) = 1;
        mx_goals(0,6) = x_max-0.100*2.5; mx_goals(1,6)  = 0; mx_goals(2,6)  = z_min+0.07*0.5 ; mx_goals(3,6) = 1;
        mx_goals(0,7) = x_max-0.100*2.5; mx_goals(1,7)  = 0; mx_goals(2,7)  = z_min+0.07*1.5 ; mx_goals(3,7) = 1;
        mx_goals(0,8) = x_max-0.100*2.5; mx_goals(1,8)  = 0; mx_goals(2,8)  = z_min+0.07*2.5 ; mx_goals(3,8) = 1;
        mx_goals(0,9) = x_max-0.100*3.5; mx_goals(1,9)  = 0; mx_goals(2,9)  = z_min+0.07*0.5 ; mx_goals(3,9) = 1;
        mx_goals(0,10)= x_max-0.100*3.5; mx_goals(1,10) = 0; mx_goals(2,10) = z_min+0.07*1.5 ; mx_goals(3,10) = 1;
        mx_goals(0,11)= x_max-0.100*3.5; mx_goals(1,11) = 0; mx_goals(2,11) = z_min+0.07*2.5 ; mx_goals(3,11) = 1;

        mx_goals = tf_ws * mx_goals;
    }
    ~KukaBoxSetup(){}

    bool isValid(double x, double y, double yaw)
    {
        double buf_hard = objects_[0].radius;
        //double buf_soft = 0.066*0.5;
        double buf_soft = 0.066*0.5;

        double z = -0.20;
        double x_tf = tf_ws_inv(0,0)*x + tf_ws_inv(0,1)*y + tf_ws_inv(0,2)*z + tf_ws_inv(0,3);
        double y_tf = tf_ws_inv(1,0)*x + tf_ws_inv(1,1)*y + tf_ws_inv(1,2)*z + tf_ws_inv(1,3);
        double z_tf = tf_ws_inv(2,0)*x + tf_ws_inv(2,1)*y + tf_ws_inv(2,2)*z + tf_ws_inv(2,3);
        
/*
cv::Mat img = cv::Mat::zeros(1000,1000,CV_8UC3);
cv::circle(img,cv::Point(500-500*y_tf,500-500*x_tf),0.066*0.5*500, cv::Scalar(255,255,255));
cv::circle(img,cv::Point(500-500*y_tf,500-500*x_tf),0.096*0.5*500, cv::Scalar(255,255,255));
cv::line(img,cv::Point(500-500*z_min,500-500*x_min),
             cv::Point(500-500*z_max,500-500*x_max), cv::Scalar(255,0,0));
cv::line(img,cv::Point(500-500*(z_min+buf_soft),500-500*(x_min+buf_soft)),
             cv::Point(500-500*(z_max-buf_soft),500-500*(x_max-buf_soft)), cv::Scalar(0,255,255));
cv::line(img,cv::Point(500-500*(z_min+buf_hard),500-500*(x_min+buf_hard)),
             cv::Point(500-500*(z_max-buf_hard),500-500*(x_max-buf_hard)), cv::Scalar(0,0,255));
cv::imshow("isvalid",img);
cv::waitKey();
*/

        if( !(x_min + buf_soft <= x_tf && x_tf <= x_max - buf_soft &&
              z_min + buf_soft <= z_tf && z_tf <= z_max - buf_soft   ) )
        {
            return false;
        }

        if(  (x_min + buf_hard <= x_tf && x_tf <= x_max - buf_hard &&
              z_min + buf_hard <= z_tf && z_tf <= z_max - buf_hard   ) )
        {
            return true;
        }

        bool res = !is_intersect(x,y,yaw);
        return res;
    }

    void visualizeSetup(cv::Mat &img)
    {
        cv::line(img,cv::Point(500-500*pts[0].y,500-500*pts[0].x),
                     cv::Point(500-500*pts[1].y,500-500*pts[1].x), cv::Scalar(0,255,0) );
        cv::line(img,cv::Point(500-500*pts[1].y,500-500*pts[1].x),
                     cv::Point(500-500*pts[2].y,500-500*pts[2].x), cv::Scalar(0,255,0) );
        cv::line(img,cv::Point(500-500*pts[2].y,500-500*pts[2].x),
                     cv::Point(500-500*pts[3].y,500-500*pts[3].x), cv::Scalar(0,255,0) );
        cv::line(img,cv::Point(500-500*pts[3].y,500-500*pts[3].x),
                     cv::Point(500-500*pts[0].y,500-500*pts[0].x), cv::Scalar(0,255,0) );
    }

    void getGoalState(int o, double* x, double* y, double* yaw)
    {
        *x = mx_goals(0,o-1);
        *y = mx_goals(1,o-1);
        *yaw = yaw_goal;
    }

private:

    static
    bool get_intersection( double x1, double y1, 
                           double x2, double y2, 
                           double x3, double y3, 
                           double x4, double y4,
                           double* x, double* y )
    {
        double tmp = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
        if( tmp==0 ) return false;

        float t1 =   ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / tmp;
        if( t1 < 0 || 1 < t1 ) return false;

        float t2 =  -((x1-x2)*(y1-y3) - (y1-y2)*(x1-x3)) / tmp;
        if( t2 < 0 || 1 < t2 ) return false;

        *x = x1 + t1*(x2-x1);
        *y = y1 + t1*(y2-y1);

        double xx = x3 + t2*(x4-x3);
        double yy = y3 + t2*(y4-y3);
        double dist = sqrt( (*x-xx)*(*x-xx) + (*y-yy)*(*y-yy) );

        if( dist < 0.00001 ) return true;
        else return false;
    }

    bool is_intersect( double x, double y, double yaw )
    {
        cv::Point2f center(x,y);
        cv::Size2f rectangleSize(objects_[0].dims[1], objects_[0].dims[2]);
        cv::RotatedRect rotatedRectangle(center, rectangleSize, yaw*180.0/M_PI);
        cv::Point2f vrt[4];
        rotatedRectangle.points(vrt);

        double x_res, y_res;
        for( int p=0; p<pts.size(); p++ )
        {
            int p1 = (p  ) % pts.size();
            int p2 = (p+1) % pts.size();
            for( int v=0; v<4; v++ )
            {
                int v1 = (v  ) % 4;
                int v2 = (v+1) % 4;       

                if( get_intersection( pts[p1].x,pts[p1].y, pts[p2].x,pts[p2].y, 
                                      vrt[v1].x,vrt[v1].y, vrt[v2].x,vrt[v2].y,
                                      &x_res, &y_res                           ))
                {
/*
cv::Mat img = cv::Mat::zeros(500,1000,CV_8UC3);
visualizeSetup(img);
cv::line(img,cv::Point(500-500*pts[p1].y,500-500*pts[p1].x),
             cv::Point(500-500*pts[p2].y,500-500*pts[p2].x), cv::Scalar(255,0,0));
cv::line(img,cv::Point(500-500*vrt[v1].y,500-500*vrt[v1].x),
             cv::Point(500-500*vrt[v2].y,500-500*vrt[v2].x), cv::Scalar(0,0,255));
cv::circle(img,cv::Point(500-500*y_res,500-500*x_res),   5, cv::Scalar(0,0,255));
std::cout << "intersect!!" << std::endl;
cv::imshow("debug",img);
cv::waitKey();
*/
                    return true;
                }
            }
        }
        return false;
    }
    
    Eigen::Matrix4d tf_ws;
    Eigen::Matrix4d tf_ws_inv;

    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;

    std::vector<cv::Point2f> pts;

    Eigen::Matrix<double,4,12> mx_goals;
    double yaw_goal;
};

std::ostream& operator<< (std::ostream& os, const RobotObjectSetup::Object &object);
std::istream& operator>> (std::istream& is, RobotObjectSetup::Object &object);

#endif
