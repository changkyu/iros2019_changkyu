#ifndef CHANGKYU_MDP_PLANNER__HPP
#define CHANGKYU_MDP_PLANNER__HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <assert.h>
#include <cmath>
#include <map>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/function.hpp>
#include <boost/graph/labeled_graph.hpp>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/geometric/PathGeometric.h>

#include "statespace.hpp"
#include "bullet_simulation/collision_simulation.hpp"
#include "bullet_simulation/bullet_simulation.hpp"

class MDP
{
public:

    struct action_t
    {
        int id;
        double x;
        double y;
        double yaw;
    };

    struct Vertex
    {
        unsigned long int idx;
        ompl::base::State* state;
    };

    struct EdgeProb
    {
        action_t action;
        std::vector<std::pair<unsigned long int, double> > idxNprobs;        
    };

    typedef boost::shared_ptr< ompl::NearestNeighbors<Vertex*> > Neighbors;    

    MDP() : is_setup(false)
    {}

    MDP( const std::vector<RobotObjectSetup::Object> &objects,
         double x_offset,
         double angular_size=20.0 * M_PI / 180.0,
         double angular_unit=10.0 * M_PI / 180.0,
         double spatial_size_x=0.08, 
         double spatial_size_y=0.08,          
         double spatial_unit=0.01
    )
     : objects_(objects),
       discount_(0.9),
       n_sim_per_action_(20),
       spatial_size_x_(spatial_size_x),
       spatial_size_y_(spatial_size_y),
       angular_size_(angular_size),
       spatial_unit_(spatial_unit),
       angular_unit_(angular_unit),
       x_offset_(x_offset)
    {
        // discretization
        int n_spatial_x_half = (int)(spatial_size_x_ * 0.5 / spatial_unit_);
        int n_spatial_y_half = (int)(spatial_size_y_ * 0.5 / spatial_unit_);
        int n_angular_half   = (int)(angular_size_   * 0.5 / angular_unit_);

        lo_spatial_x_ = -n_spatial_x_half * spatial_unit_ + x_offset;
        hi_spatial_x_ =  n_spatial_x_half * spatial_unit_ + x_offset;
        lo_spatial_y_ = -n_spatial_y_half * spatial_unit_;
        hi_spatial_y_ =  n_spatial_y_half * spatial_unit_;
        lo_angular_   = -n_angular_half   * angular_unit_;
        hi_angular_   =  n_angular_half   * angular_unit_;

        is_setup = false;
    }

    ~MDP()
    {
        if( nn_ )
        {
            std::vector<Vertex *> vs;
            nn_->list(vs);
            for( int i=0; i<vs.size(); i++ ) delete vs[i];
        }
        if( si_ )
        {
            for( int i=0; i<states_.size(); i++ )
            {
                si_->freeState(states_[i]);
            }
        }
    }
    
    void setup()
    {
        if( is_setup==false )
        {
            // state space
            ompl::base::RealVectorBounds bounds(2);
            bounds.setLow( 0,lo_spatial_x_);
            bounds.setHigh(0,hi_spatial_x_);
            bounds.setLow( 1,lo_spatial_y_);
            bounds.setHigh(1,hi_spatial_y_);            
            
            si_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(
                    ompl::base::StateSpacePtr(new ObjectRelativeStateSpace(bounds))));
            si_->setStateValidityChecker(boost::bind(&MDP::isValid, this, _1));

            // nearest neighbors
            nn_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<Vertex*>(si_->getStateSpace()));
            nn_->setDistanceFunction(boost::bind(&MDP::distanceFunction, this, _1, _2));

            is_setup = true;
        }        
    }

    Vertex* getNearest(ompl::base::State* state)
    {
        Vertex v_query;
        v_query.idx = -1;
        v_query.state = state;
        return nn_->nearest(&v_query);
    }

    const std::vector<ompl::base::State*>& getStates(){ return states_; }

    double valueIteration()
    {
        int n_vertices = graph_.size();
        int n_actions = actions_.size();

        double err = 0;
        std::vector<double> values_prev = values_;
        for( int s0=0; s0<n_vertices; s0++ )
        {
            int n_edges = graph_[s0].size();

            double v_max = 0;
            for( int e=0; e<n_edges; e++ )
            {
                EdgeProb &ep = graph_[s0][e];

                double v = rewards_[s0];
                for( int p=0; p<ep.idxNprobs.size(); p++ )
                {
                    int s1 = ep.idxNprobs[p].first;
                    double T = ep.idxNprobs[p].second;                    
                    v += T * discount_ * values_prev[s1];
                }                
                if( v_max < v ) v_max = v;
            }

            values_[s0] = v_max;

            err += std::abs(values_[s0] - values_prev[s0]);
        }
        return err;
    }

    int valueIterations(const double thresh = 0)
    {
        // Initialize Values        
        for( int i=0; i<values_.size(); i++ ) values_[i] = 0;
        
        // Do value interations
        int iter = 0;        
        double err = 0;
        std::cout << "<< Value Iterations Convergence >>" << std::endl;
        do
        {
            iter++;
            err = valueIteration();
            printf("\r[%.5f]",err);
            std::cout << std::flush;
        } while( err > thresh );
        std::cout << " with " << iter << " iterations" << std::endl;
        return iter;
    }

    bool policy(ompl::base::State* state, action_t &action)
    {        
        Vertex* nvertex = getNearest(state);
        int s0 = nvertex->idx;
        return policy(s0, action);
    }

    int policy(int s0, action_t &action)
    {
        int s1_max = -1;
        double v_max = values_[s0];
        action_t action_max;
        action_max.x = NAN;
        action_max.y = NAN;
        action_max.yaw = NAN;
        
        for( int e=0; e<graph_[s0].size(); e++ )        
        {
            EdgeProb &ep = graph_[s0][e];
            
            int s1_most_likely = -1;
            double v = 0;
            double prob_max = -1;            
            for( int p=0; p<ep.idxNprobs.size(); p++ )
            {
                int s1 = ep.idxNprobs[p].first;
                double prob = ep.idxNprobs[p].second;
                v += (prob*values_[s1]);

                if( prob_max < prob )
                {
                    prob_max = prob;
                    s1_most_likely = s1;
                }
            }

            if( v_max < v )
            {                
                v_max = v;
                action_max = ep.action;
                s1_max = s1_most_likely;
            }
        }
        
        action = action_max;
        return s1_max;
    }

    void policies( ompl::base::State* state, 
                   std::vector<action_t> &actions,
                   std::vector<ompl::base::State*> &states_res )
    {
        action_t action;

        Vertex* nvertex = getNearest(state);
        if( si_->distance(nvertex->state,state)>0.001 )
        {
            ompl::base::State* state_init = si_->allocState();
            si_->copyState(state_init,nvertex->state);
            states_res.push_back(state_init);
            action.x   = STATE_OBJREL_1(state_init)->getX()
                       - STATE_OBJREL_1(state)->getX();
            action.y   = STATE_OBJREL_1(state_init)->getY()
                       - STATE_OBJREL_1(state)->getY();
            action.yaw = distance_angle( STATE_OBJREL_1(state_init)->getYaw(),
                                         STATE_OBJREL_1(state)->getYaw()      );
            actions.push_back(action);
        }
        
        int s0 = nvertex->idx;
        int s1;

        bool cycle = false;
        std::vector<int> idxes_added;
        idxes_added.push_back(s0);
        while( cycle==false && (s1 = policy(s0, action)) >= 0 )
        {
            ompl::base::State* state_res = si_->allocState();
            si_->copyState(state_res,states_[s1]);
            states_res.push_back(state_res);
            actions.push_back(action);

            s0 = s1;

            for( int i=0; i<idxes_added.size(); i++ )
            {
                if( idxes_added[i]==s0 )
                {
                    cycle = true;
                    break;
                } 
            }
            idxes_added.push_back(s0);
        }        
    }

    void constructGraph()
    {        
        constructVertex();
        constructEdge();
    }

    std::vector<RobotObjectSetup::Object>& getObjects()
    {
        return objects_;
    }

    friend std::ostream& operator<< (std::ostream&, const MDP&);
    friend std::istream& operator>> (std::istream&, MDP&);

    ompl::base::SpaceInformationPtr si_;

protected:

    virtual bool isValid(const ompl::base::State* state) = 0;
    virtual double getReward(const ompl::base::State* state) = 0;
    virtual void simulate( const ompl::base::State* state, const action_t &a, 
                           ompl::base::State* state_res                   ) = 0;

    bool isValidRange( const ompl::base::State* state )
    {
        const ObjectState* obj1 = STATE_OBJREL_1(state);

        double x_i   = 0;
        double y_i   = 0;
        double yaw_i = STATE_OBJREL_0(state)->value;
        double x_j   = obj1->getX();
        double y_j   = obj1->getY();
        double yaw_j = obj1->getYaw();

        if( (-angular_unit_ + lo_angular_   < yaw_i && yaw_i < hi_angular_   + angular_unit_ &&
             -spatial_unit_ + lo_spatial_x_ < x_j   && x_j   < hi_spatial_x_ + spatial_unit_ &&
             -spatial_unit_ + lo_spatial_y_ < y_j   && y_j   < hi_spatial_y_ + spatial_unit_ &&
             -angular_unit_ + lo_angular_   < yaw_j && yaw_j < hi_angular_   + angular_unit_    ) )
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    double lo_spatial_x_;
    double hi_spatial_x_;
    double lo_spatial_y_;
    double hi_spatial_y_;
    double lo_angular_;
    double hi_angular_;

    double spatial_size_x_;
    double spatial_size_y_;
    double angular_size_;

    double spatial_unit_;
    double angular_unit_;

    double x_offset_;

    std::vector<RobotObjectSetup::Object> objects_;    
    std::vector<action_t> actions_;

private:

    void constructVertex()
    {
        ompl::base::State* state_tmp = si_->allocState();
        ompl::base::SO2StateSpace::StateType* obj0 = STATE_OBJREL_0(state_tmp);        
        ObjectState* obj1 = STATE_OBJREL_1(state_tmp);

        int n_spatial_x_half = (int)(spatial_size_x_ * 0.5 / spatial_unit_);
        int n_spatial_y_half = (int)(spatial_size_y_ * 0.5 / spatial_unit_);
        int n_angular_half   = (int)(angular_size_   * 0.5 / angular_unit_);

        int prog = 0;
        int n_total = (n_spatial_x_half*2 + 1) *
                      (n_spatial_y_half*2 + 1) *
                      (n_angular_half  *2 + 1) *
                      (n_angular_half  *2 + 1) ;

        int i = 0;        
        for( int idx_x1=-n_spatial_x_half; idx_x1<=n_spatial_x_half; idx_x1++ )
        for( int idx_y1=-n_spatial_y_half; idx_y1<=n_spatial_y_half; idx_y1++ )
        for( int idx_a1=-n_angular_half;   idx_a1<=n_angular_half;   idx_a1++ )
        for( int idx_a0=-n_angular_half;   idx_a0<=n_angular_half;   idx_a0++ )
        {
            obj0->value = angular_unit_ * idx_a0;
            obj1->setX(   spatial_unit_ * idx_x1 + x_offset_ );
            obj1->setY(   spatial_unit_ * idx_y1             );
            obj1->setYaw( angular_unit_ * idx_a1             );

            if( isValid(state_tmp) )
            {
                ompl::base::State* state = si_->allocState();
                si_->copyState(state,state_tmp);
                
                addState(state);                    
                i++;
            }

            prog++;
            printf("\r[%.2f %%] (%d/%d)",prog/(float)n_total*100.0, prog, n_total);
            std::cout << std::flush;
        }
        std::cout << std::endl;
        std::cout << states_.size() << " valid states are generated." << std::endl;

        si_->freeState(state_tmp);
    }

    void constructEdge()
    {
        int n_edges = 0;
        int n_isolated = 0;

        // construct edges
        int n_states = states_.size();
        for( int i=0; i<n_states; i++ )
        {            
            ompl::base::State* state = states_[i];

            int n_edges_local = 0;
            
            int n_actions = actions_.size();
            for( int a=0; a<n_actions; a++ )
            {
                action_t action = actions_[a]; // must be a copy

                ompl::base::State* states_res[n_sim_per_action_];
                for( int n=0; n<n_sim_per_action_; n++ )
                {
                    states_res[n] = si_->allocState();
                }

                for( int n=0; n<n_sim_per_action_; n++ )
                {
                    /*
                    action.x += (-0.0025 + (rand()/(double)RAND_MAX)*0.005);
                    action.y += (-0.0025 + (rand()/(double)RAND_MAX)*0.005);
                    */                    
                    simulate(state, action, states_res[n]);
                }

                double sum_weight = 0;
                typedef std::map<unsigned long int,double> IDX2PROB_t;
                IDX2PROB_t idx2prob;
                
                for( int n=0; n<n_sim_per_action_; n++ )
                {
                    if( isValidRange(states_res[n])==false ) continue;
                    
                    Vertex* nvertex = getNearest(states_res[n]);
                    //if( si_->distance(nvertex->state,states_res[n]) < 0.03 )

                    if( i != nvertex->idx )
                    {
                        IDX2PROB_t::iterator it = idx2prob.find(nvertex->idx);
                        if(it!=idx2prob.end()) it->second = it->second+1;
                        else idx2prob.insert(std::make_pair(nvertex->idx,1));

                        sum_weight++;
                    }
                }

                if( sum_weight > 0 )
                {
                    EdgeProb e;
                    e.action = action;
                    for( IDX2PROB_t::iterator it = idx2prob.begin(); it!=idx2prob.end(); it++ )
                    {
                        e.idxNprobs.push_back(std::make_pair(it->first,it->second / sum_weight));                        
                    }
                    graph_[i].push_back(e);

                    n_edges++;
                    n_edges_local++;
                }

                for( int n=0; n<n_sim_per_action_; n++ )
                {
                    si_->freeState(states_res[n]);
                }
            }

            if( n_edges_local==0 ) n_isolated++;

            printf("\r[%.2f %%]",(i+1)/(float)n_states*100.0);
            std::cout << std::flush;
        }
        std::cout << std::endl;
        std::cout << n_edges << " pairs are connected" << std::endl;
        std::cout << n_isolated << " vertices are isolated" << std::endl;
    }

    void addState(ompl::base::State* state)
    {        
        Vertex* v = new Vertex;
        v->idx = states_.size();
        v->state = state;
        nn_->add(v);

        states_.push_back(state);
        rewards_.push_back(getReward(state));

        std::vector<EdgeProb> es;
        graph_.push_back(es);
        values_.push_back(0);
    }

    double distanceFunction(const Vertex* a, const Vertex* b) const
    {
        return si_->distance(a->state, b->state);
    }

    double discount_;
    int n_sim_per_action_;    

    Neighbors nn_;    
    bool is_setup;

    std::vector<ompl::base::State*> states_;
    std::vector<double> values_;
    std::vector<double> rewards_;

    std::vector<std::vector<EdgeProb> > graph_; // vertex - action - (prob, vertex)
};

std::ostream& operator<< (std::ostream&, const MDP&);
std::istream& operator>> (std::istream&, MDP&);

class MDP_Rotator : public MDP
{
public:
    MDP_Rotator() : MDP(){}

    MDP_Rotator( double yaw, double yaw_base, double x_offset,
                 const std::vector<RobotObjectSetup::Object> &objects )
     : MDP( objects, x_offset, 
            fabs(yaw)*2, fabs(yaw)*0.5, 
            0.06, 0.06, 0.005      ),
       yaw_(yaw),
       yaw_base_(yaw_base)
    {
        // Simulation
        q_base_.setEulerZYX(yaw_base_,0,0);

        sim_.SetGravity(btVector3(0,0,-0.1));
        sim_.AddPlaneShape(btVector4(0,0,1,0));
        sim_idxes_.resize(2);
        for( int o=0; o<2; o++ )
        {            
            sim_idxes_[o] = sim_.AddCollisionShape( objects_[o].shape, 
                                                    btVector3(0,0,0),
                                                    btQuaternion(0,0,0,1), 
                                                    objects_[o].mass, 1.0, false);
        }

        // Actions
        action_t a;        
        a.id = 0;
        a.id++; a.x = -0.01; a.y = 0; a.yaw = 0; actions_.push_back(a);
        a.id++; a.x =  0.01; a.y = 0; a.yaw = 0; actions_.push_back(a);        
        a.id++; a.x = -0.02; a.y = 0; a.yaw = 0; actions_.push_back(a);
        a.id++; a.x =  0.02; a.y = 0; a.yaw = 0; actions_.push_back(a);
        a.id++; a.x = 0; a.y = -0.01; a.yaw = 0; actions_.push_back(a);
        a.id++; a.x = 0; a.y =  0.01; a.yaw = 0; actions_.push_back(a);        
        a.id++; a.x = 0; a.y = -0.02; a.yaw = 0; actions_.push_back(a);
        a.id++; a.x = 0; a.y =  0.02; a.yaw = 0; actions_.push_back(a);
        a.id++; a.x = 0; a.y = 0; a.yaw = -angular_unit_; actions_.push_back(a);
        a.id++; a.x = 0; a.y = 0; a.yaw =  angular_unit_; actions_.push_back(a);
        a.id++; a.x = 0; a.y = 0; a.yaw = -angular_unit_*2; actions_.push_back(a);
        a.id++; a.x = 0; a.y = 0; a.yaw =  angular_unit_*2; actions_.push_back(a);
    }

    ~MDP_Rotator()
    {

    }

    double getGoalYaw(){ return yaw_; }
    double getBaseYaw(){ return yaw_base_; }

    friend std::ostream& operator<< (std::ostream&, const MDP_Rotator&);
    friend std::istream& operator>> (std::istream&, MDP_Rotator&);

protected:

    bool isValid( const ompl::base::State* state )
    {
        if( isValidRange(state)==false ) return false;
        if( isCollide(state)==true ) return false;

        return true;
    }

    double getReward( const ompl::base::State* state )
    {
        if( distance_angle(STATE_OBJREL_0(state)->value,yaw_)==0 )
        {            
            return 1.0;
        }
        else
        {
            return 0.0;
        }
    }

    void simulate( const ompl::base::State* state, const action_t &action, 
                         ompl::base::State* state_res                     )
    {
        for( int o=1; o<=2; o++ )            
        {
            double x,y,a;
            if( o==1 )
            {
                x = 0;
                y = 0;
                a = STATE_OBJREL_0(state)->value;
            }
            else
            {
                const ObjectState* obj1 = STATE_OBJREL_1(state);
                x = obj1->getX();
                y = obj1->getY();
                a = obj1->getYaw();
            }

            btVector3 T( x, y, objects_[o-1].z_offset );

            btQuaternion q;
            q.setEulerZYX(a,0,0);
            q = q * q_base_ * objects_[o-1].q_offset;

            sim_.SetObjectPose(sim_idxes_[o-1], q, T);
        }        

        int idx_sim = sim_idxes_[1];
        sim_.MoveRotateObject(idx_sim, btVector3(action.x,action.y,0), action.yaw);

        double x_offset=0, y_offset=0;
        for( int o=1; o<=2; o++ )
        {   
            btVector3 T;
            btQuaternion q;
            sim_.GetObjectPose(sim_idxes_[o-1], q, T);

            q = q * (q_base_ * objects_[o-1].q_offset).inverse();
            btScalar yaw, pitch, roll;
            q.getEulerZYX(yaw, pitch, roll);
           
            if( o==1 )
            {
                STATE_OBJREL_0(state_res)->value = yaw;
                x_offset = -T[0];
                y_offset = -T[1];
            }
            else
            {
                ObjectState* state_obj = STATE_OBJREL_1(state_res);
                state_obj->setX(T[0] + x_offset);
                state_obj->setY(T[1] + y_offset);
                state_obj->setYaw(yaw);
            }
        }
    }

private:

    bool isCollide( const ompl::base::State* state )
    {
        const ObjectState* obj1 = STATE_OBJREL_1(state);

        double x_i   = 0;
        double y_i   = 0;
        double yaw_i = STATE_OBJREL_0(state)->value;
        double x_j   = obj1->getX();
        double y_j   = obj1->getY();
        double yaw_j = obj1->getYaw();

        CollisionSimulation sim;
        btVector3 dims_i( objects_[0].dims[0]*0.5,
                          objects_[0].dims[1]*0.5,
                          objects_[0].dims[2]*0.5  );
        btVector3 dims_j( objects_[1].dims[0]*0.5,
                          objects_[1].dims[1]*0.5,
                          objects_[1].dims[2]*0.5  );

        btQuaternion q_i;
        q_i.setEulerZYX(yaw_i,0,0);
        q_i = q_i * q_base_ * objects_[0].q_offset;
        btQuaternion q_j;
        q_j.setEulerZYX(yaw_j,0,0);
        q_j = q_j * q_base_ * objects_[1].q_offset;

        sim.AddBox(btVector3(x_i,y_i,0),q_i,dims_i);
        sim.AddBox(btVector3(x_j,y_j,0),q_j,dims_j);

        return sim.Test();
    }

    double yaw_;
    double yaw_base_;
    btQuaternion q_base_;    

#if DEBUG
    BulletSimulationGui sim_;
#else
    BulletSimulation sim_;
#endif
    std::vector<int> sim_idxes_;
};

std::ostream& operator<< (std::ostream&, const MDP_Rotator&);
std::istream& operator>> (std::istream&, MDP_Rotator&);

class SoapRotators
{
public:
    enum DOCKING_SIDE {
        DOCKING_270=0,
        DOCKING_180,
        DOCKING_90,
        DOCKING_0,
    };

    struct DockingPoint
    {
        ompl::base::State* state;
        DOCKING_SIDE docking_side;
        double yaw_goal;
    };

    SoapRotators( const std::vector<RobotObjectSetup::Object> &objects,
                  double yaw = 10*M_PI/180.0 )
     : yaw_base_{ -M_PI*0.5,      0, M_PI*0.5,   M_PI },
         x_base_{    -0.066, -0.096,   -0.066, -0.096 }
    {
        for( int i=0; i<4; i++ )
        {
            RotatorMeta meta1;
            meta1.rotator = new MDP_Rotator(-yaw,yaw_base_[i],x_base_[i],objects);
            meta1.rotator->setup();
            meta1.rotator->constructGraph();
            meta1.rotator->valueIterations();
            meta1.yaw_goal = -yaw;
            meta1.yaw_base = yaw_base_[i];
            rotatorSets_.push_back(meta1);

            RotatorMeta meta2;
            meta2.rotator = new MDP_Rotator( yaw,yaw_base_[i],x_base_[i],objects);
            meta2.rotator->setup();
            meta2.rotator->constructGraph();
            meta2.rotator->valueIterations();
            meta2.yaw_goal = yaw;
            meta2.yaw_base = yaw_base_[i];
            rotatorSets_.push_back(meta2);
        }

        // Query State Space
        si_query_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(
                      ompl::base::StateSpacePtr(new RobotObjectStateSpace(2))));
    }

    SoapRotators( std::ifstream &is )
     : yaw_base_{ -M_PI*0.5,      0, M_PI*0.5,   M_PI },
         x_base_{    -0.066, -0.096,   -0.066, -0.096 }
    {
        int n_rotators;
        is.read(reinterpret_cast<char*>(&n_rotators), sizeof(n_rotators));
        for( int i=0; i<n_rotators; i++ )
        {
            RotatorMeta meta;
            meta.rotator = new MDP_Rotator;
            is >> *(meta.rotator);
            meta.yaw_goal = meta.rotator->getGoalYaw();
            meta.yaw_base = meta.rotator->getBaseYaw();
            rotatorSets_.push_back(meta);            
        }

        // Query State Space
        si_query_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(
                      ompl::base::StateSpacePtr(new RobotObjectStateSpace(2))));
    }

    ~SoapRotators()
    {
        for( int i=0; i<rotatorSets_.size(); i++ )
        {
            delete rotatorSets_[i].rotator;
        }
    }

    void store( std::ofstream &os )
    {
        int n_rotators = rotatorSets_.size();
        os.write(reinterpret_cast<const char*>(&n_rotators), sizeof(n_rotators));
        for( int i=0; i<n_rotators; i++ )
        {
            os << *(rotatorSets_[i].rotator);
        }
    }

    void record( double x, double y, double yaw, double yaw_base,
                 const std::vector<MDP::action_t> &actions,
                 const std::vector<RobotObjectSetup::Object> &objects )
    {
        BulletSimulationGui sim;
        sim.SetGravity(btVector3(0,0,-0.1));
        sim.AddPlaneShape(btVector4(0,0,1,0));

        std::vector<int> sim_idxes(2);
        for( int o=1; o<=2; o++ )
        {            
            sim_idxes[o-1] = sim.AddCollisionShape( objects[o-1].shape, 
                                                    btVector3(0,0,0),
                                                    btQuaternion(0,0,0,1), 
                                                    objects[o-1].mass, 1.0, false );
            btVector3 T;
            btQuaternion q;
            if( o==1 )
            {
                T[0] = 0; T[1] = 0; T[2] = 0;
                q.setEulerZYX(0,0,0);
            }
            else
            {
                T[0] = x; T[1] = y; T[2] = objects[o-1].z_offset;
                q.setEulerZYX(yaw,0,0);
            }

            btQuaternion q_base;
            q_base.setEulerZYX(yaw_base,0,0);

            q = q * q_base * objects[o-1].q_offset;

            sim.SetObjectPose(sim_idxes[o-1], q, T);
        }        

        sim.SpinInit();
        sim.ResetCamera( 0.3,-90,80, 0.0,0.0,0 );

        int idx_sim = sim_idxes[1];
        for( int i=0; i<actions.size(); i++ )
        {
            char tmp[256];
            sprintf(tmp,"/home/cs1080/tmp_video/action%03d_%%03d.png", i);
            std::string fp_format = tmp;
            //sim.MoveRotateObjectRecord(idx_sim, btVector3(actions[i].x,actions[i].y,0), actions[i].yaw, fp_format);
            sim.MoveRotateObject(idx_sim, btVector3(actions[i].x,actions[i].y,0), actions[i].yaw);
        }
    }

    void rotate2D( double x_in, double y_in, double yaw, 
                   double* x_out, double* y_out          )
    {
        double x_tmp =  x_in * std::cos(yaw) - y_in * std::sin(yaw);
        double y_tmp =  x_in * std::sin(yaw) + y_in * std::cos(yaw);
        
        *x_out = x_tmp;
        *y_out = y_tmp;
    }

    void Rel2Abs( double x, double y, double yaw, 
                  double x_target, double y_target, double yaw_target, double yaw_base,
                  double* x_res, double *y_res, double *yaw_res=NULL )
    {
        rotate2D( x,     y,    -yaw_base,   x_res,y_res);
        rotate2D(*x_res,*y_res, yaw_target, x_res,y_res);
        *x_res += x_target;
        *y_res += y_target;

        if( yaw_res )
        {
            //*yaw_res = yaw - yaw_base + yaw_target;
            *yaw_res = yaw + yaw_target;
            *yaw_res = (*yaw_res) > M_PI ? (*yaw_res)-(2*M_PI) : (*yaw_res);
        }
    }

    bool compareState(DockingPoint a, DockingPoint b, const ompl::base::State* c)
    {
        //return si_query_->distance(a.state,c) < si_query_->distance(b.state,c);

        ompl::base::StateSpacePtr oss = si_query_->getStateSpace()->as<RobotObjectStateSpace>()->getSubspace(0);
       
        double dist_a = oss->distance(STATE_OBJECT(a.state,1),STATE_OBJECT(a.state,2));
        double dist_b = oss->distance(STATE_OBJECT(b.state,1),STATE_OBJECT(b.state,2));

        return dist_a < dist_b;
    }

    void getRelAngle( double x_target, double y_target, double yaw_target,
                      double x_goal,   double y_goal,   double* angle_rel)    
    {
        double x_rel, y_rel;
        rotate2D(x_goal-x_target,y_goal-y_target,-yaw_target, &x_rel,&y_rel);
        *angle_rel = std::atan2(y_rel,x_rel);
    }

    double getYawRel( double x_target, double y_target, double yaw_target,
                      double x_goal,   double y_goal, const DOCKING_SIDE docking_side )
    {
        double angle_rel;
        getRelAngle( x_target, y_target, yaw_target, x_goal, y_goal, &angle_rel);
        return getYawRel(docking_side, angle_rel);
    }

    double getYawRel( const DOCKING_SIDE docking_side, double angle_rel )
    {
        double yaw_goal = yaw_base_[docking_side] + angle_rel;

        // opposite side
        std::vector<DOCKING_SIDE> order_dockingside;
        if(      fabs(distance_angle(angle_rel,0       )) <= M_PI*0.25 )
        {
            order_dockingside.push_back(DOCKING_180);
            order_dockingside.push_back(DOCKING_270);
            order_dockingside.push_back(DOCKING_90);
            order_dockingside.push_back(DOCKING_0);
        }
        else if( fabs(distance_angle(angle_rel,M_PI*0.5)) <= M_PI*0.25 )
        {
            order_dockingside.push_back(DOCKING_270);
            order_dockingside.push_back(DOCKING_0);
            order_dockingside.push_back(DOCKING_180);
            order_dockingside.push_back(DOCKING_90);
        }
        else if( fabs(distance_angle(angle_rel,M_PI    )) <= M_PI*0.25 )
        {
            order_dockingside.push_back(DOCKING_0);
            order_dockingside.push_back(DOCKING_90);
            order_dockingside.push_back(DOCKING_270);
            order_dockingside.push_back(DOCKING_180);
        }
        else if( fabs(distance_angle(angle_rel,M_PI*1.5)) <= M_PI*0.25 )
        {
            order_dockingside.push_back(DOCKING_90);
            order_dockingside.push_back(DOCKING_180);
            order_dockingside.push_back(DOCKING_0);
            order_dockingside.push_back(DOCKING_270);
        }
        int d=-1;
        for( int i=0; i<4; i++ )
        {
            if( order_dockingside[i]==docking_side )
            {
                d = i;
                break;
            }
        }

        if(      d==1 ) yaw_goal += M_PI*0.5;
        else if( d==2 ) yaw_goal -= M_PI*0.5;
        else if( d==3 ) yaw_goal += M_PI;
        yaw_goal = ((int)(yaw_goal * 180.0 / M_PI) % 360) * M_PI / 180.0;
        yaw_goal = yaw_goal > M_PI ? yaw_goal-(2*M_PI) : yaw_goal;
        return yaw_goal;
    }

    void getDockingPoints( const ompl::base::State* state, 
                           double x_goal, double y_goal, 
                           std::vector<DockingPoint> &possible_dockings,
                           bool aligned=false )
    {
        double x_target   = STATE_OBJECT(state,1)->getX();
        double y_target   = STATE_OBJECT(state,1)->getY();
        double yaw_target = STATE_OBJECT(state,1)->getYaw();

        double angle_rel;
        getRelAngle(x_target, y_target, yaw_target, x_goal, y_goal, &angle_rel);

        // opposite side
        std::vector<DOCKING_SIDE> order_dockingside;
        if(      fabs(distance_angle(angle_rel,0       )) <= M_PI*0.25 )
        {
            order_dockingside.push_back(DOCKING_180);
            order_dockingside.push_back(DOCKING_270);
            order_dockingside.push_back(DOCKING_90);
            order_dockingside.push_back(DOCKING_0);
        }
        else if( fabs(distance_angle(angle_rel,M_PI*0.5)) <= M_PI*0.25 )
        {
            order_dockingside.push_back(DOCKING_270);
            order_dockingside.push_back(DOCKING_0);
            order_dockingside.push_back(DOCKING_180);
            order_dockingside.push_back(DOCKING_90);
        }
        else if( fabs(distance_angle(angle_rel,M_PI    )) <= M_PI*0.25 )
        {
            order_dockingside.push_back(DOCKING_0);
            order_dockingside.push_back(DOCKING_90);
            order_dockingside.push_back(DOCKING_270);
            order_dockingside.push_back(DOCKING_180);
        }
        else if( fabs(distance_angle(angle_rel,M_PI*1.5)) <= M_PI*0.25 )
        {
            order_dockingside.push_back(DOCKING_90);
            order_dockingside.push_back(DOCKING_180);
            order_dockingside.push_back(DOCKING_0);
            order_dockingside.push_back(DOCKING_270);
        }

        for( int d=0; d<order_dockingside.size(); d++ )
        {
            DOCKING_SIDE &docking_side = order_dockingside[d];
            
            double yaw_goal = yaw_base_[docking_side] + angle_rel;
            if(      d==1 ) yaw_goal += M_PI*0.5;
            else if( d==2 ) yaw_goal -= M_PI*0.5;
            else if( d==3 ) yaw_goal += M_PI;
            yaw_goal = ((int)(yaw_goal * 180.0 / M_PI) % 360) * M_PI / 180.0;
            yaw_goal = yaw_goal > M_PI ? yaw_goal-(2*M_PI) : yaw_goal;
            
            int i = docking_side*2 + (yaw_goal<0?0:1);

            MDP_Rotator* rotator = rotatorSets_[i].rotator;
            std::vector<DockingPoint> possible_tmp;
            getDockingPoints( rotator, 
                              x_target, y_target, yaw_target, yaw_base_[docking_side],
                              possible_tmp, aligned                    );
            std::sort( possible_tmp.begin(), possible_tmp.end(), 
                       std::bind(&SoapRotators::compareState,this,
                                 std::placeholders::_1,std::placeholders::_2,state));

            for( int p=0; p<possible_tmp.size(); p++ )
            {
                possible_tmp[p].docking_side = docking_side;
                possible_tmp[p].yaw_goal = yaw_goal;
                possible_dockings.push_back(possible_tmp[p]);
            }
        }
    }

    void getDockingPoint_aligned( const ompl::base::State* state, 
                                  double x_goal, double y_goal, 
                                  DOCKING_SIDE docking_side,
                                  DockingPoint* aligned_docking  )
    {
        std::vector<SoapRotators::DockingPoint> aligned_dockings;
std::cout << __LINE__ << std::endl;

        getDockingPoints( state, x_goal, y_goal, aligned_dockings, true );
std::cout << __LINE__ << std::endl;

        int d_min = -1;
        double dist_min = INFINITY;        
        for( int d=0; d<aligned_dockings.size(); d++ )
        {
            ObjectState* state_target = STATE_OBJECT(aligned_dockings[d].state,1);
            ObjectState* state_pusher = STATE_OBJECT(aligned_dockings[d].state,2);
            double dist = sqrt( (state_target->getX() - state_pusher->getX())*
                                (state_target->getX() - state_pusher->getX())+
                                (state_target->getY() - state_pusher->getY())*
                                (state_target->getY() - state_pusher->getY())  );
            if( aligned_dockings[d].docking_side == docking_side && dist_min > dist )
            {
                d_min = d;
                dist_min = dist;
            }
        }
std::cout << __LINE__ << std::endl;
std::cout << "d_min: " << d_min << std::endl;
        
        aligned_docking->yaw_goal     = aligned_dockings[d_min].yaw_goal;
        aligned_docking->docking_side = aligned_dockings[d_min].docking_side;
        si_query_->copyState( aligned_docking->state, 
                              aligned_dockings[d_min].state );
std::cout << __LINE__ << std::endl;
        for( int d=0; d<aligned_dockings.size(); d++ )
        {
            si_query_->freeState(aligned_dockings[d].state);
        }
std::cout << __LINE__ << std::endl;
    }

    void plan( const DockingPoint &docking,
               std::vector<MDP::action_t> &actions_res, 
               ompl::geometric::PathGeometric &path    )
    {
        double x_target   = STATE_OBJECT(docking.state,1)->getX();
        double y_target   = STATE_OBJECT(docking.state,1)->getY();
        double yaw_target = STATE_OBJECT(docking.state,1)->getYaw();
        
        DOCKING_SIDE docking_side = docking.docking_side;
        int i = docking_side*2 + (docking.yaw_goal<0?0:1);

        if(fabs(docking.yaw_goal)<=fabs(rotatorSets_[i].yaw_goal)*0.5) return;

        MDP_Rotator* rotator = rotatorSets_[i].rotator;

        double x_query, y_query;
        double x_rel = STATE_OBJECT(docking.state,2)->getX() - x_target;
        double y_rel = STATE_OBJECT(docking.state,2)->getY() - y_target;
        rotate2D(x_rel, y_rel, -yaw_target, &x_rel, &y_rel);
        rotate2D(x_rel, y_rel, rotatorSets_[i].yaw_base, &x_query, &y_query);
        
        ompl::base::State* state_curr = rotator->si_->allocState();
                
        STATE_OBJREL_0(state_curr)->value = 0;
        STATE_OBJREL_1(state_curr)->setX( x_query );
        STATE_OBJREL_1(state_curr)->setY( y_query );
        STATE_OBJREL_1(state_curr)->setYaw( distance_angle(
            STATE_OBJECT(docking.state,2)->getYaw(), yaw_target ));
        
        std::vector<ompl::base::State*> states_res;        
        rotator->policies(state_curr,actions_res,states_res);        
        
        double x_1   = STATE_OBJREL_1(state_curr)->getX();
        double y_1   = STATE_OBJREL_1(state_curr)->getY();
        double yaw_1 = STATE_OBJREL_1(state_curr)->getYaw();
        ompl::base::State* state_res = si_query_->allocState();
        for( int i=0; i<actions_res.size(); i++ )
        {
            x_1   += actions_res[i].x;
            y_1   += actions_res[i].y;
            yaw_1 = STATE_OBJREL_1(states_res[i])->getYaw();

            double x_0 = x_1 - STATE_OBJREL_1(states_res[i])->getX();
            double y_0 = y_1 - STATE_OBJREL_1(states_res[i])->getY();
            double yaw_0 = STATE_OBJREL_0(states_res[i])->value;

            double x_0_res, y_0_res, yaw_0_res, 
                   x_1_res, y_1_res, yaw_1_res;
            Rel2Abs( x_0, y_0, yaw_0,
                     x_target, y_target, yaw_target, yaw_base_[docking_side],
                     &x_0_res, &y_0_res, &yaw_0_res );
            Rel2Abs( x_1, y_1, yaw_1,
                     x_target, y_target, yaw_target, yaw_base_[docking_side],
                     &x_1_res, &y_1_res, &yaw_1_res );
            Rel2Abs( actions_res[i].x, actions_res[i].y, actions_res[i].yaw, 
                     0, 0, yaw_target, yaw_base_[docking_side],
                     &actions_res[i].x, &actions_res[i].y );

            STATE_OBJECT(state_res,1)->setX(x_0_res);
            STATE_OBJECT(state_res,1)->setY(y_0_res);
            STATE_OBJECT(state_res,1)->setYaw(yaw_0_res);
            STATE_OBJECT(state_res,2)->setX(x_1_res);
            STATE_OBJECT(state_res,2)->setY(y_1_res);
            STATE_OBJECT(state_res,2)->setYaw(yaw_1_res);
            path.append(state_res);
        }

        std::vector<MDP::action_t> actions_res_next; 
        ompl::geometric::PathGeometric path_next(si_query_);

        DockingPoint docking_next = docking;
        docking_next.yaw_goal -= rotatorSets_[i].yaw_goal;
        docking_next.state = si_query_->allocState();
        si_query_->copyState(docking_next.state, state_res);

        si_query_->freeState(state_res);
        rotator->si_->freeState(state_curr);
        for( int i=0; i<states_res.size(); i++ )
        {
            rotator->si_->freeState(states_res[i]);
        } 

        plan( docking_next, actions_res_next, path_next );
        
        actions_res.insert(actions_res.end(), actions_res_next.begin(), actions_res_next.end());
        path.append(path_next);

        si_query_->freeState(docking_next.state);        
    }

    ompl::base::SpaceInformationPtr getQuerySpaceInformation()
    {
        return si_query_;
    }

private:

    struct RotatorMeta
    {
        MDP_Rotator* rotator;
        double yaw_goal;
        double yaw_base;        
    };

    void getDockingPoints( MDP_Rotator* rotator,
                           double x_target, double y_target, double yaw_target, double yaw_base,
                           std::vector<DockingPoint> &possible_dockings,
                           bool aligned=false                                  )
    {   
        const std::vector<ompl::base::State*> &states = rotator->getStates();
        for( int s=0; s<states.size(); s++ )
        {
            if( STATE_OBJREL_0(states[s])->value==0 )
            {
                const ObjectState* state_obj = STATE_OBJREL_1(states[s]);
                double x   = state_obj->getX();
                double y   = state_obj->getY();
                double yaw = state_obj->getYaw();

                //if( aligned && (y < -0.01 || 0.01 < y || yaw!=0) ) continue;
                if( aligned && yaw!=0 ) continue;

                double x_res, y_res, yaw_res;
                Rel2Abs( x, y, yaw, 
                         x_target, y_target, yaw_target, yaw_base,
                         &x_res, &y_res, &yaw_res );

                ompl::base::State* state_res = si_query_->allocState();
                STATE_OBJECT(state_res,1)->setX(x_target);
                STATE_OBJECT(state_res,1)->setY(y_target);
                STATE_OBJECT(state_res,1)->setYaw(yaw_target);
                STATE_OBJECT(state_res,2)->setX(x_res);
                STATE_OBJECT(state_res,2)->setY(y_res);
                STATE_OBJECT(state_res,2)->setYaw(yaw_res);

                DockingPoint docking_point;
                docking_point.state = state_res;
                possible_dockings.push_back(docking_point);
            }            
        }
    }

    const double yaw_base_[4];
    const double x_base_[4];

    std::vector<RotatorMeta> rotatorSets_;
    ompl::base::SpaceInformationPtr si_query_;
};

std::ostream& operator<< (std::ostream& os, const SoapRotators::DOCKING_SIDE &ds);

#endif