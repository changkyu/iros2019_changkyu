#include "mdp_planner.hpp"

#include <ompl/base/StateStorage.h>

using namespace std;

template<typename Scalar>
static
std::ostream& operator<< (std::ostream& os, const std::vector<Scalar>& vec)
{
    int len = vec.size();
    os.write(reinterpret_cast<const char*>(&len),sizeof(len));
    for( int i=0; i<len; i++ )
    {
        Scalar val = vec[i];
        os.write(reinterpret_cast<const char*>(&val),sizeof(val));
    }
}

template<typename Scalar>
static
std::istream& operator>> (std::istream& is, std::vector<Scalar>& vec)
{
    int len;
    is.read(reinterpret_cast<char*>(&len),sizeof(len));

    if( is.eof() ) return is;

    vec.resize(len);
    for( int i=0; i<len; i++ )
    {
        Scalar val;
        is.read(reinterpret_cast<char*>(&val),sizeof(val));
        vec[i] = val;
    }
}

std::ostream& operator<< (std::ostream& os, const MDP& mdp)
{
    os.write(reinterpret_cast<const char*>(&mdp.lo_spatial_x_),sizeof(mdp.lo_spatial_x_));
    os.write(reinterpret_cast<const char*>(&mdp.hi_spatial_x_),sizeof(mdp.hi_spatial_x_));
    os.write(reinterpret_cast<const char*>(&mdp.lo_spatial_y_),sizeof(mdp.lo_spatial_y_));
    os.write(reinterpret_cast<const char*>(&mdp.hi_spatial_y_),sizeof(mdp.hi_spatial_y_));
    os.write(reinterpret_cast<const char*>(&mdp.lo_angular_),   sizeof(mdp.lo_angular_));
    os.write(reinterpret_cast<const char*>(&mdp.hi_angular_),   sizeof(mdp.hi_angular_));
    
    os.write(reinterpret_cast<const char*>(&mdp.spatial_size_x_),sizeof(mdp.spatial_size_x_));
    os.write(reinterpret_cast<const char*>(&mdp.spatial_size_y_),sizeof(mdp.spatial_size_y_));
    os.write(reinterpret_cast<const char*>(&mdp.angular_size_),  sizeof(mdp.angular_size_));
    os.write(reinterpret_cast<const char*>(&mdp.spatial_unit_),  sizeof(mdp.spatial_unit_));
    os.write(reinterpret_cast<const char*>(&mdp.angular_unit_),  sizeof(mdp.angular_unit_));

    os.write(reinterpret_cast<const char*>(&mdp.x_offset_),      sizeof(mdp.x_offset_));

    int n_objects = mdp.objects_.size();
    os.write(reinterpret_cast<const char*>(&n_objects),sizeof(n_objects));    
    for( int i=0; i<n_objects; i++ )
    {
        os << mdp.objects_[i];
    }

    os << mdp.actions_;
    
    os.write(reinterpret_cast<const char*>(&mdp.discount_),         sizeof(mdp.discount_));
    os.write(reinterpret_cast<const char*>(&mdp.n_sim_per_action_), sizeof(mdp.n_sim_per_action_));
    
    os << mdp.values_;
    os << mdp.rewards_;    
    
    int n_vertex = mdp.graph_.size();
    os.write(reinterpret_cast<const char*>(&n_vertex),sizeof(n_vertex));
    for( int v=0; v<n_vertex; v++ )
    {
        int n_edge = mdp.graph_[v].size();
        os.write(reinterpret_cast<const char*>(&n_edge),sizeof(n_edge));
        for( int e=0; e<n_edge; e++ )
        {
            const MDP::EdgeProb &ep = mdp.graph_[v][e];
            os.write(reinterpret_cast<const char*>(&ep.action),sizeof(ep.action));
            int n_prob = ep.idxNprobs.size();
            os.write(reinterpret_cast<const char*>(&n_prob),sizeof(n_prob));
            for( int p=0; p<n_prob; p++ )
            {
                unsigned long int idx = ep.idxNprobs[p].first;
                double prob = ep.idxNprobs[p].second;                
                os.write(reinterpret_cast<const char*>(&idx),sizeof(idx));
                os.write(reinterpret_cast<const char*>(&prob),sizeof(prob));
            }
        }        
    }

    ompl::base::StateStorage ss(mdp.si_->getStateSpace());
    for( int i=0; i<mdp.states_.size(); i++ )
    {
        ss.addState(mdp.states_[i]);
    }
    ss.store(os);
}

std::istream& operator>> (std::istream& is, MDP& mdp)
{
    is.read(reinterpret_cast<char*>(&mdp.lo_spatial_x_),sizeof(mdp.lo_spatial_x_));
    is.read(reinterpret_cast<char*>(&mdp.hi_spatial_x_),sizeof(mdp.hi_spatial_x_));
    is.read(reinterpret_cast<char*>(&mdp.lo_spatial_y_),sizeof(mdp.lo_spatial_y_));
    is.read(reinterpret_cast<char*>(&mdp.hi_spatial_y_),sizeof(mdp.hi_spatial_y_));    
    is.read(reinterpret_cast<char*>(&mdp.lo_angular_),   sizeof(mdp.lo_angular_));
    is.read(reinterpret_cast<char*>(&mdp.hi_angular_),   sizeof(mdp.hi_angular_));

    is.read(reinterpret_cast<char*>(&mdp.spatial_size_x_),sizeof(mdp.spatial_size_x_));
    is.read(reinterpret_cast<char*>(&mdp.spatial_size_y_),sizeof(mdp.spatial_size_y_));
    is.read(reinterpret_cast<char*>(&mdp.angular_size_),  sizeof(mdp.angular_size_));
    is.read(reinterpret_cast<char*>(&mdp.spatial_unit_),  sizeof(mdp.spatial_unit_));
    is.read(reinterpret_cast<char*>(&mdp.angular_unit_),  sizeof(mdp.angular_unit_));

    is.read(reinterpret_cast<char*>(&mdp.x_offset_),      sizeof(mdp.x_offset_));

    int n_objects;    
    is.read(reinterpret_cast<char*>(&n_objects),sizeof(n_objects));    

    mdp.objects_.resize(n_objects);
    for( int i=0; i<n_objects; i++ )
    {
        is >> mdp.objects_[i];
    }    

    is >> mdp.actions_;

    is.read(reinterpret_cast<char*>(&mdp.discount_),        sizeof(mdp.discount_));
    is.read(reinterpret_cast<char*>(&mdp.n_sim_per_action_),sizeof(mdp.n_sim_per_action_));
    
    is >> mdp.values_;
    is >> mdp.rewards_;

    int n_vertex;
    is.read(reinterpret_cast<char*>(&n_vertex),sizeof(n_vertex));

    mdp.graph_.resize(n_vertex);
    for( int v=0; v<n_vertex; v++ )
    {
        int n_edge;
        is.read(reinterpret_cast<char*>(&n_edge),sizeof(n_edge));

        mdp.graph_[v].resize(n_edge);
        for( int e=0; e<n_edge; e++ )
        {
            MDP::EdgeProb ep;
            is.read(reinterpret_cast<char*>(&ep.action),sizeof(ep.action));
            int n_prob;
            is.read(reinterpret_cast<char*>(&n_prob),sizeof(n_prob));

            ep.idxNprobs.resize(n_prob);
            for( int p=0; p<n_prob; p++ )
            {
                unsigned long int idx;
                double prob;
                is.read(reinterpret_cast<char*>(&idx),sizeof(idx));
                is.read(reinterpret_cast<char*>(&prob),sizeof(prob));

                ep.idxNprobs[p].first = idx;
                ep.idxNprobs[p].second = prob;
            }
            mdp.graph_[v].push_back(ep);
        }        
    }

    mdp.setup();
    
    ompl::base::StateStorage ss(mdp.si_->getStateSpace());
    ss.load(is);
    const vector<const ompl::base::State*> &states = ss.getStates();
    for( int i=0; i<states.size(); i++ )
    {
        ompl::base::State* state = mdp.si_->allocState();
        mdp.si_->copyState(state, states[i]);
        
        MDP::Vertex* v = new MDP::Vertex;
        v->idx = mdp.states_.size();
        v->state = state;
        mdp.nn_->add(v);
        mdp.states_.push_back(state);
    }
}

std::ostream& operator<< (std::ostream& os, const MDP_Rotator &rot)
{
    os << *(MDP*)&rot;
    os.write(reinterpret_cast<const char*>(&rot.yaw_),      sizeof(rot.yaw_));
    os.write(reinterpret_cast<const char*>(&rot.yaw_base_), sizeof(rot.yaw_base_));
}

std::istream& operator>> (std::istream& is, MDP_Rotator &rot)
{
    is >> *(MDP*)&rot;
    is.read(reinterpret_cast<char*>(&rot.yaw_),      sizeof(rot.yaw_));
    is.read(reinterpret_cast<char*>(&rot.yaw_base_), sizeof(rot.yaw_base_));
}

std::ostream& operator<< (std::ostream& os, const SoapRotators::DOCKING_SIDE &ds)
{
    if(      ds == SoapRotators::DOCKING_0 )   os << "DOCKING_0";
    else if( ds == SoapRotators::DOCKING_90 )  os << "DOCKING_90";
    else if( ds == SoapRotators::DOCKING_180 ) os << "DOCKING_180";
    else if( ds == SoapRotators::DOCKING_270 ) os << "DOCKING_270";
    else                                       os << "DOCKING_UNKNOWN:" << ((int)ds);
}