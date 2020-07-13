#include <algorithm>
#include <sstream>

#include "planner.hpp"
#include "kinodynamicRRT.hpp"

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>
#define LOG BOOST_LOG_TRIVIAL(trace)

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;


static double compute_cost(const ompl::base::State* state1, const ompl::base::State* state2)
{
    int o_1 = STATE_ROBOT(state1);
    int o_2 = STATE_ROBOT(state2);

    if( o_1 == o_2 )
    {
        const ObjectState* obj_1 = STATE_OBJECT(state1,o_1);
        const ObjectState* obj_2 = STATE_OBJECT(state2,o_2);

        double dist = sqrt( (obj_2->getX()-obj_1->getX())*
                            (obj_2->getX()-obj_1->getX())+
                            (obj_2->getY()-obj_1->getY())*
                            (obj_2->getY()-obj_1->getY())  );
        return dist;
    }
    else
    {
        const ob::SE2StateSpace::StateType *end_1   = STATE_OBJECT(state1,o_1);
        const ObjectStateSpace::StateType *obj_prev = STATE_OBJECT(state1,o_2);
        const ObjectStateSpace::StateType *obj_post = STATE_OBJECT(state2,o_2);

        double dist1 = sqrt( (obj_prev->getX()-end_1->getX())*
                             (obj_prev->getX()-end_1->getX())+
                             (obj_prev->getY()-end_1->getY())*
                             (obj_prev->getY()-end_1->getY())  );

        double dist2 = sqrt( (obj_post->getX()-obj_prev->getX())*
                             (obj_post->getX()-obj_prev->getX())+
                             (obj_post->getY()-obj_prev->getY())*
                             (obj_post->getY()-obj_prev->getY())  );
                
        return dist1 + dist2;
    }
}


static void drawSoap(cv::Mat& img, double x, double y, double yaw, cv::Scalar color, double alpha=1.0, bool fill=false)
{
    cv::Point point(500 - y*500, 500 - x*500);

    cv::Size rectangleSize(0.66 * 50,0.96 * 50);
    cv::RotatedRect rotatedRectangle(point, rectangleSize, -yaw * 180.0 / M_PI);
    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    cv::Mat tmp = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
    if( fill )
    {
        cv::Point vertices[4];    
        for(int i = 0; i < 4; ++i)
        {
            vertices[i] = vertices2f[i];
        }
        cv::fillConvexPoly( tmp, vertices, 4, color );
    }
    else
    {
        for(int i = 0; i < 4; ++i)
        {
            cv::line(tmp, vertices2f[i], vertices2f[(i+1)%4], color);
        }
    }
    cv::addWeighted(tmp,alpha,img,1.0, 0.0,img);    
}

class MyIterationTerminationCondition
{
public:
    MyIterationTerminationCondition(unsigned int numIterations)
     : maxCalls_(numIterations), timesCalled_(0u)
    {
    }

    bool eval()
    {
        ++timesCalled_;
        return (timesCalled_ > maxCalls_);
    }

    void reset()
    {
        timesCalled_ = 0u;
    }

    operator ompl::base::PlannerTerminationCondition()
    {
        return ompl::base::PlannerTerminationCondition([this]
                                                       {
                                                           return eval();
                                                       });
    }

    unsigned int getTimesCalled() const
    {
        return timesCalled_;
    }

private:
    unsigned int maxCalls_;
    unsigned int timesCalled_;
};

class MyLazyPRMstar : public og::LazyPRMstar
{
public:
    MyLazyPRMstar(const ob::SpaceInformationPtr &si)
     : og::LazyPRMstar(si)
    {
    }

    ~MyLazyPRMstar(){}

    void resetValidity()
    {        
        std::pair<vertex_iter, vertex_iter> vp;
        for(vp = boost::vertices(g_); vp.first != vp.second; ++vp.first)
        {
            vertexValidityProperty_[*vp.first] = VALIDITY_UNKNOWN;
        }

        boost::graph_traits<Graph>::edge_iterator ei, ei_end;
        for( boost::tie(ei, ei_end) = edges(g_); ei != ei_end; ++ei)
        {
            edgeValidityProperty_[*ei] = VALIDITY_UNKNOWN;
        }
    }

    void visualize()
    {
        const int width = 1000;
        const int width_half = width * 0.5;
        const int height = 500;
        const double factor = height;

        cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);

        std::pair<vertex_iter, vertex_iter> vp;
        for(vp = boost::vertices(g_); vp.first != vp.second; ++vp.first)
        {
            ObjectState* state = stateProperty_[*vp.first]->as<ObjectState>();
            if( si_->isValid(state) )
            {
                int r = height     - state->getX()*factor;
                int c = width_half - state->getY()*factor;

                img.at<cv::Vec3b>(r,c)[0] = 0;
                img.at<cv::Vec3b>(r,c)[1] = 255;
                img.at<cv::Vec3b>(r,c)[2] = 255;
            }
        }

        boost::graph_traits<Graph>::edge_iterator ei, ei_end;
        for( boost::tie(ei, ei_end) = edges(g_); ei != ei_end; ++ei)
        {
            const Vertex v1 = boost::source(*ei, g_);
            const Vertex v2 = boost::target(*ei, g_);
            ObjectState* s1 = stateProperty_[v1]->as<ObjectState>();
            ObjectState* s2 = stateProperty_[v2]->as<ObjectState>();

            int r1 = height     - s1->getX()*factor;
            int c1 = width_half - s1->getY()*factor;
            int r2 = height     - s2->getX()*factor;
            int c2 = width_half - s2->getY()*factor;

            cv::Point pt1(c1,r1), pt2(c2,r2);
            if( edgeValidityProperty_[*ei] )
            {
                line(img, pt1, pt2, cv::Scalar(0,0,255) );
            }
        }

        cv::imshow("prm",img);
        cv::waitKey();
    }

private:
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
};

Planner::Planner(RobotObjectSetup &env)
 : env_(env),
   objects_(env.getObjects()),
   soap_rotator_(NULL),
   thresh_goal(0.005),
   use_kino(false)
{
    doAction = NULL;

    solve_time_ = 10.0;
    n_objs_ = env.getObjectCount();

    si_single_ = env.getSingleSpaceInformation();
    si_single4all_ = env.getSingleForAllSpaceInformation();
    si_single4clear_ = env.getSingleForAllSpaceInformation();
    si_all4all_ = env.getAllForAllSpaceInformation();

    env.setup();

    opt_inf = ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si_single4all_));
    opt_inf->setCostThreshold(opt_inf->infiniteCost());    
    
    sim_.SetGravity(btVector3(0,0,-0.1));
    sim_.AddPlaneShape(btVector4(0,0,1,0));
    sim_idxes_.resize(n_objs_);
    for( int o=0; o<n_objs_; o++ )
    {            
        sim_idxes_[o] = sim_.AddCollisionShape( objects_[o].shape, 
                                                btVector3(0,0,0),
                                                btQuaternion(0,0,0,1), 
                                                objects_[o].mass, 1.0, false);
    }

    vector<double> workspace = env_.getWorkspace();
    if( workspace.size() > 0 )
    {
        vector<float> workspacef(workspace.begin(), workspace.end());
        workspacef[7] -= 0.02;
        workspacef[9] -= 0.02;
        workspacef[2] = workspacef[8]*0.5;
        sim_.AddBucketShape(workspacef,0.01,0,0);
    }

    colors_ = cv::Mat(n_objs_,1,CV_8UC3);   
    for( int o=0; o<n_objs_; o++ )
    {
        colors_.at<cv::Vec3b>(o,0)[0] = o * 180.0 / n_objs_;
        colors_.at<cv::Vec3b>(o,0)[1] = 255;
        colors_.at<cv::Vec3b>(o,0)[2] = 255;
    }
    cv::cvtColor(colors_, colors_, cv::COLOR_HSV2RGB);

#if DEBUG
    //sim_.SpinInit();
    //sim_.ResetCamera( 1,-90,80, 0.0,0.0,0 );

    simgui_.SetGravity(btVector3(0,0,-0.1));

    simgui_.AddPlaneShape(btVector4(0,0,1,-0.1));
    simgui_.AddColor(btVector4( 0.3, 0.3, 0.3, 1.0 ));

    table_shape = new btBoxShape(btVector3(0.5,1.0,0.5));
    simgui_.AddCollisionShape( table_shape, 
                               btVector3(0.5,0,-0.5),
                               btQuaternion(0,0,0,1), 
                               0, 1.0, false);

    simgui_.AddColor(btVector4( 255/255., 199/255., 117/255., 1.0 ));
    simgui_idxes_.resize(n_objs_);    
    for( int o=0; o<n_objs_; o++ )
    {            
        simgui_idxes_[o] = simgui_.AddCollisionShape( objects_[o].shape, 
                                                      btVector3(0,0,0),
                                                      btQuaternion(0,0,0,1), 
                                                      objects_[o].mass, 1.0, false);
        simgui_.AddColor(btVector4( colors_.at<cv::Vec3b>(o,0)[0] / 255., 
                                    colors_.at<cv::Vec3b>(o,0)[1] / 255., 
                                    colors_.at<cv::Vec3b>(o,0)[2] / 255., 1.0 ));
    }
    

    //simgui_stick_ = new btCylinderShapeZ(btVector3(0.005, 0.005, 0.15));
    simgui_stick_ = new btCylinderShapeZ(btVector3(0.01, 0.01, 0.15));
    simgui_stick_idx_
     = simgui_.AddCollisionShape( simgui_stick_, 
                                  btVector3(0,0,0), btQuaternion(0,0,0,1), 
                                  0, 1.0, false );
    simgui_.AddColor(btVector4(1,1,1,1));

    if( workspace.size() > 0 )
    {
        vector<float> workspacef(workspace.begin(), workspace.end());        
        workspacef[7] -= 0.02;
        workspacef[9] -= 0.02;
        workspacef[2] = workspacef[8]*0.5;
        simgui_.AddBucketShape(workspacef,0.01,0,0);
    }
    simgui_.AddColor(btVector4(0.80,0.52,0,1));

#endif

}

Planner::~Planner()
{
    if( soap_rotator_ ) delete soap_rotator_;

    delete table_shape;

#if DEBUG
    delete simgui_stick_;
    sim_.SpinExit();
#endif
}

#if DEBUG
void Planner::simulate_gui(const og::PathGeometric &path, const string &fp_record)
{
    bool record = fp_record.compare("")!=0;

    simgui_.SpinInit();
    simgui_.ResetCamera( 1,90,70, 0.65,0.0,0 );

    int r = 0;
    int o_prev = -1;
    int len_path = path.getStateCount();
    for( int p=1; p<len_path; p++ )
    {
        const ob::State* state_prev = path.getState(p-1);
        const ob::State* state_curr = path.getState(p  );

        for( int o=1; o<=n_objs_; o++ )
        {
            const ObjectState* state_obj = STATE_OBJECT(state_prev,o);

            btVector3 T(state_obj->getX(), state_obj->getY(), objects_[o-1].z_offset );
            btQuaternion q;
            q.setEulerZYX(state_obj->getYaw(),0,0);
            q = q * objects_[o-1].q_offset;

            simgui_.SetObjectPose(simgui_idxes_[o-1], q, T);
        }

        int o = STATE_ROBOT(state_prev);
        int idx_sim = simgui_idxes_[o-1];

        const ObjectState* state_obj_prev = STATE_OBJECT(state_prev,o);
        const ObjectState* state_obj_curr = STATE_OBJECT(state_curr,o);

        if( o_prev>0 && o_prev!=o )
        {
            // transition
            const ObjectState* state_obj_tmp = STATE_OBJECT(state_prev,o_prev);

            double x_delta = state_obj_prev->getX() - state_obj_tmp->getX();
            double y_delta = state_obj_prev->getY() - state_obj_tmp->getY();

            if( record )
            {
                char tmp[256];
                sprintf(tmp,"%s/%04d_%%04d.png",fp_record.c_str(),r);
                simgui_.MoveRotateObjectRecord(simgui_stick_idx_, btVector3(x_delta,y_delta,0), 0, simgui_stick_idx_, string(tmp));
                r++;
            }            
        }

        double x_delta   = state_obj_curr->getX() - state_obj_prev->getX();
        double y_delta   = state_obj_curr->getY() - state_obj_prev->getY();
        double yaw_delta = distance_angle(state_obj_curr->getYaw(),state_obj_prev->getYaw());

        if( record )
        {
            char tmp[256];
            sprintf(tmp,"%s/%04d_%%04d.png",fp_record.c_str(),r);
            simgui_.MoveRotateObjectRecord(idx_sim, btVector3(x_delta,y_delta,0), yaw_delta, simgui_stick_idx_, string(tmp));
            r++;
        }
        else
        {
            simgui_.MoveRotateObject(idx_sim, btVector3(x_delta,y_delta,0), yaw_delta);
        }        
    }

    simgui_.Spin(0.1);
}

void Planner::simulate_gui( const ompl::geometric::PathGeometric &path,
                            const std::vector<Action> &actions )
{
    simgui_.SpinInit();
    simgui_.ResetCamera( 1,-90,80, 0.0,0.0,0 );

    for( int o=1; o<=n_objs_; o++ )
    {
        const ObjectState* state_obj = STATE_OBJECT(path.getState(0),o);

        btVector3 T(state_obj->getX(), state_obj->getY(), objects_[o-1].z_offset );
        btQuaternion q;
        q.setEulerZYX(state_obj->getYaw(),0,0);
        q = q * objects_[o-1].q_offset;

        simgui_.SetObjectPose(simgui_idxes_[o-1], q, T);
    }

    int len_actions = actions.size();
    for( int a=1; a<len_actions; a++ )
    {        
        if( actions[a].type == ACTION_TRANSFER )
        {
            int idx = simgui_idxes_[actions[a].idx_target-1];

            btVector3 T;
            btQuaternion q;
            simgui_.GetObjectPose(idx, q, T);
            q = q * objects_[actions[a].idx_target-1].q_offset.inverse();
            btScalar yaw, pitch, roll;
            q.getEulerZYX(yaw, pitch, roll);

            double x_delta = actions[a].x - T[0];
            double y_delta = actions[a].y - T[1];
            double yaw_delta = distance_angle(actions[a].yaw,yaw);

            simgui_.MoveRotateObject(idx, btVector3(x_delta,y_delta,0), yaw_delta);
        }
        else if( actions[a].type == ACTION_PUSHING )
        {
            int o_pusher = actions[a].idx_target;
            int o_target = actions[a].idx_target2;
            int idx_pusher = simgui_idxes_[o_pusher-1];
            int idx_target = simgui_idxes_[o_target-1];

            btVector3 T;
            btQuaternion q;
            btScalar yaw, pitch, roll;
            simgui_.GetObjectPose(idx_pusher, q, T);
            q = q * objects_[actions[a].idx_target-1].q_offset.inverse();            
            q.getEulerZYX(yaw, pitch, roll);

            double x_delta = actions[a].x - T[0];
            double y_delta = actions[a].y - T[1];
            double yaw_delta = distance_angle(actions[a].yaw,yaw);

            simgui_.MoveRotateObject(idx_pusher, btVector3(x_delta,y_delta,0), yaw_delta);

            simgui_.GetObjectPose(idx_target, q, T);
            q = q * objects_[actions[a].idx_target-1].q_offset.inverse();            
            q.getEulerZYX(yaw, pitch, roll);

            const ObjectState* state_target = STATE_OBJECT(path.getState(a),o_target);
            double dist = sqrt((T[0]-state_target->getX())*
                               (T[0]-state_target->getX())+
                               (T[1]-state_target->getY())*
                               (T[1]-state_target->getY()));
            if( dist > 0.02 )
            {
                cout << "out of track - re-planning..." << endl;

                og::PathGeometric path_psh(si_single4all_);
                while( true )
                {
                    if( actions[a].type != ACTION_PUSHING ) break;
                    path_psh.append(STATE_OBJECT(path.getState(a),o_target));
                    a++;
                }

                ob::State* state_curr = si_all4all_->allocState();
                for( int o=1; o<=n_objs_; o++ )
                {
                    ObjectState* state_obj = STATE_OBJECT(state_curr,o);

                    btVector3 T;
                    btQuaternion q;
                    btScalar yaw, pitch, roll;
                    simgui_.GetObjectPose(simgui_idxes_[o-1], q, T);
                    q = q * objects_[o-1].q_offset.inverse();            
                    q.getEulerZYX(yaw, pitch, roll);

                    state_obj->setX(T[0]);
                    state_obj->setY(T[1]);
                    state_obj->setYaw(yaw);
                }

                og::PathGeometric path_pushing(si_all4all_);                
                plan_pushing(state_curr, o_pusher, o_target, path_psh, path_pushing, true);

                si_all4all_->freeState(state_curr);

                cout << "out of track - re-planning... done" << endl;
            }
        }
    }

    simgui_.Spin(0.1);
}

void Planner::simulate_gui( int o_obj, 
                            const og::PathGeometric &path_obj,
                            ob::State* state_res )
{
    int idx_sim = simgui_idxes_[o_obj-1];
    int len_path = path_obj.getStateCount();    
    for( int p=1; p<len_path; p++ )
    {
        const ObjectState* state_obj_curr = path_obj.getState(p)->as<ObjectState>();

        btVector3 T;
        btQuaternion q;
        btScalar yaw, pitch, roll;
        simgui_.GetObjectPose(idx_sim, q, T);
        q = q * objects_[o_obj-1].q_offset.inverse();
        q.getEulerZYX(yaw, pitch, roll);

        double x_delta = state_obj_curr->getX() - T[0];
        double y_delta = state_obj_curr->getY() - T[1];
        double yaw_delta = distance_angle(state_obj_curr->getYaw(),yaw);

        simgui_.MoveRotateObject(idx_sim, btVector3(x_delta,y_delta,0), yaw_delta);
    }
    
    for( int o=1; o<=n_objs_; o++ )
    {
        ObjectState* state_obj = STATE_OBJECT(state_res,o);

        btVector3 T;
        btQuaternion q;
        btScalar yaw, pitch, roll;

        simgui_.GetObjectPose(simgui_idxes_[o-1], q, T);

        q = q * objects_[o-1].q_offset.inverse();        
        q.getEulerZYX(yaw, pitch, roll);

        state_obj->setX(T[0]);
        state_obj->setY(T[1]);
        state_obj->setYaw(yaw);
    }
}
#endif

bool Planner::simulate( const ob::State* state, int o_obj, 
                        const og::PathGeometric &path_obj,
                        vector<ob::State*> &states_res       )
{
    for( int o=1; o<=n_objs_; o++ )
    {
        const ObjectState* state_obj = STATE_OBJECT(state,o);

        btVector3 T(state_obj->getX(), state_obj->getY(), objects_[o-1].z_offset );
        btQuaternion q;
        q.setEulerZYX(state_obj->getYaw(),0,0);
        q = q * objects_[o-1].q_offset;

        sim_.SetObjectPose(sim_idxes_[o-1], q, T);
    }

    ob::State* state_prev = si_all4all_->allocState();
    si_all4all_->copyState(state_prev,state);
    ObjectState* state_obj_prev = STATE_OBJECT(state_prev,o_obj);

    int idx_sim = sim_idxes_[o_obj-1];
    int len_path = path_obj.getStateCount();
    for( int p=1; p<len_path; p++ )
    {        
        const ObjectState* state_obj_curr = path_obj.getState(p)->as<ObjectState>();

        double x_delta   = state_obj_curr->getX() - state_obj_prev->getX();
        double y_delta   = state_obj_curr->getY() - state_obj_prev->getY();
        double yaw_delta = distance_angle(state_obj_curr->getYaw(),state_obj_prev->getYaw());
        
        sim_.MoveRotateObject(idx_sim, btVector3(x_delta,y_delta,0), yaw_delta);

        for( int o=1; o<=n_objs_; o++ )
        {
            ObjectState* state_obj = STATE_OBJECT(state_prev,o);

            btVector3 T;
            btQuaternion q;
            btScalar yaw, pitch, roll;

            sim_.GetObjectPose(sim_idxes_[o-1], q, T);

            q = q * objects_[o-1].q_offset.inverse();            
            q.getEulerZYX(yaw, pitch, roll);

            if( env_.isValid(T[0],T[1],yaw)==false )
            {
/*
{
    std::cout << "p: " << p << endl;
    std::cout << T[0] << ", " << T[1] << yaw << std::endl;
    cv::Mat img = cv::Mat::zeros(500,1000,CV_8UC3);
    env_.visualizeSetup(img);
    for( int z=0; z<states_res.size(); z++ )
    {
        visualizeState(img,states_res[z]);
    }
    drawSoap(img,T[0],T[1],yaw,cv::Scalar(255,255,255));
    cv::imshow("debug",img);
    cv::waitKey();
}
*/
                return false;
            }

            state_obj->setX(T[0]);
            state_obj->setY(T[1]);
            state_obj->setYaw(yaw);
        }

        ompl::base::State* state_add = si_all4all_->allocState();
        si_all4all_->copyState(state_add,state_prev);
        states_res.push_back(state_add);
    }

    si_all4all_->freeState(state_prev);

    return true;
}

bool Planner::simulate( const ob::State* state, int idx, 
                        const vector<btVector3> &pos_delta, 
                        const vector<double> &yaw_delta,
                        vector<ob::State*> &states_res       )
{
    for( int o=1; o<=n_objs_; o++ )
    {
        const ObjectState* state_obj = STATE_OBJECT(state,o);

        btVector3 T(state_obj->getX(), state_obj->getY(), objects_[o-1].z_offset );
        btQuaternion q;
        q.setEulerZYX(state_obj->getYaw(),0,0);
        q = q * objects_[o-1].q_offset;

        sim_.SetObjectPose(sim_idxes_[o-1], q, T);    
    }

    int idx_sim = sim_idxes_[idx-1];
    int len_path = pos_delta.size();
    for( int p=0; p<len_path; p++ )
    {
        bool res = sim_.MoveRotateObject(idx_sim, pos_delta[p], yaw_delta[p]);
        if( res==false ) 
        {
            cout << "collide!!" << endl;
            return false;
        }

        for( int o=1; o<=n_objs_; o++ )
        {
            const ObjectState* state_prv;
            if( p==0 )   state_prv = STATE_OBJECT(state,o);
            else         state_prv = STATE_OBJECT(states_res[p-1],o);
            ObjectState* state_obj = STATE_OBJECT(states_res[p  ],o);

            btVector3 T;
            btQuaternion q;
            sim_.GetObjectPose(sim_idxes_[o-1], q, T);

            q = q * objects_[o-1].q_offset.inverse();
            btScalar yaw, pitch, roll;
            q.getEulerZYX(yaw, pitch, roll);

            if( env_.isValid(T[0],T[1],yaw)==false )
            {
                cout << "invalid push!! object: " << o << "(" << T[0] << "," << T[1] << ")" << endl;
                return false;
            } 
           
            //cout << "yaw: " << yaw << ", pitch: " << pitch << ", roll: " << roll << endl;

            state_obj->setX(T[0]);
            state_obj->setY(T[1]);
            state_obj->setYaw(yaw);
        }
    }

    return true;
}

bool Planner::isValidPath( ompl::base::State* state, int o_target,
                           const ompl::geometric::PathGeometric &path,
                           std::vector<ompl::base::State*> &states_res,
                           const bool do_action )
{
    if( do_action==false )
    {
        return simulate(state, o_target, path, states_res);
    }
    else
    {
        std::vector<ompl::base::State*> states_tmp;
        bool res = simulate(state, o_target, path, states_tmp);
        for( int i=0; i<states_tmp.size(); i++ ) si_all4all_->freeState(states_tmp[i]);
        
        if( res )
        {
            if( doAction )
            {
                doAction(state, o_target, path, states_res);
            }
            else
            {
                ob::State* state_res = si_all4all_->allocState();
#if DEBUG
                simulate_gui(o_target, path, state_res);
                states_res.push_back(state_res);
#else                
                states_res.push_back(states_tmp.back());                
#endif
                
            }
            return true;
        }
        else
        {
            return false;
        }
    }
}

double Planner::compute_cost( const ompl::geometric::PathGeometric &path )
{
    double cost = 0;
    for( int i=1; i<path.getStateCount(); i++ )
    {
        const ob::State* state_prev = path.getState(i-1);
        const ob::State* state_curr = path.getState(i  );

        int o_prev = STATE_ROBOT(state_prev);
        int o_curr = STATE_ROBOT(state_curr);

        if( o_prev==o_curr )
        {
            const ObjectState* obj_prev = STATE_OBJECT(state_prev,o_prev);
            const ObjectState* obj_curr = STATE_OBJECT(state_curr,o_curr);
            double dist = sqrt( (obj_curr->getX()-obj_prev->getX())*
                                (obj_curr->getX()-obj_prev->getX())+
                                (obj_curr->getY()-obj_prev->getY())*
                                (obj_curr->getY()-obj_prev->getY()) );
            cost += dist;
        }
        else
        {            
            const ObjectState* obj_prev  = STATE_OBJECT(state_prev,o_prev);
            const ObjectState* obj_prev2 = STATE_OBJECT(state_prev,o_curr);
            const ObjectState* obj_curr  = STATE_OBJECT(state_curr,o_curr);
            double dist1 = sqrt( (obj_prev2->getX()-obj_prev->getX())*
                                 (obj_prev2->getX()-obj_prev->getX())+
                                 (obj_prev2->getY()-obj_prev->getY())*
                                 (obj_prev2->getY()-obj_prev->getY()) );
            double dist2 = sqrt( (obj_curr->getX()-obj_prev2->getX())*
                                 (obj_curr->getX()-obj_prev2->getX())+
                                 (obj_curr->getY()-obj_prev2->getY())*
                                 (obj_curr->getY()-obj_prev2->getY()) );
            cost += (dist1 + dist2);
       }
    }
    return cost;
}

void Planner::path2Actions( const ompl::geometric::PathGeometric &path, 
                            std::vector<Action> &actions               )
{
    for( int i=1; i<path.getStateCount(); i++ )
    {
        const ob::State* state_prev = path.getState(i-1);
        const ob::State* state_curr = path.getState(i  );

        int o_prev = STATE_ROBOT(state_prev);
        int o_curr = STATE_ROBOT(state_curr);
        
        const ObjectState* s_prev = STATE_OBJECT(state_prev,o_curr);
        const ObjectState* s_curr = STATE_OBJECT(state_curr,o_curr);        
        
        if( i==1 || o_prev != o_curr )
        {
            Action action;
            action.type = ACTION_TRANSITION;
            action.x   = s_prev->getX();
            action.y   = s_prev->getY();
            action.yaw = s_prev->getYaw();
            action.idx_target = o_curr;
            action.idx_target2 = -1;
            actions.push_back(action);
        }

        Action action;
        action.type = ACTION_TRANSFER;
        action.x   = s_curr->getX();
        action.y   = s_curr->getY();
        action.yaw = s_curr->getYaw();
        action.idx_target = o_curr;
        action.idx_target2 = -1;
        actions.push_back(action);
    }
}

bool Planner::plan_pushing( const ompl::base::State* state, int o_pusher, int o_target,
                            const og::PathGeometric &path_target,
                            og::PathGeometric &path_res,
                            const bool do_action                    )
{
    double len_max = 0.20;
    if( use_kino ) len_max = 0.05;

//    const double len_min = 0.10;
    const double len_min = 0.03;

    og::PathGeometric path_pushing(si_all4all_);

    ompl::base::State* state_curr = si_all4all_->allocState();
    ObjectState* state_target_mid = si_single4all_->allocState()->as<ObjectState>();
    si_all4all_->copyState(state_curr, state);

    const ObjectState* state_goal = path_target.getState(path_target.getStateCount()-1)->as<ObjectState>();
    double x_vec_goal = state_goal->getX() - STATE_OBJECT(state,o_target)->getX();
    double y_vec_goal = state_goal->getY() - STATE_OBJECT(state,o_target)->getY();
    double dist_goal = sqrt(x_vec_goal*x_vec_goal + y_vec_goal*y_vec_goal);
    x_vec_goal /= dist_goal;
    y_vec_goal /= dist_goal;

    bool res = false;
    for( int p=1; p<path_target.getStateCount(); p++ )
    {
        const ObjectState* state_target_curr = STATE_OBJECT(state_curr,o_target);
        const ObjectState* state_target_next = path_target.getState(p)->as<ObjectState>();

        double dist = sqrt( (state_goal->getX()-state_target_curr->getX())*
                            (state_goal->getX()-state_target_curr->getX())+
                            (state_goal->getY()-state_target_curr->getY())*
                            (state_goal->getY()-state_target_curr->getY())  );
        //if( dist < 0.15 )
        if( dist < 0.03 )
        {
            cout << "[plan_pushing] almost there" << endl;
            break;
        }

        double dot_goal = x_vec_goal*(state_goal->getX()-state_target_curr->getX())+
                          y_vec_goal*(state_goal->getY()-state_target_curr->getY());
        if( dot_goal < 0 )
        {            
            cout << "[plan_pushing] already passed - all" << endl;
            res = true;
            break;
        }

        double d = sqrt((state_target_next->getX()-state_target_curr->getX())*
                        (state_target_next->getX()-state_target_curr->getX())+
                        (state_target_next->getY()-state_target_curr->getY())*
                        (state_target_next->getY()-state_target_curr->getY()));

        if( d <= len_min ) continue;
        else if( d > len_max )
        {
            bool stop = false;
            int n_middle = d/len_max + 1;
            for( int m=1; m<=n_middle; m++ )
            {
                si_single4all_->getStateSpace()->interpolate( 
                    state_target_curr,state_target_next,
                    m/(double)n_middle, state_target_mid);

                og::PathGeometric path_tmp(si_all4all_);
                if( use_kino )
                {
                    res = plan_pushing_kino(state_curr, o_pusher, o_target, 
                                            state_target_mid->getX(),
                                            state_target_mid->getY(), path_tmp, &stop);
                }
                else
                {
                    res = plan_pushing(state_curr, o_pusher, o_target, 
                                       state_target_mid->getX(),
                                       state_target_mid->getY(), path_tmp, &stop, do_action);
                }
                
                if( res==false ) break;

                if( path_tmp.getStateCount() > 1 )
                {
                    si_all4all_->copyState(state_curr,path_tmp.getState(path_tmp.getStateCount()-1));                    
                    for( int p=1; p<path_tmp.getStateCount(); p++ ) // skip duplicattion
                    {
                        path_pushing.append(path_tmp.getState(p));
                    }
                }
                if( stop ) break;
            }
            if( stop ) break;
        }
        else
        {
            bool stop = false;
            og::PathGeometric path_tmp(si_all4all_);
            if( use_kino )
            {
                res = plan_pushing_kino(state_curr, o_pusher, o_target, 
                                        state_target_mid->getX(),
                                        state_target_mid->getY(), path_tmp, &stop);
            }
            else
            {
                res = plan_pushing( state_curr, o_pusher, o_target, 
                                    state_target_next->getX(), 
                                    state_target_next->getY(), path_tmp, &stop, do_action);
            }

            if( res==false ) break;

            if( path_tmp.getStateCount() > 1 )
            {
                si_all4all_->copyState(state_curr,path_tmp.getState(path_tmp.getStateCount()-1));
                for( int p=1; p<path_tmp.getStateCount(); p++ ) // skip duplicattion
                {
                    path_pushing.append(path_tmp.getState(p));
                }
            }
            if( stop ) break;
        }

/*
if( res )
{
    cv::Mat img = cv::Mat::zeros(500,1000,CV_8UC3);
    visualizePath(img, path_pushing);
    cv::line(img, cv::Point(500-500*STATE_OBJECT(state_curr,o_target)->getY(),
                            500-500*STATE_OBJECT(state_curr,o_target)->getX()),
                  cv::Point(500-500*state_target_next->getY(),
                            500-500*state_target_next->getX()),
                  cv::Scalar(0,255,0));

    for( int p=1; p<path_target.getStateCount(); p++ )
    {
        cv::circle(img,cv::Point(500-500*path_target.getState(p  )->as<ObjectState>()->getY(),
                                 500-500*path_target.getState(p  )->as<ObjectState>()->getX()), 10, cv::Scalar(255,255,255) );
        cv::line(img, cv::Point(500-500*path_target.getState(p-1)->as<ObjectState>()->getY(),
                                500-500*path_target.getState(p-1)->as<ObjectState>()->getX()),
                      cv::Point(500-500*path_target.getState(p  )->as<ObjectState>()->getY(),
                                500-500*path_target.getState(p  )->as<ObjectState>()->getX()),
                      cv::Scalar(255,255,255) );
    }
    cv::imshow("plan_pushing path",img);
    cv::waitKey();
}
*/

        if( res==false ) break;
    }

    if( path_pushing.getStateCount() > 0 )
    {
        si_all4all_->copyState(state_curr, state);
        STATE_ROBOT(state_curr) = o_pusher;
        path_res.append(state_curr);
        path_res.append(path_pushing);
    }

    si_all4all_->freeState(state_curr);
    si_single4all_->freeState(state_target_mid);

    return path_pushing.getStateCount() > 0;
}

bool Planner::plan_pushing_kino( const ompl::base::State* state, 
                                 int o_pusher, int o_target,
                                 double x_goal, double y_goal,
                                 og::PathGeometric &path_res,
                                 bool* stop )
{
    const double len_range = 0.02;

    *stop = false;

    ompl::base::State* state_curr = si_all4all_->allocState();
    ObjectState* state_tmp = si_single4all_->allocState()->as<ObjectState>();
    si_all4all_->copyState(state_curr, state);
    
    RobotObjectSetup::ParamSingleForAll param_org = env_.getParamSingleForAll();
    vector<int> idxes_done = param_org.idxes_obs;
    
    ObjectState* state_target = STATE_OBJECT(state_curr,o_target);
    ObjectState* state_pusher = STATE_OBJECT(state_curr,o_pusher);

    double x_vec_to_go = x_goal - state_target->getX();
    double y_vec_to_go = y_goal - state_target->getY();
    double dist_to_go = sqrt( x_vec_to_go*x_vec_to_go +
                              y_vec_to_go*y_vec_to_go   );
    x_vec_to_go /= dist_to_go;
    y_vec_to_go /= dist_to_go;

    if( len_range >= dist_to_go )
    {
        cout << "[plan_pushing] already got there" << endl;
        // already achieved
        path_res.append(state_curr);
        si_all4all_->freeState(state_curr);
        return true;
    }

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si_all4all_));
    pdef->setStartAndGoalStates(state, state, 0.005);
    pdef->setOptimizationObjective(opt_inf);

    std::vector<KinodynamicRRT::SimShape> simshapes(objects_.size());
    for( int o=0; o<n_objs_; o++)
    {
        simshapes[o].shape = objects_[o].shape;
        simshapes[o].z_offset = objects_[o].z_offset;
        simshapes[o].q_offset = objects_[o].q_offset;
        simshapes[o].mass = objects_[o].mass;
    }

    KinodynamicRRT kino(si_all4all_, 20, n_objs_, o_target, o_pusher);
    kino.setTargetGoalXY(x_goal, y_goal);
    kino.AddShapes(simshapes);
    kino.setRange(0.05);

    kino.setProblemDefinition(pdef);        

cout << "KINO" << endl;

    kino.setup();
    ob::PlannerStatus solved = kino.solve(ob::timedPlannerTerminationCondition(60.0));

cout << "KINO - done" << endl;

    if( solved )
    {

        og::PathGeometric &path = static_cast<og::PathGeometric &>(*pdef->getSolutionPath());
        cout << "len_path: " << path.getStateCount() << endl;
        for( int p=0; p<path.getStateCount(); p++ )
        {
            for( int o=1; o<=n_objs_; o++ )
            {
                cout << STATE_OBJECT(path.getState(p),o)->getX() << ","
                     << STATE_OBJECT(path.getState(p),o)->getY() << ","
                     << STATE_OBJECT(path.getState(p),o)->getYaw() << " | ";
            }
            cout << endl;
        }

        ObjectState* state_target_last = STATE_OBJECT(path.getState(path.getStateCount()-1),o_target);



        double dist = sqrt( (state_target_last->getX()-x_goal)*
                            (state_target_last->getX()-x_goal)+
                            (state_target_last->getY()-y_goal)*
                            (state_target_last->getY()-y_goal) );
        cout << "kino dist : " << dist << endl;


        if( path.getStateCount() > 2 && dist < 0.01 )
        {            
            path_res.append(path);
            return true;
        }
    }

    return false;
}

bool Planner::plan_pushing( const ompl::base::State* state, 
                            int o_pusher, int o_target,
                            double x_goal, double y_goal,
                            og::PathGeometric &path_res,
                            bool* stop,
                            const bool do_action )
{
    const double len_range = 0.02;

    *stop = false;

    ompl::base::State* state_curr = si_all4all_->allocState();
    ObjectState* state_tmp = si_single4all_->allocState()->as<ObjectState>();
    si_all4all_->copyState(state_curr, state);
    
    RobotObjectSetup::ParamSingleForAll param_org = env_.getParamSingleForAll();
    vector<int> idxes_done = param_org.idxes_obs;
    
    ObjectState* state_target = STATE_OBJECT(state_curr,o_target);
    ObjectState* state_pusher = STATE_OBJECT(state_curr,o_pusher);

    double x_vec_to_go = x_goal - state_target->getX();
    double y_vec_to_go = y_goal - state_target->getY();
    double dist_to_go = sqrt( x_vec_to_go*x_vec_to_go +
                              y_vec_to_go*y_vec_to_go   );
    x_vec_to_go /= dist_to_go;
    y_vec_to_go /= dist_to_go;

    if( len_range >= dist_to_go )
    {
        cout << "[plan_pushing] already got there" << endl;
        // already achieved
        path_res.append(state_curr);
        si_all4all_->freeState(state_curr);
        return true;
    }

    ompl::base::SpaceInformationPtr si_rotator(soap_rotator_->getQuerySpaceInformation());

    og::PathGeometric path_pushing(si_all4all_);
    og::PathGeometric path_docking(si_all4all_);
    SoapRotators::DOCKING_SIDE docking_side;
    if( plan_docking( state, o_pusher, o_target, x_goal, y_goal, 
                      path_docking, &docking_side, do_action     )==false )
    {
        cout << "[plan_pushing] failed in docking" << endl;
        si_all4all_->freeState( state_curr );
        return false;
    }
    else
    {
        for( int p=1; p<path_docking.getStateCount(); p++ ) // skip duplicate
        {
            path_pushing.append(path_docking.getState(p));
        }
        si_all4all_->copyState( 
            state_curr, path_docking.getState(path_docking.getStateCount()-1) );
    }

    int n_pushes = 0;
    bool res = false;    
    do
    {
        og::PathGeometric path(si_single4all_);
        path.append(state_pusher);

        double x_target   = state_target->getX();
        double y_target   = state_target->getY();
        double yaw_target = state_target->getYaw();
        double x_pusher   = state_pusher->getX();
        double y_pusher   = state_pusher->getY();
        double yaw_pusher = state_pusher->getYaw();

        dist_to_go = (x_goal - x_target)*x_vec_to_go + 
                     (y_goal - y_target)*y_vec_to_go;
        double x_subgoal_pusher, y_subgoal_pusher;
        if( dist_to_go < 0 )
        {
            cout << "[plan_pushing] passed the goal" << endl;
            // already passed            
            res = true;
            break;
        }
        else
        {
            double x_vec = x_target - x_pusher;
            double y_vec = y_target - y_pusher;
            double dist = sqrt(x_vec*x_vec + y_vec*y_vec);
            x_vec /= dist;
            y_vec /= dist;

            x_subgoal_pusher = x_vec * len_range + x_pusher;
            y_subgoal_pusher = y_vec * len_range + y_pusher;
        }        

        state_tmp->setX(x_subgoal_pusher);
        state_tmp->setY(y_subgoal_pusher);
        state_tmp->setYaw(yaw_pusher);
        path.append(state_tmp);

        vector<ob::State*> states_res;
        res = isValidPath(state_curr, o_pusher, path, states_res, do_action);
        for( int p=0; p<states_res.size(); p++ )
        {            
            for( int d=0; d<idxes_done.size(); d++ )
            {
                ObjectState* state_done     = STATE_OBJECT(state_curr,   idxes_done[d]);
                ObjectState* state_done_res = STATE_OBJECT(states_res[p],idxes_done[d]);

                double dist = sqrt((state_done->getX()-state_done_res->getX())*
                                   (state_done->getX()-state_done_res->getX())+
                                   (state_done->getY()-state_done_res->getY())*
                                   (state_done->getY()-state_done_res->getY()));
                if( dist > 0.05 )
                {
                    *stop =true;
                    res = false;
                    break;
                }
            }

            env_.setParamSingleForAll(o_target, idxes_done, state_curr);
            if( si_single4all_->isValid(STATE_OBJECT(states_res[p],o_target))==false )
            {
                *stop = true;
                res = false;
                break;
            }
            env_.setParamSingleForAll(o_pusher, idxes_done, state_curr);
            if( si_single4all_->isValid(STATE_OBJECT(states_res[p],o_pusher))==false )
            {
                *stop = true;
                res = false;
                break;
            }

            if( *stop==true ) break;
        }

        if( res )
        {
            for( int p=0; p<states_res.size(); p++ )
            {
                STATE_ROBOT(states_res[p]) = o_pusher;
                path_pushing.append(states_res[p]);
            }
            si_all4all_->copyState(state_curr,states_res.back());

            n_pushes++;
        }
        for( int p=0; p<states_res.size(); p++ ) 
        {
            si_all4all_->freeState(states_res[p]);
        }
/*
{
    cv::Mat img = cv::Mat::zeros(500,1000,CV_8UC3);
    visualizePath(img, path_pushing);
    cv::circle(img,cv::Point(500-500*y_goal, 500-500*x_goal),10,cv::Scalar(0,255,0));
    cv::imshow("debug",img);
    cv::waitKey();
}
*/
        cout << "dist_to_go: " << dist_to_go << ", res=" << res << endl;

        if( res==false ) break;
    } while( dist_to_go > len_range );

    env_.setParamSingleForAll(param_org);
    si_single4all_->freeState(state_tmp);
    si_all4all_->freeState(state_curr);
    
    if( n_pushes > 0 )
    {
        path_res.append(path_pushing);
    }

    cout << "[plan_pushing] local planning done: " << (n_pushes > 0) << ", length: " << path_pushing.getStateCount() << endl;

    return (n_pushes > 0);
}

bool Planner::plan_docking( const ompl::base::State* state, 
                            int o_pusher, int o_target,
                            double x_goal, double y_goal,
                            og::PathGeometric &path_res, 
                            SoapRotators::DOCKING_SIDE* docking_side,
                            const bool do_action                      )
{
    assert(soap_rotator_!=NULL && "load/create soap_rotator_ before calling plan_docking");

    og::PathGeometric path_docking(si_all4all_);

    ompl::base::State* state_curr = si_all4all_->allocState();
    si_all4all_->copyState(state_curr, state);

    ompl::base::SpaceInformationPtr si_rotator(soap_rotator_->getQuerySpaceInformation());
    ompl::base::State* state_query = si_rotator->allocState();
    ObjectState* state_target = STATE_OBJECT(state_query,1);
    ObjectState* state_pusher = STATE_OBJECT(state_query,2);

    si_single_->copyState(state_pusher,STATE_OBJECT(state_curr,o_pusher));
    si_single_->copyState(state_target,STATE_OBJECT(state_curr,o_target));
    
    RobotObjectSetup::ParamSingleForAll param_org = env_.getParamSingleForAll();
    vector<int> idxes_done = param_org.idxes_obs;
    idxes_done.push_back(o_target);

    cout << "idxes_done for docking: [";
    for( int i=0; i<idxes_done.size(); i++ )
    {
        cout << idxes_done[i] << ",";
    }
    cout << endl;

    env_.setParamSingleForAll(o_pusher, idxes_done, state_curr);

    std::vector<SoapRotators::DockingPoint> possible_dockings;
    soap_rotator_->getDockingPoints( state_query, x_goal, y_goal, possible_dockings );

    if( si_single4all_->isValid(state_pusher)==false )
    {
        if( env_.validateSingleForAll( state_pusher, o_pusher, idxes_done, 
                                       state_pusher) )
        {
            cout << "[validating] succeed" << endl;
        }
        else
        {
            cout << "[validating] failed" << endl;
            si_rotator->freeState(state_query);
            si_all4all_->freeState(state_curr);
            return false;
        }
    }

    ob::OptimizationObjectivePtr opt(new ob::PathLengthOptimizationObjective(si_single4all_));
    opt->setCostThreshold(opt->infiniteCost());

    // plan to a docking point
    og::RRTstar planner(si_single4all_);
    planner.setRange(0.02);
    planner.setGoalBias(0.2);
    planner.setup();

    bool res = false;
    for( int i=0; i<possible_dockings.size(); i++ )
    {
        const ObjectState* state_docking = STATE_OBJECT(possible_dockings[i].state,2);

        if( env_.isValid(state_docking->getX(),state_docking->getY(),state_docking->getYaw())==false )
        {
            continue;
        }

        ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si_single4all_));
        pdef->setStartAndGoalStates(state_pusher, state_docking, 0.005);
        pdef->setOptimizationObjective(opt);
        
        planner.setProblemDefinition(pdef);        
        ob::PlannerStatus solved = planner.solve(ob::timedPlannerTerminationCondition(10.0));
  
        og::PathGeometric &path = static_cast<og::PathGeometric &>(*pdef->getSolutionPath());
        path.append(state_docking);

        og::PathSimplifier ps(si_single4all_);
        ps.simplifyMax(path);

        vector<MDP::action_t> actions;
        og::PathGeometric path_rot(soap_rotator_->getQuerySpaceInformation());
        soap_rotator_->plan(possible_dockings[i], actions, path_rot);
        for( int p=0; p<path_rot.getStateCount(); p++ )
        {
            ompl::base::State* state_rotate = STATE_OBJECT(path_rot.getState(p),2);
            path.append(state_rotate);
        }        

        vector<ob::State*> states_res;
        res = isValidPath(state_curr, o_pusher, path, states_res, do_action);

        bool stop = false;
        for( int p=0; p<states_res.size(); p++ )
        {
            for( int d=0; d<idxes_done.size(); d++ )
            {
                ObjectState* state_done     = STATE_OBJECT(state_curr,   idxes_done[d]);
                ObjectState* state_done_res = STATE_OBJECT(states_res[p],idxes_done[d]);

                double dist = sqrt((state_done->getX()-state_done_res->getX())*
                                   (state_done->getX()-state_done_res->getX())+
                                   (state_done->getY()-state_done_res->getY())*
                                   (state_done->getY()-state_done_res->getY()));
                if( dist > 0.05 )
                {
                    stop = true;
                    res = false;
                    break;
                }
            }
            if( res == false ) break;

            env_.setParamSingleForAll(o_target, param_org.idxes_obs, state_curr);
            if( si_single4all_->isValid(STATE_OBJECT(states_res[p],o_target))==false )
            {
                stop = true;
                res = false;
                break;
            }
            env_.setParamSingleForAll(o_pusher, param_org.idxes_obs, state_curr);
            if( si_single4all_->isValid(STATE_OBJECT(states_res[p],o_pusher))==false )
            {
                stop = true;
                res = false;
                break;
            }
        }
        if( stop ) break;

        if( res )
        {
            STATE_ROBOT(state_curr) = o_pusher;
            path_docking.append(state_curr);
            for( int p=0; p<states_res.size(); p++ )
            {
                STATE_ROBOT(states_res[p]) = o_pusher;
                path_docking.append(states_res[p]);
            }

            if( docking_side )
            {
                *docking_side = possible_dockings[i].docking_side;
            }

            si_all4all_->copyState(state_curr,states_res.back());            
        }

        for( int p=0; p<states_res.size(); p++ )
        {
            si_all4all_->freeState(states_res[p]);
        }
    
        if( res ) break;
    }

    for( int i=0; i<possible_dockings.size(); i++ )
    {
        si_rotator->freeState(possible_dockings[i].state);
    }
    si_rotator->freeState(state_query);
    si_all4all_->freeState(state_curr);

    env_.setParamSingleForAll(param_org);

    cout << "[docking] planning done: " << res 
         << ", path length: " << path_docking.getStateCount() << endl;

    if( res )
    {
        path_res.append(path_docking);
    }

    return res;
}

#define USE_LAZYPRM 0

void Planner::plan( const ompl::base::State *state_start,
                    const ompl::base::State *state_goal,
                    og::PathGeometric &path_res,
                    std::vector<Action> &actions_res,
                    bool do_merge          )
{
    LOG << "plan started";
    vector<int> idxes_none;
    vector<int> order_objs(n_objs_);
    vector<int> idxes_all(n_objs_);
    for( int o=1; o<=n_objs_; o++ )
    {
        order_objs[o-1] = o;
        idxes_all[o-1] = o;        
    } 

    ob::State* state_curr = si_all4all_->allocState();
    bool succ = true;
    do
    {
        succ = true;
        og::PathGeometric path_tmp(si_all4all_);
        si_all4all_->copyState(state_curr,state_start);

        vector<int> idxes_done;
        for( int i=0; i<n_objs_; i++ )
        {
            //clock_t begin, end;
            //begin = clock();

            // select object
            int o = order_objs[i];

            env_.setParamSingleForAll(o, idxes_done, state_curr);

            // 1) find a selfish path
            const ObjectState* state_0 = STATE_OBJECT(state_curr,o);
            const ObjectState* state_1 = STATE_OBJECT(state_goal,o);

            if( si_single4all_->isValid(state_0)==false )
            {
                if( env_.validateSingleForAll( state_0, o, idxes_done, 
                                               STATE_OBJECT(state_curr,o)) )
                {
                    cout << "[validating] succeed" << endl;
                    STATE_ROBOT(state_curr) = o;
                    path_tmp.append(state_curr);                    
                }
                else
                {
                    cout << "invalid simulation result" << endl;
                    succ = false;
                    break;
                }
            }

            ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si_single4all_));
            pdef->setStartAndGoalStates(state_0, state_1, thresh_goal);
            pdef->setOptimizationObjective(opt_inf);

            ob::PlannerStatus solved;

            og::RRTstar planner(si_single4all_);
            planner.setRange(0.05);
            //planner.setGoalBias(0.5);
            planner.setProblemDefinition(pdef);
            planner.setup();
            solved = planner.solve(ob::timedPlannerTerminationCondition(10.0));

            if( solved ) 
            {
                cout << "[SELFISH] idxes_done: [";
                for( int d=0; d<idxes_done.size(); d++ )
                {
                    cout << idxes_done[d] << ",";
                }
                cout << "]" << endl;                
            }

            //cout << "cost: " << opt_inf->motionCost(state_0, state_1) << endl;
            //end = clock();
            //double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
            //LOG << "RRT Cont FindPath - done";
            //LOG << "Duration Time: " << time_spent;

            og::PathGeometric &path = static_cast<og::PathGeometric &>(*pdef->getSolutionPath());
            double dist = si_single4all_->distance(path.getState(path.getStateCount()-1),state_1);

            //if( solved && dist < thresh_goal )
            if( solved && dist < thresh_goal )
            {
                // Check Planning                
                path.append(state_1); // to make sure

                og::PathSimplifier ps(si_single4all_);
                ps.simplifyMax(path);

                vector<ob::State*> states_res;
                bool res = isValidPath(state_curr, o, path, states_res);
                if( res )
                {
                    for( int d=0; d<idxes_done.size(); d++ )
                    {
                        ObjectState* state_done     = STATE_OBJECT(state_curr,                     idxes_done[d]);
                        ObjectState* state_done_res = STATE_OBJECT(states_res[states_res.size()-1],idxes_done[d]);

                        double dist = sqrt((state_done->getX()-state_done_res->getX())*
                                           (state_done->getX()-state_done_res->getX())+
                                           (state_done->getY()-state_done_res->getY())*
                                           (state_done->getY()-state_done_res->getY()));
                        if( dist > 0.01 )
                        {
                            succ = false;
                            res = false;
                            break;
                        }
                    }

                    if( res )
                    {
                        STATE_ROBOT(state_curr) = o;
                        path_tmp.append(state_curr);
                        for( int p=0; p<states_res.size(); p++ )
                        {
                            STATE_ROBOT(states_res[p]) = o;
                            path_tmp.append(states_res[p]);
                        }
                        si_all4all_->copyState(state_curr,states_res.back());

                        // to make sure
                        si_single4all_->copyState(STATE_OBJECT(state_curr,o),state_1);
                        path_tmp.append(state_curr);
                    }
                }
                else
                {
                    /*
                    cout << "try to avoid others" << endl;

                    env_.setParamSingleForAll(o, idxes_all, state_curr);

                    ob::ProblemDefinitionPtr pdef_avoid(new ob::ProblemDefinition(si_single4all_));
                    pdef_avoid->setStartAndGoalStates(state_0, state_1, thresh_goal);
                    pdef_avoid->setOptimizationObjective(opt_inf);                    

                    og::RRTstar planner_rrt(si_single4all_);
                    planner_rrt.setRange(0.05);
                    planner_rrt.setProblemDefinition(pdef_avoid);
                    planner_rrt.setup();
                    solved = planner_rrt.solve(ob::timedPlannerTerminationCondition(10.0));

                    if( solved )
                    {
                        cout << "[AVOIDING] " << endl;
                        //planner.visualize();
                    }
                    
                    og::PathGeometric &path_avoid = static_cast<og::PathGeometric &>(*pdef_avoid->getSolutionPath());
                    double dist_avoid = si_single4all_->distance(path_avoid.getState(path_avoid.getStateCount()-1),state_1);
                    if( solved && dist_avoid < thresh_goal)
                    {
                        // to make sure
                        path_avoid.append(state_1);

                        og::PathSimplifier ps(si_single4all_);
                        ps.simplifyMax(path_avoid);

                        STATE_ROBOT(state_curr) = o;
                        path_tmp.append(state_curr);
                        for( int p=0; p<path_avoid.getStateCount(); p++ )
                        {                            
                            STATE_ROBOT(state_curr) = o;
                            si_single4all_->copyState(STATE_OBJECT(state_curr,o),path_avoid.getState(p));
                            path_tmp.append(state_curr);
                        }                        
                        res = true;
                    }
                    */
                }

                if( res )
                {
                    STATE_OBJECT(state_curr,o)->setX(  state_1->getX());
                    STATE_OBJECT(state_curr,o)->setY(  state_1->getY());
                    STATE_OBJECT(state_curr,o)->setYaw(state_1->getYaw());
                    
                    idxes_done.push_back(o);
                    cout << "added done objects: [";
                    for( int d=0; d<idxes_done.size(); d++ ) cout << idxes_done[d] << ",";
                    cout << "]" << endl;
                }
                else
                {
                    cout << "failed to find a path (selfish & avoiding)" << endl;
                    succ = false;
                    break;
                }

                for( int p=0; p<states_res.size(); p++ )
                {
                    si_all4all_->freeState(states_res[p]);
                }
            }
            else
            {
                cout << "failed to find a path (selfish)" << endl;                
                succ = false;
                break;
            }
        }

        if( succ==true )
        {
            path_res = path_tmp;
            break;            
        }        

    } while( next_permutation(order_objs.begin(), order_objs.end()) );

LOG << "plan - done";
    if( do_merge )
    {
LOG << "merge path";
/*
cv::Mat img = cv::Mat::zeros(500,1000,CV_8UC3);
env_.visualizeSetup(img);
visualizePath(img,path_res);
cv::imshow("debug",img);
cv::waitKey();
*/
        og::PathGeometric path_merge(si_all4all_);
        vector<Action> actions;
        if( mergePath(path_res, path_merge, actions) )
        {
            double cost_merge = 0;
            for( int p=1; p<path_merge.getStateCount(); p++ )
            {
                const ob::State* state_prev = path_merge.getState(p-1);
                const ob::State* state_curr = path_merge.getState(p);
                
                cost_merge += ::compute_cost(state_prev,state_curr);                
            }

            double cost_selfish = 0;
            for( int p=1; p<path_res.getStateCount(); p++ )
            {
                const ob::State* state_prev = path_res.getState(p-1);
                const ob::State* state_curr = path_res.getState(p);
                
                cost_selfish += ::compute_cost(state_prev,state_curr);                
            }

            //if( cost_selfish > cost_merge )
            if( true )
            {
                path_res = path_merge;
                actions_res = actions;
            }
            else
            {
                path2Actions(path_res,actions_res);
            }
        }
        else
        {
            path2Actions(path_res,actions_res);
        }
LOG << "merge path - done";
    }
    else
    {
        path2Actions(path_res,actions_res);
    }
    si_single4all_->freeState(state_curr);
}

void Planner::plan_plRS( const ompl::base::State *state_start,
                         const ompl::base::State *state_goal,
                         og::PathGeometric &path_res,
                         std::vector<Action> &actions_res )
{
    LOG << "plan started (plRS)";
    vector<int> idxes_all(n_objs_);
    vector<int> order_objs(n_objs_);
    for( int o=1; o<=n_objs_; o++ )
    {
        order_objs[o-1] = o;
        idxes_all[o-1] = o;
    } 

    ob::State* state_curr = si_all4all_->allocState();
    bool succ = true;
    do
    {
        succ = true;
        og::PathGeometric path_tmp(si_all4all_);
        si_all4all_->copyState(state_curr,state_start);

        vector<int> idxes_done;
        for( int i=0; i<n_objs_; i++ )
        {
            // select object
            int o = order_objs[i];

            env_.setParamSingleForAll(o, idxes_done, state_curr);

            // 1) find a selfish path
            const ObjectState* state_0 = STATE_OBJECT(state_curr,o);
            const ObjectState* state_1 = STATE_OBJECT(state_goal,o);

            if( si_single4all_->isValid(state_0)==false )
            {
                if( env_.validateSingleForAll( state_0, o, idxes_done, 
                                               STATE_OBJECT(state_curr,o)) )
                {
                    cout << "[validating] succeed" << endl;
                    STATE_ROBOT(state_curr) = o;
                    path_tmp.append(state_curr);                    
                }
                else
                {
                    cout << "invalid simulation result" << endl;
                    succ = false;
                    break;
                }
            }

            ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si_single4all_));
            pdef->setStartAndGoalStates(state_0, state_1, thresh_goal);
            pdef->setOptimizationObjective(opt_inf);

            ob::PlannerStatus solved;
            og::RRTstar planner(si_single4all_);
            planner.setRange(0.05);
            //planner.setGoalBias(0.5);
            planner.setProblemDefinition(pdef);
            planner.setup();
            solved = planner.solve(ob::timedPlannerTerminationCondition(10.0));

            og::PathGeometric &path = static_cast<og::PathGeometric &>(*pdef->getSolutionPath());
            double dist = si_single4all_->distance(path.getState(path.getStateCount()-1),state_1);

            if( solved && dist < thresh_goal )
            {
                // Check Planning                
                path.append(state_1); // to make sure

                og::PathSimplifier ps(si_single4all_);
                ps.simplifyMax(path);

                // already done
                vector<int> idxes_collide;
                {
                    RobotObjectSetup::ParamSingleForAll param_org = env_.getParamSingleForAll();

                    ompl::base::State* state_midd = si_single4all_->allocState();
                    for( int p=1; p<path.getStateCount(); p++ )
                    {
                        const ObjectState* state_path0 = path.getState(p-1)->as<ObjectState>();
                        const ObjectState* state_path1 = path.getState(p  )->as<ObjectState>();
                        
                        double dist = sqrt( (state_path1->getX()-state_path0->getX())*
                                            (state_path1->getX()-state_path0->getX())+
                                            (state_path1->getY()-state_path0->getY())*
                                            (state_path1->getY()-state_path0->getY())  );

                        int n_middle = ((int)(dist / 0.05)) + 1;
                        for( int m=1; m<=n_middle; m++ )
                        {
                            si_single4all_->getStateSpace()->interpolate( 
                                    state_path0,state_path1, m/(double)n_middle, state_midd);
                            
                            for( int c=1; c<=n_objs_; c++ )
                            {
                                if( c==o ) continue;

                                vector<int> idxes_c(1); idxes_c[0] = c;                                
                                env_.setParamSingleForAll(o,idxes_c,state_curr);

                                if( si_single4all_->isValid(state_midd)==false )
                                {
                                    bool already_have = false;
                                    for( int ii=0; ii<idxes_collide.size(); ii++ )
                                    {
                                        if( c==idxes_collide[ii] ) 
                                        {
                                            already_have = true;
                                            break;
                                        }
                                    }
                                    if( already_have==false )
                                    {
                                        idxes_collide.push_back(c);
                                    }
                                }
                            }
                        }
                    }
                    si_single4all_->freeState(state_midd);                    

                    env_.setParamSingleForAll(param_org);
                }

                if( idxes_collide.size()>0 )
                {
                    ompl::base::StateSamplerPtr ss_single = si_single4clear_->allocStateSampler();
                    vector<pair<int,ObjectState*> > obstacles;
                    for( int oo=1; oo<=n_objs_; oo++ )
                    {
                        obstacles.push_back(make_pair(oo,STATE_OBJECT(state_curr,oo)));
                    }

                    for( int ii=0; ii<idxes_collide.size(); ii++ )
                    {
                        int c = idxes_collide[ii];
                        env_.setParamSingleForClear(c, o,path, obstacles);

                        ObjectState* state_c = STATE_OBJECT(state_curr,c);

                        // reverse search
                        ob::ProblemDefinitionPtr pdef_clear(new ob::ProblemDefinition(si_single4clear_));
                        pdef_clear->setOptimizationObjective(opt_inf);
                        pdef_clear->setGoalState(state_c);
                        ob::State* state_clear = si_single4clear_->allocState();
                        for( int cc=0; cc<100; cc++ )
                        {                            
                            ss_single->sampleUniform(state_clear);
                            pdef_clear->addStartState(state_clear);
                        }
                        si_single4clear_->freeState(state_clear);                        

                        og::RRTstar planner_clear(si_single4clear_);
                        planner_clear.setRange(0.05);
                        //planner.setGoalBias(0.5);
                        planner_clear.setProblemDefinition(pdef_clear);
                        planner_clear.setup();
                        solved = planner_clear.solve(ob::timedPlannerTerminationCondition(10.0));

                        og::PathGeometric &path_clear = static_cast<og::PathGeometric &>(*pdef_clear->getSolutionPath());
                        dist = si_single4clear_->distance(path_clear.getState(path_clear.getStateCount()-1),state_c);
                        if( solved && dist < thresh_goal )
                        {                            
                            og::PathSimplifier ps(si_single4clear_);
                            ps.simplifyMax(path_clear);

                            STATE_ROBOT(state_curr) = c;
                            path_tmp.append(state_curr);
                            for( int p=path_clear.getStateCount()-1; p>=0; p-- )
                            {
                                ObjectState* state_p = path_clear.getState(p)->as<ObjectState>();                        
                                state_c->setX(state_p->getX());
                                state_c->setY(state_p->getY());
                                state_c->setYaw(state_p->getYaw());
                                path_tmp.append(state_curr);
                            }

                            cout << "cleared c=" << c << endl;
                        }
                        else
                        {
                            cout << "failed to find a path (clear c=" << c << ")" << endl;
                            succ = false;
                            break;
                        }
                    }
                }

                STATE_ROBOT(state_curr) = o;
                path_tmp.append(state_curr);
                ObjectState* state_o = STATE_OBJECT(state_curr,o);
                for( int p=1; p<path.getStateCount(); p++ )
                {
                    ObjectState* state_p = path.getState(p)->as<ObjectState>();                        
                    state_o->setX(state_p->getX());
                    state_o->setY(state_p->getY());
                    state_o->setYaw(state_p->getYaw());
                    path_tmp.append(state_curr);
                }

                idxes_done.push_back(o);
                cout << "added done objects: [";
                for( int d=0; d<idxes_done.size(); d++ ) cout << idxes_done[d] << ",";
                cout << "]" << endl;
            }
            else
            {
                cout << "failed to find a path (selfish)" << endl;                
                succ = false;
                break;
            }
        }

        if( succ==true )
        {
            path_res = path_tmp;
            break;            
        }        

    } while( next_permutation(order_objs.begin(), order_objs.end()) );

    path2Actions(path_res,actions_res);

    si_single4all_->freeState(state_curr);
}
/*
void Planner::plan_kino( const ompl::base::State *state_start,
                         const ompl::base::State *state_goal,
                         ompl::geometric::PathGeometric &path_res,
                         std::vector<Action> &actions_res )
{    
    std::vector<KinodynamicRRT::SimShape> simshapes(objects_.size());
    for( int o=0; o<n_objs_; o++)
    {
        simshapes[o].shape = objects_[o].shape;
        simshapes[o].z_offset = objects_[o].z_offset;
        simshapes[o].q_offset = objects_[o].q_offset;
        simshapes[o].mass = objects_[o].mass;
    }

    KinodynamicRRT kinoRRT(si_all4all_, 20, n_objs_, false);
    kinoRRT.AddShapes(simshapes);
    kinoRRT.setRange(0.05);
    kinoRRT.setGoalBias(0.5);

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si_all4all_));
    pdef->setStartAndGoalStates(state_start,state_goal,thresh_goal * n_objs_);
    pdef->setOptimizationObjective(opt_inf);

    kinoRRT.setProblemDefinition(pdef);
    kinoRRT.setGoalState(state_goal);
    kinoRRT.setup();

    ob::PlannerStatus solved = kinoRRT.ob::Planner::solve(70.0 * n_objs_);
    
    path_res = static_cast<og::PathGeometric &>(*pdef->getSolutionPath());
    path2Actions(path_res,actions_res);
}
*/
bool Planner::plan_pushing( const ompl::base::State* state, int o_pusher, int o_target,
                            const og::PathGeometric &path_mrg,
                            const og::PathGeometric &path_psh,
                            const og::PathGeometric &path_sep,
                            const ompl::base::State* state_target_goal,
                            const ompl::base::State* state_pusher_goal,
                            og::PathGeometric &path_res,
                            vector<Action> &actions_res )
{
    RobotObjectSetup::ParamSingleForAll param_org = env_.getParamSingleForAll();
    vector<int> idxes_done = param_org.idxes_obs;

    og::PathGeometric path_target_ready(si_all4all_);
    og::PathGeometric path_pushing(si_all4all_);
    og::PathGeometric path_target_goal(si_all4all_);
    og::PathGeometric path_pusher_goal(si_all4all_);

    ompl::base::State* state_curr = si_all4all_->allocState();
    si_all4all_->copyState(state_curr,state);
    env_.setParamSingleForAll(o_target, idxes_done, state_curr);

    bool res;
    // move the target to the merging point

cout << "path_mrg size: " <<  path_mrg.getStateCount() << endl;

    if( path_mrg.getStateCount() > 0 )
    {
        vector<ob::State*> states_mrg;
        res = isValidPath(state_curr, o_target, path_mrg, states_mrg);

        if( res )
        {
            path_target_ready.append(state_curr);
            for( int p=0; p<states_mrg.size(); p++ )
            {
                STATE_ROBOT(states_mrg[p]) = o_target;
                path_target_ready.append(states_mrg[p]);
            }
        }
        si_all4all_->copyState(state_curr,states_mrg.back());
        for( int p=0; p<states_mrg.size(); p++ ) 
        {
            si_all4all_->freeState(states_mrg[p]);
        }

        if(res==false)
        {
            si_all4all_->freeState(state_curr);
            return false;
        }
    }

    // push the target with the pusher to the separating point
    if( path_psh.getStateCount() > 0 )
    {
        res = plan_pushing(state_curr, o_pusher, o_target, path_psh, path_pushing);
        if(res==false)
        {
            si_all4all_->freeState(state_curr);
            return false;
        }
        si_all4all_->copyState(state_curr,path_pushing.getState(path_pushing.getStateCount()-1));
    }

    // place target object to its goal    

    bool do_target = false;
    {
        ObjectState* state_0 = STATE_OBJECT(state_curr,o_target);

        double dist = sqrt( (state_target_goal->as<ObjectState>()->getX()-state_0->getX())* 
                            (state_target_goal->as<ObjectState>()->getX()-state_0->getX())+
                            (state_target_goal->as<ObjectState>()->getY()-state_0->getY())* 
                            (state_target_goal->as<ObjectState>()->getY()-state_0->getY()) );
        double dist_yaw = distance_angle(state_target_goal->as<ObjectState>()->getYaw(),state_0->getYaw());

        //if( dist > 0.01 || dist_yaw > 0.2 )
        if( dist > 0.0001 || dist_yaw > 0.001 )
        { 
            do_target = true;
            if( si_single4all_->isValid(state_0)==false )
            {
                if( env_.validateSingleForAll( state_0, o_target, idxes_done, 
                                               state_0 ) )
                {
                    cout << "[validating] succeed" << endl;
                    path_target_goal.append(state_curr);
                }
                else
                {
                    cout << "[validating] failed... invalid simulation result" << endl;
                    si_all4all_->freeState(state_curr);
                    return false;
                }
            }

            ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si_single4all_));
            pdef->setStartAndGoalStates(state_0,state_target_goal,thresh_goal);
            pdef->setOptimizationObjective(opt_inf);

            og::RRTstar planner(si_single4all_);
            planner.setRange(0.02);
            planner.setProblemDefinition(pdef);
            planner.setup();

            ob::PlannerStatus solved = planner.solve(ob::timedPlannerTerminationCondition(10.0));
            if(!solved)
            {
                si_all4all_->freeState(state_curr);
                return false;
            }

            og::PathGeometric &path_sep_target
             = static_cast<og::PathGeometric &>(*pdef->getSolutionPath());
            path_sep_target.append(state_target_goal);

            og::PathSimplifier ps(si_single4all_);
            ps.simplifyMax(path_sep_target);

            vector<ob::State*> states_sep_target;
            res = isValidPath(state_curr, o_target, path_sep_target, states_sep_target);
            if( res )
            {
                STATE_ROBOT(state_curr) = o_target;
                path_target_goal.append(state_curr);
                for( int p=0; p<states_sep_target.size(); p++ )
                {
                    STATE_ROBOT(states_sep_target[p]) = o_target;
                    path_target_goal.append(states_sep_target[p]);
                }

                idxes_done.push_back(o_target);
                si_all4all_->copyState(state_curr,states_sep_target.back());
            }
            
            for( int p=0; p<states_sep_target.size(); p++ ) 
            {
                si_all4all_->freeState(states_sep_target[p]);
            }

            if(res==false)
            {
                si_all4all_->freeState(state_curr);
                return false;
            }
        }
    }    

    // place pusher object to its goal    
    bool do_pusher = false;
    {
        env_.setParamSingleForAll(o_pusher, idxes_done, state_curr);
        ObjectState* state_0 = STATE_OBJECT(state_curr,o_pusher);

        double dist = sqrt( (state_pusher_goal->as<ObjectState>()->getX()-state_0->getX())* 
                            (state_pusher_goal->as<ObjectState>()->getX()-state_0->getX())+
                            (state_pusher_goal->as<ObjectState>()->getY()-state_0->getY())* 
                            (state_pusher_goal->as<ObjectState>()->getY()-state_0->getY()) );
        double dist_yaw = distance_angle(state_pusher_goal->as<ObjectState>()->getYaw(),state_0->getYaw());

        //if( dist > 0.01 || dist_yaw > 0.2 )
        //if( dist > 0.005 || dist_yaw > 0.1 )
        if( dist > 0.0001 || dist_yaw > 0.001 )
        { 
            do_pusher = true;
            if( si_single4all_->isValid(state_0)==false )
            {
                cout << "before: " << state_0->getX() << ", "
                                   << state_0->getY() << ", "
                                   << state_0->getYaw() << endl;
    
                if( env_.validateSingleForAll( state_0, o_pusher, idxes_done, 
                                               state_0 ) )
                {
                    cout << "after:  " << state_0->getX() << ", "
                                       << state_0->getY() << ", "
                                       << state_0->getYaw() << endl;
    
                    cout << "[validating] succeed" << endl;
                    path_pusher_goal.append(state_curr);                    
                }
                else
                {
                    cout << "[validating] failed.. invalid simulation result" << endl;
    /*
    cv::Mat img = cv::Mat::zeros(500,1000,CV_8UC3);
    visualizeState(img,state_curr);
    cv::imshow("debug",img);
    cv::waitKey();
    */
                    si_all4all_->freeState(state_curr);
                    return false;
                }
            }
    
            ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si_single4all_));
            pdef->setStartAndGoalStates(STATE_OBJECT(state_curr,o_pusher),state_pusher_goal,thresh_goal);
            pdef->setOptimizationObjective(opt_inf);
    
            og::RRTstar planner(si_single4all_);
            planner.setRange(0.02);
            planner.setProblemDefinition(pdef);
            planner.setup();
    
            ob::PlannerStatus solved = planner.solve(ob::timedPlannerTerminationCondition(10.0));
            if(!solved)
            {
                si_all4all_->freeState(state_curr);
                return false;
            }
    
            og::PathGeometric &path_sep_pusher
             = static_cast<og::PathGeometric &>(*pdef->getSolutionPath());
            path_sep_pusher.append(state_pusher_goal);
    
            og::PathSimplifier ps(si_single4all_);
            ps.simplifyMax(path_sep_pusher);
    
            vector<ob::State*> states_sep_pusher;
            res = isValidPath(state_curr, o_pusher, path_sep_pusher, states_sep_pusher);
            if( res )
            {
                STATE_ROBOT(state_curr) = o_pusher;
                path_pusher_goal.append(state_curr);
                for( int p=0; p<states_sep_pusher.size(); p++ )
                {
                    si_single4all_->copyState(
                      STATE_OBJECT(states_sep_pusher[p],o_target),state_target_goal);
    
                    STATE_ROBOT(states_sep_pusher[p]) = o_pusher;
                    path_pusher_goal.append(states_sep_pusher[p]);
                }
                si_all4all_->copyState(state_curr,states_sep_pusher.back());
            }
    
            for( int p=0; p<states_sep_pusher.size(); p++ ) 
            {
                si_all4all_->freeState(states_sep_pusher[p]);
            }
    /*
    {
        cv::Mat img = cv::Mat::zeros(500,1000,CV_8UC3);
        visualizePath(img,path_pushing);
        cv::circle(img, cv::Point(500-500*state_pusher_goal->as<ObjectState>()->getY(),
                                  500-500*state_pusher_goal->as<ObjectState>()->getX()),
                   10, cv::Scalar(255,255,255) );
        cv::imshow("path_psh",img);
        cv::waitKey();
    }
    */
        }
    }

    if( res )
    {
        si_all4all_->copyState(state_curr,state);
        if( path_mrg.getStateCount() > 0 )
        {
            STATE_ROBOT(state_curr) =  o_target;
            path_res.append(state_curr);
            path_res.append(path_target_ready);
            path2Actions(path_target_ready,actions_res);
        }        
        path_res.append(path_pushing);
        if( do_target )
            path_res.append(path_target_goal);
        if( do_pusher )
            path_res.append(path_pusher_goal);

        vector<Action> actions_pushing;
        path2Actions(path_pushing,actions_pushing);
        for( int i=0; i<actions_pushing.size(); i++ )
        {
            if( actions_pushing[i].type == ACTION_TRANSFER )
            {
                actions_pushing[i].type = ACTION_PUSHING;
                actions_pushing[i].idx_target2 = o_target;
            }
        }
        actions_res.insert( actions_res.end(), 
                            actions_pushing.begin(), actions_pushing.end() );

        if( do_target )
            path2Actions(path_target_goal,actions_res);        
        if( do_pusher )
            path2Actions(path_pusher_goal,actions_res);        
    }
    si_all4all_->freeState(state_curr);

cout << "[plan_pushing] mrg psh sep " << res << ", length: " << path_res.getStateCount() << endl;

    return res;
}

bool Planner::mergePath( const og::PathGeometric &path, 
                               og::PathGeometric &path_res,
                               vector<Action> &actions      )
{    
    bool res = false;
    bool updated = false;

    vector<int> idxes_all(n_objs_);
    for( int o=1; o<=n_objs_; o++ ) idxes_all[o-1] = o;

    vector<int> idxes_done;
    RobotObjectSetup::ParamSingleForAll param_org = env_.getParamSingleForAll();

    vector<og::PathGeometric*> paths_obj;
    vector<int> order_res;
    vector<double> accum_costs;
    vector<pair<int,int> > idxes_startend;

    splitPath( path, paths_obj, order_res, idxes_startend, accum_costs );

    ob::State* state_target_mrg = si_single4all_->allocState();
    ob::State* state_target_sep = si_single4all_->allocState();
    ob::State* state_curr = si_all4all_->allocState();
    si_all4all_->copyState(state_curr, path.getState(0));
    
    for( int i=1; i<order_res.size(); i++ )
    {
        int o_target = order_res[i-1];
        int o_pusher = order_res[i  ];

        ObjectState* state_target_init = STATE_OBJECT(state_curr,o_target);
        const ob::State* state_target_goal = paths_obj[o_target-1]->getState(paths_obj[o_target-1]->getStateCount()-1);
        const ob::State* state_pusher_init = STATE_OBJECT(state_curr,o_pusher);
        const ob::State* state_pusher_goal = paths_obj[o_pusher-1]->getState(paths_obj[o_pusher-1]->getStateCount()-1);

        env_.setParamSingleForAll(o_target, idxes_done, state_curr);

        if( si_single4all_->isValid(state_target_init)==false )
        {
            RobotObjectSetup::ParamSingleForAll param = env_.getParamSingleForAll();
            vector<int> idxes_target = param.idxes_obs;
            if( env_.validateSingleForAll( state_target_init, o_target, idxes_target, 
                                           state_target_init) )
            {
                cout << "[validating] succeed" << endl;
            }
            else
            {
                cout << "[validating] failed" << endl;                
                return false;
            }
        }

        ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si_single4all_));
        pdef->setStartAndGoalStates(state_target_init, state_target_goal, thresh_goal);
        pdef->setOptimizationObjective(opt_inf);                    

        og::RRTstar planner(si_single4all_);
        planner.setRange(0.05);
        planner.setProblemDefinition(pdef);
        planner.setup();
        planner.solve(ob::timedPlannerTerminationCondition(30.0));

        og::PathGeometric &path_target = static_cast<og::PathGeometric&>(*pdef->getSolutionPath());
        path_target.append(state_target_goal);

        og::PathSimplifier ps(si_single4all_);
        ps.simplifyMax(path_target);

        vector<ob::State*> states_res;
        res = isValidPath(state_curr, o_target, path_target, states_res);
        if( res==false )
        {
            cout << "cancel the plan so far" << endl;
            path_res = og::PathGeometric(si_all4all_);
            for( int j=0; j<i; j++ )
            {
                for( int p =idxes_startend[j].first; 
                         p<=idxes_startend[j].second; p++ )
                {
                    cout << "p=" << p << endl;
                    path_res.append(path.getState(p));
                }
            }
            for( int p=0; p<states_res.size(); p++ )
            {            
                si_all4all_->freeState(states_res[p]);
            }

            si_all4all_->copyState(state_curr,path_res.getState(path_res.getStateCount()-1));                
            idxes_done.push_back(o_target);

            actions.clear();
            path2Actions(path_res,actions);

            cout << "path_res.length: " << path_res.getStateCount() << endl;

            continue;
        }

        og::PathGeometric path_mrg(si_single4all_);
        og::PathGeometric path_psh(si_single4all_);
        og::PathGeometric path_sep(si_single4all_);

        double x_mrg, y_mrg, x_sep, y_sep;
        double cost;
        bool res_mrg
         = findMergingSeparatingPoints( path_target, 
                                        state_pusher_init, state_pusher_goal, 
                                        state_target_mrg,  state_target_sep,
                                        path_mrg, path_psh, path_sep,
                                        &cost  );
/*
{
    cv::Mat img = cv::Mat::zeros(500,1000,CV_8UC3);
    env_.visualizeSetup(img);
    for( int o=1; o<n_objs_; o++ )
    {
        drawSoap( img, STATE_OBJECT(state_curr,o)->getX(),
                       STATE_OBJECT(state_curr,o)->getY(),
                       STATE_OBJECT(state_curr,o)->getYaw(),
                       cv::Scalar(20,20,20) );
    }

    for( int p=0; p<path_mrg.getStateCount(); p++ )
    {
        drawSoap( img, path_mrg.getState(p)->as<ObjectState>()->getX(),
                       path_mrg.getState(p)->as<ObjectState>()->getY(),
                       path_mrg.getState(p)->as<ObjectState>()->getYaw(),
                       cv::Scalar(0,0,255) );
    }
    for( int p=0; p<path_psh.getStateCount(); p++ )
    {
        drawSoap( img, path_psh.getState(p)->as<ObjectState>()->getX(),
                       path_psh.getState(p)->as<ObjectState>()->getY(),
                       path_psh.getState(p)->as<ObjectState>()->getYaw(),
                       cv::Scalar(0,255,0) );
    }
    for( int p=0; p<path_sep.getStateCount(); p++ )
    {
        drawSoap( img, path_sep.getState(p)->as<ObjectState>()->getX(),
                       path_sep.getState(p)->as<ObjectState>()->getY(),
                       path_sep.getState(p)->as<ObjectState>()->getYaw(),
                       cv::Scalar(255,0,0) );
    }

    drawSoap( img, state_pusher_init->as<ObjectState>()->getX(),
                   state_pusher_init->as<ObjectState>()->getY(),
                   state_pusher_init->as<ObjectState>()->getYaw(),
                   cv::Scalar(0,128,255) );

    cv::imshow("pushing?",img);
    cv::waitKey();
}
*/
        double cost_curr = accum_costs[i] - accum_costs[i-1];
        cout << "cost before: " << cost_curr << ", cost expected: " << cost << endl;

        res = false;
        //if( res_mrg && cost_curr > cost )
        if( res_mrg )
        {
            og::PathGeometric path_tmp(si_all4all_);
            if( plan_pushing( state_curr, o_pusher, o_target,
                              path_mrg, path_psh, path_sep, 
                              state_target_goal,
                              state_pusher_goal, 
                              path_tmp,
                              actions) )
            {
                double actual_cost = ::compute_cost(state_curr, path_tmp.getState(path_tmp.getStateCount()-1));
                cout << "cost actual: " << actual_cost << endl;
                //if( cost_curr > actual_cost )
                {
                    path_res.append(path_tmp);
                    si_all4all_->copyState(state_curr,path_res.getState(path_res.getStateCount()-1));
                    res = true;
                }
            }
        }
        
        bool one_last = false;
        if( res )
        {
            idxes_done.push_back(o_target);
            idxes_done.push_back(o_pusher);
            i++;

            cout << "pushing together" << o_target << "," << o_pusher << endl;

            if( i==order_res.size()-1 )
            {
                one_last = true;
                cout << "together one_last: " << one_last << endl;
            }
            
            updated = true;
        }
        else
        {
            idxes_done.push_back(o_target);

            {
                Action action;
                action.type = ACTION_TRANSITION;
                action.x   = state_target_init->getX();
                action.y   = state_target_init->getY();
                action.yaw = state_target_init->getYaw();
                action.idx_target  = o_target;
                action.idx_target2 = -1;
                actions.push_back(action);
            }
            for( int p=0; p<path_target.getStateCount(); p++ )
            {
                Action action;
                action.type = ACTION_TRANSFER;
                action.x   = path_target.getState(p)->as<ObjectState>()->getX();
                action.y   = path_target.getState(p)->as<ObjectState>()->getY();
                action.yaw = path_target.getState(p)->as<ObjectState>()->getYaw();
                action.idx_target  = o_target;
                action.idx_target2 = -1;
                actions.push_back(action);            
            }

            STATE_ROBOT(state_curr) = o_target;
            path_res.append(state_curr);
            for( int p=0; p<states_res.size(); p++ )
            {
                STATE_ROBOT(states_res[p]) = o_target;
                path_res.append(states_res[p]);
            }
            si_all4all_->copyState(state_curr,path_res.getState(path_res.getStateCount()-1));

            if( i==order_res.size()-1 )
            {
                one_last = true;
                cout << "alone and one_last: " << order_res[i] << one_last << endl;
            }
            else
            {
                cout << "pushing alone: " << order_res[i] << endl;                
            }
        }

        for( int p=0; p<states_res.size(); p++ )
        {            
            si_all4all_->freeState(states_res[p]);
        }

        if( one_last )
        {
            cout << __LINE__ << endl;
            ObjectState* state_last_init = STATE_OBJECT(state_curr,order_res[i]);
            const ompl::base::State* state_last_goal = paths_obj[order_res[i]-1]->getState(paths_obj[order_res[i]-1]->getStateCount()-1);

            env_.setParamSingleForAll(order_res[i], idxes_done, state_curr);

            if( si_single4all_->isValid(state_last_init)==false )
            {
                RobotObjectSetup::ParamSingleForAll param = env_.getParamSingleForAll();
                vector<int> idxes_target = param.idxes_obs;
                if( env_.validateSingleForAll( state_last_init, order_res[i], idxes_target, 
                                               state_last_init) )
                {
                    cout << "[validating] succeed" << endl;
                }
                else
                {
                    cout << "[validating] failed" << endl;                
                    return false;
                }
            }

            ob::ProblemDefinitionPtr pdef_last(new ob::ProblemDefinition(si_single4all_));
            pdef_last->setStartAndGoalStates(state_last_init,state_last_goal,thresh_goal);
            pdef_last->setOptimizationObjective(opt_inf);                    

            og::RRTstar planner_last(si_single4all_);
            planner_last.setRange(0.05);
            planner_last.setProblemDefinition(pdef_last);
            planner_last.setup();
            planner_last.solve(ob::timedPlannerTerminationCondition(10.0));

            og::PathGeometric& path_last = static_cast<og::PathGeometric &>(*pdef_last->getSolutionPath());
            path_last.append(state_last_goal);

            og::PathSimplifier ps(si_single4all_);
            ps.simplifyMax(path_last);

            vector<ob::State*> states_res_last;
            res = isValidPath(state_curr, order_res[i], path_last, states_res_last);
            for( int p=0; p<states_res_last.size(); p++ )
            {
                STATE_ROBOT(states_res_last[p]) = order_res[i];
                path_res.append(states_res_last[p]);
                si_all4all_->freeState(states_res_last[p]);
            }

            idxes_done.push_back(order_res[i]);
            {
                Action action;
                action.type = ACTION_TRANSITION;
                action.x   = STATE_OBJECT(state_curr,order_res[i])->getX();
                action.y   = STATE_OBJECT(state_curr,order_res[i])->getY();
                action.yaw = STATE_OBJECT(state_curr,order_res[i])->getYaw();
                action.idx_target  = order_res[i];
                action.idx_target2 = -1;
                actions.push_back(action);
            }
            for( int p=0; p<path_last.getStateCount(); p++ )
            {
                Action action;
                action.type = ACTION_TRANSFER;
                action.x   = path_last.getState(p)->as<ObjectState>()->getX();
                action.y   = path_last.getState(p)->as<ObjectState>()->getY();
                action.yaw = path_last.getState(p)->as<ObjectState>()->getYaw();
                action.idx_target  = order_res[i];
                action.idx_target2 = -1;
                actions.push_back(action);
            }
            cout << __LINE__ << endl;

            break;
        }
    }

    for( int i=0; i<order_res.size(); i++ )
    {
        cout << order_res[i] << " -> ";
    }
    cout << endl;

    si_all4all_->freeState(state_curr);
    si_single4all_->freeState(state_target_mrg);
    si_single4all_->freeState(state_target_sep);

    for( int i=0; i<paths_obj.size(); i++ ) delete paths_obj[i];

    env_.setParamSingleForAll(param_org);

    return updated;
}

bool Planner::findMergingSeparatingPoints( const og::PathGeometric &path, 
                                           const ob::State* state_pusher_init, 
                                           const ob::State* state_pusher_goal, 
                                           ob::State* state_target_mrg,
                                           ob::State* state_target_sep,
                                           og::PathGeometric &path_mrg,
                                           og::PathGeometric &path_psh,
                                           og::PathGeometric &path_sep,                                           
                                           double* cost )
{
    const double len = 0.01;

    // discretize path
    vector<int> idxes_path;
    vector<double> xs;    
    vector<double> ys;
    vector<double> yaws;
    const ObjectState* state_init = path.getState(0)->as<ObjectState>();
    xs.push_back(state_init->getX());
    ys.push_back(state_init->getY());
    yaws.push_back(state_init->getYaw());
    idxes_path.push_back(0);

    int len_path = path.getStateCount();
    for( int p=0; p<len_path-1; p++ )
    {
        const ObjectState* state_curr = path.getState(p  )->as<ObjectState>();
        const ObjectState* state_next = path.getState(p+1)->as<ObjectState>();

        double x_curr   = state_curr->getX();
        double y_curr   = state_curr->getY();
        double yaw_curr = state_curr->getYaw();
        double x_next   = state_next->getX();
        double y_next   = state_next->getY();
        double yaw_next = state_next->getYaw();        
        
        double x_vec = (x_next-x_curr);
        double y_vec = (y_next-y_curr);        
        double dist = sqrt(x_vec*x_vec + y_vec*y_vec);        

        double yaw_vec = distance_angle(yaw_next, yaw_curr);
        if( dist > len )
        {
            int n_middle = dist / len;

            x_vec   /= (double)n_middle;
            y_vec   /= (double)n_middle;
            yaw_vec /= (double)n_middle;
            
            for( int m=1; m<=n_middle; m++ )
            {
                double x_middle   = x_curr   + x_vec*m;
                double y_middle   = y_curr   + y_vec*m;
                double yaw_middle = yaw_curr + yaw_vec*m;

                xs.push_back(x_middle);
                ys.push_back(y_middle);                
                yaws.push_back(yaw_middle);
                idxes_path.push_back(p);
            }
        }

        xs.push_back(x_next);
        ys.push_back(y_next);
        yaws.push_back(yaw_next);
        idxes_path.push_back(p);
    }

    // find the closest point
    double x_pusher_init = state_pusher_init->as<ObjectState>()->getX();
    double y_pusher_init = state_pusher_init->as<ObjectState>()->getY();
    double x_pusher_goal = state_pusher_goal->as<ObjectState>()->getX();
    double y_pusher_goal = state_pusher_goal->as<ObjectState>()->getY();
    double x_target_goal = path.getState(path.getStateCount()-1)->as<ObjectState>()->getX();
    double y_target_goal = path.getState(path.getStateCount()-1)->as<ObjectState>()->getY();

    int n_pts = xs.size();
    int i_mrg = -1;
    double dist_init_min = INFINITY;
    for( int i=0; i<n_pts; i++ )
    {
        double x   = xs[i];
        double y   = ys[i];
        double yaw = yaws[i];
        double dist_init = sqrt((x-x_pusher_init)*(x-x_pusher_init)+
                                (y-y_pusher_init)*(y-y_pusher_init)) * 2;
        if( dist_init_min > dist_init )
        {
            i_mrg = i;
            dist_init_min = dist_init;
            state_target_mrg->as<ObjectState>()->setX(x);
            state_target_mrg->as<ObjectState>()->setY(y);
            state_target_mrg->as<ObjectState>()->setYaw(yaw);
        }
    }

    if( i_mrg>0 )
    {
        i_mrg += 5;
        if( i_mrg >= n_pts ) return false;
    }

    int i_sep = -1;
    double dist_goal_min = INFINITY;
    for( int i=i_mrg; i<n_pts; i++ )
    {
        double x   = xs[i];
        double y   = ys[i];
        double yaw = yaws[i];
        double dist_goal = sqrt((x-x_target_goal)*(x-x_target_goal)+
                                (y-y_target_goal)*(y-y_target_goal)) + 
                           sqrt((x-x_pusher_goal)*(x-x_pusher_goal)+
                                (y-y_pusher_goal)*(y-y_pusher_goal));
        if( dist_goal_min > dist_goal )
        {
            i_sep = i;
            dist_goal_min = dist_goal;
            state_target_sep->as<ObjectState>()->setX(x);
            state_target_sep->as<ObjectState>()->setY(y);
            state_target_sep->as<ObjectState>()->setYaw(yaw);
        }
    }

    //*cost = path.length() + dist_init_min + dist_goal_min;
    //cout << "cost: " << path.length() << " + " << dist_init_min << " + " << dist_goal_min << endl;

    *cost = 0;
    for( int p=1; p<path.getStateCount(); p++ )
    {
        double x_prev = path.getState(p-1)->as<ObjectState>()->getX();
        double y_prev = path.getState(p-1)->as<ObjectState>()->getY();
        double x_curr = path.getState(p  )->as<ObjectState>()->getX();
        double y_curr = path.getState(p  )->as<ObjectState>()->getY();

        *cost += sqrt( (x_curr-x_prev)*(x_curr-x_prev)+
                       (y_curr-y_prev)*(y_curr-y_prev)  );
    }
    *cost += dist_init_min;
    *cost += dist_goal_min;

    if( i_mrg >= 0 && i_sep >= 0 && i_mrg < i_sep )
    {
        ObjectState* state_tmp = si_single4all_->allocState()->as<ObjectState>();
        
        if( i_mrg>0 )
        {
            for( int p=0; p<=idxes_path[i_mrg]; p++ ) 
            {
                path_mrg.append(path.getState(p));
            }
            state_tmp->setX(xs[i_mrg]);
            state_tmp->setY(ys[i_mrg]);
            state_tmp->setYaw(yaws[i_mrg]);
            path_mrg.append(state_tmp);
        }

        path_psh.append(state_tmp);
        for( int p=idxes_path[i_mrg]+1; p<=idxes_path[i_sep]; p++ ) 
        {
            path_psh.append(path.getState(p));
        }
        state_tmp->setX(xs[i_sep]);
        state_tmp->setY(ys[i_sep]);
        state_tmp->setYaw(yaws[i_sep]);
        path_psh.append(state_tmp);
        
        path_sep.append(state_tmp);
        for( int p=idxes_path[i_sep]+1; p<path.getStateCount(); p++ ) 
        {
            path_sep.append(path.getState(p));
        }

        si_single4all_->freeState(state_tmp);
        return true;
    }
    else
    {
        cout << "i_mrg: " << i_mrg << ", i_sep: " << i_sep << endl;
        return false;
    }
}

void Planner::splitPath( const og::PathGeometric &path, 
                         vector<og::PathGeometric*> &paths_res, 
                         vector<int> &order_res,
                         vector<pair<int,int> > &idxes_startend,
                         vector<double> &accum_costs )
{
    int len_path = path.getStateCount();

    paths_res.resize(n_objs_);
    for( int o=0; o<n_objs_; o++ )
    {
        paths_res[o] = new og::PathGeometric(si_single4all_);
    }

    int idx_start = -1, idx_end = -1;
    double cost = 0;
    for( int p=0; p<len_path; p++ )
    {
        const ob::State* state = path.getState(p);

        int o = STATE_ROBOT(state);
        paths_res[o-1]->append(STATE_OBJECT(state,o));

        if( order_res.empty() )
        {
            order_res.push_back(o);            
            idx_start = 0;
        }
        else if( order_res.back() != o )
        {
            order_res.push_back(o);
            accum_costs.push_back(cost);
            idx_end = p-1;            
            idxes_startend.push_back(make_pair(idx_start,idx_end));
            idx_start = p;
        }
        
        if( p > 0 )
        {
            const ob::State* state_prev = path.getState(p-1);
            //cost += si_all4all_->distance(state_prev,state);
            cost += ::compute_cost(state_prev,state);
        }
    }
    accum_costs.push_back(cost);
    idx_end = len_path-1;
    idxes_startend.push_back(make_pair(idx_start,idx_end));
}

void Planner::visualizePath(cv::Mat& img, const og::PathGeometric &path)
{
    for( int p=0; p<path.getStateCount(); p++ )
    {
        double alpha = 0.5 + 0.5*((p+1)/(double)path.getStateCount());
        const ompl::base::State* state = path.getState(p);
        for( int q=1; q<=n_objs_; q++ )
        {
            cv::Scalar color = cv::Scalar(colors_.at<cv::Vec3b>(q-1,0)[2],
                                          colors_.at<cv::Vec3b>(q-1,0)[1],
                                          colors_.at<cv::Vec3b>(q-1,0)[0]);

            const ObjectState* state_obj = STATE_OBJECT(state,q);
            drawSoap( img,state_obj->getX(), state_obj->getY(), state_obj->getYaw(),
                      color, alpha, p==0 || p==path.getStateCount()-1 );

            if( p>0 )
            {
                const ObjectState* state_prv = STATE_OBJECT(path.getState(p-1),q);

                int x1 = 500-state_prv->getY()*500;
                int y1 = 500-state_prv->getX()*500;

                int x2 = 500-state_obj->getY()*500;
                int y2 = 500-state_obj->getX()*500;
                line(img, cv::Point(x1,y1), cv::Point(x2,y2), color );
            }
        }
    }
}

void Planner::visualizeState(cv::Mat& img, const ompl::base::State* state)
{
    for( int o=1; o<=n_objs_; o++ )
    {
        cv::Scalar color = cv::Scalar(colors_.at<cv::Vec3b>(o-1,0)[2],
                                      colors_.at<cv::Vec3b>(o-1,0)[1],
                                      colors_.at<cv::Vec3b>(o-1,0)[0]);

        const ObjectState* state_obj = STATE_OBJECT(state,o);
        drawSoap( img,state_obj->getX(), state_obj->getY(), state_obj->getYaw(), color);
    }
}

void Planner::test()
{
    ompl::base::State *start, *goal;

    start = si_single_->allocState();
    goal = si_single_->allocState();

    start->as<ob::SE2StateSpace::StateType>()->setX(0.10);
    start->as<ob::SE2StateSpace::StateType>()->setY(0.55);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0);

    goal->as<ob::SE2StateSpace::StateType>()->setX(0.10);
    goal->as<ob::SE2StateSpace::StateType>()->setY(-0.55);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(0);

    //plan(start, goal);

    si_single_->freeState(start);
    si_single_->freeState(goal);
}

void Planner::save_plan( const std::string &fp_save,
                         const std::string &name,
                         int n_objs,
                         double time_spent,
                         double time_moving,
                         const ompl::base::State* state_init,
                         const ompl::base::State* state_goal,
                         const ompl::geometric::PathGeometric &path,
                         const std::vector<Action> &actions          )
{
    double cost = compute_cost(path);
    //double dist = distance(state_goal,path.getState(path.getStateCount()-1));
    ofstream ofs(fp_save);
    ofs << "name: " << name << endl;
    ofs << "time_spent: " << time_spent << endl;
    ofs << "time_moving: " << time_moving << endl;
    ofs << "cost: " << cost << endl;
    //ofs << "dist: " << dist << endl;
    ofs << "actions: [" << endl;
    for( int a=0; a<actions.size(); a++ )
    {
        ofs << "[" << actions[a].type << ","
                   << actions[a].idx_target  << ","
                   << actions[a].idx_target2 << ","
                   << actions[a].x    << ","
                   << actions[a].y    << ","
                   << actions[a].yaw  << "]," << endl;
    }
    ofs << "]" << endl;
    ofs << "path: [" << endl;
    for( int i=0; i<path.getStateCount(); i++ )
    {
        const ompl::base::CompoundStateSpace::StateType* cs
         = path.getState(i)->as<ompl::base::CompoundStateSpace::StateType>();

        ofs << "[";
        ofs << cs->as<ob::RealVectorStateSpace::StateType>(0)->values[0] << ",";
        for( int o=1; o<=n_objs; o++ )
        {
            ofs << cs->as<ob::SE2StateSpace::StateType>(o)->getX() << ",";
            ofs << cs->as<ob::SE2StateSpace::StateType>(o)->getY() << ",";
            ofs << cs->as<ob::SE2StateSpace::StateType>(o)->getYaw() << ",";
        }
        ofs << "]," << endl;
    }
    ofs << "]" << endl;
    ofs << "init: [";
    for( int o=1; o<=n_objs; o++ )
    {
        ofs << STATE_OBJECT(state_init,o)->getX()   << ",";
        ofs << STATE_OBJECT(state_init,o)->getY()   << ",";
        ofs << STATE_OBJECT(state_init,o)->getYaw() << ",";
    }
    ofs << "]" << endl;

    ofs << "goal: [";
    for( int o=1; o<=n_objs; o++ )
    {
        ofs << STATE_OBJECT(state_goal,o)->getX()   << ",";
        ofs << STATE_OBJECT(state_goal,o)->getY()   << ",";
        ofs << STATE_OBJECT(state_goal,o)->getYaw() << ",";
    }
    ofs << "]" << endl;
    ofs.close();
}
