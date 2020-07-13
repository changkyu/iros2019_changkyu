#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>
#define LOG BOOST_LOG_TRIVIAL(trace)

#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

#include "rl_msgs/seg_scene_srv.h"
#include "rl_msgs/rl_msgs_visualization.hpp"

#include "camera/camera_config.hpp"
#include "camera/camera_utils.hpp"

#include "segmentation/seg_lccp_2Dseg.hpp"
#include "segmentation/seg_2dbox.hpp"

#include "utils/utils_inline.hpp"
#include "utils/utils.hpp"

#include "planner.hpp"
#include "ros_markers.hpp"
#include "ros_kuka.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace std;
using namespace pcl;
using namespace changkyu;

map<string,camerainfo_t> name2camerainfo;
static const double resolution = 0.0025;
static const double resolution_half = resolution*0.5;

SegLCCP2DSeg* seglccp2dseg;

bool cropPointCloud( const vector<float> &workspace,
                     const PointCloud<PointXYZRGB> &cloud_world,
                     PointCloud<PointXYZRGB> &cloud_crop )
{
    Eigen::Matrix3f tf_ws_rot
     = Eigen::Quaternionf(workspace[3],
                          workspace[4],
                          workspace[5],
                          workspace[6]).toRotationMatrix();
    Eigen::Matrix4f tf_ws;
    tf_ws << tf_ws_rot(0,0),tf_ws_rot(0,1),tf_ws_rot(0,2),workspace[0],
             tf_ws_rot(1,0),tf_ws_rot(1,1),tf_ws_rot(1,2),workspace[1],
             tf_ws_rot(2,0),tf_ws_rot(2,1),tf_ws_rot(2,2),workspace[2],
                          0,             0,             0,           1;

    PointCloud<PointXYZRGB>::Ptr cloud_tmp(new PointCloud<PointXYZRGB>);
    transformPointCloud(cloud_world, *cloud_tmp, tf_ws.inverse());

    double x_min = -workspace[7]*0.5;
    double x_max =  workspace[7]*0.5;
    double y_min = -workspace[8]*0.5;
    double y_max =  workspace[8]*0.5;
    double z_min = -workspace[9]*0.5;
    double z_max =  workspace[9]*0.5;

    PointCloud<PointXYZRGB> cloud_out;
    for( size_t p=0; p<cloud_tmp->size(); p++ )
    {
        if( x_min <= cloud_tmp->points[p].x &&
            x_max >= cloud_tmp->points[p].x &&
            y_min <= cloud_tmp->points[p].y &&
            y_max >= cloud_tmp->points[p].y &&
            z_min <= cloud_tmp->points[p].z &&
            z_max >= cloud_tmp->points[p].z    )
        {
            cloud_out.push_back(cloud_world[p]);
        }        
    }
    // cloud_cam & cloud_crop could refer the same memory space
    copyPointCloud(cloud_out, cloud_crop); 
}

int main(int argc, char* argv[])
{    
    double ee_z = 0.417;
    double plane_z = -0.21;
    string fp_save_rotator;
    string camera_name;
    vector<float> workspace;
    string fmt_goal;
    string env_name;
    string planner_name;
    string dir_save;
    int exp_idx;
    bool do_merge;

    ros::init(argc, argv, "tabletop_kuka");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/env_name",env_name);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/planner",planner_name);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/dir_save",dir_save);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/exp_idx",exp_idx);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/rotator",fp_save_rotator);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/endeff_z",ee_z);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/plane_z",plane_z);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/camera_name",camera_name);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/crop_space",workspace);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/fmt_goal",fmt_goal);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/do_merge",do_merge);
    changkyu::ParseParam("/home/cs1080/projects/ijcai2019/software/ros-package/src/perception/camera/config/camera_info_all.yaml", name2camerainfo);
    camerainfo_t &caminfo = name2camerainfo.find(camera_name)->second;

    Markers markers(nh);
    tf::Quaternion q_dove;
    q_dove.setRPY(0, -89.9999/ 180.0 * M_PI, 0);
    markers.AddMesh( "dove_beauty_bar", q_dove, 0.035*0.5 + plane_z );

    KukaRobot kuka(nh,ee_z);
    kuka.GotoHome();

    // Estimate Object Poses
    vector<PointCloud<PointXYZRGB>::Ptr> clouds_obj;
    bool repeat;
    do
    {
        do
        {
            repeat = false;
            clouds_obj.clear();

            sensor_msgs::Image msg_image, msg_depth;       
            PointCloud<PointXYZRGB>::Ptr cloud_cam(new PointCloud<PointXYZRGB>);    
            changkyu::GetInputFromCamera(nh, msg_image, msg_depth, *cloud_cam, caminfo);
            
            PointCloud<PointXYZRGB>::Ptr cloud_world(new PointCloud<PointXYZRGB>);
            transformPointCloud(*cloud_cam,*cloud_world,caminfo.tf_cam2world);

            cropPointCloud(workspace, *cloud_world, *cloud_world);

            PointCloud<PointXYZRGB>::Ptr cloudvox_world(new PointCloud<PointXYZRGB>);
            utils::PointCloud2PointCloudVox(*cloud_world, *cloudvox_world, resolution);

            PointCloud<PointXYZRGB>::Ptr cloudvox_cam(new PointCloud<PointXYZRGB>);    
            transformPointCloud(*cloudvox_world,*cloudvox_cam,caminfo.tf_cam2world.inverse());

            // segmentation
            seglccp2dseg = new SegLCCP2DSeg(&nh);
            rl_msgs::SegmentationScene scene;
            seglccp2dseg->Segment( &scene,
                                   msg_image,
                                   cloudvox_cam, 
                                   "remove_background=0",
                                   caminfo.camera_K,
                                   vector<float>()            );

            Seg2DBox::boxinfo_t boxinfo;
            boxinfo.width  = 0.060;
            boxinfo.height = 0.090;
            boxinfo.area = boxinfo.width*boxinfo.height;

            for( int o=0; o<scene.objects.size(); o++ )
            {
                PointCloud<PointXYZRGB>::Ptr cloud_obj(new PointCloud<PointXYZRGB>);
                fromROSMsg(scene.objects[o].cloud, *cloud_obj);
                transformPointCloud(*cloud_obj,*cloud_obj,caminfo.tf_cam2world);

                // project to the plane
                ModelCoefficients::Ptr coef(new ModelCoefficients);            
                coef->values.resize(4);
                coef->values[0] = 0;
                coef->values[1] = 0;
                coef->values[2] = 1;
                coef->values[3] = 0;

                ProjectInliers<PointXYZRGB> proj_prj;
                proj_prj.setModelType(SACMODEL_PLANE);
                proj_prj.setModelCoefficients(coef);
                proj_prj.setInputCloud(cloud_obj);
                proj_prj.filter(*cloud_obj);

                utils::PointCloud2PointCloudVox(*cloud_obj, *cloud_obj, resolution);

                RadiusOutlierRemoval<PointXYZRGB> outrem;
                outrem.setInputCloud(cloud_obj);
                outrem.setRadiusSearch(resolution);
                outrem.setMinNeighborsInRadius(2);
                outrem.filter (*cloud_obj);

                if( cloud_obj->size() > boxinfo.area*100000*1.5 )
                {
                    vector<PointCloud<PointXYZRGB>::Ptr> clouds_res;
                    Seg2DBox seg2dbox;
                    seg2dbox.Split<PointXYZRGB>(
                        clouds_res, cloud_obj, Eigen::Vector3f(0,0,1), boxinfo, resolution);

                    if( clouds_res.size()==0 )
                    {
                        cout << "err" << endl;
                        repeat = true;
                    }

                    for( int r=0; r<clouds_res.size(); r++ )
                    {
                        clouds_obj.push_back(clouds_res[r]);
                    }
                }
                else
                {
                    clouds_obj.push_back(cloud_obj);
                }
            }
    
            vector<PointCloud<PointXYZRGB>::Ptr> cloud_observe(1);
            cloud_observe[0] = cloud_world;
            markers.PublishPointClouds(cloud_observe);

        } while( repeat );

        markers.PublishPointClouds(clouds_obj);

        std::cout << "# of objs: " << clouds_obj.size() << std::endl;

        std::cout << "[DETECTION] \'r\' for repeat, otherwise [Enter]" << std::endl;        
        char dummy = getchar();
        if( dummy=='r' ) repeat = true;
        std::cin.ignore(INT_MAX,'\n');

    } while( repeat );

    // Provide Object Information
    int n_objs = clouds_obj.size();
    vector<string> names_obj(n_objs);
    vector<RobotObjectSetup::Object> objects(n_objs);
    for( int o=0; o<n_objs; o++ )
    {
        names_obj[o] = "dove_beauty_bar";
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

    // Load Goal
    RobotObjectSetup* env = NULL;
    if( env_name.compare("redbox_kuka")==0 || 
        env_name.compare("bluebox_kuka")==0 || 
        env_name.compare("amazonbox_kuka")==0 )
    {
        vector<double> workspace(caminfo.workspace.begin(), caminfo.workspace.end());
        env = new KukaBoxSetup(objects,workspace);
    }
    else
    {
        env = new KukaTableSetup(objects);
    }

    ompl::base::State* state_init = env->allocState();
    ompl::base::State* state_goal = env->allocState();

    char fp_goal[256];
    sprintf(fp_goal, fmt_goal.c_str(), n_objs);

    cout << fp_goal << endl;

    YAML::Node node_goal = YAML::LoadFile(fp_goal);
    STATE_ROBOT(state_goal) = -1;
    for( int o=1; o<=n_objs; o++ )
    {
        STATE_OBJECT(state_goal,o)->setX(  node_goal["state"][0+(o-1)*3].as<double>()-0.01);
        STATE_OBJECT(state_goal,o)->setY(  node_goal["state"][1+(o-1)*3].as<double>()-0.01);
        STATE_OBJECT(state_goal,o)->setYaw(node_goal["state"][2+(o-1)*3].as<double>());
    }

    // Assign Labels
    vector<double> xs, ys, yaws;
    vector<double> conf_objs;
    for( int o=0; o<n_objs; o++ )
    {
        PointCloud<PointXYZRGB>::Ptr cloud_boundary(new PointCloud<PointXYZRGB>);
        utils::FindBoundary2D(clouds_obj[o], *cloud_boundary, resolution );

        vector<PointXYZRGB> pts_corner;
        utils::FindCorners2D(*cloud_boundary, pts_corner);

        PointXYZRGB pt_center;
        pt_center.x = 0;
        pt_center.y = 0;
        pt_center.z = 0;
        for( int p=0; p<pts_corner.size(); p++ )
        {
            pt_center.x += pts_corner[p].x;
            pt_center.y += pts_corner[p].y;
            pt_center.z += pts_corner[p].z;
        }
        pt_center.x /= pts_corner.size();
        pt_center.y /= pts_corner.size();
        pt_center.z /= pts_corner.size();

        PointXYZRGB vecs[2];
        vecs[0].x = ((pts_corner[0].x - pts_corner[2].x)+ 
                     (pts_corner[3].x - pts_corner[1].x))*0.5;
        vecs[0].y = ((pts_corner[0].y - pts_corner[2].y)+ 
                     (pts_corner[3].y - pts_corner[1].y))*0.5;

        vecs[1].x = ((pts_corner[0].x - pts_corner[3].x)+ 
                     (pts_corner[2].x - pts_corner[1].x))*0.5;
        vecs[1].y = ((pts_corner[0].y - pts_corner[3].y)+ 
                     (pts_corner[2].y - pts_corner[1].y))*0.5;

        double dists[2];
        dists[0] = sqrt(vecs[0].x*vecs[0].x+vecs[0].y*vecs[0].y);
        dists[1] = sqrt(vecs[1].x*vecs[1].x+vecs[1].y*vecs[1].y);

        double yaw;
        if( dists[0]>dists[1] )
        {
            yaw = std::atan2(vecs[0].y,vecs[0].x);
        }
        else
        {
            yaw = std::atan2(vecs[1].y,vecs[1].x);
        }

        {
            int degree = yaw * 180.0 / M_PI;            
            double d = degree % 180;
            double res;
            if( d > 90 ) res = 180 - d;
            else if( d < -90 ) res = 180 + d;
            else res = d;

            yaw = res * M_PI / 180.0;  
        }

        xs.push_back(pt_center.x);
        ys.push_back(pt_center.y);
        yaws.push_back(yaw);
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
//            x   = STATE_OBJECT(state_goal,o)->getX();
//            y   = STATE_OBJECT(state_goal,o)->getY();
              x = 1.00;
              y = 0;
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

        conf_objs.push_back(xs[i_min]);
        conf_objs.push_back(ys[i_min]);
        conf_objs.push_back(yaws[i_min]);

        xs[i_min] = INFINITY;
        ys[i_min] = INFINITY;
    }

    for( int o=1; o<=n_objs; o++ )
    {
        cout << STATE_OBJECT(state_init,o)->getX()   << ", "
             << STATE_OBJECT(state_init,o)->getY()   << ", "
             << STATE_OBJECT(state_init,o)->getYaw() << ", ";
    }
    cout << endl;

    // Publish Markers
    markers.PublishWorkspace();
    markers.PublishObjects(conf_objs,names_obj);

    // Do Plan
    ompl::geometric::PathGeometric path(env->getAllForAllSpaceInformation());
    vector<Planner::Action> actions;
    do
    {
        path = ompl::geometric::PathGeometric(env->getAllForAllSpaceInformation());
        actions.clear();

        Planner planner(*env);
        if( planner_name.compare("plrs")==0 )
        {    
            planner.plan_plRS(state_init, state_goal, path, actions);
        }
        else if( planner_name.compare("ours_selfish")==0 )
        {
            planner.plan(state_init, state_goal, path, actions, false);
        }
        else if( planner_name.compare("kino")==0 )
        {
            planner.UseKino();
            planner.plan(state_init, state_goal, path, actions, true);
        } 
        else
        {
            ifstream ifs(fp_save_rotator);
            planner.load_precomputed_planners(ifs);
            ifs.close();

            planner.plan(state_init, state_goal, path, actions, true);
        }
        markers.PublishObjectsPath(path,names_obj);

        for( int p=0; p<path.getStateCount(); p++ )
        {
            ompl::base::State* state = path.getState(p);
            int o = STATE_ROBOT(state);
            cout << "o=" << o << ": ";
            for( o=1; o<=n_objs; o++ )
            {
                cout << STATE_OBJECT(state,o)->getX() << ","
                     << STATE_OBJECT(state,o)->getY() << ","
                     << STATE_OBJECT(state,o)->getYaw() << " | ";
            }
            cout << endl;
        }
        for( int a=0; a<actions.size(); a++ )
        {
            if( actions[a].type==Planner::ACTION_TRANSITION ) cout << "ACTION_TRANSITION: ";
            if( actions[a].type==Planner::ACTION_TRANSFER   ) cout << "ACTION_TRANSFER:   ";
            if( actions[a].type==Planner::ACTION_PUSHING    ) cout << "ACTION_PUSHING:    ";
     
            cout << actions[a].x   << ",";
            cout << actions[a].y   << ",";
            cout << actions[a].yaw << endl;
        }
        
        std::cout << "[PLANNING] \'r\' for repeat, otherwise [Enter]" << std::endl;        
        char dummy = getchar();
        if( dummy=='r' ) repeat = true;
        else repeat = false;
        std::cin.ignore(INT_MAX,'\n');

    } while( repeat );

    // Actions
    kuka.AddMesh("dove_beauty_bar",q_dove, 0.035 + plane_z );
    kuka.SetObjectNames(names_obj);


    std::cout << "Ready to go [Enter]" << std::endl;
    std::cin.ignore(INT_MAX,'\n');

    if( camera_name.compare("camera1")==0 )
        kuka.GotoDefault(1);
    else if( camera_name.compare("camera2")==0 )
        kuka.GotoDefault(-1);


    double time_moving = 0;

    ros::Time time_begin = ros::Time::now();
    for( int a=0; a<actions.size(); a++ )
    {
        if( actions[a].type==Planner::ACTION_TRANSITION )
        {       
            cout << "ACTION_TRANSITION" << ": "
                 << actions[a].x    << ", "
                 << actions[a].y    << ", "
                 << actions[a].yaw  << endl;

            ros::Time time_begin_local = ros::Time::now();

            kuka.Transition(actions[a].idx_target,actions[a].x,actions[a].y,actions[a].yaw);

            ros::Time time_end_local = ros::Time::now();
            time_moving += (time_end_local-time_begin_local).toSec();
        }
        else if( actions[a].type==Planner::ACTION_TRANSFER ||
                 actions[a].type==Planner::ACTION_PUSHING     )
        {
            bool is_pushing = actions[a].type==Planner::ACTION_PUSHING;
            vector<double> xs, ys, yaws;
            xs.push_back(actions[a-1].x);
            ys.push_back(actions[a-1].y);
            yaws.push_back(actions[a-1].yaw);
            int o = actions[a].idx_target;
            while( a<actions.size() && 
                   (actions[a].type==Planner::ACTION_TRANSFER ||
                   (actions[a].type==Planner::ACTION_PUSHING    ) ) )
            {
                cout << ((actions[a].type==Planner::ACTION_TRANSFER)?"ACTION_TRANSFER: ":"ACTION_PUSHING: ")
                     << actions[a].x    << ", "
                     << actions[a].y    << ", "
                     << actions[a].yaw  << endl;
                xs.push_back(actions[a].x);
                ys.push_back(actions[a].y);
                yaws.push_back(actions[a].yaw);
                a++;
            }
            
            kuka.GraspClose();

            ros::Time time_begin_local = ros::Time::now();

            kuka.Transfer(o,xs,ys,yaws,is_pushing);

            ros::Time time_end_local = ros::Time::now();
            time_moving += (time_end_local-time_begin_local).toSec();

            kuka.GraspOpen();
            a--;
        }
    }
            
    ros::Time time_end = ros::Time::now();
    double time_spent = (time_end - time_begin).toSec();
    cout << "name: " << planner_name << ", execution time: " << time_spent << ", moving time: " << time_moving << endl;

    char fp_save[256];
    sprintf(fp_save,"%s/%s/robot/%s.%s.%s.n%d.%03d.id%03d.res", dir_save.c_str(), env_name.c_str(), planner_name.c_str(), planner_name.c_str(), "dove_beauty_bar", n_objs, exp_idx, 0);
    Planner::save_plan( string(fp_save), "dove_beauty_bar", n_objs, time_spent, time_moving, state_init, state_goal, path, actions );

    kuka.GotoHome();

    env->freeState(state_goal);
    env->freeState(state_init);
    delete env;
    delete seglccp2dseg;

    return 0;
}
