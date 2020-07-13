#include <iostream>

#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include "Utils/b3Clock.h"

#include "bullet_simulation/bullet_simulation.hpp"

using namespace std;

static
btConvexHullShape* ReduceConvexShape(btConvexHullShape* originalConvexShape )
{
    //create a hull approximation
    btShapeHull* hull = new btShapeHull(originalConvexShape);
    btScalar margin = originalConvexShape->getMargin();
    hull->buildHull(margin);
    return new btConvexHullShape((const btScalar *)hull->getVertexPointer(),hull->numVertices());
}

BulletSimulation::BulletSimulation()
{
    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    //m_collisionConfiguration->setConvexConvexMultipointIterations();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new  btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    m_solver = new btSequentialImpulseConstraintSolver;        

    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);

    SetGravity(btVector3(0, 0, -10));
}   

BulletSimulation::~BulletSimulation()
{
    ExitWorld();

    delete m_dynamicsWorld;
    m_dynamicsWorld=0;

    delete m_solver;
    m_solver=0;

    delete m_broadphase;
    m_broadphase=0;

    delete m_dispatcher;
    m_dispatcher=0;

    delete m_collisionConfiguration;
    m_collisionConfiguration=0;
}

void BulletSimulation::SetGravity(const btVector3 gravity)
{
    m_dynamicsWorld->setGravity(gravity);
}

void BulletSimulation::InitWorld()
{

}

void BulletSimulation::ExitWorld()
{    
    if (m_dynamicsWorld)
    {
        int i;
        for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
        {
            m_dynamicsWorld->removeConstraint(m_dynamicsWorld->getConstraint(i));
        }
        for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
        {
            btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
            btRigidBody* body = btRigidBody::upcast(obj);
            if (body && body->getMotionState())
            {
                delete body->getMotionState();
            }
            m_dynamicsWorld->removeCollisionObject(obj);
            delete obj;
        }
    }
    //delete collision shapes
    for (int j = 0; j<m_collisionShapes.size(); j++)
    {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }
    m_collisionShapes.clear();
}

btRigidBody* BulletSimulation::CreateRigidBody( btCollisionShape* shape,                                    
                                                const btVector3 &position,
                                                const btQuaternion &pose,
                                                const float mass,
                                                const float friction,
                                                const bool  popping        )
{ 
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    shape->setMargin(0);    

    btVector3 localInertia(0,0,0);
    if( mass!=0.f ) shape->calculateLocalInertia(mass, localInertia);

    btTransform transform;
    transform.setIdentity();
    transform.setRotation(pose);
    transform.setOrigin(position);
    btDefaultMotionState* noMotion = new btDefaultMotionState(transform);

    btRigidBody::btRigidBodyConstructionInfo 
      cInfo(mass, noMotion, shape, localInertia);

    //btVector3 center;
    //btScalar radius;
    //shape->getBoundingSphere(center, radius);
    //btTransform transform_cm;
    //transform_cm.setIdentity();
    //transform_cm.setOrigin(center);    

    btRigidBody* body = new btRigidBody(cInfo);    
    body->setUserIndex(-1);
    body->setFriction(friction);
    body->setRestitution(0);
    //body->setCenterOfMassTransform(transform_cm);
    if( popping == false ) body->setDamping(0.99,0.99);
    else                   body->setDamping(0.00,0.00);
    
    m_dynamicsWorld->addRigidBody(body);
    return body;
}

void BulletSimulation::RemoveLastRigidBody()
{
    if (m_dynamicsWorld)
    {
        int i = m_dynamicsWorld->getNumCollisionObjects() - 1;
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject(obj);
        delete obj;        
    }
    //delete collision shapes
    if( m_collisionShapes.size() > 0 )
    {
        int i = m_collisionShapes.size()-1;
        btCollisionShape* shape = m_collisionShapes[i];
        delete shape;
        m_collisionShapes.pop_back();
    }    
}

void BulletSimulation::AddPlaneShape( const btVector4 &vec_plane,
                                      const float friction            )
{
    btStaticPlaneShape* shape
     = new btStaticPlaneShape(btVector3(vec_plane[0],vec_plane[1],vec_plane[2]), 
                              btScalar(vec_plane[3]) );
    m_collisionShapes.push_back(shape);
    CreateRigidBody(shape,btVector3(0,0,0),btQuaternion(0,0,0,1),0,friction);    
}

void BulletSimulation::AddBucketShape( std::vector<float> bucket,                                       
                                       const float width,
                                       const float mass,
                                       const float friction       )
{
    if( bucket.size() == 10 )
    {
        btVector3 position( bucket[0],bucket[1],bucket[2] );        
        btQuaternion rotation( bucket[4],bucket[5],bucket[6],bucket[3] );
        btVector3 dims( bucket[7],bucket[9],bucket[8] );

        btQuaternion q_offset( -0.707,0,0,0.707 );

        btBoxShape* front   = new btBoxShape(
            btVector3(  width*0.5, dims[1]*0.5, dims[2]*0.5));
        btBoxShape* back    = new btBoxShape(
            btVector3(  width*0.5, dims[1]*0.5, dims[2]*0.5));
        btBoxShape* left    = new btBoxShape(
            btVector3(dims[0]*0.5,   width*0.5, dims[2]*0.5));
        btBoxShape* right   = new btBoxShape(
            btVector3(dims[0]*0.5,   width*0.5, dims[2]*0.5));
        btBoxShape* bottom  = new btBoxShape(
            btVector3(dims[0]*0.5, dims[1]*0.5, width*0.5));
            
        btTransform tf_front;
        tf_front.setIdentity();
        tf_front.setOrigin(  btVector3(-(dims[0]*0.5 + width*0.5),0,0 ) );
        btTransform tf_back;
        tf_back.setIdentity();
        tf_back.setOrigin(   btVector3( (dims[0]*0.5 + width*0.5),0,0 ) );
        btTransform tf_left;
        tf_left.setIdentity();
        tf_left.setOrigin( btVector3(0, -(dims[1]*0.5 + width*0.5),0 ) );
        btTransform tf_right;
        tf_right.setIdentity();
        tf_right.setOrigin( btVector3(0,  (dims[1]*0.5 + width*0.5),0 ) );
        btTransform tf_bottom;
        tf_bottom.setIdentity();
        tf_bottom.setOrigin(  btVector3(0,0, -(dims[2]*0.5 + width*0.5)) );
                
        btCompoundShape* shape = new btCompoundShape();
        shape->addChildShape(tf_front,  front);
        shape->addChildShape(tf_back,   back);
        shape->addChildShape(tf_left,   left);
        shape->addChildShape(tf_right,  right);
        shape->addChildShape(tf_bottom, bottom);

        m_collisionShapes.push_back(shape);
        CreateRigidBody(shape,position,rotation*q_offset,mass,friction);
    }
}

int BulletSimulation::AddCollisionShape(  btCollisionShape* shape,
                                          const btVector3 &position,
                                          const btQuaternion &pose,
                                          const float mass,
                                          const float friction,
                                          const bool  popping        )
{
    m_objects.push_back(CreateRigidBody(shape,position,pose,mass,friction,popping));
    return (m_objects.size()-1);
}

void BulletSimulation::SetObjectPose(int idx, const btQuaternion &q, const btVector3 &T)
{
    if( idx<0 || m_objects.size()<= idx ) return;

    btTransform tf;
    tf.setOrigin(T);
    tf.setRotation(q);
    m_objects[idx]->setWorldTransform(tf);
    m_objects[idx]->getMotionState()->setWorldTransform(tf);
}

bool BulletSimulation::GetObjectPose(int idx, btQuaternion &q, btVector3 &T)
{
    if( idx<0 || m_objects.size()<= idx ) return false;

    btTransform transform;
    m_objects[idx]->getMotionState()->getWorldTransform(transform);
    q = transform.getRotation();
    T = transform.getOrigin();
}

bool BulletSimulation::GetObjectPose(int idx, Eigen::Matrix4f &pose)
{
    btQuaternion q;
    btVector3 T;
    if( GetObjectPose(idx,q,T) )
    {           
        btMatrix3x3 R(q);
        pose << R[0][0], R[0][1], R[0][2], T[0],
                R[1][0], R[1][1], R[1][2], T[1],
                R[2][0], R[2][1], R[2][2], T[2],
                      0,       0,       0,    1;
        return true;
    }
    else
    {
        return false;
    }
}

bool BulletSimulation::MoveRotateObject(int idx, const btVector3 &pos, double yaw)
{    
    if( idx<0 || m_objects.size()<= idx ) return false;

    btTransform tf;
    m_objects[idx]->getMotionState()->getWorldTransform(tf);
    btVector3    pos_curr = tf.getOrigin();
    btQuaternion rot_curr = tf.getRotation();

    btVector3 pos_goal = pos + pos_curr;
    btQuaternion rot_goal;
    {
        btQuaternion rot;
        rot.setEulerZYX(yaw,0,0);
        rot_goal = rot * rot_curr;
    }

    const double time_delta = 0.1;
    double time_req = 0;
    {
        // 30 degree per second OR 3cm per second
        double dist = sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
        double time_pos = dist / 0.01;
        double time_rot = yaw * 6.0 / M_PI;
        time_req = time_pos > time_rot ? time_pos : time_rot;
        time_req = time_req > 0.1 ? time_req : 0.1;
    }    
    
    int n_iters = round(time_req / time_delta);

    btVector3 pos_per_sec = pos/(double)time_req;
    btVector3 pos_per_iter = pos/(double)n_iters;
    btQuaternion rot_per_iter;
    rot_per_iter.setEulerZYX(yaw/(double)n_iters,0,0);
    
    for( int i=0; i<n_iters; i++ )
    {
        m_objects[idx]->activate(true);

        btTransform tf;
        pos_curr = pos_per_iter + pos_curr;
        rot_curr = rot_per_iter * rot_curr;
        rot_curr.normalize();
        tf.setOrigin(pos_curr);
        tf.setRotation(rot_curr);
        m_objects[idx]->setWorldTransform(tf);
        m_objects[idx]->getMotionState()->setWorldTransform(tf);
        StepSimulation(time_delta);
    }

    //btTransform tf;
    tf.setOrigin(pos_goal);
    tf.setRotation(rot_goal);
    m_objects[idx]->activate(true);
    m_objects[idx]->setWorldTransform(tf);
    m_objects[idx]->getMotionState()->setWorldTransform(tf);
    StepSimulation(10.0);

    double dist = sqrt((pos_goal[0]-pos_curr[0])*(pos_goal[0]-pos_curr[0])+ 
                       (pos_goal[1]-pos_curr[1])*(pos_goal[1]-pos_curr[1])+ 
                       (pos_goal[2]-pos_curr[2])*(pos_goal[2]-pos_curr[2]) );
    return dist <= 0.01;
}

bool BulletSimulationGui::MoveRotateObjectRecord(int idx, const btVector3 &pos, double yaw, int idx_stick, const string &fp_format)
{
    if( idx<0 || m_objects.size()<= idx ) return false;
    
    btTransform tf;
    m_objects[idx]->getMotionState()->getWorldTransform(tf);
    btVector3    pos_curr = tf.getOrigin();
    btQuaternion rot_curr = tf.getRotation();

    btVector3 pos_goal = pos + pos_curr;
    btQuaternion rot_goal;
    {
        btQuaternion rot;
        rot.setEulerZYX(yaw,0,0);
        rot_goal = rot * rot_curr;
    }

    const double time_delta = 0.1;
    double time_req = 0;
    {
        // 30 degree per second OR 3cm per second
        double dist = sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
        double time_pos = dist / 0.01;
        double time_rot = yaw * 6.0 / M_PI;
        time_req = time_pos > time_rot ? time_pos : time_rot;
        time_req = time_req > 0.1 ? time_req : 0.1;
    }    
    
    int n_iters = round(time_req / time_delta);

    btVector3 pos_per_sec = pos/(double)time_req;
    btVector3 pos_per_iter = pos/(double)n_iters;
    btQuaternion rot_per_iter;
    rot_per_iter.setEulerZYX(yaw/(double)n_iters,0,0);
        
    for( int i=0; i<n_iters; i++ )
    {
        char fp_save[256];
        sprintf(fp_save, fp_format.c_str(), i);
        saveNextFrame(fp_save);

        m_objects[idx]->activate(true);        

        btTransform tf;        
        pos_curr = pos_per_iter + pos_curr;
        rot_curr = rot_per_iter * rot_curr;
        rot_curr.normalize();
        tf.setOrigin(pos_curr);
        tf.setRotation(rot_curr);
        m_objects[idx]->setWorldTransform(tf);
        m_objects[idx]->getMotionState()->setWorldTransform(tf);

        m_objects[idx_stick]->activate(true);
        btVector3 pos_stick = pos_curr;
        pos_stick[2] = 0.19;
        tf.setOrigin(pos_stick);
        tf.setRotation(btQuaternion(0,0,0,1));
        m_objects[idx_stick]->setWorldTransform(tf);
        m_objects[idx_stick]->getMotionState()->setWorldTransform(tf);

        StepSimulation(time_delta);
    }
    StepSimulation(1.0);

    double dist = sqrt((pos_goal[0]-pos_curr[0])*(pos_goal[0]-pos_curr[0])+ 
                       (pos_goal[1]-pos_curr[1])*(pos_goal[1]-pos_curr[1])+ 
                       (pos_goal[2]-pos_curr[2])*(pos_goal[2]-pos_curr[2]) );
    return dist <= 0.01;
}

static double compute_dist(btQuaternion &q1, btQuaternion &q2)
{
    btQuaternion q11 = q1.normalize();
    btQuaternion q22 = q2.normalize();

    return 1 - (q11.x()*q22.x() + q11.y()*q22.y() + q11.z()*q22.z() + q11.w()*q22.w())*
               (q11.x()*q22.x() + q11.y()*q22.y() + q11.z()*q22.z() + q11.w()*q22.w());
}

static double compute_dist(btVector3 &vec1, btVector3 &vec2)
{
    return sqrt( (vec1[0] - vec2[0]) * (vec1[0] - vec2[0]) +
                 (vec1[1] - vec2[1]) * (vec1[1] - vec2[1]) +
                 (vec1[2] - vec2[2]) * (vec1[2] - vec2[2])   );
}

double BulletSimulation::SpinIter(int iter_until)
{
    const double eps = 0.000001;// 0.0001 = 0.01 dist * 0.01 sec
    vector<btVector3> locations_init;
    vector<btVector3> locations_prev;
    vector<btQuaternion> rotations_init;
    vector<btQuaternion> rotations_prev;

    int n_objs = m_dynamicsWorld->getNumCollisionObjects();

    int iter=0;
    vector<double> dist_sum(n_objs);
    for (int o=0; o<n_objs; o++) dist_sum[o] = 0;

    bool loop=true;
    do
    {
        double dist_local_max = -1;
        for (int o=0; o<n_objs; o++)
        {
            double dist_local = 0;

            btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[o];
            btRigidBody* body = btRigidBody::upcast(obj);

            if (body && body->getMotionState())
            {                
                btTransform tr = body->getWorldTransform();
                btVector3 location = tr.getOrigin();
                btQuaternion rotation = tr.getRotation();

                if( o < locations_prev.size() )
                {
                    double dist_loc = compute_dist(location, locations_prev[o]);
                    double dist_rot = compute_dist(rotation, rotations_prev[o]);

                    //cout << "dist loc: "<< dist_loc << ", dist rot: " << dist_rot << endl;
                    dist_local = (dist_loc + dist_rot);
                    locations_prev[o] = location;
                    rotations_prev[o] = rotation;
                }
                else
                {
                    locations_init.push_back(location);
                    locations_prev.push_back(location);
                    rotations_init.push_back(rotation);
                    rotations_prev.push_back(rotation);
                }
            }

            if( dist_local_max < dist_local )
            {
                dist_local_max = dist_local;
            }
            dist_sum[o] += dist_local;
        }
        
        StepSimulation();

        if( 0 < iter_until && iter > iter_until )
        {
            loop = false;            
        }      
        /*  
        else if( iter > 100 && dist_local_max < eps )
        {
            loop = false;
        }
        */

        iter++;
    } while(loop);

    double dist_max = -1;
    for (int o=0; o<n_objs; o++)
    {
        /*
        double dist_loc = compute_dist(locations_init[o], locations_prev[o]);
        double dist_rot = compute_dist(rotations_init[o], rotations_prev[o]);
        double dist = dist_loc + dist_rot;
        if( dist_max < dist ) dist_max = dist;
        */

        if( dist_max < dist_sum[o] ) dist_max = dist_sum[o];
    }

    return dist_max;
}

static b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove( float x, float y)
{
    bool handled = false; 
    //handled = example->mouseMoveCallback(x,y);
    if (!handled)
    {
        if (prevMouseMoveCallback)
            prevMouseMoveCallback (x,y);
    }
}

static b3MouseButtonCallback prevMouseButtonCallback  = 0;
static void OnMouseDown(int button, int state, float x, float y) 
{
    bool handled = false;
    //handled = example->mouseButtonCallback(button, state, x,y); 
    if (!handled)
    {
        if (prevMouseButtonCallback )
            prevMouseButtonCallback (button,state,x,y);
    }
}

static double colors_random[][4] = 
{
    {100, 100, 100, 255},
    {230,  25,  75, 255},
    { 60, 180,  75, 255},
    {255, 225,  25, 255},
    {  0, 130, 200, 255},
    {245, 130,  48, 255},
    {145,  30, 180, 255},
    { 70, 240, 240, 255},
    {240,  50, 230, 255},
    {210, 245,  60, 255},
    {250, 190, 190, 255},
    {  0, 128, 128, 255},
    {230, 190, 255, 255},
    {170, 110,  40, 255},
    {255, 250, 200, 255},
    {128,   0,   0, 255},
    {170, 255, 195, 255},
    {128, 128,   0, 255},
    {255, 215, 180, 255},
    {  0,   0, 128, 255},
    {128, 128, 128, 255},
    {255, 255,  25, 255}
};

//vector<btVector4> MyOpenGLGuiHelper::colors;

void MyOpenGLGuiHelper::autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld)
{
    btAlignedObjectArray<btCollisionObject*> sortedObjects;
    sortedObjects.reserve(rbWorld->getNumCollisionObjects());
    for (int i=0;i<rbWorld->getNumCollisionObjects();i++)
    {
        btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];  
        createCollisionShapeGraphicsObject(colObj->getCollisionShape());

        btVector4 color;
        if( i >= colors.size() )
        {
            double* clr
             = colors_random[ (i-colors.size())
                               % (sizeof(colors_random)/(sizeof(double)*4)) ];
            color = btVector4(clr[0]/255.,clr[1]/255.,clr[2]/255.,clr[3]/255.);
        }
        else
        {
            color = colors[i];
        }

        createCollisionObjectGraphicsObject(colObj,color);
    }
}

BulletSimulationGui::BulletSimulationGui() : BulletSimulation()
{
    m_gui = NULL;
    m_app = NULL;

    InitWorld();

    b_init_gui = false;
}

BulletSimulationGui::~BulletSimulationGui()
{
    ExitWorld();
}

void BulletSimulationGui::AddColor(btVector4 color)
{
    colors.push_back(color);
}

void BulletSimulationGui::ResetCamera( float dist, float pitch, float yaw, 
                                       float x, float y, float z           )
{
    m_gui->resetCamera(dist,pitch,yaw,x,y,z);
}

void BulletSimulationGui::SpinInit()
{    
    SpinExit();

    m_app = new SimpleOpenGL3App("Bullet Standalone Example",1024,768,true);
    
    prevMouseButtonCallback = m_app->m_window->getMouseButtonCallback();
    prevMouseMoveCallback = m_app->m_window->getMouseMoveCallback();
    m_app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
    m_app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

    m_gui = new MyOpenGLGuiHelper(m_app,false);
    m_gui->setUpAxis(2);
    m_gui->createPhysicsDebugDrawer(m_dynamicsWorld);

    if (m_dynamicsWorld->getDebugDrawer())
        m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

    m_gui->colors = colors;
    m_gui->autogenerateGraphicsObjects(m_dynamicsWorld);
    //ResetCamera();

    b_init_gui = true;
}

void BulletSimulationGui::SpinExit()
{
    if( b_init_gui==false ) return;
    
    for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
    {        
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        obj->getCollisionShape()->setUserIndex(-1);
        obj->setUserIndex(-1);
    }    
#if 1    
    if(m_gui)
    {
        delete m_gui;
        m_gui = NULL;
    } 
    if(m_app)
    {
        delete m_app;
        m_app = NULL;        
    } 
#else

#endif
    b_init_gui = false;
}

void BulletSimulationGui::SpinOnce(float duration)
{
    if( !b_init_gui ) SpinInit();

    StepSimulation(duration);
}

void BulletSimulationGui::Spin(float speed)
{
    if( !b_init_gui ) SpinInit();

    b3Clock clock;    
    do
    {
        btScalar dtSec = btScalar(clock.getTimeInSeconds());
        dtSec *= speed;
        if (dtSec<0.01) dtSec = 0.01;

        StepSimulation(dtSec);

        clock.reset();
    } while (!m_app->m_window->requestedExit());
}

void BulletSimulationGui::StepSimulation(float duration)
{
    if( m_app )
    {
        m_app->m_instancingRenderer->init();
        m_app->m_instancingRenderer->updateCamera(m_app->getUpAxis());
    }

    BulletSimulation::StepSimulation(duration);
    
    if( m_gui )
    {
        m_gui->syncPhysicsToGraphics(m_dynamicsWorld);
        m_gui->render(m_dynamicsWorld);
    }

    if( m_app )
    {
#if 0
        DrawGridData dg;        
        dg.upAxis = m_app->getUpAxis();
        m_app->drawGrid(dg);
#endif     
        m_app->swapBuffer();
    }
}