#ifndef _BULLET_SIMULATION__H__
#define _BULLET_SIMULATION__H__

#include <Eigen/Dense>

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"

#include <shape_msgs/Plane.h>

typedef class BulletSimulation
{
public:
    BulletSimulation();    
    virtual ~BulletSimulation();

    virtual void StepSimulation(float deltaTime=0.01)
    {
        // timeStep < maxSubSteps * fixedTimeStep
        if (m_dynamicsWorld) m_dynamicsWorld->stepSimulation(deltaTime);        
    }

    #define DEFAULT_MASS (1.0)
    #define DEFAULT_FRICTION (0.5)
    //#define DEFAULT_MASS (0.01)
    //#define DEFAULT_FRICTION (100)

    int AddCollisionShape(  btCollisionShape* shape,
                            const btVector3 &position,
                            const btQuaternion &pose,
                            const float mass=DEFAULT_MASS,
                            const float friction=DEFAULT_FRICTION,
                            const bool  popping=true        );

    void AddPlaneShape(      const btVector4 &plane=btVector4(0,0,1,0),
                             const float friction=DEFAULT_FRICTION         );

    void AddBucketShape(     std::vector<float> bucket,
                             const float width=0.01,
                             const float mass=DEFAULT_MASS,
                             const float friction=DEFAULT_FRICTION         );

    void InitWorld();
    void ExitWorld();

    btRigidBody* CreateRigidBody( btCollisionShape* shape,
                                  const btVector3 &position,
                                  const btQuaternion &pose=btQuaternion(0,0,0,1),
                                  const float mass=DEFAULT_MASS,
                                  const float friction=DEFAULT_FRICTION,
                                  const bool  popping=true               );

    void RemoveLastRigidBody();

    double SpinIter(int iter_until);
    double SpinUntilStable(){return SpinIter(10000);}
    void SetGravity(const btVector3 gravity);

    void SetObjectPose(int idx, const btQuaternion &q, const btVector3 &T);
    bool GetObjectPose(int idx, btQuaternion &q, btVector3 &T);
    bool GetObjectPose(int idx, Eigen::Matrix4f &pose);

    bool MoveRotateObject(int idx, const btVector3 &pos, double yaw);
    
protected:
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    btCollisionDispatcher*  m_dispatcher;
    btBroadphaseInterface*  m_broadphase;
    btConstraintSolver* m_solver;
    btDiscreteDynamicsWorld* m_dynamicsWorld;
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

    std::vector<btRigidBody*> m_objects;

} BulletSimulation;

struct MyOpenGLGuiHelper : public OpenGLGuiHelper
{
    MyOpenGLGuiHelper(struct CommonGraphicsApp* glApp, bool useOpenGL2)
     : OpenGLGuiHelper(glApp, useOpenGL2){}

    virtual ~MyOpenGLGuiHelper(){};

    virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld);

    std::vector<btVector4> colors;
};

typedef class BulletSimulationGui : public BulletSimulation
{
public:
    BulletSimulationGui();    
    virtual ~BulletSimulationGui();

    virtual void StepSimulation(float deltaTime);

    void Spin(float speed);
    void SpinOnce(float duration=0.1);

    void ResetCamera( float dist=1, float pitch=180, float yaw=30, 
                      float x=0, float y=0, float z=0              );

    void AddColor(btVector4 color);

    void saveNextFrame(const char* filename)
    {
        m_app->dumpNextFrameToPng(filename);
    }

    bool MoveRotateObjectRecord(int idx, const btVector3 &pos, double yaw, int idx_stick, const std::string &fp_format);

    void SpinInit();
    void SpinExit();
private:        

    SimpleOpenGL3App* m_app;
    //struct GUIHelperInterface* m_gui;
    struct MyOpenGLGuiHelper* m_gui;

    bool b_init_gui;

    std::vector<btVector4> colors;    
} BulletSimulationGui;

#endif