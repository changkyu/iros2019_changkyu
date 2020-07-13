#ifndef COLLISION_SIMULATION__HPP
#define COLLISION_SIMULATION__HPP

#include <vector>
#include <btBulletCollisionCommon.h>

typedef class CollisionSimulation
{
public:
    CollisionSimulation();
    ~CollisionSimulation();

    void AddBox( const btVector3 &position,
                 const btQuaternion &pose, 
                 const btVector3 &dims     )
    {
        btVector3 dims_box = dims;
        dims_box[0] -= 0.001f;
        dims_box[1] -= 0.001f;
        dims_box[2] -= 0.001f;

        btBoxShape* box = new btBoxShape(dims_box);
        box->setMargin(0.0f);
        m_shapes.push_back(box);

        AddShape(box, position, pose);
    }

    void AddShape( btCollisionShape* shape,
                   const btVector3 &position,
                   const btQuaternion &pose   )
    {
        btCollisionObject* obj = new btCollisionObject();
        m_objects.push_back(obj);

        obj->getWorldTransform().setOrigin(position);
        obj->getWorldTransform().setRotation(pose);
        obj->setCollisionShape(shape);

        m_collisionWorld->addCollisionObject(obj);
    }

    void RemoveLast()
    {
        btCollisionObject* obj = m_objects.back();
        if( obj )
        {
            m_collisionWorld->removeCollisionObject(obj);            
            m_objects.pop_back();
            delete obj;
        }        
    }

    bool Test();

private:

    btDefaultCollisionConfiguration* m_collisionConfiguration;
    btCollisionDispatcher*           m_dispatcher;

    btAxisSweep3*                    m_broadphase;
    btCollisionWorld*                m_collisionWorld;

    std::vector<btCollisionShape*>   m_shapes;
    std::vector<btCollisionObject*>  m_objects;   

} CollisionSimulation;

#endif