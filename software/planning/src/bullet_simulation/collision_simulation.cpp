#include <iostream>
#include "bullet_simulation/collision_simulation.hpp"

CollisionSimulation::CollisionSimulation()
{
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    btVector3 worldAabbMin(-1000,-1000,-1000);
    btVector3 worldAabbMax( 1000, 1000, 1000);
    m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);
    m_collisionWorld
     = new btCollisionWorld(m_dispatcher,m_broadphase,m_collisionConfiguration);
}

CollisionSimulation::~CollisionSimulation()
{
    for( int i=0; i<m_objects.size(); i++ ) delete m_objects[i];
    m_objects.clear();

    for( int i=0; i<m_shapes.size(); i++ ) delete m_shapes[i];
    m_shapes.clear();

    delete m_collisionWorld;
    delete m_broadphase;
    delete m_dispatcher;
    delete m_collisionConfiguration;
}

bool
CollisionSimulation::Test()
{
    m_collisionWorld->performDiscreteCollisionDetection();
    int numManifolds = m_collisionWorld->getDispatcher()->getNumManifolds();

    for( int i=0 ; i<numManifolds; i++ )
    {
        btPersistentManifold* contactManifold
         = m_collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);        
        int numContacts = contactManifold->getNumContacts();

        if( numContacts > 0 ) return true;
    }
    return false;
}