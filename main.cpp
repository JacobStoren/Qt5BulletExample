// A minimal example on how to combine Qt3D and Bullet physics
// Copyright (C) 2017 Jens Jacob Støren
// Permission is granted to anyone to use this software for any purpose 

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

#include "BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"

#include "BulletCollision/CollisionShapes/btCollisionShape.h"

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

class BulletSimulator
{
public:
    BulletSimulator()
    {
        m_collisionConfiguration = new btDefaultCollisionConfiguration();
        m_collisionDispatcher    = new btCollisionDispatcher(m_collisionConfiguration);
        m_physicsSolver          = new btSequentialImpulseConstraintSolver;
        m_broadphaseInterface    = new btDbvtBroadphase();
        m_dynamicsWorld          = new btDiscreteDynamicsWorld(m_collisionDispatcher, m_broadphaseInterface, m_physicsSolver, m_collisionConfiguration);
        m_dynamicsWorld->setGravity(btVector3(0, 0, -10));
    }

    virtual ~BulletSimulator()
    {
        for (int i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0 ; i--)
        {
            btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
            m_dynamicsWorld->removeCollisionObject(obj);

            delete obj;
        }

        for (int j = 0; j < m_collisionShapesStorage.size(); j++)
        {
            btCollisionShape* shape = m_collisionShapesStorage[j];
            m_collisionShapesStorage[j] = 0;
            delete shape;
        }

        delete m_dynamicsWorld;
        delete m_physicsSolver;
        delete m_broadphaseInterface;
        delete m_collisionDispatcher;
        delete m_collisionConfiguration;
        m_collisionShapesStorage.clear();
    }

    void addRigidBody(btCollisionShape* collisionShape, 
                      float mass,
                      btMotionState* bodyMotionUpdaterWithStartPos)
    {
        m_collisionShapesStorage.push_back(collisionShape);

        btVector3 localInertia(0, 0, 0);
        if (mass > 0.0) collisionShape->calculateLocalInertia(mass, localInertia);

        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, bodyMotionUpdaterWithStartPos, collisionShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        m_dynamicsWorld->addRigidBody(body);
    }

    void calculateNextTimeStep()
    {
        m_dynamicsWorld->stepSimulation(1.f/60.f, 10);
    }

private:
    btDefaultCollisionConfiguration*        m_collisionConfiguration;
    btCollisionDispatcher*                  m_collisionDispatcher;
    btBroadphaseInterface*                  m_broadphaseInterface;
    btSequentialImpulseConstraintSolver*    m_physicsSolver;
    btDiscreteDynamicsWorld*                m_dynamicsWorld;
    btAlignedObjectArray<btCollisionShape*> m_collisionShapesStorage;
};

#include <QGuiApplication>

#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>

#include <Qt3DRender/QCamera>
#include <Qt3DRender/QCameraLens>

#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QOrbitCameraController>

#include <QTimer>


#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"

class BodyMotionUpdater : public Qt3DCore::QTransform, public btMotionState
{
public:
    virtual void getWorldTransform(btTransform& worldTrans) const override
    {
        QQuaternion q = this->rotation();
        QVector3D t = this->translation();
        worldTrans = btTransform(btQuaternion(q.x(), q.y(), q.z(), q.scalar()), btVector3(t.x(), t.y(), t.z()));
    }

    virtual void setWorldTransform(const btTransform& worldTrans) override
    {
        QMatrix4x4 mx;
        worldTrans.getOpenGLMatrix(mx.data());
        this->setMatrix(mx);
    }
};

class SimulationSceneCreator
{
public:
    explicit SimulationSceneCreator(BulletSimulator* bulletSimulator) : m_bulletSimulator(bulletSimulator), m_sceneRoot(nullptr) { }
    
    virtual ~SimulationSceneCreator() 
    {
        delete m_sceneRoot;
    }

    Qt3DCore::QEntity* getScene()
    {
        auto temp = m_sceneRoot;
        m_sceneRoot = nullptr;   
        return temp;
    }

    void addBody( Qt3DRender::QGeometryRenderer* visualGeometry,
                  Qt3DRender::QMaterial*         renderMaterial,
                  btCollisionShape*              collisionShape,
                  float                          mass,
                  QVector3D                      position,
                  QQuaternion                    orientation)
    {
        if (m_sceneRoot == nullptr) m_sceneRoot = new Qt3DCore::QEntity;

        Qt3DCore::QEntity *bodyEntity = new Qt3DCore::QEntity(m_sceneRoot);

        BodyMotionUpdater *bodyTransformUpdater = new BodyMotionUpdater;
        bodyTransformUpdater->setTranslation(position);
        bodyTransformUpdater->setRotation(orientation);

        m_bulletSimulator->addRigidBody(collisionShape, mass, bodyTransformUpdater);

        bodyEntity->addComponent(visualGeometry);
        bodyEntity->addComponent(bodyTransformUpdater);
        bodyEntity->addComponent(renderMaterial);
    }

private:
    Qt3DCore::QEntity *m_sceneRoot;
    BulletSimulator* m_bulletSimulator;
};

Qt3DCore::QEntity *createScene(BulletSimulator* simulator)
{
    SimulationSceneCreator sceneCreator(simulator);

    {
        auto material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(QColor(0,100,20));
        auto colShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
        auto vizGeom = new Qt3DExtras::QCuboidMesh;
        vizGeom->setXExtent(100);
        vizGeom->setYExtent(100);
        vizGeom->setZExtent(100);

        sceneCreator.addBody(vizGeom, material, colShape, 0.0f, QVector3D(0, 0, -56), QQuaternion());
    }

    {
        auto material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(QColor(200, 0, 20));
        btCollisionShape* colShape = new btSphereShape(btScalar(1.0f));
        auto vizGeom = new Qt3DExtras::QSphereMesh;
        vizGeom->setRadius(1.0f);

        sceneCreator.addBody(vizGeom, material, colShape, 100.0f, QVector3D(2, 0, 10), QQuaternion());
    }

    {
        auto material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(QColor(0, 0, 255));
        btCollisionShape* colShape = new btConeShape(btScalar(1.0f), 2.0f);
        auto vizGeom = new Qt3DExtras::QConeMesh;
        vizGeom->setBottomRadius(1.0f);
        vizGeom->setTopRadius(0.0f);
        vizGeom->setLength(2.0f);

        sceneCreator.addBody(vizGeom, material, colShape, 30.0f, QVector3D(2.3f, 0, 8), QQuaternion());
    }

    return sceneCreator.getScene();
}

int main(int argc, char* argv[])
{
    QGuiApplication app(argc, argv);

    BulletSimulator simulator;
    Qt3DCore::QEntity *scene = createScene(&simulator);

    Qt3DExtras::Qt3DWindow view;

    Qt3DRender::QCamera *camera = view.camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setPosition(QVector3D(0, -40, 40));
    camera->setViewCenter(QVector3D(0, 0, 0));

    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(scene);
    camController->setLinearSpeed( 50.0f );
    camController->setLookSpeed( 180.0f );
    camController->setCamera(camera);

    view.setRootEntity(scene);
    view.show();

    QTimer timer;
    timer.setInterval(1000*1.0f/60);
    QObject::connect(&timer, &QTimer::timeout, [&] { simulator.calculateNextTimeStep(); });
    timer.start();

    return app.exec();
}



