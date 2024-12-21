#ifndef _PHYSICS_RAGDOLL_H
#define _PHYSICS_RAGDOLL_H

#include "g_local.h"
#include "../ghoul2/G2.h"
#include "../../build/bullet3-master/src/btBulletDynamicsCommon.h"
#include "../../build/bullet3-master/src/BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include <vector>

struct RagdollBone {
    char boneName[MAX_QPATH];
    qhandle_t boneIndex;
    float mass;
    qboolean isActive;
    vec3_t position;
    btRigidBody* rigidBody;
    btCollisionShape* collisionShape;
    btTransform initialTransform;
};

class SimpleRagdoll {
public:
    SimpleRagdoll(gentity_t* ent);
    ~SimpleRagdoll();

    void Enable(vec3_t force);
    void Update(float deltaTime);
    void ApplyForce(vec3_t force, int boneIndex);

private:
    void Initialize();
    void InitializeBone(const char* boneName);
    void CreateConstraints();
    void CreateConstraint(RagdollBone* parent, RagdollBone* child, const btVector3& parentAxis, float limit);

    gentity_t* owner;
    bool isEnabled;
    float gravity;
    std::vector<RagdollBone> bones;

    btSoftRigidDynamicsWorld* dynamicsWorld;
    btBroadphaseInterface* broadphase;
    btDefaultCollisionConfiguration* collisionConfig;
    btCollisionDispatcher* dispatcher;
    btSequentialImpulseConstraintSolver* solver;
    btRigidBody* groundBody;
    std::vector<btTypedConstraint*> constraints;
};

#endif