#ifndef _PHYSICS_RAGDOLL_H
#define _PHYSICS_RAGDOLL_H

#include "g_local.h"
#include "../ghoul2/G2.h"
#include "../../build/bullet3-master/src/btBulletDynamicsCommon.h"
#include <vector>

enum BoneOrientation {
    BONE_ORIENT_X = 0,
    BONE_ORIENT_Y = 1,
    BONE_ORIENT_Z = 2
};

class SimpleRagdoll {
public:
    SimpleRagdoll(gentity_t* ent);
    ~SimpleRagdoll();

    void Enable(vec3_t force);
    void Update(float deltaTime);
    void ApplyForce(vec3_t force, int boneIndex);

private:
    void CreateBoneConstraints();

    struct Bone {
        char boneName[MAX_QPATH];
        qhandle_t boneIndex;
        float mass;
        qboolean isActive;
        vec3_t position;
        vec3_t lastPosition;
        vec3_t velocity;
        btRigidBody* rigidBody;
    };

    gentity_t* owner;
    bool isEnabled;
    float gravity;
    std::vector<Bone> bones;

    // Bullet Physics components
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;
};

#endif