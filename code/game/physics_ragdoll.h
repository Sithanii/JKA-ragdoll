﻿#ifndef _PHYSICS_RAGDOLL_H
#define _PHYSICS_RAGDOLL_H

#include "g_local.h"
#include "../ghoul2/G2.h"
#include "../../build/bullet3-master/src/btBulletDynamicsCommon.h"
#include <vector>

int G_RagAnimForPositioning(gentity_t* ent);

enum BoneOrientation {
    BONE_ORIENT_X = 0,
    BONE_ORIENT_Y = 1,
    BONE_ORIENT_Z = 2
};

class CRagDollParams;
class CGameRagDollUpdateParams;

class SimpleRagdoll {
public:
    SimpleRagdoll(gentity_t* ent);
    ~SimpleRagdoll();

    void Enable(vec3_t force);
    void Update(float deltaTime);
    void ApplyForce(vec3_t force, int boneIndex);
    void ApplyDamageForce(vec3_t direction, float damage, int mod);

private:
    void Initialize();
    void InitializeBone(const char* boneName);
    void CreateBoneConstraints();
    void CreateConstraint(const char* parentName, const char* childName, float strength);
    void CheckBoltPoints();
    void ProcessEffectorCollision(const trace_t& tr, int boneIndex);

    struct Bone {
        char boneName[MAX_QPATH];
        qhandle_t boneIndex;
        float mass;
        qboolean isActive;
        vec3_t position;
        vec3_t lastPosition;
        vec3_t velocity;
        btRigidBody* rigidBody;
        btCollisionShape* collisionShape;
    };

    gentity_t* owner;
    bool isEnabled;
    float gravity;
    vec3_t ownerAngles;
    std::vector<Bone> bones;
    btRigidBody* groundBody;

    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;

    vec3_t effectorTotal;
    qboolean hasEffectorData;
};

#endif