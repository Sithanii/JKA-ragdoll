#include "physics_ragdoll.h"
#include "g_local.h"
#include "../ghoul2/G2.h"
#include "../cgame/cg_local.h"

#ifdef _MSC_VER
#pragma comment(lib, "BulletCollision.lib")
#pragma comment(lib, "BulletDynamics.lib")
#pragma comment(lib, "BulletSoftBody.lib")
#pragma comment(lib, "LinearMath.lib")
#endif

int G_RagAnimForPositioning(gentity_t* ent) {
    vec3_t dir;
    mdxaBone_t matrix;
    assert(ent->client);
    vec3_t G2Angles = { 0, ent->client->ps.viewangles[YAW], 0 };
    assert(ent->ghoul2.size() > 0);
    assert(ent->crotchBolt > -1);

    gi.G2API_GetBoltMatrix(ent->ghoul2, ent->playerModel, ent->crotchBolt, &matrix, G2Angles, ent->client->ps.origin,
        (cg.time ? cg.time : level.time), NULL, ent->s.modelScale);
    gi.G2API_GiveMeVectorFromMatrix(matrix, NEGATIVE_Z, dir);

    return (dir[2] > 0.1f) ? BOTH_DEADFLOP2 : BOTH_DEADFLOP1;
}

static const char* g_effectorStringTable[] = {
   "lhand",
   "rtibia",
   "ltibia",
   "rtalus",
   "ltalus",
   "lradiusX",
   "rfemurX",
   "lfemurX",
   NULL
};

SimpleRagdoll::SimpleRagdoll(gentity_t* ent) {
	owner = ent;
	isEnabled = false;
	gravity = -900.0f;
	hasEffectorData = qfalse;
	VectorClear(effectorTotal);

	Initialize();
}

void SimpleRagdoll::ApplyDamageForce(vec3_t direction, float damage, int mod) {
    if (!isEnabled) return;

    btVector3 force(direction[0], direction[1], direction[2]);
    force.normalize();
    force *= damage * 10.0f;

    for (auto& bone : bones) {
        if (!bone.rigidBody) continue;
        bone.rigidBody->activate(true);
        bone.rigidBody->applyCentralImpulse(force);
    }
}

void SimpleRagdoll::Initialize() {
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    overlappingPairCache = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, 0, -900.0f));
    btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
    info.m_erp = 0.4f;
    info.m_erp2 = 0.2f;
    info.m_linearSlop = 0.0f;

    vec3_t dir;
    mdxaBone_t matrix;
    gi.G2API_GetBoltMatrix(owner->ghoul2, owner->playerModel, owner->crotchBolt, 
        &matrix, ownerAngles, owner->currentOrigin, level.time, NULL, owner->s.modelScale);
    gi.G2API_GiveMeVectorFromMatrix(matrix, NEGATIVE_Z, dir);

    // Ustaw pocz¹tkow¹ orientacjê dla wszystkich koœci
    btQuaternion baseRotation;
    if (dir[2] > 0.1f) {
        baseRotation.setEulerZYX(0, M_PI/2, 0);
    } else {
        baseRotation.setEulerZYX(0, -M_PI/2, 0);
    }

    // Prosta p³aszczyzna dla kolizji
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRbInfo(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    groundBody = new btRigidBody(groundRbInfo);
    groundBody->setFriction(0.8f);
    groundBody->setRestitution(0.1f);
    dynamicsWorld->addRigidBody(groundBody);

    const char* boneNames[] = {
        "pelvis", "lower_lumbar", "upper_lumbar", "thoracic", "cervical",
        "cranium", "rclavical", "rhumerus", "rhumerusX", "rradius",
        "rradiusX", "rhand", "lclavical", "lhumerus", "lhumerusX",
        "lradius", "lradiusX", "lhand", "rfemurYZ", "rfemurX",
        "rtibia", "rtalus", "rtail", "lfemurYZ", "lfemurX",
        "ltibia", "ltalus", "ltail"
    };

    for (const char* boneName : boneNames) {
        InitializeBone(boneName);
    }

    CreateBoneConstraints();
}

void SimpleRagdoll::InitializeBone(const char* boneName) {
    if (!owner || !owner->client || !owner->ghoul2.size()) return;

    qhandle_t boneIndex = gi.G2API_GetBoneIndex(&owner->ghoul2[0], boneName, qtrue);
    if (boneIndex == -1) return;

    Bone bone;
    Q_strncpyz(bone.boneName, boneName, sizeof(bone.boneName));
    bone.boneIndex = boneIndex;
    bone.isActive = qfalse;

    // Get bone position
    mdxaBone_t boneMatrix;
    vec3_t angles;
    VectorSet(angles, 0, owner->client->ps.viewangles[YAW], 0);

    gi.G2API_GetBoltMatrix(
        owner->ghoul2, 0, boneIndex,
        &boneMatrix, angles, owner->currentOrigin,
        level.time, NULL, owner->s.modelScale
    );

    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(
        boneMatrix.matrix[0][3],
        boneMatrix.matrix[1][3],
        boneMatrix.matrix[2][3]
    ));

    bone.position[0] = transform.getOrigin().getX();
    bone.position[1] = transform.getOrigin().getY();
    bone.position[2] = transform.getOrigin().getZ();
    VectorCopy(bone.position, bone.lastPosition);
    VectorClear(bone.velocity);

    // Set mass and create collision shapes
    if (strcmp(boneName, "pelvis") == 0) {
        bone.mass = 10.0f;
        bone.collisionShape = new btCapsuleShape(6.0f, 8.0f);
    }
    else if (strstr(boneName, "lumbar") || strcmp(boneName, "thoracic") == 0) {
        bone.mass = 8.0f;
        bone.collisionShape = new btCapsuleShape(5.0f, 10.0f);
    }
    else if (strstr(boneName, "femur") || strstr(boneName, "tibia")) {
        bone.mass = 7.0f;
        bone.collisionShape = new btCapsuleShape(3.0f, 15.0f);
    }
    else if (strstr(boneName, "humerus") || strstr(boneName, "radius")) {
        bone.mass = 5.0f;
        bone.collisionShape = new btCapsuleShape(2.5f, 12.0f);
    }
    else if (strcmp(boneName, "cranium") == 0) {
        bone.mass = 6.0f;
        bone.collisionShape = new btSphereShape(5.0f);
    }
    else {
        bone.mass = 3.0f;
        bone.collisionShape = new btCapsuleShape(2.0f, 4.0f);
    }

    btVector3 localInertia(0, 0, 0);
    bone.collisionShape->calculateLocalInertia(bone.mass, localInertia);

    btDefaultMotionState* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(bone.mass, motionState, bone.collisionShape, localInertia);
    bone.rigidBody = new btRigidBody(rbInfo);

    bone.rigidBody->setDamping(0.1f, 0.1f);
    bone.rigidBody->setFriction(0.8f);
    bone.rigidBody->setRestitution(0.2f);

    dynamicsWorld->addRigidBody(bone.rigidBody);
    bones.push_back(bone);

    // Set physics properties
    if (strcmp(boneName, "pelvis") == 0) {
        bone.rigidBody->setDamping(0.1f, 0.1f);
        bone.rigidBody->setAngularFactor(btVector3(0.5f, 0.5f, 0.5f));
    }
    else {
        bone.rigidBody->setDamping(0.05f, 0.85f);
    }

    bone.rigidBody->setFriction(0.8f);
    bone.rigidBody->setRestitution(0.2f);
    bone.rigidBody->setActivationState(DISABLE_DEACTIVATION);

    dynamicsWorld->addRigidBody(bone.rigidBody);
    bones.push_back(bone);
}

void SimpleRagdoll::CheckBoltPoints() {
    int boltChecks[5];
    vec3_t boltPoints[5];
    vec3_t tAng;

    boltChecks[0] = gi.G2API_AddBolt(&owner->ghoul2[owner->playerModel], "rhand");
    boltChecks[1] = gi.G2API_AddBolt(&owner->ghoul2[owner->playerModel], "lhand");
    boltChecks[2] = gi.G2API_AddBolt(&owner->ghoul2[owner->playerModel], "cranium");
    boltChecks[3] = gi.G2API_AddBolt(&owner->ghoul2[owner->playerModel], "rtalus");
    boltChecks[4] = gi.G2API_AddBolt(&owner->ghoul2[owner->playerModel], "ltalus");

    VectorSet(tAng, 0, owner->client->ps.viewangles[YAW], 0);

    // Get head first
    mdxaBone_t boltMatrix;
    gi.G2API_GetBoltMatrix(owner->ghoul2, owner->playerModel, boltChecks[2],
        &boltMatrix, tAng, owner->currentOrigin, level.time, NULL, owner->s.modelScale);
    gi.G2API_GiveMeVectorFromMatrix(boltMatrix, ORIGIN, boltPoints[2]);

    for (int i = 0; i < 5; i++) {
        if (i == 2) continue;  // Skip head, already done

        gi.G2API_GetBoltMatrix(owner->ghoul2, owner->playerModel, boltChecks[i],
            &boltMatrix, tAng, owner->currentOrigin, level.time, NULL, owner->s.modelScale);
        gi.G2API_GiveMeVectorFromMatrix(boltMatrix, ORIGIN, boltPoints[i]);

        trace_t tr;
        vec3_t trStart, trEnd;

        if (i < 2) { // Hands trace to head
            VectorCopy(boltPoints[i], trStart);
            VectorCopy(boltPoints[2], trEnd);
        }
        else {
            VectorCopy(boltPoints[i], trStart);
            VectorCopy(owner->currentOrigin, trEnd);
        }

        gi.trace(&tr, trStart, NULL, NULL, trEnd, owner->s.number, MASK_SOLID, (EG2_Collision)0, 0);

        if (tr.fraction != 1.0f || tr.startsolid || tr.allsolid) {
            ProcessEffectorCollision(tr, i);
        }
    }
}

void SimpleRagdoll::ProcessEffectorCollision(const trace_t& tr, int boneIndex) {
    vec3_t effectorPosDif;
    float magicFactor42 = 64.0f;
    VectorScale(tr.plane.normal, magicFactor42, effectorPosDif);
    VectorAdd(effectorTotal, effectorPosDif, effectorTotal);
    hasEffectorData = qtrue;
}

void SimpleRagdoll::Enable(vec3_t force) {
    if (isEnabled) return;

    isEnabled = true;

    owner->client->ps.pm_type = PM_DEAD;
    owner->client->ps.eFlags |= EF_RAG;

    btVector3 impactForce(force[0], force[1], force[2]);
    float forceMagnitude = impactForce.length();

    if (forceMagnitude > 2000.0f) {
        impactForce *= 2000.0f / forceMagnitude;
    }

    for (auto& bone : bones) {
        if (!bone.rigidBody) continue;

        bone.isActive = qtrue;
        bone.rigidBody->setDamping(0.1f, 0.1f);
        bone.rigidBody->setActivationState(DISABLE_DEACTIVATION);
        bone.rigidBody->applyCentralImpulse(impactForce);

        btVector3 boneForce = impactForce;
        if (strcmp(bone.boneName, "pelvis") == 0) {
            boneForce *= 1.5f;
        }
        else if (strstr(bone.boneName, "thoracic") || strcmp(bone.boneName, "cranium") == 0) {
            boneForce *= 1.3f;
        }
        else if (strstr(bone.boneName, "femur") || strstr(bone.boneName, "tibia")) {
            boneForce *= 0.8f;
        }

        boneForce += btVector3(
            Q_flrand(-50.0f, 50.0f),
            Q_flrand(-50.0f, 50.0f),
            Q_flrand(-50.0f, 50.0f)
        );

        bone.rigidBody->applyCentralImpulse(boneForce);
    }
}

void SimpleRagdoll::Update(float deltaTime) {
    if (!isEnabled || !owner || !owner->client || !dynamicsWorld) return;

    // Limit maximum timestep
    const float maxTimeStep = 1.0f / 60.0f;
    deltaTime = (deltaTime > maxTimeStep) ? maxTimeStep : deltaTime;

    dynamicsWorld->stepSimulation(deltaTime, 10);
    CheckBoltPoints();

    if (hasEffectorData) {
        VectorNormalize(effectorTotal);
        VectorScale(effectorTotal, 7.0f, effectorTotal);

        int i = 0;
        while (g_effectorStringTable[i]) {
            for (auto& bone : bones) {
                if (!bone.rigidBody) continue;
                if (strcmp(bone.boneName, g_effectorStringTable[i]) == 0) {
                    btVector3 force(effectorTotal[0], effectorTotal[1], effectorTotal[2]);
                    bone.rigidBody->applyCentralImpulse(force);
                    break;
                }
            }
            i++;
        }

        VectorClear(effectorTotal);
        hasEffectorData = qfalse;
    }

    // Update bone positions and angles
    for (auto& bone : bones) {
        if (!bone.rigidBody) continue;

        btTransform trans;
        bone.rigidBody->getMotionState()->getWorldTransform(trans);

        const btVector3& pos = trans.getOrigin();
        bone.position[0] = pos.getX();
        bone.position[1] = pos.getY();
        bone.position[2] = pos.getZ();

        btMatrix3x3 rot = trans.getBasis();
        btScalar yaw, pitch, roll;
        rot.getEulerYPR(yaw, pitch, roll);

        vec3_t angles;
        angles[0] = RAD2DEG(pitch);
        angles[1] = RAD2DEG(yaw);
        angles[2] = RAD2DEG(roll);

        gi.G2API_SetBoneAngles(
            &owner->ghoul2[0],
            bone.boneName,
            angles,
            BONE_ANGLES_POSTMULT,
            POSITIVE_X,
            POSITIVE_Y,
            POSITIVE_Z,
            NULL,
            100,
            level.time
        );
    }
}

void SimpleRagdoll::CreateBoneConstraints() {
    struct BoneConnection {
        const char* parent;
        const char* child;
        float breakingThreshold;
    };

    BoneConnection connections[] = {
        {"pelvis", "lower_lumbar", 1000.0f},
        {"lower_lumbar", "upper_lumbar", 1000.0f},
        {"upper_lumbar", "thoracic", 1000.0f}
    };

    for (const auto& conn : connections) {
        Bone* parent = nullptr;
        Bone* child = nullptr;

        for (auto& bone : bones) {
            if (strcmp(bone.boneName, conn.parent) == 0) parent = &bone;
            if (strcmp(bone.boneName, conn.child) == 0) child = &bone;
        }

        if (!parent || !child || !parent->rigidBody || !child->rigidBody) continue;

        btTransform localA, localB;
        localA.setIdentity();
        localB.setIdentity();

        btGeneric6DofConstraint* constraint = new btGeneric6DofConstraint(
            *parent->rigidBody,
            *child->rigidBody,
            localA,
            localB,
            true
        );

        constraint->setBreakingImpulseThreshold(conn.breakingThreshold);
        constraint->setLinearLowerLimit(btVector3(-SIMD_PI / 8, -SIMD_PI / 8, -SIMD_PI / 8));
        constraint->setLinearUpperLimit(btVector3(SIMD_PI / 8, SIMD_PI / 8, SIMD_PI / 8));

        dynamicsWorld->addConstraint(constraint, true);
    }
}

void SimpleRagdoll::CreateConstraint(const char* parentName, const char* childName, float strength) {
    Bone* parent = nullptr;
    Bone* child = nullptr;

    for (auto& bone : bones) {
        if (strcmp(bone.boneName, parentName) == 0) parent = &bone;
        if (strcmp(bone.boneName, childName) == 0) child = &bone;
    }

    if (!parent || !child || !parent->rigidBody || !child->rigidBody) return;

    btTransform frameInA, frameInB;
    frameInA.setIdentity();
    frameInB.setIdentity();

    btPoint2PointConstraint* p2p = new btPoint2PointConstraint(
        *parent->rigidBody,
        *child->rigidBody,
        btVector3(0, 0, 0),
        btVector3(0, 0, 0)
    );

    p2p->setParam(BT_CONSTRAINT_STOP_CFM, 0.8f);
    p2p->setParam(BT_CONSTRAINT_STOP_ERP, 0.6f);

    dynamicsWorld->addConstraint(p2p, true);
}

void SimpleRagdoll::ApplyForce(vec3_t force, int boneIndex) {
    if (!isEnabled) return;

    if (boneIndex >= 0 && boneIndex < bones.size() && bones[boneIndex].rigidBody) {
        bones[boneIndex].rigidBody->activate(true);
        btVector3 btForce(force[0], force[1], force[2]);
        bones[boneIndex].rigidBody->applyCentralImpulse(btForce);
    }
}

SimpleRagdoll::~SimpleRagdoll() {
    while (dynamicsWorld->getNumConstraints() > 0) {
        btTypedConstraint* constraint = dynamicsWorld->getConstraint(0);
        dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }

    for (auto& bone : bones) {
        if (bone.rigidBody) {
            dynamicsWorld->removeRigidBody(bone.rigidBody);
            delete bone.rigidBody->getMotionState();
            delete bone.collisionShape;
            delete bone.rigidBody;
        }
    }

    delete groundBody->getMotionState();
    delete groundBody->getCollisionShape();
    dynamicsWorld->removeRigidBody(groundBody);
    delete groundBody;

    delete dynamicsWorld;
    delete solver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;
}