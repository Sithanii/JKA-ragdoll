#include "physics_ragdoll.h"
#include "g_local.h"
#include "../ghoul2/G2.h"

#ifdef _MSC_VER
#pragma comment(lib, "BulletCollision.lib")
#pragma comment(lib, "BulletDynamics.lib")
#pragma comment(lib, "BulletSoftBody.lib")
#pragma comment(lib, "LinearMath.lib")
#endif

SimpleRagdoll::SimpleRagdoll(gentity_t* ent) {
    owner = ent;
    isEnabled = false;
    gravity = -900.0f;

    Initialize();
    DisableJAAnimations();
}

void SimpleRagdoll::Initialize() {
    // Initialize Bullet Physics
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    overlappingPairCache = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, 0, -980.0f));

    const char* boneNames[] = {
        "pelvis", "lower_lumbar", "upper_lumbar", "thoracic", "cervical",
        "cranium", "rclavical", "rhumerus", "rhumerusX", "rradius",
        "rradiusX", "rhand", "lclavical", "lhumerus", "lhumerusX",
        "lradius", "lradiusX", "lhand", "rfemurYZ", "rfemurX",
        "rtibia", "rtalus", "rtail", "lfemurYZ", "lfemurX",
        "ltibia", "ltalus", "ltail"
    };

    // Initialize all bones
    for (const char* boneName : boneNames) {
        InitializeBone(boneName);
    }

    CreateBoneConstraints();
}

void SimpleRagdoll::InitializeBone(const char* boneName) {
    qhandle_t boneIndex = gi.G2API_GetBoneIndex(&owner->ghoul2[0], boneName, qtrue);
    if (boneIndex == -1) return;

    Bone bone;
    Q_strncpyz(bone.boneName, boneName, sizeof(bone.boneName));
    bone.boneIndex = boneIndex;
    bone.isActive = qfalse;

    // Masses
    if (strcmp(boneName, "pelvis") == 0) {
        bone.mass = 12.0f;
    }
    else if (strcmp(boneName, "thoracic") == 0) {
        bone.mass = 10.0f;
    }
    else if (strstr(boneName, "lumbar")) {
        bone.mass = 8.0f;
    }
    else if (strstr(boneName, "femur")) {
        bone.mass = 7.0f;
    }
    else if (strstr(boneName, "humerus")) {
        bone.mass = 5.0f;
    }
    else {
        bone.mass = 3.0f;
    }

    // Bones position
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

    // Collision types
    if (strcmp(boneName, "pelvis") == 0) {
        bone.collisionShape = new btCapsuleShape(6.0f, 8.0f);
    }
    else if (strstr(boneName, "lumbar") || strcmp(boneName, "thoracic") == 0) {
        bone.collisionShape = new btCapsuleShape(5.0f, 10.0f);
    }
    else if (strstr(boneName, "femur") || strstr(boneName, "tibia")) {
        bone.collisionShape = new btCapsuleShape(3.0f, 15.0f);
    }
    else if (strstr(boneName, "humerus") || strstr(boneName, "radius")) {
        bone.collisionShape = new btCapsuleShape(2.5f, 12.0f);
    }
    else if (strcmp(boneName, "cranium") == 0) {
        bone.collisionShape = new btSphereShape(5.0f);
    }
    else {
        bone.collisionShape = new btCapsuleShape(2.0f, 4.0f);
    }

    btVector3 localInertia(0, 0, 0);
    bone.collisionShape->calculateLocalInertia(bone.mass, localInertia);

    btMotionState* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(bone.mass, motionState, bone.collisionShape, localInertia);
    bone.rigidBody = new btRigidBody(rbInfo);

    bone.rigidBody->setDamping(0.3f, 0.5f);
    bone.rigidBody->setFriction(0.8f);
    bone.rigidBody->setRestitution(0.3f);
    bone.rigidBody->setLinearFactor(btVector3(1, 1, 1));
    bone.rigidBody->setAngularFactor(btVector3(0.8f, 0.8f, 0.8f));

    // Ground
    btCollisionShape* groundShape = new btBoxShape(btVector3(1000, 1000, 10));
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -100)));

    btRigidBody::btRigidBodyConstructionInfo groundRbInfo(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    groundBody = new btRigidBody(groundRbInfo);
    groundBody->setFriction(1.0f);
    groundBody->setRestitution(0.1f);
    groundBody->setCollisionFlags(groundBody->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
    dynamicsWorld->addRigidBody(groundBody, 1, 1);

    for (auto& bone : bones) {
        if (bone.rigidBody) {
            bone.rigidBody->setCollisionFlags(bone.rigidBody->getCollisionFlags() |
                btCollisionObject::CF_DYNAMIC_OBJECT |
                btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
        }
    }

    dynamicsWorld->addRigidBody(bone.rigidBody);
    bones.push_back(bone);
}

void SimpleRagdoll::DisableJAAnimations() {
    if (!owner || !owner->client) return;

    owner->client->ps.legsAnim = 0;
    owner->client->ps.torsoAnim = 0;
    owner->client->ps.pm_type = PM_DEAD;
    owner->client->ps.pm_flags |= PMF_TIME_KNOCKBACK;

    const char* bonesToDisable[] = {
        "upper_lumbar", "lower_lumbar", "thoracic", "cervical",
        "pelvis", "cranium"
    };

    for (const char* boneName : bonesToDisable) {
        gi.G2API_SetBoneAngles(&owner->ghoul2[0], boneName, vec3_origin,
            BONE_ANGLES_POSTMULT,
            static_cast<Eorientations>(BONE_ORIENT_X),
            static_cast<Eorientations>(BONE_ORIENT_Y),
            static_cast<Eorientations>(BONE_ORIENT_Z),
            NULL, 100, level.time);
    }
}

void SimpleRagdoll::CreateBoneConstraints() {
    struct BoneConnection {
        const char* parent;
        const char* child;
        float strength;
    };

    BoneConnection connections[] = {
        {"pelvis", "lower_lumbar", 1.0f},
        {"lower_lumbar", "upper_lumbar", 1.0f},
        {"upper_lumbar", "thoracic", 1.0f},
        {"thoracic", "cervical", 0.8f},
        {"cervical", "cranium", 0.8f},
        {"thoracic", "rclavical", 0.7f},
        {"rclavical", "rhumerus", 0.6f},
        {"rhumerus", "rhumerusX", 0.6f},
        {"rhumerusX", "rradius", 0.5f},
        {"rradius", "rradiusX", 0.5f},
        {"rradiusX", "rhand", 0.4f},
        {"thoracic", "lclavical", 0.7f},
        {"lclavical", "lhumerus", 0.6f},
        {"lhumerus", "lhumerusX", 0.6f},
        {"lhumerusX", "lradius", 0.5f},
        {"lradius", "lradiusX", 0.5f},
        {"lradiusX", "lhand", 0.4f},
        {"pelvis", "rfemurYZ", 0.8f},
        {"rfemurYZ", "rfemurX", 0.8f},
        {"rfemurX", "rtibia", 0.7f},
        {"rtibia", "rtalus", 0.6f},
        {"rtalus", "rtail", 0.5f},
        {"pelvis", "lfemurYZ", 0.8f},
        {"lfemurYZ", "lfemurX", 0.8f},
        {"lfemurX", "ltibia", 0.7f},
        {"ltibia", "ltalus", 0.6f},
        {"ltalus", "ltail", 0.5f}
    };

    for (const auto& conn : connections) {
        CreateConstraint(conn.parent, conn.child, conn.strength);
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

void SimpleRagdoll::HandleCollisions() {
    int numManifolds = dispatcher->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
        const btCollisionObject* objA = contactManifold->getBody0();
        const btCollisionObject* objB = contactManifold->getBody1();

        for (auto& bone : bones) {
            if (bone.rigidBody == objA || bone.rigidBody == objB) {
                int numContacts = contactManifold->getNumContacts();
                for (int j = 0; j < numContacts; j++) {
                    btManifoldPoint& pt = contactManifold->getContactPoint(j);
                    if (pt.getDistance() < 0.0f) {
                        btVector3 normalForce = pt.m_normalWorldOnB * pt.m_appliedImpulse;
                        bone.rigidBody->applyCentralImpulse(normalForce * 0.1f);
                    }
                }
            }
        }
    }
}

void SimpleRagdoll::ApplyDamageForce(vec3_t direction, float damage, int mod) {
    if (!isEnabled) return;

    btVector3 force(direction[0], direction[1], direction[2]);
    force.normalize();

    float baseMult = 15.0f;
    switch (mod) {
    case MOD_EXPLOSIVE:
    case MOD_ROCKET:
        force *= damage * 25.0f;
        force[2] += damage * 0.5f;
        break;
    case MOD_SABER:
        force *= damage * 10.0f;
        force[2] += damage * 0.3f;
        break;
    default:
        force *= damage * baseMult;
        force[2] += damage * 0.2f;
    }

    for (auto& bone : bones) {
        if (!bone.rigidBody) continue;

        btVector3 boneForce = force;
        boneForce += btVector3(
            Q_flrand(-damage * 0.2f, damage * 0.2f),
            Q_flrand(-damage * 0.2f, damage * 0.2f),
            Q_flrand(0, damage * 0.3f)
        );

        bone.rigidBody->applyCentralImpulse(boneForce);
    }
}

void SimpleRagdoll::Enable(vec3_t force) {
    if (isEnabled) return;

    // Turn off bones anims
    gi.G2API_SetBoneAngles(&owner->ghoul2[owner->playerModel], "upper_lumbar", vec3_origin,
        BONE_ANGLES_POSTMULT, POSITIVE_X, NEGATIVE_Y, NEGATIVE_Z, NULL, 100, level.time);
    gi.G2API_SetBoneAngles(&owner->ghoul2[owner->playerModel], "lower_lumbar", vec3_origin,
        BONE_ANGLES_POSTMULT, POSITIVE_X, NEGATIVE_Y, NEGATIVE_Z, NULL, 100, level.time);
    gi.G2API_SetBoneAngles(&owner->ghoul2[owner->playerModel], "thoracic", vec3_origin,
        BONE_ANGLES_POSTMULT, POSITIVE_X, NEGATIVE_Y, NEGATIVE_Z, NULL, 100, level.time);
    gi.G2API_SetBoneAngles(&owner->ghoul2[owner->playerModel], "cervical", vec3_origin,
        BONE_ANGLES_POSTMULT, POSITIVE_X, NEGATIVE_Y, NEGATIVE_Z, NULL, 100, level.time);
    gi.G2API_SetBoneAngles(&owner->ghoul2[owner->playerModel], "pelvis", vec3_origin,
        BONE_ANGLES_POSTMULT, POSITIVE_X, NEGATIVE_Y, NEGATIVE_Z, NULL, 100, level.time);

    owner->client->ps.pm_type = PM_DEAD;
    owner->client->ps.pm_flags |= PMF_TIME_KNOCKBACK;
    owner->contents = CONTENTS_CORPSE;
    owner->clipmask = MASK_DEADSOLID;
    owner->takedamage = qfalse;

    owner->client->ps.stats[STAT_HEALTH] = 0;
    owner->client->ps.eFlags &= ~(EF_TELEPORT_BIT);
    owner->client->ps.pm_flags &= ~PMF_ALL_TIMES;
    gi.G2API_SetRagDoll(owner->ghoul2, NULL);

    owner->s.groundEntityNum = 0;

    for (auto& bone : bones) {
        if (bone.rigidBody) {
            // Collision callback
            bone.rigidBody->setCollisionFlags(bone.rigidBody->getCollisionFlags() |
                btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

            // Minimal height
            btTransform trans = bone.rigidBody->getWorldTransform();
            btVector3 origin = trans.getOrigin();
            if (origin.getZ() < -64.0f) {
                origin.setZ(-64.0f);
                trans.setOrigin(origin);
                bone.rigidBody->setWorldTransform(trans);
            }
        }
    }

    // Turn of root motion
    gi.G2API_SetBoneAnim(&owner->ghoul2[0], "model_root", 0, 1,
        BONE_ANIM_OVERRIDE_FREEZE, 0.0f, level.time, -1, 0);

    // Freeze anims
    float currentFrame;
    int startFrame, endFrame;
    int flags;
    float animSpeed;

    if (gi.G2API_GetBoneAnim(&owner->ghoul2[0], "model_root", level.time, &currentFrame,
        &startFrame, &endFrame, &flags, &animSpeed, NULL)) {
        gi.G2API_SetBoneAnim(&owner->ghoul2[0], "lower_lumbar", currentFrame, currentFrame + 1,
            flags, animSpeed, level.time, currentFrame, 500);
        gi.G2API_SetBoneAnim(&owner->ghoul2[0], "model_root", currentFrame, currentFrame + 1,
            flags, animSpeed, level.time, currentFrame, 500);
        gi.G2API_SetBoneAnim(&owner->ghoul2[0], "Motion", currentFrame, currentFrame + 1,
            flags, animSpeed, level.time, currentFrame, 500);
        gi.G2API_SetBoneAnim(&owner->ghoul2[0], "pelvis", currentFrame, currentFrame + 1,
            flags, animSpeed, level.time, currentFrame, 500);
    }

    btVector3 impactForce(force[0], force[1], force[2]);
    float forceMagnitude = impactForce.length();

    if (forceMagnitude > 2000.0f) {
        impactForce *= 2000.0f / forceMagnitude;
    }

    for (auto& bone : bones) {
        if (!bone.rigidBody) continue;

        bone.isActive = qtrue;
        bone.rigidBody->activate(true);

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

    isEnabled = true;
}

void SimpleRagdoll::UpdateCollisions() {
    trace_t trace;
    for (auto& bone : bones) {
        if (!bone.rigidBody) continue;

        btTransform trans = bone.rigidBody->getWorldTransform();
        btVector3 pos = trans.getOrigin();

        vec3_t start, end;
        VectorSet(start, pos.getX(), pos.getY(), pos.getZ());
        VectorSet(end, pos.getX(), pos.getY(), pos.getZ() - 10.0f);

        gi.trace(&trace, start, NULL, NULL, end, owner->s.number, MASK_SOLID, (EG2_Collision)0, 0);


        if (trace.fraction < 1.0f) {
            trans.setOrigin(btVector3(pos.getX(), pos.getY(), pos.getZ() + (10.0f * (1.0f - trace.fraction))));
            bone.rigidBody->setWorldTransform(trans);
            bone.rigidBody->setLinearVelocity(btVector3(0, 0, 0));
        }
    }
}

void SimpleRagdoll::Update(float deltaTime) {
    if (!isEnabled) return;

    dynamicsWorld->stepSimulation(deltaTime, 10);
    UpdateCollisions();
    HandleCollisions();

    const float maxStep = 1.0f / 60.0f;
    deltaTime = (deltaTime > maxStep) ? maxStep : deltaTime;

    dynamicsWorld->stepSimulation(deltaTime, 10, 1.0f / 240.0f);
    HandleCollisions();

    for (auto& bone : bones) {
        if (!bone.rigidBody) continue;

        btTransform trans;
        bone.rigidBody->getMotionState()->getWorldTransform(trans);

        // Position
        const btVector3& pos = trans.getOrigin();
        bone.position[0] = pos.getX();
        bone.position[1] = pos.getY();
        bone.position[2] = pos.getZ();

        // Angles
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
            static_cast<Eorientations>(BONE_ORIENT_X),
            static_cast<Eorientations>(BONE_ORIENT_Y),
            static_cast<Eorientations>(BONE_ORIENT_Z),
            NULL,
            100,
            level.time
        );

        // Speed
        const btVector3& vel = bone.rigidBody->getLinearVelocity();
        bone.velocity[0] = vel.getX();
        bone.velocity[1] = vel.getY();
        bone.velocity[2] = vel.getZ();
    }
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
    for (auto& bone : bones) {
        if (bone.rigidBody) {
            dynamicsWorld->removeRigidBody(bone.rigidBody);
            delete bone.rigidBody->getMotionState();
            delete bone.collisionShape;
            delete bone.rigidBody;
        }
    }

    while (dynamicsWorld->getNumConstraints() > 0) {
        btTypedConstraint* constraint = dynamicsWorld->getConstraint(0);
        dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }

    delete dynamicsWorld;
    delete solver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;
}