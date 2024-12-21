#include "physics_ragdoll.h"

SimpleRagdoll::SimpleRagdoll(gentity_t* ent) : owner(ent), isEnabled(false) {
    Initialize();
}

SimpleRagdoll::~SimpleRagdoll() {
    // Usuñ wszystkie constraints
    for (auto constraint : constraints) {
        dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }
    constraints.clear();

    // Usuñ wszystkie koœci
    for (auto& bone : bones) {
        if (bone.rigidBody) {
            dynamicsWorld->removeRigidBody(bone.rigidBody);
            delete bone.rigidBody->getMotionState();
            delete bone.rigidBody;
            delete bone.collisionShape;
        }
    }
    bones.clear();

    // Usuñ pod³o¿e
    if (groundBody) {
        dynamicsWorld->removeRigidBody(groundBody);
        delete groundBody->getMotionState();
        delete groundBody->getCollisionShape();
        delete groundBody;
    }

    // Usuñ œwiat fizyki
    delete dynamicsWorld;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete collisionConfig;
}

void SimpleRagdoll::Initialize() {
    // Inicjalizacja œwiata fizyki
    collisionConfig = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfig);
    broadphase = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
    // Zwiêkszona grawitacja
    dynamicsWorld->setGravity(btVector3(0, 0, -1600.0f));

    // Wiêcej iteracji dla lepszej detekcji kolizji
    btContactSolverInfo& solverInfo = dynamicsWorld->getSolverInfo();
    solverInfo.m_numIterations = 20;
    solverInfo.m_splitImpulse = true;
    solverInfo.m_splitImpulsePenetrationThreshold = -0.1f;
    solverInfo.m_erp = 0.2f;  // Dodane dla lepszej penetracji
    solverInfo.m_erp2 = 0.8f; // Dodane dla lepszej stabilnoœci

    // Lepsze pod³o¿e
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), -1);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -1)));

    btRigidBody::btRigidBodyConstructionInfo groundRbInfo(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    groundBody = new btRigidBody(groundRbInfo);
    groundBody->setFriction(1.0f);
    groundBody->setRestitution(0.0f);  // Brak odbicia
    groundBody->setCollisionFlags(groundBody->getCollisionFlags() |
        btCollisionObject::CF_STATIC_OBJECT |
        btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
    dynamicsWorld->addRigidBody(groundBody);

    // Inicjalizacja koœci ragdolla
    const char* boneNames[] = {
        "pelvis",
        "lower_lumbar", "upper_lumbar", "thoracic",
        "cranium",
        "rhumerus", "rradius", "rhand",
        "lhumerus", "lradius", "lhand",
        "rfemur", "rtibia", "rtalus",
        "lfemur", "ltibia", "ltalus"
    };

    for (const char* boneName : boneNames) {
        InitializeBone(boneName);
    }

    // Stwórz po³¹czenia miêdzy koœæmi
    CreateConstraints();
}

void SimpleRagdoll::InitializeBone(const char* boneName) {
    if (!owner || !owner->client || !owner->ghoul2.size()) return;

    qhandle_t boneIndex = gi.G2API_GetBoneIndex(&owner->ghoul2[0], boneName, qtrue);
    if (boneIndex == -1) return;

    RagdollBone bone = {};
    Q_strncpyz(bone.boneName, boneName, sizeof(bone.boneName));
    bone.boneIndex = boneIndex;
    bone.isActive = qfalse;

    // Pobierz pozycjê koœci
    mdxaBone_t boneMatrix;
    vec3_t angles;
    VectorSet(angles, 0, owner->client->ps.viewangles[YAW], 0);

    gi.G2API_GetBoltMatrix(
        owner->ghoul2, 0, boneIndex,
        &boneMatrix, angles, owner->currentOrigin,
        level.time, NULL, owner->s.modelScale
    );

    bone.position[0] = boneMatrix.matrix[0][3];
    bone.position[1] = boneMatrix.matrix[1][3];
    bone.position[2] = boneMatrix.matrix[2][3];

    bone.initialTransform.setIdentity();
    bone.initialTransform.setOrigin(btVector3(bone.position[0], bone.position[1], bone.position[2]));

    // Ustaw parametry fizyki dla ka¿dej koœci
    if (strcmp(boneName, "pelvis") == 0) {
        bone.mass = 15.0f;
        bone.collisionShape = new btCapsuleShape(4.0f, 8.0f);
    }
    else if (strstr(boneName, "lumbar") || strcmp(boneName, "thoracic") == 0) {
        bone.mass = 10.0f;
        bone.collisionShape = new btCapsuleShape(3.5f, 7.0f);
    }
    else if (strstr(boneName, "femur")) {
        bone.mass = 8.0f;
        bone.collisionShape = new btCapsuleShape(3.0f, 14.0f);
    }
    else if (strstr(boneName, "tibia")) {
        bone.mass = 6.0f;
        bone.collisionShape = new btCapsuleShape(2.5f, 12.0f);
    }
    else if (strstr(boneName, "humerus")) {
        bone.mass = 4.0f;
        bone.collisionShape = new btCapsuleShape(2.0f, 10.0f);
    }
    else if (strstr(boneName, "radius")) {
        bone.mass = 2.0f;
        bone.collisionShape = new btCapsuleShape(1.5f, 8.0f);
    }
    else if (strcmp(boneName, "cranium") == 0) {
        bone.mass = 5.0f;
        bone.collisionShape = new btSphereShape(4.0f);
    }
    else {
        bone.mass = 1.0f;
        bone.collisionShape = new btCapsuleShape(1.0f, 4.0f);
    }

    btVector3 localInertia(0, 0, 0);
    bone.collisionShape->calculateLocalInertia(bone.mass, localInertia);

    btDefaultMotionState* motionState = new btDefaultMotionState(bone.initialTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(bone.mass, motionState, bone.collisionShape, localInertia);

    rbInfo.m_friction = 0.8f;
    rbInfo.m_restitution = 0.1f;
    rbInfo.m_linearDamping = 0.05f;
    rbInfo.m_angularDamping = 0.85f;

    bone.rigidBody = new btRigidBody(rbInfo);
    bone.rigidBody->setActivationState(DISABLE_DEACTIVATION);

    dynamicsWorld->addRigidBody(bone.rigidBody);
    bones.push_back(bone);
}

void SimpleRagdoll::CreateConstraints() {
    struct ConstraintInfo {
        const char* parent;
        const char* child;
        btVector3 axis;
        float limit;
    };

    ConstraintInfo constraintInfos[] = {
        {"pelvis", "lower_lumbar", btVector3(0,0,1), 30.0f},
        {"lower_lumbar", "upper_lumbar", btVector3(0,0,1), 30.0f},
        {"upper_lumbar", "thoracic", btVector3(0,0,1), 30.0f},
        {"thoracic", "cranium", btVector3(0,0,1), 45.0f},
        {"thoracic", "rhumerus", btVector3(1,0,0), 90.0f},
        {"thoracic", "lhumerus", btVector3(1,0,0), 90.0f},
        {"rhumerus", "rradius", btVector3(0,1,0), 120.0f},
        {"lhumerus", "lradius", btVector3(0,1,0), 120.0f},
        {"pelvis", "rfemur", btVector3(1,0,0), 90.0f},
        {"pelvis", "lfemur", btVector3(1,0,0), 90.0f},
        {"rfemur", "rtibia", btVector3(0,1,0), 120.0f},
        {"lfemur", "ltibia", btVector3(0,1,0), 120.0f}
    };

    for (const auto& info : constraintInfos) {
        RagdollBone* parent = nullptr;
        RagdollBone* child = nullptr;

        for (auto& bone : bones) {
            if (strcmp(bone.boneName, info.parent) == 0) parent = &bone;
            if (strcmp(bone.boneName, info.child) == 0) child = &bone;
        }

        if (parent && child) {
            CreateConstraint(parent, child, info.axis, info.limit);
        }
    }
}

void SimpleRagdoll::CreateConstraint(RagdollBone* parent, RagdollBone* child, const btVector3& axis, float limit) {
    btTransform parentFrame, childFrame;
    parentFrame.setIdentity();
    childFrame.setIdentity();

    btVector3 parentPos = parent->rigidBody->getCenterOfMassPosition();
    btVector3 childPos = child->rigidBody->getCenterOfMassPosition();

    btVector3 center = (parentPos + childPos) * 0.5f;

    parentFrame.setOrigin(parent->rigidBody->getCenterOfMassTransform().inverse()(center));
    childFrame.setOrigin(child->rigidBody->getCenterOfMassTransform().inverse()(center));

    btConeTwistConstraint* constraint = new btConeTwistConstraint(
        *parent->rigidBody,
        *child->rigidBody,
        parentFrame,
        childFrame
    );

    float limitRad = DEG2RAD(limit);
    constraint->setLimit(limitRad, limitRad, limitRad);
    constraint->setDamping(1.0f);

    dynamicsWorld->addConstraint(constraint, true);
    constraints.push_back(constraint);
}

void SimpleRagdoll::Enable(vec3_t force) {
    if (isEnabled) return;
    isEnabled = true;

    // Zachowaj oryginaln¹ orientacjê
    btQuaternion initialRot;
    initialRot.setEulerZYX(
        0, // roll
        DEG2RAD(owner->client->ps.viewangles[YAW]), // yaw
        0  // pitch
    );

    btVector3 impactForce(force[0], force[1], force[2]);
    float forceMagnitude = impactForce.length();

    // Dostosuj si³ê impactu
    if (forceMagnitude < 200.0f) {
        impactForce = btVector3(force[0], force[1], force[2]);
        impactForce.normalize();
        impactForce *= 200.0f;
    }
    else {
        impactForce *= 2.0f;  // Zmniejszone z 3.0f
    }

    // Mniejsza dodatkowa si³a w górê
    impactForce.setZ(impactForce.getZ() + 100.0f);  // Zmniejszone z 200.0f

    for (auto& bone : bones) {
        if (!bone.rigidBody) continue;
        bone.isActive = qtrue;

        // Zachowaj pocz¹tkow¹ orientacjê dla ka¿dej koœci
        btTransform current = bone.initialTransform;
        current.setRotation(initialRot);
        bone.rigidBody->setWorldTransform(current);

        // Zablokuj kompletnie rotacjê w osiach X i Z na pocz¹tku
        bone.rigidBody->setAngularFactor(btVector3(0, 1, 0));

        // Aplikuj si³ê zale¿nie od koœci
        btVector3 totalForce;
        if (strcmp(bone.boneName, "pelvis") == 0) {
            totalForce = impactForce * 1.0f;  // Zwiêkszone z 0.8f
            bone.rigidBody->setAngularFactor(btVector3(0, 1, 0));
            bone.rigidBody->setDamping(0.3f, 0.3f);  // Zmniejszone t³umienie
        }
        else if (strstr(bone.boneName, "lumbar") || strcmp(bone.boneName, "thoracic") == 0) {
            totalForce = impactForce * 0.9f;  // Zwiêkszone z 0.7f
            bone.rigidBody->setAngularFactor(btVector3(0, 1, 0));
            bone.rigidBody->setDamping(0.3f, 0.3f);
        }
        else {
            totalForce = impactForce * 0.8f;  // Zwiêkszone z 0.6f
            bone.rigidBody->setDamping(0.3f, 0.3f);
        }

        // Dodaj si³ê w kierunku uderzenia
        bone.rigidBody->setLinearVelocity(totalForce * 0.4f);  // Zwiêkszone z 0.3f
        bone.rigidBody->applyCentralImpulse(totalForce);
    }

    bones[0].rigidBody->setActivationState(DISABLE_DEACTIVATION);
}

void SimpleRagdoll::Update(float deltaTime) {
    if (!isEnabled || !owner || !owner->client || !dynamicsWorld) return;

    static float timeElapsed = 0.0f;
    timeElapsed += deltaTime;

    const float maxTimeStep = 1.0f / 60.0f;
    deltaTime = (deltaTime > maxTimeStep) ? maxTimeStep : deltaTime;

    dynamicsWorld->stepSimulation(deltaTime, 10, 1.0f / 60.0f);

    // Stopniowo odblokowuj rotacjê po 0.2 sekundy
    if (timeElapsed > 0.2f && timeElapsed < 0.7f) {
        for (auto& bone : bones) {
            if (!bone.rigidBody) continue;

            float factor = (timeElapsed - 0.2f) / 0.5f;
            factor = factor > 1.0f ? 1.0f : factor;

            if (strcmp(bone.boneName, "pelvis") == 0) {
                bone.rigidBody->setAngularFactor(btVector3(0.3f * factor, 1, 0.3f * factor));
            }
            else {
                bone.rigidBody->setAngularFactor(btVector3(factor, 1, factor));
            }
        }
    }

    // SprawdŸ czy postaæ jest na ziemi
    bool isOnGround = false;
    static float onGroundTime = 0.0f;

    for (auto& bone : bones) {
        if (!bone.rigidBody) continue;

        btTransform trans;
        bone.rigidBody->getMotionState()->getWorldTransform(trans);
        const btVector3& pos = trans.getOrigin();
        bone.position[0] = pos.getX();
        bone.position[1] = pos.getY();
        bone.position[2] = pos.getZ();

        // SprawdŸ kolizje ze œcianami
        trace_t trace;
        vec3_t start, end;
        VectorCopy(bone.position, start);
        VectorCopy(bone.position, end);

        gi.trace(&trace, start, NULL, NULL, end, owner->s.number, MASK_SOLID, (EG2_Collision)0, 0);
        if (trace.startsolid || trace.allsolid) {
            // Jeœli koœæ jest w œcianie, wypchnij j¹
            btVector3 pushDir(trace.plane.normal[0], trace.plane.normal[1], trace.plane.normal[2]);
            bone.rigidBody->applyCentralImpulse(pushDir * 100.0f);
        }

        // SprawdŸ kolizjê z pod³o¿em
        if (pos.getZ() < 1.0f) {
            isOnGround = true;
            if (pos.getZ() < -1.0f) {
                btTransform newTrans = trans;
                newTrans.getOrigin().setZ(-1.0f);
                bone.rigidBody->getMotionState()->setWorldTransform(newTrans);
                bone.rigidBody->setWorldTransform(newTrans);

                btVector3 velocity = bone.rigidBody->getLinearVelocity();
                if (velocity.getZ() < 0) {
                    velocity.setZ(0);
                    bone.rigidBody->setLinearVelocity(velocity);
                }
            }
        }

        // Ogranicz maksymaln¹ prêdkoœæ
        btVector3 velocity = bone.rigidBody->getLinearVelocity();
        float speed = velocity.length();
        const float maxSpeed = 800.0f;  // Zmniejszona maksymalna prêdkoœæ
        if (speed > maxSpeed) {
            velocity *= (maxSpeed / speed);
            bone.rigidBody->setLinearVelocity(velocity);
        }

        // Zwiêksz t³umienie jeœli postaæ jest na ziemi
        if (isOnGround) {
            onGroundTime += deltaTime;
            if (onGroundTime > 1.0f) {  // Po sekundzie na ziemi
                bone.rigidBody->setDamping(1.5f, 1.5f);  // Zwiêksz t³umienie
                // Ogranicz prêdkoœæ k¹tow¹
                btVector3 angVel = bone.rigidBody->getAngularVelocity();
                if (angVel.length() > 3.0f) {
                    angVel *= 3.0f / angVel.length();
                    bone.rigidBody->setAngularVelocity(angVel);
                }
            }
        }

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

        if (strcmp(bone.boneName, "pelvis") == 0) {
            VectorCopy(bone.position, owner->currentOrigin);
            VectorCopy(bone.position, owner->s.origin);
            VectorCopy(bone.position, owner->s.pos.trBase);
            gi.linkentity(owner);
        }
    }
}

void SimpleRagdoll::ApplyForce(vec3_t force, int boneIndex) {
    if (!isEnabled || boneIndex >= bones.size()) return;

    auto& bone = bones[boneIndex];
    if (!bone.rigidBody) return;

    bone.rigidBody->applyCentralImpulse(btVector3(force[0], force[1], force[2]));
}