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
    isSettled = false;
    settleThreshold = 0.1f;
    gravity = -9.8f;

    // Inicjalizacja Bullet Physics
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    overlappingPairCache = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

    // Ustawienie grawitacji
    dynamicsWorld->setGravity(btVector3(0, 0, -980.0f));

    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(
        btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));

    btRigidBody::btRigidBodyConstructionInfo groundInfo(
        0,                  // masa = 0 dla obiektów statycznych
        groundMotionState,
        groundShape,
        btVector3(0, 0, 0)
    );

    btRigidBody* groundBody = new btRigidBody(groundInfo);
    groundBody->setFriction(0.8f);
    groundBody->setRestitution(0.1f);
    groundBody->setCollisionFlags(groundBody->getCollisionFlags() |
        btCollisionObject::CF_STATIC_OBJECT);
    dynamicsWorld->addRigidBody(groundBody);

    // Pozycja początkowa postaci
    btVector3 characterPos(
        owner->client->ps.origin[0],
        owner->client->ps.origin[1],
        owner->client->ps.origin[2]
    );

    // Inicjalizacja kości z pełną hierarchią
    const char* boneNames[] = {
        // Kręgosłup i głowa
        "pelvis",
        "lower_lumbar",
        "upper_lumbar",
        "thoracic",
        "cervical",
        "cranium",

        // Prawa strona
        "rclavical",
        "rhumerus",
        "rhumerusX",
        "rradius",
        "rradiusX",
        "rhand",

        // Lewa strona
        "lclavical",
        "lhumerus",
        "lhumerusX",
        "lradius",
        "lradiusX",
        "lhand",

        // Prawa noga
        "rfemurYZ",
        "rfemurX",
        "rtibia",
        "rtalus",
        "rtail",

        // Lewa noga
        "lfemurYZ",
        "lfemurX",
        "ltibia",
        "ltalus",
        "ltail"
    };

    for (const char* boneName : boneNames) {
        qhandle_t boneIndex = gi.G2API_GetBoneIndex(&owner->ghoul2[0], boneName, qtrue);
        if (boneIndex != -1) {
            Bone bone;
            Q_strncpyz(bone.boneName, boneName, sizeof(bone.boneName));
            bone.boneIndex = boneIndex;
            bone.mass = 10.0f;
            bone.isActive = qfalse;

            // Tworzenie collision shape dla kości
            bone.collisionShape = new btSphereShape(8.0f);

            // Pobierz pozycję kości
            mdxaBone_t boneMatrix;
            vec3_t angles = { 0, 0, 0 };
            gi.G2API_GetBoltMatrix(owner->ghoul2, 0, boneIndex, &boneMatrix, angles,
                owner->currentOrigin, level.time, NULL, owner->s.modelScale);

            // Oblicz pozycję względem postaci
            btVector3 boneOffset(
                boneMatrix.matrix[0][3],
                boneMatrix.matrix[1][3],
                boneMatrix.matrix[2][3]
            );

            btTransform transform;
            transform.setIdentity();
            transform.setOrigin(characterPos + boneOffset);

            // Zapisz pozycję początkową
            bone.position[0] = transform.getOrigin().getX();
            bone.position[1] = transform.getOrigin().getY();
            bone.position[2] = transform.getOrigin().getZ();
            VectorCopy(bone.position, bone.lastPosition);
            VectorClear(bone.velocity);

            // Utworzenie rigid body
            btVector3 localInertia(0, 0, 0);
            bone.collisionShape->calculateLocalInertia(bone.mass, localInertia);

            btMotionState* motionState = new btDefaultMotionState(transform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(bone.mass, motionState, bone.collisionShape, localInertia);
            bone.rigidBody = new btRigidBody(rbInfo);

            bone.rigidBody->setCollisionFlags(bone.rigidBody->getCollisionFlags() &
                ~btCollisionObject::CF_NO_CONTACT_RESPONSE);
            bone.rigidBody->setBroadphaseHandle(nullptr);

            // Konfiguracja parametrów fizycznych
            bone.rigidBody->setDamping(0.05f, 0.85f);
            bone.rigidBody->setFriction(0.8f);
            bone.rigidBody->setRestitution(0.3f);
            bone.rigidBody->setLinearFactor(btVector3(1, 1, 1));
            bone.rigidBody->setAngularFactor(btVector3(0.2f, 0.2f, 0.2f));
            bone.rigidBody->setActivationState(DISABLE_DEACTIVATION);

            // Konfiguracja parametrów fizycznych
            bone.rigidBody->setDamping(0.05f, 0.85f);
            bone.rigidBody->setFriction(0.8f);
            bone.rigidBody->setRestitution(0.3f);
            bone.rigidBody->setLinearFactor(btVector3(1, 1, 1));
            bone.rigidBody->setAngularFactor(btVector3(0.2f, 0.2f, 0.2f));
            bone.rigidBody->setActivationState(DISABLE_DEACTIVATION);

            dynamicsWorld->addRigidBody(bone.rigidBody);
            bones.push_back(bone);

            Com_Printf("Added bone %s to ragdoll at position %f %f %f\n",
                boneName, bone.position[0], bone.position[1], bone.position[2]);
        }
    }

    CreateBoneConstraints();
}

SimpleRagdoll::~SimpleRagdoll() {
    // Usuwanie constraints
    int numConstraints = dynamicsWorld->getNumConstraints();
    for (int i = numConstraints - 1; i >= 0; i--) {
        btTypedConstraint* constraint = dynamicsWorld->getConstraint(i);
        dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }

    // Usuwanie rigid bodies i powiązanych obiektów
    for (auto& bone : bones) {
        if (bone.rigidBody) {
            dynamicsWorld->removeRigidBody(bone.rigidBody);
            delete bone.rigidBody->getMotionState();
            delete bone.collisionShape;
            delete bone.rigidBody;
        }
    }

    delete dynamicsWorld;
    delete solver;
    delete overlappingPairCache;
    delete dispatcher;
    delete collisionConfiguration;
}

void SimpleRagdoll::Enable(vec3_t force) {
    if (isEnabled) {
        Com_Printf("Ragdoll is already enabled.\n");
        return;
    }

    Com_Printf("Enabling ragdoll with force: %f %f %f\n", force[0], force[1], force[2]);

    // Aktualizuj pozycję postaci
    btVector3 characterPos(
        owner->client->ps.origin[0],
        owner->client->ps.origin[1],
        owner->client->ps.origin[2]
    );

    // Oblicz kierunek siły
    btVector3 forceDir(force[0], force[1], force[2]);
    float forceMagnitude = forceDir.length();
    if (forceMagnitude > 0) {
        forceDir.normalize();
    }

    // Podstawowa siła w górę plus kierunek śmierci
    btVector3 upForce(0, 0, 1000.0f);
    btVector3 deathForce = forceDir * 500.0f;
    btVector3 totalForce = upForce + deathForce;

    for (auto& bone : bones) {
        if (bone.rigidBody) {
            bone.isActive = qtrue;

            // Aktualizuj pozycję kości względem postaci
            btTransform transform;
            bone.rigidBody->getMotionState()->getWorldTransform(transform);
            transform.setOrigin(characterPos + (transform.getOrigin() - characterPos));
            bone.rigidBody->getMotionState()->setWorldTransform(transform);

            bone.rigidBody->activate(true);

            // Różne siły dla różnych części ciała
            float forceMult = 1.0f;
            if (strcmp(bone.boneName, "pelvis") == 0) {
                // Pozwól na swobodny ruch podstawy
                bone.rigidBody->setAngularFactor(btVector3(1, 1, 1));
                bone.rigidBody->setLinearFactor(btVector3(1, 1, 1));
                bone.rigidBody->setFlags(0);
                bone.rigidBody->setActivationState(DISABLE_DEACTIVATION);

                // Zwiększ masę dla stabilności
                bone.mass = 20.0f;
                btVector3 localInertia(0, 0, 0);
                bone.collisionShape->calculateLocalInertia(bone.mass, localInertia);
                bone.rigidBody->setMassProps(bone.mass, localInertia);

                // Wyłącz kinematyczne ograniczenia
                bone.rigidBody->setCollisionFlags(bone.rigidBody->getCollisionFlags() &
                    ~btCollisionObject::CF_KINEMATIC_OBJECT);
            }
            else if (strstr(bone.boneName, "thoracic") || strstr(bone.boneName, "cranium")) {
                forceMult = 1.1f;
            }
            else if (strstr(bone.boneName, "femur") || strstr(bone.boneName, "tibia")) {
                forceMult = 0.9f;
            }

            bone.rigidBody->applyCentralForce(totalForce * forceMult);
        }
    }

    isEnabled = true;
    isSettled = false;
}

void SimpleRagdoll::Update(float deltaTime) {
    if (!isEnabled) return;

    const float maxTimeStep = 1.0f / 60.0f;
    deltaTime = (deltaTime > maxTimeStep) ? maxTimeStep : deltaTime;

    if (!isSettled) {
        dynamicsWorld->stepSimulation(deltaTime, 10);

        // Sprawdzanie czy ragdoll się uspokoił
        bool allBonesSettled = true;
        for (auto& bone : bones) {
            if (!bone.isActive || !bone.rigidBody) continue;

            const btVector3& vel = bone.rigidBody->getLinearVelocity();
            const btVector3& angVel = bone.rigidBody->getAngularVelocity();

            float linearSpeed = vel.length();
            float angularSpeed = angVel.length();

            if (linearSpeed > settleThreshold || angularSpeed > settleThreshold) {
                allBonesSettled = false;
                break;
            }
        }

        if (allBonesSettled) {
            isSettled = true;
            for (auto& bone : bones) {
                if (bone.rigidBody) {
                    bone.rigidBody->setActivationState(DISABLE_SIMULATION);
                }
            }
            Com_Printf("Ragdoll settled\n");
        }
    }

    for (auto& bone : bones) {
        if (!bone.isActive || !bone.rigidBody) continue;

        VectorCopy(bone.position, bone.lastPosition);

        btTransform trans;
        bone.rigidBody->getMotionState()->getWorldTransform(trans);

        // Aktualizacja pozycji
        btVector3 pos = trans.getOrigin();
        bone.position[0] = pos.getX();
        bone.position[1] = pos.getY();
        bone.position[2] = pos.getZ();

        // Aktualizacja prędkości
        const btVector3& vel = bone.rigidBody->getLinearVelocity();
        bone.velocity[0] = vel.getX();
        bone.velocity[1] = vel.getY();
        bone.velocity[2] = vel.getZ();

        // Konwersja rotacji
        btMatrix3x3 rot = trans.getBasis();
        btScalar yaw, pitch, roll;
        rot.getEulerYPR(yaw, pitch, roll);

        vec3_t angles;
        angles[0] = RAD2DEG(pitch);
        angles[1] = RAD2DEG(yaw);
        angles[2] = RAD2DEG(roll);

        // Aktualizacja kości w modelu
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
    }
}

void SimpleRagdoll::CreateBoneConstraints() {
    struct BoneConnection {
        const char* parent;
        const char* child;
        float strength;
    };

    BoneConnection connections[] = {
        // Kręgosłup i głowa
        {"pelvis", "lower_lumbar", 1.0f},
        {"lower_lumbar", "upper_lumbar", 1.0f},
        {"upper_lumbar", "thoracic", 1.0f},
        {"thoracic", "cervical", 0.8f},
        {"cervical", "cranium", 0.8f},

        // Prawa ręka
        {"thoracic", "rclavical", 0.7f},
        {"rclavical", "rhumerus", 0.6f},
        {"rhumerus", "rhumerusX", 0.6f},
        {"rhumerusX", "rradius", 0.5f},
        {"rradius", "rradiusX", 0.5f},
        {"rradiusX", "rhand", 0.4f},

        // Lewa ręka
        {"thoracic", "lclavical", 0.7f},
        {"lclavical", "lhumerus", 0.6f},
        {"lhumerus", "lhumerusX", 0.6f},
        {"lhumerusX", "lradius", 0.5f},
        {"lradius", "lradiusX", 0.5f},
        {"lradiusX", "lhand", 0.4f},

        // Prawa noga
        {"pelvis", "rfemurYZ", 0.8f},
        {"rfemurYZ", "rfemurX", 0.8f},
        {"rfemurX", "rtibia", 0.7f},
        {"rtibia", "rtalus", 0.6f},
        {"rtalus", "rtail", 0.5f},

        // Lewa noga
        {"pelvis", "lfemurYZ", 0.8f},
        {"lfemurYZ", "lfemurX", 0.8f},
        {"lfemurX", "ltibia", 0.7f},
        {"ltibia", "ltalus", 0.6f},
        {"ltalus", "ltail", 0.5f}
    };

    for (const auto& conn : connections) {
        Bone* parentBone = nullptr;
        Bone* childBone = nullptr;

        for (auto& bone : bones) {
            if (strcmp(bone.boneName, conn.parent) == 0) parentBone = &bone;
            if (strcmp(bone.boneName, conn.child) == 0) childBone = &bone;
        }

        if (parentBone && childBone && parentBone->rigidBody && childBone->rigidBody) {
            btTransform frameInA, frameInB;
            frameInA.setIdentity();
            frameInB.setIdentity();

            btGeneric6DofSpringConstraint* constraint = new btGeneric6DofSpringConstraint(
                *parentBone->rigidBody,
                *childBone->rigidBody,
                frameInA,
                frameInB,
                true
            );

            // Bardziej elastyczne limity
            constraint->setLinearLowerLimit(btVector3(-0.2f, -0.2f, -0.2f));
            constraint->setLinearUpperLimit(btVector3(0.2f, 0.2f, 0.2f));
            constraint->setAngularLowerLimit(btVector3(-0.8f, -0.8f, -0.8f));
            constraint->setAngularUpperLimit(btVector3(0.8f, 0.8f, 0.8f));

            // Dostosowane parametry sprężyn
            for (int i = 0; i < 6; i++) {
                constraint->enableSpring(i, true);
                constraint->setStiffness(i, 200.0f * conn.strength);
                constraint->setDamping(i, 5.0f * conn.strength);
            }

            dynamicsWorld->addConstraint(constraint, true);
            Com_Printf("Created constraint between %s and %s\n", conn.parent, conn.child);
        }
    }
}

void SimpleRagdoll::ApplyForce(vec3_t force, int boneIndex) {
    if (!isEnabled) return;

    // Aplikuj siłę do wszystkich kości w miejscu uderzenia
    btVector3 impactForce(force[0], force[1], force[2]);

    for (auto& bone : bones) {
        if (bone.rigidBody) {
            float distanceFactor = 1.0f;
            if (boneIndex != -1) {
                // Jeśli znamy konkretną kość, siła maleje z odległością
                distanceFactor = (bone.boneIndex == boneIndex) ? 1.0f : 0.5f;
            }
            bone.rigidBody->activate(true);
            bone.rigidBody->applyCentralImpulse(impactForce * distanceFactor);
        }
    }
}