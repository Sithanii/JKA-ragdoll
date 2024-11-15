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

    // Ustawienie grawitacji - mniejsza niż wcześniej dla dłuższego lotu
    dynamicsWorld->setGravity(btVector3(0, 0, -800.0f));

    // Dodaj podłogę
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo groundRbInfo(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundBody = new btRigidBody(groundRbInfo);
    groundBody->setFriction(0.8f);
    groundBody->setRestitution(0.1f);  // Małe odbicie od podłoża
    btCollisionShape* spineShape = new btCapsuleShape(15.0f, 50.0f);  // Główny kształt dla tułowia
    btTransform spineTransform;
    spineTransform.setIdentity();
    spineTransform.setOrigin(btVector3(
        owner->client->ps.origin[0],
        owner->client->ps.origin[1],
        owner->client->ps.origin[2]
    ));

    btRigidBody* spineBody = new btRigidBody(btRigidBodyConstructionInfo(
        20.0f,  // Większa masa dla stabilności
        new btDefaultMotionState(spineTransform),
        spineShape,
        btVector3(0, 0, 0)
    ));
    dynamicsWorld->addRigidBody(groundBody);

    // Inicjalizacja kości
    const char* boneNames[] = {
        "pelvis", "thoracic", "cranium", "rhumerus", "lhumerus",
        "rradius", "lradius", "rfemur", "lfemur", "rtibia", "ltibia"
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
            bone.collisionShape = new btSphereShape(10.0f);

            // Pobierz pozycję kości
            mdxaBone_t boneMatrix;
            vec3_t angles = { 0, 0, 0 };
            gi.G2API_GetBoltMatrix(owner->ghoul2, 0, boneIndex, &boneMatrix, angles,
                owner->currentOrigin, level.time, NULL, owner->s.modelScale);

            btTransform transform;
            transform.setIdentity();
            transform.setOrigin(btVector3(
                boneMatrix.matrix[0][3],
                boneMatrix.matrix[1][3],
                boneMatrix.matrix[2][3]
            ));

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

            // Konfiguracja parametrów fizycznych
            bone.rigidBody->setDamping(0.9f, 0.9f);  // Większe tłumienie
            bone.rigidBody->setFriction(0.9f);     // Większe tarcie
            bone.rigidBody->setRestitution(0.1f);  // Mniejsze odbicie
            bone.rigidBody->setAngularFactor(btVector3(0.1f, 0.1f, 0.1f));
            bone.rigidBody->setSleepingThresholds(1.0f, 1.0f);
            bone.rigidBody->setActivationState(DISABLE_DEACTIVATION);

            // Dodaj rigid body do świata
            dynamicsWorld->addRigidBody(bone.rigidBody);

            bones.push_back(bone);

            Com_Printf("Added bone %s to ragdoll\n", boneName);
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
    if (isEnabled) return;

    // Duża siła w górę
    btVector3 btForce(force[0], force[1], force[2] + 2000.0f);

    for (auto& bone : bones) {
        if (bone.rigidBody) {
            bone.isActive = qtrue;
            bone.rigidBody->activate(true);

            // Siła głównie w górę dla wszystkich kości
            btVector3 upForce(0, 0, 2000.0f);
            bone.rigidBody->applyCentralForce(upForce);
            // Dodatkowa siła w kierunku śmierci
            bone.rigidBody->applyCentralForce(btForce * 0.3f);
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
            Com_Printf("Ragdoll settled and frozen\n");
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
        {"pelvis", "thoracic", 1.0f},
        {"thoracic", "cranium", 0.8f},
        {"thoracic", "rhumerus", 0.6f},
        {"thoracic", "lhumerus", 0.6f},
        {"rhumerus", "rradius", 0.4f},
        {"lhumerus", "lradius", 0.4f},
        {"pelvis", "rfemur", 0.8f},
        {"pelvis", "lfemur", 0.8f},
        {"rfemur", "rtibia", 0.6f},
        {"lfemur", "ltibia", 0.6f}
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

            // Limity dla ruchu liniowego
            constraint->setLinearLowerLimit(btVector3(-0.01f, -0.01f, -0.01f));
            constraint->setLinearUpperLimit(btVector3(0.01f, 0.01f, 0.01f));

            // Limity dla rotacji - dostosowane do typu połączenia
            if (strstr(conn.parent, "humerus") || strstr(conn.parent, "femur")) {
                // Większy zakres ruchu dla kończyn
                constraint->setAngularLowerLimit(btVector3(-0.5f, -0.5f, -0.5f));
                constraint->setAngularUpperLimit(btVector3(0.5f, 0.5f, 0.5f));
            }
            else {
                // Mniejszy zakres dla tułowia i głowy
                constraint->setAngularLowerLimit(btVector3(-0.05f, -0.05f, -0.05f));
                constraint->setAngularLowerLimit(btVector3(-0.05f, -0.05f, -0.05f));
            }

            // Konfiguracja sprężyn
            for (int i = 0; i < 6; i++) {
                constraint->enableSpring(i, true);
                constraint->setStiffness(i, 1000.0f);
                constraint->setDamping(i, 100.0f);
            }

            dynamicsWorld->addConstraint(constraint, true);
            Com_Printf("Created constraint between %s and %s\n", conn.parent, conn.child);
        }
    }
}

void SimpleRagdoll::ApplyForce(vec3_t force, int boneIndex) {
    if (!isEnabled || isSettled) return;

    if (boneIndex >= 0 && boneIndex < bones.size() && bones[boneIndex].rigidBody) {
        btVector3 btForce(force[0], force[1], force[2]);
        bones[boneIndex].rigidBody->activate(true);
        bones[boneIndex].rigidBody->applyCentralForce(btForce);
        Com_Printf("Additional force applied to bone %d\n", boneIndex);
    }
}