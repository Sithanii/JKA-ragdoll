#include "physics_ragdoll.h"

// Użyjmy stałych z OpenJK zamiast własnych enumów
#define POSITIVE_X BONE_ANGLES_POSTMULT
#define NEGATIVE_Y BONE_ANGLES_POSTMULT
#define NEGATIVE_Z BONE_ANGLES_POSTMULT

struct BoneInfo {
    const char* name;
    float mass;
};

static BoneInfo boneMapping[] = {
    {"pelvis", 3.0f},
    {"thoracic", 2.5f},
    {"cervical", 1.0f},
    {"cranium", 1.0f},
    {"rhumerus", 0.75f},
    {"lhumerus", 0.75f},
    {"rfemuryx", 1.0f},
    {"lfemuryx", 1.0f}
};

SimpleRagdoll::SimpleRagdoll(gentity_t* ent) {
    owner = ent;
    isEnabled = false;
    gravity = -9.8f * 0.01f;

    // Initialize Bullet Physics
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
    overlappingPairCache = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver;
    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

    // Inicjalizacja kości
    for (const auto& mapping : boneMapping) {
        qhandle_t boneIndex = gi.G2API_GetBoneIndex(&owner->ghoul2[0], mapping.name, qtrue);
        if (boneIndex != -1) {
            Bone bone;
            bone.boneIndex = boneIndex;
            Q_strncpyz(bone.boneName, mapping.name, sizeof(bone.boneName));
            bone.mass = mapping.mass;
            bone.isActive = qfalse;

            mdxaBone_t boneMatrix;
            vec3_t angles;
            VectorSet(angles, 0, owner->client->ps.viewangles[YAW], 0);

            // Pobieranie macierzy transformacji kości
            gi.G2API_GetBoltMatrix(
                owner->ghoul2[0],         // ghoul2
                0,                        // modelIndex
                boneIndex,                // boltIndex
                &boneMatrix,              // matrix
                angles,                   // angles
                owner->client->ps.origin, // position
                level.time,               // frameNum
                nullptr,                  // modelList
                owner->s.modelScale       // scale
            );

            // Ekstrakcja pozycji z macierzy
            bone.position[0] = boneMatrix.matrix[0][3];
            bone.position[1] = boneMatrix.matrix[1][3];
            bone.position[2] = boneMatrix.matrix[2][3];

            // Inicjalizacja pozostałych parametrów kości
            VectorCopy(bone.position, bone.lastPosition);
            VectorClear(bone.velocity);

            // Tworzenie obiektu fizycznego dla kości
            btCollisionShape* shape = new btSphereShape(0.1f);
            btTransform transform;
            transform.setIdentity();
            transform.setOrigin(btVector3(bone.position[0], bone.position[1], bone.position[2]));

            btVector3 localInertia(0, 0, 0);
            if (bone.mass != 0.0f) {
                shape->calculateLocalInertia(bone.mass, localInertia);
            }

            btDefaultMotionState* motionState = new btDefaultMotionState(transform);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(bone.mass, motionState, shape, localInertia);
            bone.rigidBody = new btRigidBody(rbInfo);

            dynamicsWorld->addRigidBody(bone.rigidBody);
            bones.push_back(bone);
        }
    }

    CreateBoneConstraints();
}

void SimpleRagdoll::CreateBoneConstraints() {
    // Znajdź kości do połączenia
    Bone* pelvis = nullptr;
    Bone* thoracic = nullptr;

    for (auto& bone : bones) {
        if (Q_stricmp(bone.boneName, "pelvis") == 0) {
            pelvis = &bone;
        }
        else if (Q_stricmp(bone.boneName, "thoracic") == 0) {
            thoracic = &bone;
        }
    }

    if (pelvis && thoracic) {
        btTransform frameInA, frameInB;
        frameInA.setIdentity();
        frameInB.setIdentity();

        btGeneric6DofConstraint* constraint = new btGeneric6DofConstraint(
            *pelvis->rigidBody,
            *thoracic->rigidBody,
            frameInA,
            frameInB,
            true
        );

        constraint->setLinearLowerLimit(btVector3(-0.1f, -0.1f, -0.1f));
        constraint->setLinearUpperLimit(btVector3(0.1f, 0.1f, 0.1f));
        constraint->setAngularLowerLimit(btVector3(-0.2f, -0.2f, -0.2f));
        constraint->setAngularUpperLimit(btVector3(0.2f, 0.2f, 0.2f));

        dynamicsWorld->addConstraint(constraint);
    }
}

void SimpleRagdoll::Update(float deltaTime) {
    if (!isEnabled) return;

    const float maxDelta = 0.05f;
    deltaTime = (deltaTime > maxDelta) ? maxDelta : deltaTime;

    dynamicsWorld->stepSimulation(deltaTime, 10);

    for (auto& bone : bones) {
        if (!bone.isActive || !bone.rigidBody) continue;

        btTransform transform;
        bone.rigidBody->getMotionState()->getWorldTransform(transform);
        btVector3 position = transform.getOrigin();

        bone.position[0] = position.x();
        bone.position[1] = position.y();
        bone.position[2] = position.z();

        // Konwersja do kątów Eulera
        btQuaternion rotation = transform.getRotation();
        btMatrix3x3 rotMatrix(rotation);
        float yaw, pitch, roll;
        rotMatrix.getEulerYPR(yaw, pitch, roll);

        vec3_t angles;
        angles[0] = RAD2DEG(pitch);
        angles[1] = RAD2DEG(yaw);
        angles[2] = RAD2DEG(roll);

        // Aktualizacja kości w modelu
        gi.G2API_SetBoneAngles(
            &owner->ghoul2[0],    // ghoul2
            0,                    // modelIndex
            bone.boneName,        // boneName
            angles,              // angles
            BONE_ANGLES_POSTMULT, // flags
            POSITIVE_X,          // up
            NEGATIVE_Y,          // right
            NEGATIVE_Z,          // forward
            NULL,                // modelList
            100,                 // blendTime
            level.time           // currentTime
        );
    }
}

SimpleRagdoll::~SimpleRagdoll() {
    for (int i = dynamicsWorld->getNumConstraints() - 1; i >= 0; i--) {
        btTypedConstraint* constraint = dynamicsWorld->getConstraint(i);
        dynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }

    for (auto& bone : bones) {
        if (bone.rigidBody) {
            dynamicsWorld->removeRigidBody(bone.rigidBody);
            delete bone.rigidBody->getMotionState();
            delete bone.rigidBody->getCollisionShape();
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
    if (!isEnabled) {
        isEnabled = true;
        for (auto& bone : bones) {
            bone.isActive = qtrue;
            if (bone.rigidBody) {
                vec3_t scaledForce;
                VectorScale(force, bone.mass, scaledForce);
                bone.rigidBody->activate(true);
                bone.rigidBody->applyCentralForce(btVector3(scaledForce[0], scaledForce[1], scaledForce[2]));
            }
        }
    }
}

void SimpleRagdoll::ApplyForce(vec3_t force, int boneIndex) {
    if (!isEnabled) return;

    for (auto& bone : bones) {
        if (bone.boneIndex == boneIndex && bone.isActive && bone.rigidBody) {
            bone.rigidBody->activate(true);
            bone.rigidBody->applyCentralForce(btVector3(force[0], force[1], force[2]));
            break;
        }
    }
}