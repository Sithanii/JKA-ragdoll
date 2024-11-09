#include "physics_ragdoll.h"
#include "../qcommon/qcommon.h"
#include "ghoul2/G2.h"

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

        btQuaternion rotation = transform.getRotation();
        btMatrix3x3 rotMatrix(rotation);
        float yaw, pitch, roll;
        rotMatrix.getEulerYPR(yaw, pitch, roll);

        vec3_t angles;
        angles[0] = RAD2DEG(pitch);
        angles[1] = RAD2DEG(yaw);
        angles[2] = RAD2DEG(roll);

        // Użycie enuma Eorientations z OpenJK
        gi.G2API_SetBoneAngles(
            &owner->ghoul2[0],    // ghoul2
            bone.boneName,        // boneName
            angles,               // angles
            BONE_ANGLES_POSTMULT, // flags
            static_cast<Eorientations>(0),  // up (BONE_UP)
            static_cast<Eorientations>(1),  // right (BONE_RIGHT)
            static_cast<Eorientations>(2),  // forward (BONE_FRONT)
            nullptr,             // modelList
            100,                 // blendTime
            level.time          // currentTime
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