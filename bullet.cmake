set(BULLET_ROOT "${CMAKE_SOURCE_DIR}/build/bullet3-master")
set(BULLET_INCLUDE_DIRS "${BULLET_ROOT}/src")
set(BULLET_LIBRARY_DIR "${BULLET_ROOT}/build/lib/Release")

set(BULLET_LIBRARIES
    "${BULLET_LIBRARY_DIR}/BulletDynamics.lib"
    "${BULLET_LIBRARY_DIR}/BulletCollision.lib"
    "${BULLET_LIBRARY_DIR}/LinearMath.lib"
    "${BULLET_LIBRARY_DIR}/BulletSoftBody.lib"
)