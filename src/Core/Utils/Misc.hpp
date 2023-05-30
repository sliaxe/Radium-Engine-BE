#pragma once

#include <Core/RaCore.hpp>
#include <Core/Types.hpp>
#include <Core/Math/Math.hpp>

namespace Ra {
namespace Core {
namespace Utils {
    
/// \return the quaternion representing the rotation between two given vectors p1 and p2.
Quaternion calculateQuaternion(const  Vector3& p1, const  Vector3& p2);

/// This function updates the maximum dot product value and corresponding axis vector based on a reference vector and an input axis vector
void updateMaxAndAxis ( Vector3 ref, Vector3 axis, Vector3& out, Scalar& max );

/// This function updates the target position based on the updated object position and direction, preserving the distance between the target and the object.
void updateTargetPosition(Vector3 position, Vector3 direction, Vector3& target);

/// \return the projection of a given point (x, y) onto a sphere of a specified size.
Scalar projectOnBall( Scalar x, Scalar y );

} // namespace Utils
} // namespace Core
} // namespace Ra
