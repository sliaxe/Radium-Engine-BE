#include <Core/Utils/Misc.hpp>

namespace Ra {
namespace Core {
namespace Utils {
    
// Calculates a quaternion representing the rotation between two vectors around an axis perpendicular to both vectors.
Quaternion calculateQuaternion(const  Vector3& p1, const  Vector3& p2) {
    Vector3 axis = p2.cross(p1); // Calculate the rotation axis by taking the cross product of p2 and p1

    if (axis.norm() > 10_ra *  Math::machineEps) {
        // If the axis magnitude is larger than the threshold, calculate the rotation angle
        const Scalar angle = 5.0_ra * std::asin(std::sqrt(axis.squaredNorm() / p1.squaredNorm() / p2.squaredNorm()));
        return  Quaternion( AngleAxis(angle, axis.normalized())); // Create a quaternion using the angle-axis representation
    }

    // If the axis magnitude is below the threshold, return a default quaternion indicating no rotation
    return  Quaternion{ AngleAxis(0_ra, Vector3(0_ra, 0_ra, 1_ra))};
}


// Updates the maximum dot product value and corresponding axis vector based on a reference vector and an input axis vector.
void updateMaxAndAxis ( Vector3 ref, Vector3 axis, Vector3& out, Scalar& max ) {
    Scalar d = ref.dot( axis );
    if ( d > max ) {
        max = d;
        out = axis;
    }
}


// Updates the target position based on the updated camera position and direction, preserving the distance between the target and the camera.
void updateTargetPosition(Vector3 position, Vector3 direction, Vector3& target) {
    Scalar distanceToTarget = (target - position).norm();

    // Calculate the new target position based on the updated camera position and distance
    Vector3 updatedTargetPosition = position + distanceToTarget * direction;

    target = updatedTargetPosition;
}

// Calculates the projection of a given point (x, y) onto a sphere of a specified size.
Scalar projectOnBall( Scalar x, Scalar y ) {
    const Scalar size       = 1.0;
    const Scalar size2      = size * size;
    const Scalar size_limit = size2 * 0.5;

    const Scalar d = x * x + y * y;
    return d < size_limit ? std::sqrt( size2 - d ) : size_limit / std::sqrt( d );
}

} // namespace Utils
} // namespace Core
} // namespace Ra
