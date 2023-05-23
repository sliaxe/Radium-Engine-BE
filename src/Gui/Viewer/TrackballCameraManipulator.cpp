#include <Gui/Viewer/TrackballCameraManipulator.hpp>

#include <Core/Asset/Camera.hpp>
#include <Core/Math/Math.hpp>
#include <Core/Utils/Log.hpp>
#include <Engine/RadiumEngine.hpp>
#include <Engine/Rendering/RenderObject.hpp>
#include <Engine/Rendering/RenderObjectManager.hpp>
#include <Engine/Scene/Light.hpp>
#include <Engine/Scene/SystemDisplay.hpp>
#include <Gui/Utils/KeyMappingManager.hpp>
#include <Gui/Utils/Keyboard.hpp>

#include <Engine/Scene/CameraComponent.hpp>
#include <QApplication>
#include <QMessageBox>
#include <algorithm>
#include <iostream>

namespace Ra {

using Core::Math::Pi;
using namespace Ra::Core::Utils;

namespace Gui {

#define KMA_VALUE( XX ) KeyMappingManager::KeyMappingAction TrackballCameraManipulator::XX;
KeyMappingCamera
#undef KMA_VALUE

void TrackballCameraManipulator::configureKeyMapping_impl()
{
    // Sets the camera context using the KeyMappingManager.
    KeyMapping::setContext(KeyMappingManager::getInstance()->getContext("CameraContext"));

    // If the context is invalid, logs an error message.
    if (KeyMapping::getContext().isInvalid())
    {
        LOG(logINFO) << "CameraContext not defined (maybe the configuration file do not contains it)";
        LOG(logERROR) << "CameraContext all keymapping invalide !";
        return;
    }

    // Macro for mapping keyboard keys to camera actions, using the key names as identifiers.
    // This mapping is stored in the KeyMappingCamera structure.
#define KMA_VALUE(XX) \
    XX = KeyMappingManager::getInstance()->getAction(KeyMapping::getContext(), #XX);
    KeyMappingCamera
#undef KMA_VALUE
}

void TrackballCameraManipulator::setupKeyMappingCallbacks() {

    // Add a callback for camera rotation
    m_keyMappingCallbackManager.addEventCallback(
        TRACKBALLCAMERA_ROTATE, [=]( QEvent* event ) { rotateCallback( event ); } );

    // Add a callback for camera panning
    m_keyMappingCallbackManager.addEventCallback( TRACKBALLCAMERA_PAN,
                                                  [=]( QEvent* event ) { panCallback( event ); } );

    // Add a callback for camera zooming
    m_keyMappingCallbackManager.addEventCallback( TRACKBALLCAMERA_ZOOM,
                                                  [=]( QEvent* event ) { zoomCallback( event ); } );

    // Add a callback for moving the camera forward
    m_keyMappingCallbackManager.addEventCallback(
        TRACKBALLCAMERA_MOVE_FORWARD, [=]( QEvent* event ) { moveForwardCallback( event ); } );

    // Add a callback for changing the camera's projection type
    m_keyMappingCallbackManager.addEventCallback( TRACKBALLCAMERA_PROJ_MODE, [=]( QEvent* ) {
        using ProjType = Ra::Core::Asset::Camera::ProjType;
        m_camera->setType( m_camera->getType() == ProjType::ORTHOGRAPHIC ? ProjType::PERSPECTIVE
                                                                         : ProjType::ORTHOGRAPHIC );
    } );


    // Add a callback for activating/disabling the camera's fast mode
    m_keyMappingCallbackManager.addEventCallback( CAMERA_TOGGLE_QUICK, [=]( QEvent* ) {
        static bool quick = false;
        quick             = !quick;
        if ( quick ) { m_quickCameraModifier = 10.0_ra; }
        else { m_quickCameraModifier = 1.0_ra; }
    } );
}

TrackballCameraManipulator::TrackballCameraManipulator() :
    CameraManipulator(), // Calls the constructor of the parent class (CameraManipulator)
    m_keyMappingCallbackManager { KeyMapping::getContext() } // Initializes the callback manager
{
    resetCamera(); // Resets the camera
    setupKeyMappingCallbacks(); // Configures the callbacks for user interaction
    m_cameraSensitivity = 1.5_ra; // Sets the camera sensitivity
}

// Copy constructor
TrackballCameraManipulator::TrackballCameraManipulator( const CameraManipulator& other ) :
    CameraManipulator( other ), // Calls the constructor of the parent class with the "other" object
    m_keyMappingCallbackManager { KeyMapping::getContext() } // Initializes the callback manager
{
    m_referenceFrame = m_camera->getFrame(); // Initializes the reference frame with the camera's frame
    m_referenceFrame.translation() =
        m_camera->getPosition() + m_distFromCenter * m_camera->getDirection(); // Updates the reference frame's translation
    m_distFromCenter = ( m_referenceFrame.translation() - m_camera->getPosition() ).norm(); // Calculates the distance between the camera and the center
    updatePhiTheta(); // Updates the phi and theta angles

    setupKeyMappingCallbacks(); // Configures the callbacks for user interaction
    m_cameraSensitivity = 1.5_ra; // Sets the camera sensitivity
}

// Destructor for the trackball camera manipulator class
TrackballCameraManipulator::~TrackballCameraManipulator() {};

// Function to reset the camera to its initial state
void TrackballCameraManipulator::resetCamera() {
    // Reset the camera transformation to identity
    m_camera->setFrame( Core::Transform::Identity() );
    // Place the camera at position (0, 0, 2) in world space
    m_camera->setPosition( Core::Vector3( 0_ra, 0_ra, 2_ra ) );
    // Orient the camera to point towards the back (negative z) of world space
    m_camera->setDirection( Core::Vector3( 0_ra, 0_ra, -1_ra ) );
    // Set the initial distance between the camera and the target point
    m_distFromCenter               = 2.0_ra;
    // Reset the camera's reference frame to identity
    m_referenceFrame               = Core::Transform::Identity();
    // Set the initial target point position to the origin of world space
    m_referenceFrame.translation() = Core::Vector3::Zero();

    // Update phi and theta angles for camera rotation
    updatePhiTheta();

    //\todo get rid of this light stuff (comment from original source code)
    // Update the light position and direction to follow the camera
    if ( m_light != nullptr ) {
        m_light->setPosition( m_camera->getPosition() );
        m_light->setDirection( m_camera->getDirection() );
    }
}

// Function to update the camera based on changes in target point position
void TrackballCameraManipulator::updateCamera() {
    // try to keep target near the previous camera's one, take it at the same distance from
    // camera, but in the new direction.
// Calculate the new distance between the camera and the target point
    m_distFromCenter = ( m_referenceFrame.translation() - m_camera->getPosition() ).norm();
    // Update the camera's reference frame from its current transformation
    m_referenceFrame = m_camera->getFrame();
    // Move the target point to the correct distance based on the new camera direction
    m_referenceFrame.translation() =
        m_camera->getPosition() + m_distFromCenter * m_camera->getDirection();

    // Update phi and theta angles for camera rotation
    updatePhiTheta();

    //\todo get rid of this light stuff (comment from original source code)
    // Update the light position and direction to follow the camera
    if ( m_light != nullptr ) {
        m_light->setPosition( m_camera->getPosition() );
        m_light->setDirection( m_camera->getDirection() );
    }
}

// Function to set the radius of the virtual sphere the camera orbits around
void TrackballCameraManipulator::setTrackballRadius( Scalar rad ) {
    // Update the distance between the camera and the center of the virtual sphere
    m_distFromCenter = rad;
    // Update the position of the center of the virtual sphere relative to the camera
    m_referenceFrame.translation() =
        m_camera->getPosition() + m_distFromCenter * m_camera->getDirection();
}

// Returns the distance between the camera and the center of the virtual sphere.
Scalar TrackballCameraManipulator::getTrackballRadius() const {
    return m_distFromCenter;
}

// Returns the position of the center of the virtual sphere.
Core::Transform::ConstTranslationPart TrackballCameraManipulator::getTrackballCenter() const {
    return m_referenceFrame.translation();
}

// Returns the mapping context for this camera.
KeyMappingManager::Context TrackballCameraManipulator::mappingContext() {
    return KeyMapping::getContext();
}

// Saves the mouse position when clicked and determines the direction of rotation (clockwise or counterclockwise).
void TrackballCameraManipulator::mousePressSaveData(const QMouseEvent* mouseEvent) {
    m_lastMouseX = mouseEvent->pos().x();
    m_lastMouseY = mouseEvent->pos().y();
    m_phiDir     = -Core::Math::signNZ(m_theta);
}

// Called when a rotation event is detected. Handles camera rotation based on mouse movement.
void TrackballCameraManipulator::rotateCallback(QEvent* event) {
    if (event->type() == QEvent::MouseMove) {
        auto mouseEvent = reinterpret_cast<QMouseEvent*>(event);
        auto [dx, dy]   = computeDeltaMouseMove(mouseEvent);
        handleCameraRotate(dx, dy);
    }
}

// Called when the user pans the camera by dragging the mouse.
void TrackballCameraManipulator::panCallback(QEvent* event) {
    if (event->type() == QEvent::MouseMove) {
        auto mouseEvent = reinterpret_cast<QMouseEvent*>(event);
        auto [dx, dy]   = computeDeltaMouseMove(mouseEvent);
        // Pass the movement to the camera panning function.
        handleCameraPan(dx, dy);
    }
}

// Called when the user zooms the camera either by scrolling the mouse wheel or by zooming with a touchpad.
void TrackballCameraManipulator::zoomCallback(QEvent* event) {
    if (event->type() == QEvent::MouseMove) {
        auto mouseEvent = reinterpret_cast<QMouseEvent*>(event);
        auto [dx, dy]   = computeDeltaMouseMove(mouseEvent);
        // Pass the movement to the camera zooming function.
        handleCameraZoom(dx, dy);
    } else if (event->type() == QEvent::Wheel) {
        auto wheelEvent = reinterpret_cast<QWheelEvent*>(event);
        // Compute the amount of zoom from the wheel movement and pass it to the camera zooming function.
        handleCameraZoom(
            (wheelEvent->angleDelta().y() * 0.01_ra + wheelEvent->angleDelta().x() * 0.01_ra) *
            m_wheelSpeedModifier);
    }
}

// Called when the user moves the camera forward by scrolling the mouse wheel or by zooming with a touchpad.
void TrackballCameraManipulator::moveForwardCallback(QEvent* event) {
    if (event->type() == QEvent::MouseMove) {
        auto mouseEvent = reinterpret_cast<QMouseEvent*>(event);
        auto [dx, dy]   = computeDeltaMouseMove(mouseEvent);
        // Pass the movement to the camera forward movement function.
        handleCameraMoveForward(dx, dy);
    } else if (event->type() == QEvent::Wheel) {
        auto wheelEvent = reinterpret_cast<QWheelEvent*>(event);
        // Compute the amount of forward movement from the wheel movement and pass it to the camera forward movement function.
        handleCameraMoveForward(
            (wheelEvent->angleDelta().y() * 0.01_ra + wheelEvent->angleDelta().x() * 0.01_ra ) *
            m_wheelSpeedModifier );
    }
}

// This function is called when the user presses a mouse button on the screen.
bool TrackballCameraManipulator::handleMousePressEvent( QMouseEvent* event,
                                                        const Qt::MouseButtons&,
                                                        const Qt::KeyboardModifiers&,
                                                        int key ) {
    // Store the current mouse position.
    m_lastMouseX = event->pos().x();
    m_lastMouseY = event->pos().y();
    // Compute the direction of the camera rotation.
    m_phiDir     = -Core::Math::signNZ( m_theta );

    // Trigger the callback for the pressed key and return whether the event was handled.
    bool handled = m_keyMappingCallbackManager.triggerEventCallback( event, key );
    return handled;
}

bool TrackballCameraManipulator::handleMouseMoveEvent(QMouseEvent* event,
                                                       const Qt::MouseButtons&,
                                                       const Qt::KeyboardModifiers&,
                                                       int key ) {
    // Calls the key event handling callback function to see if the event was handled
    bool handled = m_keyMappingCallbackManager.triggerEventCallback(event, key);

    // Updates the last mouse position
    m_lastMouseX = event->pos().x();
    m_lastMouseY = event->pos().y();

    // Updates the position and direction of the light (if it exists) to align it with the camera
    if (m_light != nullptr) {
        m_light->setPosition(m_camera->getPosition());
        m_light->setDirection(m_camera->getDirection());
    }

    return handled;
}

bool TrackballCameraManipulator::handleMouseReleaseEvent(QMouseEvent* /*event*/) {
    // Does nothing and returns false
    return false;
}

bool TrackballCameraManipulator::handleWheelEvent(QWheelEvent* event,
                                                   const Qt::MouseButtons&,
                                                   const Qt::KeyboardModifiers&,
                                                   int key) {
    // Calls the key event handling callback function to see if the event was handled
    bool handled = m_keyMappingCallbackManager.triggerEventCallback(event, key, true);

    // Updates the position and direction of the light (if it exists) to align it with the camera
    if (m_light != nullptr) {
        m_light->setPosition(m_camera->getPosition());
        m_light->setDirection(m_camera->getDirection());
    }

    return handled;
}

bool TrackballCameraManipulator::handleKeyPressEvent(QKeyEvent* event,
                                                      const KeyMappingManager::KeyMappingAction& action) {
    // Calls the key event handling callback function to see if the event was handled
    return m_keyMappingCallbackManager.triggerEventCallback(action, event);
}

void TrackballCameraManipulator::setCameraPosition(const Core::Vector3& position) {
    // Checks that the position is not equal to the target point
    if (position == m_referenceFrame.translation()) {
        QMessageBox::warning(nullptr, "Error", "The position cannot be set as the target point.");
        return;
    }

    // Updates the camera position and reference position
    m_camera->setPosition(position);
    m_referenceFrame.translation() = position + m_distFromCenter * m_camera->getDirection();

    // Updates the phi and theta angles
    updatePhiTheta();

    // Updates the position and direction of the light (if it exists) to align it with the camera
    if (m_light != nullptr) {
        m_light->setPosition(m_camera->getPosition());
        m_light->setDirection(m_camera->getDirection());
    }
}

void TrackballCameraManipulator::setCameraTarget( const Core::Vector3& target ) {
    // Check if the camera is already at the target position
    if ( m_camera->getPosition() == m_referenceFrame.translation() ) {
        // Show a warning message and return if the camera is already at the target position
        QMessageBox::warning( nullptr, "Error", "Target cannot be set to current camera position" );
        return;
    }

    // Update the reference frame's translation to the target position
    m_referenceFrame.translation() = target;

    // Set the camera's direction to point towards the target position
    m_camera->setDirection( ( target - m_camera->getPosition() ).normalized() );

    // Compute the distance between the camera position and the target position
    m_distFromCenter = ( target - m_camera->getPosition() ).norm();

    // Update the phi and theta angles to look towards the target position
    updatePhiTheta();

    // If a light is attached to the camera, update its direction to match the camera's direction
    if ( m_light != nullptr ) {
        m_light->setDirection( m_camera->getDirection() );
    }
}

/**
 * This method adjusts the camera to fit the bounding box of the scene.
 * @param aabb The axis-aligned bounding box of the scene to be fit.
 */
void TrackballCameraManipulator::fitScene( const Core::Aabb& aabb ) {

    // Get camera's field of view and aspect ratio.
    Scalar f = m_camera->getFOV();
    Scalar a = m_camera->getAspect();

    // Calculate the distance between the camera and the center of the scene.
    const Scalar r = ( aabb.max() - aabb.min() ).norm() / 2_ra;
    const Scalar x = r / std::sin( f / 2_ra );
    const Scalar y = r / std::sin( f * a / 2_ra );
    Scalar d       = std::max( std::max( x, y ), 0.001_ra );

    // Reset the camera frame and set its position to be at the center of the scene, 
    // with the appropriate distance.
    m_camera->setFrame( Core::Transform::Identity() );
    Core::Vector3 camPos { aabb.center().x(), aabb.center().y(), aabb.center().z() + d };
    m_camera->setPosition( camPos );

    // Set the camera direction to look towards the center of the scene.
    Core::Vector3 camDir { aabb.center() - camPos };
    m_distFromCenter = camDir.norm();
    m_camera->setDirection( camDir / m_distFromCenter );

    // There's no reference frame here, so set the identity matrix as the reference.
    m_referenceFrame.setIdentity();
    m_referenceFrame.translation() = aabb.center();

    // Update the camera angles.
    updatePhiTheta();

    // Update the light source direction and position if there is one.
    if ( m_light != nullptr ) {
        m_light->setPosition( m_camera->getPosition() );
        m_light->setDirection( m_camera->getDirection() );
    }
}

void handleCameraRotateCalcul(Scalar* pdphi,Scalar* pdtheta,Scalar* pphi,Scalar* ptheta){
    // Calculate change in phi and theta angles based on input values and sensitivity modifiers
    *dphi   = m_phiDir * dx * m_cameraSensitivity * m_quickCameraModifier;
    *dtheta = -dy * m_cameraSensitivity * m_quickCameraModifier;

    // Update phi and theta angles with change in angles
    *phi   = m_phi + (*dphi);
    *theta = m_theta + (*dtheta);
}


void TrackballCameraManipulator::handleCameraRotate( Scalar dx, Scalar dy ) {
    // variables declaration
    Scalar dphi,dtheta,phi,theta;

    handleCameraRotateCalcul(&dphi,&dtheta,&phi,&theta);

    // Compute new direction vector of camera based on phi and theta angles using trigonometric functions
    Core::Vector3 dir { std::sin( phi ) * std::sin( theta ),
                        std::cos( theta ),
                        std::cos( phi ) * std::sin( theta ) };

    // Compute right vector of camera based on direction vector and normalize it
    Core::Vector3 right { -dir[2], 0, dir[0] };
    right.normalize();

    // Check if camera right vector is pointing in opposite direction of calculated right vector
    if ( ( m_referenceFrame.linear().inverse() * m_camera->getRightVector() ).dot( right ) < 0 )
        right = -right;

    // Compute up vector of camera using cross product of direction and right vectors and normalize it
    Core::Vector3 up = dir.cross( right ).normalized();

    // Apply the reference frame transformation to direction, right, and up vectors
    dir   = m_referenceFrame.linear() * dir;
    right = m_referenceFrame.linear() * right;
    up    = m_referenceFrame.linear() * up;

    // Create a 3x3 matrix from right, up, and direction vectors
    Core::Matrix3 m;
    m << right[0], up[0], dir[0],
         right[1], up[1], dir[1],
         right[2], up[2], dir[2];

    // Create a new transformation matrix using the computed 3x3 matrix and camera position
    Core::Transform t;
    t.setIdentity();
    t.linear()        = m;
    Core::Vector3 pos = m_referenceFrame.translation() + m_distFromCenter * dir;
    t.translation()   = pos;

    // Set the new camera frame using the new transformation matrix
    m_camera->setFrame( t );

    // Update phi and theta angles while clamping them within their valid range
    m_phi   = phi;
    m_theta = theta;
    clampThetaPhi(); // This function ensures that the theta and phi angles are within their valid range
}

void TrackballCameraManipulator::handleCameraPanScalar( Scalar* px, Scalar* py){
    // Calculate movement amount in x and y directions based on input values and sensitivity modifiers
    *x = dx * m_cameraSensitivity * m_quickCameraModifier * m_distFromCenter * 0.1_ra;
    *y = dy * m_cameraSensitivity * m_quickCameraModifier * m_distFromCenter * 0.1_ra;
}

void TrackballCameraManipulator::handleCameraPan( Scalar dx, Scalar dy ) {
    
    Scalar x,y;
    handleCameraPanScalar( &x, &y );

    // Compute camera right and up vectors
    Core::Vector3 R = -m_camera->getRightVector();
    Core::Vector3 U = m_camera->getUpVector();

    // Create a new transformation matrix with a translation component based on the computed movement amount
    Core::Transform T( Core::Transform::Identity() );
    Core::Vector3 t = x * R + y * U;
    T.translate( t );

    // Move the camera and trackball center by applying the new transformation matrix
    m_camera->applyTransform( T );
    m_referenceFrame.translation() += t;
}

void TrackballCameraManipulator::handleCameraMoveForward( Scalar dx, Scalar dy ) {
    // Calculate the magnitude of the 2D vector (dx, dy) and multiply it by the sign of dy
    Scalar magnitude = Ra::Core::Vector2 { dx, dy }.norm();
    Scalar sign = Ra::Core::Math::sign( dy );

    // Call the other version of this function with the adjusted parameter value
    handleCameraMoveForward( sign * magnitude );
}

void TrackballCameraManipulator::handleCameraMoveForward( Scalar z ) {
    // Calculate the amount of movement, based on the input value and the camera's properties
    Scalar moveFactor = z * m_distFromCenter * m_cameraSensitivity * m_quickCameraModifier;

    // Create a translation matrix based on the camera's current direction and the movement factor
    Core::Transform T( Core::Transform::Identity() );
    T.translate( moveFactor * m_camera->getDirection() );

    // Apply the transformation to the camera's frame
    m_camera->applyTransform( T );

    // Update the distance from the camera to the trackball center
    m_distFromCenter = ( m_referenceFrame.translation() - m_camera->getPosition() ).norm();
}

void TrackballCameraManipulator::handleCameraZoom( Scalar dx, Scalar dy ) {
    // Calculate the zoom factor from the input dx and dy values
    Scalar z = Ra::Core::Math::sign( dy ) * Ra::Core::Vector2 { dx, dy }.norm();
    
    // Call the handleCameraZoom(Scalar z) function with the computed zoom factor
    handleCameraZoom( z );
}

void TrackballCameraManipulator::handleCameraZoom( Scalar z ) {
    // Calculate the new zoom factor
    Scalar zoom = m_camera->getZoomFactor() - z * m_cameraSensitivity * m_quickCameraModifier;

    // Set the new zoom factor
    m_camera->setZoomFactor( zoom );
}

void TrackballCameraManipulator::updatePhiTheta() {
    using Core::Math::areApproxEqual;
    const Core::Vector3 R = m_referenceFrame.linear().inverse() * ( -m_camera->getDirection() );

    m_theta = std::acos( R.y() );

    // unlikely to have z and x to 0, unless direction is perfectly aligned with
    // m_referenceFrame.z() in this case phi is given by the relative orientation of right/up in
    // the z/x plane of m_reference frame.
    if ( UNLIKELY( areApproxEqual( R.z(), 0_ra ) && areApproxEqual( R.x(), 0_ra ) ) ) {
        Scalar fx = m_referenceFrame.matrix().block<3, 1>( 0, 2 ).dot( m_camera->getRightVector() );
        Scalar fy = m_referenceFrame.matrix().block<3, 1>( 0, 2 ).dot( m_camera->getUpVector() );
        m_phi     = std::atan2( fx, fy );
    }
    else {
        m_phi = std::atan2( R.x(), R.z() );
    }

    // no need to clamp, atan2 is by def \in [-pi,pi]
    // acos in [0, pi]
    // clampThetaPhi();
    CORE_ASSERT( std::isfinite( m_theta ) && std::isfinite( m_phi ), "Error in trackball camera" );
}

void TrackballCameraManipulator::clampThetaPhi() {
    // Keep phi between 0 and 2pi
    if ( m_phi < 0_ra ) { m_phi += 2_ra * Pi; }
    // Keep theta in [-pi, pi] (instead of [0,pi]) to allows scene flip
    if ( m_theta < -Pi ) { m_theta += 2_ra * Pi; }
    if ( m_theta > Pi ) { m_theta -= 2_ra * Pi; }
}

bool TrackballCameraManipulator::checkIntegrity( const std::string& mess ) const {
    // Compute the camera center position from its position and distance from center
    Core::Vector3 c = m_camera->getPosition() + m_distFromCenter * m_camera->getDirection();

    // Compute the distance between the trackball center and the computed camera center
    Scalar d        = ( m_referenceFrame.translation() - c ).norm();

    // If the distance is greater than a threshold, log a warning message and output debug information
    if ( d > 0.001_ra ) {
        LOG( logWARNING ) << "TrackballCameraManipulator Integrity problem : " << mess;
        LOG( logWARNING ) << "\t Position  " << m_camera->getPosition().transpose();
        LOG( logWARNING ) << "\t Ref       "
                          << ( m_referenceFrame.translation() +
                               m_distFromCenter * ( -m_camera->getDirection() ) )
                                 .transpose();
        LOG( logWARNING ) << "\t Direction " << m_camera->getDirection().transpose();
        LOG( logWARNING ) << "\t Center    " << c.transpose();
        LOG( logWARNING ) << "\t Distance  " << d;
        LOG( logWARNING ) << "\t angles    " << m_phi << " " << m_theta;
    }

    // Return true if the distance is below the threshold, false otherwise
    return d < 0.001_ra;
}

} // namespace Gui
} // namespace Ra