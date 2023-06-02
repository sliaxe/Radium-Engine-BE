#include <Core/Asset/CameraManipulator.hpp>

#include <Core/Math/Math.hpp>
#include <Engine/Scene/CameraManager.hpp>
#include <Engine/Scene/Light.hpp>

#include <Core/Asset/Camera.hpp>
#include <Gui/Viewer/Viewer.hpp>

namespace Core {
namespace Asset {
using namespace Core::Utils; // log

CameraManipulator::CameraManipulator( const CameraManipulator& other ) :
    QObject(),
    m_cameraSensitivity( other.m_cameraSensitivity ),
    m_quickCameraModifier( other.m_quickCameraModifier ),
    m_wheelSpeedModifier( other.m_wheelSpeedModifier ),
    m_targetedAabbVolume( other.m_targetedAabbVolume ),
    m_mapCameraBahaviourToAabb( other.m_mapCameraBahaviourToAabb ),
    m_target( other.m_target ),
    m_camera( other.m_camera ),
    m_light( other.m_light ) {}

/**
 * @brief Constructs a CameraManipulator object.
 *
 * This constructor initializes a CameraManipulator object by retrieving the active camera from the
 * default camera manager in the RadiumEngine. It assumes the existence of the "DefaultCameraManager"
 * system in the RadiumEngine.
 */
CameraManipulator::CameraManipulator() {

    auto cameraManager = static_cast<Ra::Engine::Scene::CameraManager*>(
        Engine::RadiumEngine::getInstance()->getSystem( "DefaultCameraManager" ) );
    m_camera = cameraManager->getActiveCamera();
}

CameraManipulator::~CameraManipulator() {}


/**
 * @brief Sets the camera sensitivity.
 *
 * This function sets the sensitivity of the camera manipulator.
 *
 * @param sensitivity The sensitivity value to set for the camera.
 */
void CameraManipulator::setCameraSensitivity( Scalar sensitivity ) {
    m_cameraSensitivity = sensitivity;
}

/**
 * @brief Sets the camera field of view (FOV).
 *
 * This function sets the field of view (FOV) of the camera controlled by the camera manipulator.
 *
 * @param fov The new field of view (FOV) value to set for the camera, in radians.
 */
void CameraManipulator::setCameraFov( Scalar fov ) {
    m_camera->setFOV( fov );
}

/**
 * @brief Sets the camera field of view (FOV) in degrees.
 *
 * This function sets the field of view (FOV) of the camera controlled by the camera manipulator,
 * using the provided angle in degrees. The FOV value is converted to radians before setting it
 * to the camera.
 *
 * @param fov The new field of view (FOV) value to set for the camera, in degrees.
 */
void CameraManipulator::setCameraFovInDegrees( Scalar fov ) {
    m_camera->setFOV( fov * Core::Math::toRad );
}

/**
 * @brief Sets the camera near clipping plane.
 *
 * This function sets the distance of the near clipping plane of the camera controlled by the camera manipulator.
 *
 * @param zNear The new value for the distance of the near clipping plane.
 */
void CameraManipulator::setCameraZNear( Scalar zNear ) {
    m_camera->setZNear( zNear );
}

/**
 * @brief Sets the camera far clipping plane.
 *
 * This function sets the distance of the far clipping plane of the camera controlled by the camera manipulator.
 *
 * @param zFar The new value for the distance of the far clipping plane.
 */
void CameraManipulator::setCameraZFar( Scalar zFar ) {
    m_camera->setZFar( zFar );
}

void CameraManipulator::updateCamera() {}

/**
 * @brief Maps camera behavior to a specified Axis-Aligned Bounding Box (AABB).
 *
 * This function maps the camera behavior to the specified Axis-Aligned Bounding Box (AABB).
 * The camera will adjust its behavior to focus on the AABB, such as maintaining it within the view or following its movement.
 *
 * @param aabb The Axis-Aligned Bounding Box (AABB) to map the camera behavior to.
 */
void CameraManipulator::mapCameraBehaviourToAabb( const Core::Aabb& aabb ) {
    m_targetedAabb             = aabb;
    m_targetedAabbVolume       = aabb.volume();
    m_mapCameraBahaviourToAabb = true;
}

/**
 * @brief Unmaps camera behavior from the previously mapped Axis-Aligned Bounding Box (AABB).
 *
 * This function unmaps the camera behavior from the previously mapped Axis-Aligned Bounding Box (AABB).
 * After calling this function, the camera behavior will no longer be tied to the AABB.
 */
void CameraManipulator::unmapCameraBehaviourToAabb() {
    m_mapCameraBahaviourToAabb = false;
}

/**
 * @brief Attaches a light to the camera manipulator.
 *
 * This function attaches the specified light to the camera manipulator.
 * The light's direction is set to match the current direction of the camera.
 *
 * @param light Pointer to the light object to attach.
 */
void CameraManipulator::attachLight( Engine::Scene::Light* light ) {
    m_light = light;
    m_light->setDirection( m_camera->getDirection() );
}

/**
 * @brief Returns the mapping context for the camera manipulator in the GUI.
 *
 * This function returns the mapping context for the camera manipulator in the GUI.
 * It creates and returns an instance of the KeyMappingManager::Context class.
 *
 * @return The mapping context for the camera manipulator in the GUI.
 */
KeyMappingManager::Context Gui::CameraManipulator::mappingContext() {
    return Gui::KeyMappingManager::Context();
}


/**
 * @brief Handles the mouse press event for the camera manipulator.
 *
 * This function is responsible for handling the mouse press event for the camera manipulator.
 * It receives the mouse event, the mouse buttons, keyboard modifiers, and the event's button value as parameters.
 *
 * @param event Pointer to the QMouseEvent representing the mouse press event.
 * @param buttons The Qt::MouseButtons indicating the buttons pressed during the event.
 * @param modifiers The Qt::KeyboardModifiers representing the keyboard modifiers pressed during the event.
 * @param button The value representing the button associated with the event.
 *
 * @return A boolean value indicating whether the mouse press event was handled successfully or not.
 */
bool CameraManipulator::handleMousePressEvent( QMouseEvent*,
                                               const Qt::MouseButtons&,
                                               const Qt::KeyboardModifiers&,
                                               int ) {
    return false;
}

/**
 * @brief Handles the mouse release event for the camera manipulator.
 *
 * This function is responsible for handling the mouse release event for the camera manipulator.
 * It receives the mouse event as a parameter.
 *
 * @param event Pointer to the QMouseEvent representing the mouse release event.
 *
 * @return A boolean value indicating whether the mouse release event was handled successfully or not.
 */
bool CameraManipulator::handleMouseReleaseEvent( QMouseEvent* ) {
    return false;
}

/**
 * @brief Handles the mouse move event for the camera manipulator.
 *
 * This function is responsible for handling the mouse move event for the camera manipulator.
 * It receives the mouse event, the mouse buttons, keyboard modifiers, and the event's button value as parameters.
 *
 * @param event Pointer to the QMouseEvent representing the mouse move event.
 * @param buttons The Qt::MouseButtons indicating the buttons pressed during the event.
 * @param modifiers The Qt::KeyboardModifiers representing the keyboard modifiers pressed during the event.
 * @param button The value representing the button associated with the event.
 *
 * @return A boolean value indicating whether the mouse move event was handled successfully or not.
 */
bool CameraManipulator::handleMouseMoveEvent( QMouseEvent*,
                                              const Qt::MouseButtons&,
                                              const Qt::KeyboardModifiers&,
                                              int ) {
    return false;
}

/**
 * @brief Handles the wheel event for the camera manipulator.
 *
 * This function is responsible for handling the wheel event for the camera manipulator.
 * It receives the wheel event, the mouse buttons, keyboard modifiers, and the event's button value as parameters.
 *
 * @param event Pointer to the QWheelEvent representing the wheel event.
 * @param buttons The Qt::MouseButtons indicating the buttons pressed during the event.
 * @param modifiers The Qt::KeyboardModifiers representing the keyboard modifiers pressed during the event.
 * @param button The value representing the button associated with the event.
 *
 * @return A boolean value indicating whether the wheel event was handled successfully or not.
 */
bool CameraManipulator::handleWheelEvent( QWheelEvent*,
                                          const Qt::MouseButtons&,
                                          const Qt::KeyboardModifiers&,
                                          int ) {
    return false;
}

/**
 * @brief Handles the key press event for the camera manipulator.
 *
 * This function is responsible for handling the key press event for the camera manipulator.
 * It receives the key event and the key mapping action associated with the event as parameters.
 *
 * @param event Pointer to the QKeyEvent representing the key press event.
 * @param action The KeyMappingManager::KeyMappingAction associated with the key press event.
 *
 * @return A boolean value indicating whether the key press event was handled successfully or not.
 */
bool CameraManipulator::handleKeyPressEvent( QKeyEvent*,
                                             const KeyMappingManager::KeyMappingAction& ) {
    return false;
}

/**
 * @brief Handles the key release event for the camera manipulator.
 *
 * This function is responsible for handling the key release event for the camera manipulator.
 * It receives the key event and the key mapping action associated with the event as parameters.
 *
 * @param event Pointer to the QKeyEvent representing the key release event.
 * @param action The KeyMappingManager::KeyMappingAction associated with the key release event.
 *
 * @return A boolean value indicating whether the key release event was handled successfully or not.
 */
bool CameraManipulator::handleKeyReleaseEvent( QKeyEvent*,
                                               const KeyMappingManager::KeyMappingAction& ) {
    return false;
}
} // namespace asset
} // namespace core
