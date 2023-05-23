#pragma once
#include <Gui/RaGui.hpp>
#include <Gui/Utils/KeyMappingManager.hpp>

#include <Core/Asset/Camera.hpp>
#include <Engine/Scene/CameraComponent.hpp>
#include <Gui/Viewer/CameraManipulator.hpp>

namespace Ra {
namespace Gui {

/**
 * @class TrackballCameraManipulator
 * A Trackball manipulator for Cameras.
 */
class TrackballCameraManipulator : public CameraManipulator,
                                   public KeyMappingManageable<TrackballCameraManipulator>
{
public:
    /**
     * Default constructor
     */
    TrackballCameraManipulator();

    /**
     * Copy constructor used when switching camera manipulator
     * Requires that m_target is on the line of sight of the camera.
     * @param other The CameraManipulator object to copy from.
     */
    explicit TrackballCameraManipulator(const CameraManipulator& other);

    /**
     * Destructor.
     */
    virtual ~TrackballCameraManipulator();

    /**
     * Handle mouse press event.
     * @param event The QMouseEvent object representing the event.
     * @param buttons The mouse buttons pressed during the event.
     * @param modifiers The keyboard modifiers pressed during the event.
     * @param key The key pressed during the event.
     * @return True if the event was handled, false otherwise.
     */
    bool handleMousePressEvent(QMouseEvent* event,
                               const Qt::MouseButtons& buttons,
                               const Qt::KeyboardModifiers& modifiers,
                               int key) override;

    /**
     * Handle mouse release event.
     * @param event The QMouseEvent object representing the event.
     * @return True if the event was handled, false otherwise.
     */
    bool handleMouseReleaseEvent(QMouseEvent* event) override;

    /**
     * Handle mouse move event.
     * @param event The QMouseEvent object representing the event.
     * @param buttons The mouse buttons pressed during the event.
     * @param modifiers The keyboard modifiers pressed during the event.
     * @param key The key pressed during the event.
     * @return True if the event was handled, false otherwise.
     */
    bool handleMouseMoveEvent(QMouseEvent* event,
                              const Qt::MouseButtons& buttons,
                              const Qt::KeyboardModifiers& modifiers,
                              int key) override;

    /**
     * Handle wheel event.
     * @param event The QWheelEvent object representing the event.
     * @param buttons The mouse buttons pressed during the event.
     * @param modifiers The keyboard modifiers pressed during the event.
     * @param key The key pressed during the event.
     * @return True if the event was handled, false otherwise.
     */
    bool handleWheelEvent(QWheelEvent* event,
                          const Qt::MouseButtons& buttons,
                          const Qt::KeyboardModifiers& modifiers,
                          int key) override;

    /**
     * Handle key press event.
     * @param event The QKeyEvent object representing the event.
     * @param action The KeyMappingManager::KeyMappingAction associated with the event.
     * @return True if the event was handled, false otherwise.
     */
    bool handleKeyPressEvent(QKeyEvent* event,
                             const KeyMappingManager::KeyMappingAction& action) override;

    /**
     * Toggle rotate around mode.
     */
    void toggleRotateAround();

    /**
     * Update the camera.
     */
    void updateCamera() override;

    /**
     * Set the distance from the camera to the target point.
     * @param rad The distance to set.
     * @note This function does not modify the camera.
     */
    void setTrackballRadius(Scalar rad);

    /**
     * Return the distance from the camera to the target point.
     * @return The distance from the camera to the target point.
     */
    Scalar getTrackballRadius() const;

    /**
     * Return the trackball center.
     * @return The trackball center.
     * @note This function does not modify the camera.
     */
    const Core::Transform::ConstTranslationPart getTrackballCenter() const;

    /**
     * Get the mapping context for key mappings.
     * @return The mapping context.
     */
    KeyMappingManager::Context mappingContext() override;

public slots:
    /**
     * Set the camera position.
     * @param position The position to set.
     */
    void setCameraPosition(const Core::Vector3& position) override;

    /**
     * Set the camera target.
     * @param target The target to set.
     */
    void setCameraTarget(const Core::Vector3& target) override;

    /**
     * Fit the scene based on the specified axis-aligned bounding box (AABB).
     * @param aabb The AABB representing the scene.
     */
    void fitScene(const Core::Aabb& aabb) override;

    /**
     * Reset the camera.
     */
    void resetCamera() override;

protected:
    /**
     * Handle camera rotation.
     * @param dx The change in x-coordinate.
     * @param dy The change in y-coordinate.
     */
    virtual void handleCameraRotate(Scalar dx, Scalar dy);

    /**
     * Handle camera panning.
     * @param dx The change in x-coordinate.
     * @param dy The change in y-coordinate.
     */
    virtual void handleCameraPan(Scalar dx, Scalar dy);

    /**
     * Handle camera zooming.
     * @param dx The change in x-coordinate.
     * @param dy The change in y-coordinate.
     */
    virtual void handleCameraZoom(Scalar dx, Scalar dy);

    /**
     * Handle camera zooming.
     * @param z The zoom value.
     */
    virtual void handleCameraZoom(Scalar z);

    /**
     * Handle moving the camera forward.
     * @param dx The change in x-coordinate.
     * @param dy The change in y-coordinate.
     */
    virtual void handleCameraMoveForward(Scalar dx, Scalar dy);

    /**
     * Handle moving the camera forward.
     * @param z The movement distance.
     */
    virtual void handleCameraMoveForward(Scalar z);

    /**
     * Update the polar coordinates of the Camera with respect to the trackball center.
     */
    void updatePhiTheta();

protected:
    // the center of the trackball is defined by the m_referenceFrame.translation()

    /// Spherical coordinates   (ISO 80000-2:2019 convention)
    /// https://en.wikipedia.org/wiki/Spherical_coordinate_system
    /// phi is azimutal
    /// theta is polar, from y which is world "up" direction
    /// rest pose correspond to camera view direction at m_referenceFrame -z;
    Scalar m_phi { 0_ra };
    Scalar m_theta { 0_ra };
    /// sign of m_theta at mousePressEvent, to guide the phi rotation direction.
    Scalar m_phiDir { 1_ra };

    /// initial frame of the camera, centered on target, to compute angles.
    Core::Transform m_referenceFrame;

    /// The distance from the camera to the trackball center.
    Scalar m_distFromCenter { 0_ra };

    KeyMappingCallbackManager m_keyMappingCallbackManager;

private:
private:
    /**
     * Set up the key mapping callbacks.
     */
    void setupKeyMappingCallbacks();

    /**
     * Check the integrity of the camera manipulator.
     * @param mess The error message to display if the integrity check fails.
     * @return True if the integrity check passes, false otherwise.
     */
    bool checkIntegrity(const std::string& mess) const;

    /**
     * Configure the key mapping implementation.
     */
    static void configureKeyMapping_impl();

    /**
     * Clamp the theta and phi values within valid ranges.
     */
    void clampThetaPhi();

    /**
     * Callback function for rotation events.
     * @param event The QEvent object representing the event.
     */
    void rotateCallback(QEvent* event);

    /**
     * Callback function for panning events.
     * @param event The QEvent object representing the event.
     */
    void panCallback(QEvent* event);

    /**
     * Callback function for move forward events.
     * @param event The QEvent object representing the event.
     */
    void moveForwardCallback(QEvent* event);

    /**
     * Callback function for zoom events.
     * @param event The QEvent object representing the event.
     */
    void zoomCallback(QEvent* event);

    /**
     * Save mouse press data.
     * @param mouseEvent The QMouseEvent object representing the event.
     */
    void mousePressSaveData(const QMouseEvent* mouseEvent);


protected:
    ///\todo move CAMERA_ to CameraManipulator, will be done soon ;)
#define KeyMappingCamera                      \
    KMA_VALUE( TRACKBALLCAMERA_ROTATE )       \
    KMA_VALUE( TRACKBALLCAMERA_PAN )          \
    KMA_VALUE( TRACKBALLCAMERA_ZOOM )         \
    KMA_VALUE( TRACKBALLCAMERA_PROJ_MODE )    \
    KMA_VALUE( TRACKBALLCAMERA_MOVE_FORWARD ) \
    KMA_VALUE( CAMERA_TOGGLE_QUICK )

#define KMA_VALUE(XX) static KeyMappingManager::KeyMappingAction XX;
    KeyMappingCamera
#undef KMA_VALUE
};


} // namespace Gui
} // namespace Ra