#include "Core/Asset/Camera.hpp"
#include "Core/Types.hpp"
#include <Gui/Viewer/CameraRecorder.hpp>
#include <cstdlib>
#include <iterator>
#include <vector>

namespace Ra {
namespace Gui {

/**
 * @brief Constructs a CameraRecorder object.
 *
 * This constructor initializes a CameraRecorder object with the specified camera and capture interval.
 *
 * @param camera The camera object to be used for recording.
 * @param captureInterval The interval at which frames will be captured, in milliseconds.
 */
CameraRecorder::CameraRecorder( Core::Asset::Camera* camera, int captureInterval ) :
    m_captureInterval( captureInterval ) {
    m_camera = camera;
    m_frames = new std::vector<Core::Transform>();
    m_timer        = new QTimer( this );
}

CameraRecorder::~CameraRecorder() {
    delete m_frames;
    delete m_timer;
}

std::vector<Core::Transform>* CameraRecorder::getFrames(){
    return m_frames;
}

/**
 * @brief Records a frame from the camera.
 *
 * This function captures a frame from the camera and adds it to the frames container.
 * The captured frame is obtained using the `getFrame()` function of the camera object.
 */
void CameraRecorder::record() {
    m_frames->emplace_back( m_camera->getFrame() );
}

void CameraRecorder::reset() {
    m_frames->clear();
}

bool CameraRecorder::isRecording() {
    return m_recording;
}

/**
 * @brief Toggles the recording state of the camera.
 *
 * This function toggles the recording state of the camera. If the camera is currently recording,
 * it stops the recording and disconnects the timer signal. If the camera is not currently recording,
 * it starts the recording by connecting the timer signal and optionally resetting the video.
 *
 * @param resetVideo If true, the video will be reset before starting the recording. Default is false.
 */
void CameraRecorder::toggleRecord( bool resetVideo ) {
    m_recording = !m_recording;

    if (!m_recording ) {
        m_timer->stop();
        QObject::disconnect( m_connection );
    } else {
        if ( resetVideo ) {
            reset();
        }
        m_timer->setInterval( m_captureInterval );
        m_connection = QObject::connect( m_timer, &QTimer::timeout, [this]() { this->record(); } );
        m_timer->start();
    }
}

void CameraRecorder::setCaptureInterval( int ms ) {
    m_captureInterval = ms;
}

int CameraRecorder::captureInterval() {
    return m_captureInterval;
}

} // namespace Gui
} // namespace Ra
