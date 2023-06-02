#pragma once

#include <Core/Asset/Camera.hpp>
#include <Core/Math/Math.hpp>
#include <Core/Types.hpp>
#include <Core/Utils/Observable.hpp>
#include <Gui/RaGui.hpp>
#include <qobject.h>
#include <qtimer.h>
#include <vector>

namespace Core {
namespace Asset {

class RA_GUI_API CameraRecorder : QObject
{
  public:
    CameraRecorder( Core::Asset::Camera* camera, int captureInterval = 500);

    ~CameraRecorder();

    void toggleRecord(bool reset);

    void setCaptureInterval( int ms );

    int captureInterval();

    void reset();

    bool isRecording();


	std::vector<Core::Transform>* getFrames();

  private:

    long m_time;
    int m_captureInterval;
    bool m_recording = false;

    Core::Asset::Camera* m_camera;

    QTimer* m_timer;
    QMetaObject::Connection m_connection; 

	  std::vector<Core::Transform>* m_frames;

    void record();
    
};

} // namespace Asset
} // namespace Core
