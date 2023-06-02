// Include Radium base application and its simple Gui
#include "Core/Animation/KeyFramedValue.hpp"
#include "Core/Animation/KeyFramedValueInterpolators.hpp"
#include "Core/Asset/Camera.hpp"
#include "Core/Types.hpp"
#include "Engine/Scene/CameraComponent.hpp"
#include "Engine/Scene/Entity.hpp"
#include "Gui/Utils/KeyMappingManager.hpp"
#include "Gui/Viewer/FlightCameraManipulator.hpp"
#include "Gui/Viewer/RotateAroundCameraManipulator.hpp"
#include "IO/AssimpLoader/AssimpCameraDataLoader.hpp"
#include <Gui/BaseApplication.hpp>
#include <Gui/RadiumWindow/SimpleWindowFactory.hpp>

// include the core geometry/appearance interface
#include <Core/Geometry/MeshPrimitives.hpp>

// include the Engine/entity/component/system/animation interface
#include <Engine/FrameInfo.hpp>
#include <Engine/Scene/EntityManager.hpp>
#include <Engine/Scene/GeometryComponent.hpp>
#include <Engine/Scene/System.hpp>

// include the task animation interface
#include <Core/Tasks/Task.hpp>

// include the camera manager interface
#include <Engine/Scene/CameraManager.hpp>

// include the viewer to add key event
#include <Gui/Viewer/Viewer.hpp>

#include <Gui/Viewer/CameraRecorder.hpp>

// To terminate the demo after a given time
#include <QTimer>
#include <cstddef>
#include <iostream>
#include <ostream>
#include <qcoreevent.h>
#include <qevent.h>
#include <vector>

using namespace Ra;
using namespace Ra::Core;
using namespace Ra::Engine;

class KeyFramedCameraComponent : public Ra::Engine::Scene::CameraComponent
{
  public:
    inline KeyFramedCameraComponent( Ra::Engine::Scene::Entity* entity, const std::string& name ) :
        Ra::Engine::Scene::CameraComponent( entity, name ),
        m_animatedEntity( entity ),
        m_transform( 0_ra, Ra::Core::Transform::Identity() ) {}

    void updateKeyFrames(const std::vector<Core::Transform>* frames) {

        //Removing the old keyframes
        for ( size_t i = 0; i < m_transform.size(); ++i ) {
            m_transform.removeKeyFrame( i );
        }

        //Adding the new keyframes
        for ( int i = 0; i < frames->size(); ++i ) {

            m_transform.insertKeyFrame( i, (*frames)[i] );
        }
        
    }

    /// This function uses the keyframes to update the camera to time \p t.
    void update( Scalar t ) {
        //! [Fetch transform from KeyFramedValue]
        auto T = m_transform.at( t, Ra::Core::Animation::linearInterpolate<Ra::Core::Transform> );
        m_animatedEntity->setTransform( T );
    }

    /// The Keyframes for the render object's tranform.
    Ra::Core::Animation::KeyFramedValue<Ra::Core::Transform> m_transform;
    Scene::Entity* m_animatedEntity;
};

//! [Define a simple animation system]
/// This system will be added to the engine. Every frame it will
/// add a task to be executed, calling the update function of the component.
/// \note This system makes time loop around.
class SimpleAnimationSystem : public Ra::Engine::Scene::System
{
  public:
    virtual void generateTasks( Ra::Core::TaskQueue* q,
                                const Ra::Engine::FrameInfo& info ) override {
        KeyFramedCameraComponent* c =
            static_cast<KeyFramedCameraComponent*>( m_components[0].second );

        // Create a new task which wil call c->spin() when executed.
        q->registerTask( std::make_unique<Ra::Core::FunctionTask>(
            std::bind( &KeyFramedCameraComponent::update, c, info.m_animationTime ), "replay" ) );
    }
};

/* ---------------------------------------------------------------------------------------------- */
/*                             main function that build the demo scene                            */
/* ---------------------------------------------------------------------------------------------- */
int main( int argc, char* argv[] ) {
    //! [Creating the application]
    Gui::BaseApplication app( argc, argv );
    app.initialize( Ra::Gui::SimpleWindowFactory {} );
    //! [Creating the application]

    //! [Create the demo animation system]
    SimpleAnimationSystem* sys = new SimpleAnimationSystem;

    //![Parameterize the Engine  time loop]
    app.m_engine->setRealTime( true );
    //![Parameterize the Engine  time loop]

    //! [Cache the camera manager]
    auto cameraManager =
        static_cast<Scene::CameraManager*>( app.m_engine->getSystem( "DefaultCameraManager" ) );
    //! [Cache the camera manager]

    //! [Add usefull custom key events]
    auto callback0 = [cameraManager]( QEvent* event ) {
        if ( event->type() == QEvent::KeyPress ) {
            auto keyEvent = static_cast<QKeyEvent*>( event );
            // Convert ascii code to camera index
            cameraManager->activate( 0 );
        }
    };

    auto callback1 = [cameraManager]( QEvent* event ) {
        if ( event->type() == QEvent::KeyPress ) {
            auto keyEvent = static_cast<QKeyEvent*>( event );
            // Convert ascii code to camera index
            cameraManager->activate( 1 );
        }
    };

    app.m_mainWindow->getViewer()->addCustomAction(
        "switchCam0",
        Gui::KeyMappingManager::createEventBindingFromStrings( "", "ControlModifier", "Key_K" ),
        callback0 );
    app.m_mainWindow->getViewer()->addCustomAction(
        "switchCam1",
        Gui::KeyMappingManager::createEventBindingFromStrings( "", "ControlModifier", "Key_L" ),
        callback1 );
    //! [Add usefull custom key events]

    //! [Create the camera animation system demonstrator]
    // auto playbackSystem = new CameraPlaybackSystem;
    // app.m_engine->registerSystem( "Camera playback system", playbackSystem );
    //! [Create the camera animation system demonstrator]

    //! [Create the demo fixed entity/component]
    {
        //! [Create the engine entity for the fixed component]
        auto e = app.m_engine->getEntityManager()->createEntity( "Fixed cube" );
        //! [Create the engine entity for the fixed component]

        //! [Creating the cube]
        auto cube = Geometry::makeSharpBox( { 0.5f, 0.5f, 0.5f }, Utils::Color::Green() );
        //! [Creating the Cube]

        //! [Create a geometry component with the cube]
        // component ownership is transfered to entity in component ctor
        new Scene::TriangleMeshComponent( "Fixed cube geometry", e, std::move( cube ), nullptr );
        //! [Create a geometry component with the cube]
    }
    //! [Create the demo fixed entity/component]

    //! [Create the reference camera]
    Asset::Camera* referenceCamera = nullptr;
    {
        auto e      = app.m_engine->getEntityManager()->createEntity( "Reference Camera" );
        auto camera = new Scene::CameraComponent( e, "Camera" );
        camera->initialize();
        camera->getCamera()->setPosition( Vector3 { 0, 0, 5 } );
        camera->getCamera()->setDirection( Vector3 { 0, 0, -1 } );
        
        auto viewer = app.m_mainWindow->getViewer();
        viewer->setCameraManipulator(
            new Gui::FlightCameraManipulator( *( viewer->getCameraManipulator() ) ) );

        cameraManager->addCamera( camera );
        cameraManager->activate( cameraManager->getCameraIndex( camera ) );

        auto window = app.m_mainWindow.get();

        //Adding forward camera movement
        viewer->addCustomAction(
            "DEMO_CAMERAFORWARD",
            Ra::Gui::KeyMappingManager::createEventBindingFromStrings( "", "", "Key_Up" ),
            [camera, cameraManager]( QEvent* event ) {

                auto rotation = camera->getCamera()->getFrame().rotation();

                Vector3 dir(0,0,-.1);
                dir = rotation * dir;
                Translation tr(dir);

                Transform transform( tr );

                camera->getCamera()->applyTransform( transform );
                cameraManager->activate( 0 );
            } );

        //Adding backward camera movement
        viewer->addCustomAction(
            "DEMO_CAMERABACKWARD",
            Ra::Gui::KeyMappingManager::createEventBindingFromStrings( "", "", "Key_Down" ),
            [camera, cameraManager]( QEvent* event ) {

                auto rotation = camera->getCamera()->getFrame().rotation();

                Vector3 dir(0,0,.1);
                dir = rotation * dir;
                Translation tr(dir);

                Transform transform( tr );

                camera->getCamera()->applyTransform( transform );
                cameraManager->activate( 0 );
            } );

        //Adding up camera movement
        viewer->addCustomAction( 
            "DEMO_CAMERAUP",
            Ra::Gui::KeyMappingManager::createEventBindingFromStrings("", "ShiftModifier", "Key_Up" ),
            [camera, cameraManager]( QEvent* event ) {

                auto rotation = camera->getCamera()->getFrame().rotation();

                Vector3 dir(0,.1,0);
                dir = rotation * dir;
                Translation tr(dir);

                Transform transform( tr );

                camera->getCamera()->applyTransform( transform );
                cameraManager->activate( 0 );
            } );

        //Adding down camera movement
        viewer->addCustomAction( "DEMO_CAMERADOWN",
            Ra::Gui::KeyMappingManager::createEventBindingFromStrings("", "ShiftModifier", "Key_Down" ),
            [camera, cameraManager]( QEvent* event ) {

                auto rotation = camera->getCamera()->getFrame().rotation();

                Vector3 dir(0,-.1,0);
                dir = rotation * dir;
                Translation tr(dir);

                Transform transform( tr );

                camera->getCamera()->applyTransform( transform );
                cameraManager->activate( 0 );
            } );

        //Adding left camera movement
        viewer->addCustomAction(
            "DEMO_CAMERALEFT",
            Ra::Gui::KeyMappingManager::createEventBindingFromStrings( "", "", "Key_Left" ),
            [camera, cameraManager]( QEvent* event ) {

                auto rotation = camera->getCamera()->getFrame().rotation();

                Vector3 dir(-.1,0,0);
                dir = rotation * dir;
                Translation tr(dir);

                Transform transform( tr );

                camera->getCamera()->applyTransform( transform );
                cameraManager->activate( 0 );
            } );

        //Adding right camera movement
        viewer->addCustomAction(
            "DEMO_CAMERARIGHT",
            Ra::Gui::KeyMappingManager::createEventBindingFromStrings( "", "", "Key_Right" ),
            [cameraManager, camera]( QEvent* event ) {

                auto rotation = camera->getCamera()->getFrame().rotation();

                Vector3 dir(.1,0,0);
                dir = rotation * dir;
                Translation tr(dir);

                Transform transform( tr );

                camera->getCamera()->applyTransform( transform );
                cameraManager->activate( 0 );
            } );

        //Adding camera rotation on y axis to look left
        viewer->addCustomAction(
            "DEMO_CAMERAROTATELEFT",
            Ra::Gui::KeyMappingManager::createEventBindingFromStrings( "", "", "Key_Q" ),
            [cameraManager, camera]( QEvent* event ) {
                auto transform = camera->getCamera()->getFrame();

                AngleAxis aa(0.01F, Vector3{0,1,0});

                transform.rotate(aa);

                camera->getCamera()->setFrame(transform);
                cameraManager->activate( 0 );
            } );

        //Adding camera rotation on y axis to look right
        viewer->addCustomAction(
            "DEMO_CAMERAROTATERIGHT",
            Ra::Gui::KeyMappingManager::createEventBindingFromStrings( "", "", "Key_D" ),
            [cameraManager, camera]( QEvent* event ) {
                auto transform = camera->getCamera()->getFrame();

                AngleAxis aa(-0.01F, Vector3{0,1,0});

                transform.rotate(aa);

                camera->getCamera()->setFrame(transform);
                cameraManager->activate( 0 );
            } );

        //Adding camera rotation on x axis to look up
        viewer->addCustomAction(
            "DEMO_CAMERAROTATEUP",
            Ra::Gui::KeyMappingManager::createEventBindingFromStrings( "", "", "Key_Z" ),
            [cameraManager, camera]( QEvent* event ) {
                auto transform = camera->getCamera()->getFrame();

                AngleAxis aa(0.01F, Vector3{1,0,0});

                transform.rotate(aa);

                camera->getCamera()->setFrame(transform);
                cameraManager->activate( 0 );
            } );

        //Adding camera rotation on x axis to look down
        viewer->addCustomAction(
            "DEMO_CAMERAROTATEDOWN",
            Ra::Gui::KeyMappingManager::createEventBindingFromStrings( "", "", "Key_S" ),
            [cameraManager, camera]( QEvent* event ) {
                auto transform = camera->getCamera()->getFrame();

                AngleAxis aa(-0.01F, Vector3{1,0,0});

                transform.rotate(aa);

                camera->getCamera()->setFrame(transform);
                cameraManager->activate( 0 );
            } );

        referenceCamera = camera->getCamera();
    }
    //! [Create the reference camera]

    //! [Create the demo animated entity/components]
    KeyFramedCameraComponent* playbackCamera = nullptr;
    {
        //! [Create the animated entity ]
        auto e              = app.m_engine->getEntityManager()->createEntity( "Animated entity" );
        Transform transform = Ra::Core::Transform::Identity();
        e->setTransform( transform );
        e->swapTransformBuffers();
        //! [Create the animated entity ]

        // playbackSystem->addEntity( e );

        playbackCamera = new KeyFramedCameraComponent( e, "Playback Camera" );
        playbackCamera->initialize();
        playbackCamera->getCamera()->setPosition( Vector3 { 0, 0, 5 } );
        playbackCamera->getCamera()->setDirection( Vector3 { 0, 0, -1 } );
        playbackCamera->show( true );
        cameraManager->addCamera( playbackCamera );


        //! [add the component to the animation system]
        sys->addComponent( e, playbackCamera );
        //! [add the component to the animation system]
    }
    app.m_engine->registerSystem( "Playback system", sys );

    //! [Create camera recorder]

    auto cameraRecorder = Gui::CameraRecorder( referenceCamera );

    //Adding recording behavior
    app.m_mainWindow->getViewer()->addCustomAction(
        "TOGGLE_RECORDING",
        Ra::Gui::KeyMappingManager::createEventBindingFromStrings( "", "", "Key_I" ),
        [&cameraRecorder]( QEvent* event ) {
            if ( event->type() == QEvent::KeyPress ) {
                cameraRecorder.toggleRecord(true);
                std::cout<<"Toggle recording \n";
            };
        } );

    bool playback = false;

    //Adding playback behavior
    app.m_mainWindow->getViewer()->addCustomAction(
        "TOGGLE_REPLAYING",
        Ra::Gui::KeyMappingManager::createEventBindingFromStrings( "", "", "Key_P" ),
        [&app, &cameraRecorder, playbackCamera, &playback]( QEvent* event ) { //TODO add safety to prevent pressing record then play before ending the recording
            size_t frameNb = cameraRecorder.getFrames()->size();
            if ( event->type() == QEvent::KeyPress ) {
                if ( !playback && frameNb > 1 && !cameraRecorder.isRecording() ) {
                    std::cout << "start replaying\n";
                    playbackCamera->updateKeyFrames( cameraRecorder.getFrames() );
                    app.m_engine->setEndTime( frameNb );
                    app.m_engine->setRealTime( true );
                    app.m_engine->play( true );
                    playback = true;
                } else if (playback) {
                    std::cout << "stop replaying\n";
                    app.m_engine->play( false );
                    playback = false;
                }
            }
        } );

    //! [Tell the window that something is to be displayed]
    // Do not call app.m_mainWindow->prepareDisplay(); as it replace the active camera by the
    // default one
    app.m_mainWindow->getViewer()->makeCurrent();
    app.m_mainWindow->getViewer()->getRenderer()->buildAllRenderTechniques();
    app.m_mainWindow->getViewer()->doneCurrent();
    //! [Tell the window that something is to be displayed]

    return app.exec();
}
