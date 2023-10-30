#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CinderImGui.h"

using namespace ci;
using namespace ci::app;
namespace gui = ImGui;

#define BALL_RADIUS 32.f // 16 pixels
#define VELO_DAMPIN 0.45f // 16 pixels

namespace PhysicsEngine {

	struct BallStruct {
		BallStruct(glm::vec2 pos) : position(pos), isDragging(false) {}
        bool isDragging;
		glm::vec2 position;
		glm::vec2 velocity;
	};
	struct BallForce {
		BallForce(const char* label, glm::vec2 force) : name(label), velocity(force) {}
		std::string name = "";
		glm::vec2 velocity;
	};
	struct EngineSettings {
		void Set(glm::ivec2 size) { bounds = size; }
		glm::ivec2 bounds;
	};

	class BallEngine {
	public:
		const std::vector<ci::Color> colors = {
			ci::Color(1.f, 0.f, 0.f), ci::Color(0.f, 1.f, 0.f)
		};
		std::vector<BallStruct*> balls;
		std::vector<BallForce*> forces;
		EngineSettings settings;
	public:
		BallEngine(glm::ivec2 bounds);
		~BallEngine();
		void Update();
		void Draw();
	};

	BallEngine::BallEngine(glm::ivec2 bounds) {
		settings.bounds = bounds;
		balls.push_back(new BallStruct(glm::vec2((float)settings.bounds.x * .25f, settings.bounds.y * .5f)));
		balls.push_back(new BallStruct(glm::vec2((float)settings.bounds.x * .75f, settings.bounds.y * .5f)));
		forces.push_back(new BallForce("Gravity", glm::vec2(0.f, .5f)));
		forces.push_back(new BallForce("Wind", glm::vec2(0.f, 0.f)));
	}

	BallEngine::~BallEngine() {
		while (balls.size()) {
			delete balls.front();
			balls.erase(balls.begin());
		}
		while (forces.size()) {
			delete forces.front();
			forces.erase(forces.begin());
		}
	}

	void BallEngine::Update() {
		for (BallStruct* ball : balls) {
            if (!ball->isDragging) {
                for(BallForce* force : forces) {
                    ball->velocity += force->velocity;
                }
            }
            
			ball->position += ball->velocity;
			if (ball->position.y >= settings.bounds.y - BALL_RADIUS * .5f) {
				if (ball->velocity.y > 0.f) {
					ball->velocity.y *= -VELO_DAMPIN;
					ball->position.y = settings.bounds.y - BALL_RADIUS * .5f;
				}
			}
			if (ball->position.y <= BALL_RADIUS * .5f) {
				if (ball->velocity.y < 0.f) {
					ball->velocity.y = 0.f;
					ball->position.y = BALL_RADIUS * .5f;
				}
			}
			if (ball->position.x <= BALL_RADIUS * .5f) {
				if (ball->velocity.x < 0.f) {
					ball->velocity.x *= -VELO_DAMPIN * VELO_DAMPIN;
					ball->position.x = BALL_RADIUS * .5f;
				}
			}
			if (ball->position.x >= settings.bounds.x - BALL_RADIUS * .5f) {
				if (ball->velocity.x > 0.f) {
					ball->velocity.x *= -VELO_DAMPIN * VELO_DAMPIN;
					ball->position.x = settings.bounds.x - BALL_RADIUS * .5f;
				}
			}
		}
	}
	void BallEngine::Draw() {
		int index = 0;
		for (BallStruct* ball : balls) {
			ci::gl::ScopedColor color(colors[index++]);
			gl::drawSolidCircle(ball->position, BALL_RADIUS);
		}
	}

}

class BasicApp : public App {

	protected:

	private:

		PhysicsEngine::BallEngine* engine = nullptr;

	public:

		// Cinder will call 'mouseDrag' when the user moves the mouse while holding one of its buttons.
		// See also: mouseMove, mouseDown, mouseUp and mouseWheel.
		void mouseDrag( MouseEvent event ) override;
		void mouseDown(MouseEvent event) override;
        void mouseUp(MouseEvent event) override;

		// Cinder will call 'keyDown' when the user presses a key on the keyboard.
		// See also: keyUp.
		void keyDown( KeyEvent event ) override;
		void keyUp( KeyEvent event ) override {}
		void resize() override;

		// Cinder will call 'draw' each time the contents of the window need to be redrawn.
		void draw() override;
		void update() override;
		void setup() override;

};

void prepareSettings( BasicApp::Settings* settings ) {
	settings->setMultiTouchEnabled( false );
}

void BasicApp::mouseDown(MouseEvent event) {
	for (PhysicsEngine::BallStruct* ball : engine->balls) {
		if (glm::distance(ball->position, (glm::vec2)event.getPos()) < BALL_RADIUS && !ball->isDragging) {
            ball->velocity = (glm::vec2)event.getPos() - (ball->position);
		}
	}
}

void BasicApp::mouseDrag( MouseEvent event ) {
    for (PhysicsEngine::BallStruct* ball : engine->balls) {
        if (glm::distance(ball->position, (glm::vec2)event.getPos()) < BALL_RADIUS) {
            ball->isDragging = true;
            ball->velocity = (glm::vec2)event.getPos() - (ball->position);
            ball->position = (glm::vec2)event.getPos();
        }
    }
}

void BasicApp::mouseUp( MouseEvent event ) {
    for (PhysicsEngine::BallStruct* ball : engine->balls) {
        ball->isDragging = false;
    }
}

void BasicApp::keyDown( KeyEvent event )
{
	if( event.getChar() == 'f' ) {
		// Toggle full screen when the user presses the 'f' key.
		setFullScreen( ! isFullScreen() );
	}
	else if( event.getCode() == KeyEvent::KEY_SPACE ) { }
	else if( event.getCode() == KeyEvent::KEY_ESCAPE ) {
		// Exit full screen, or quit the application, when the user presses the ESC key.
		if( isFullScreen() )
			setFullScreen( false );
		else
			quit();
	}
}

void BasicApp::resize() {
	engine->settings.Set(ci::app::getWindowSize());
}

void BasicApp::setup() {
	gui::Initialize();
	engine = new PhysicsEngine::BallEngine(ci::app::getWindowSize());
}

void BasicApp::update() {
	engine->Update();
}

void BasicApp::draw() {

	// Clear the contents of the window. This call will clear
	// both the color and depth buffers. 
	gl::clear( Color::gray( 0.1f ) );
	engine->Draw();

	{
		if (gui::Begin("Settings")) {
			for (PhysicsEngine::BallStruct* ball : engine->balls) {
				gui::PushID(ball);
				gui::DragFloat2("Position", &ball->position);
				gui::DragFloat2("Velocity", &ball->velocity);
				gui::PopID();
			}
			gui::Separator();
			for (PhysicsEngine::BallForce* force : engine->forces) {
				gui::PushID(force);
				gui::TextDisabled(force->name.c_str());
				gui::DragFloat2("Force", &force->velocity);
				gui::PopID();
			}
			gui::End();
		}
	}

}

// This line tells Cinder to actually create and run the application.
CINDER_APP( BasicApp, RendererGl, prepareSettings )
