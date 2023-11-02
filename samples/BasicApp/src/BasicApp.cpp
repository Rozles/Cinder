#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/CinderImGui.h"
#include "cinder/Rand.h"

using namespace ci;
using namespace ci::app;
namespace gui = ImGui;



namespace PhysicsEngine {

	struct BallStruct {
		BallStruct(glm::vec2 pos, float rad, ci::Color col) : position(pos), radius(rad), color(col) {}
		glm::vec2 position;
		glm::vec2 velocity;
		float radius;
		ci::Color color;
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
	struct MousePointerStruct {
		MousePointerStruct() : ball(nullptr), dragging(false) {}
		BallStruct* ball;
		bool dragging;
		glm::vec2 grabbingVec;
	};

	class BallEngine {
	public:
		std::vector<BallStruct*> balls;
		std::vector<BallForce*> forces;
		EngineSettings settings;
		MousePointerStruct* mousePointer;
		float airDragCoefficient;
		float groundFrictionCoefficient;
		float minBallRadius;
		float maxBallRadius;
		Rand* randPtr;

	public:
		BallEngine(glm::ivec2 bounds);
		~BallEngine();
		void Update();
		void Draw();
		void AddBall();
	private:
		void CheckBoundingBox(BallStruct* ball);
		void ApplyForces(BallStruct* ball);
		void CheckCollision(BallStruct* ball, int index);
	};

	BallEngine::BallEngine(glm::ivec2 bounds) {
		randPtr = new Rand();
		airDragCoefficient = .02f;
		groundFrictionCoefficient = .15f;
		minBallRadius = 8.f;
		maxBallRadius = 48.f;
		settings.bounds = bounds;
		mousePointer = new MousePointerStruct();
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

	void BallEngine::AddBall() {
		float radius = randPtr->nextFloat(minBallRadius, maxBallRadius);
		ci::Color color = ci::Color(randPtr->nextFloat(), randPtr->nextFloat(), randPtr->nextFloat());
		glm::vec2 position = glm::vec2(randPtr->nextFloat(radius, (float)settings.bounds.x - radius), randPtr->nextFloat(radius, (float)settings.bounds.y - radius));
		balls.push_back(new BallStruct(position, radius, color));
	}


	void BallEngine::CheckBoundingBox(BallStruct* ball) {
		/*  Ground friction equation                    
		 *               |              |         Cf
		 *  F = Cf' * N  |  N ~ v * m   | Cf' = ------
		 *               |              |       avg(m)              
		 */
		glm::vec2 groundFriction = -(groundFrictionCoefficient / (((minBallRadius + maxBallRadius) / 2) * ((minBallRadius + maxBallRadius) / 2))) * ball->velocity * (ball->radius * ball->radius);

		if (ball->position.y >= settings.bounds.y - ball->radius * .5f) {
			if (ball->velocity.y > 0.f)
				ball->velocity.y = -(ball->velocity.y + groundFriction.y);	
			ball->velocity.x += groundFriction.x;
			ball->position.y = settings.bounds.y - ball->radius * .5f;
		}

		if (ball->position.y <= ball->radius * .5f) {
			if (ball->velocity.y < 0.f) 
				ball->velocity.y = -(ball->velocity.y + groundFriction.y);	
			ball->velocity.x += groundFriction.x;
			ball->position.y = ball->radius * .5f;
		}

		if (ball->position.x <= ball->radius * .5f) {
			if (ball->velocity.x < 0.f) 
				ball->velocity.x = -(ball->velocity.x + groundFriction.x);
			ball->velocity.y += groundFriction.y;
			ball->position.x = ball->radius * .5f;
		}

		if (ball->position.x >= settings.bounds.x - ball->radius * .5f) {
			if (ball->velocity.x > 0.f) 
				ball->velocity.x = -(ball->velocity.x + groundFriction.x);
			ball->velocity.y += groundFriction.y;
			ball->position.x = settings.bounds.x - ball->radius * .5f;
		}
	}

	void BallEngine::CheckCollision(BallStruct* ball1, int index) {
		for (int i = index + 1; i < balls.size(); i++) {
			// if (i == index) continue;
			
			float distance = glm::distance(ball1->position, balls[i]->position);
			if (distance <= ball1->radius + balls[i]->radius) {

				// Resolve overlapping

				BallStruct* ball2 = balls[i];
				glm::vec2 distanceVec = ball2->position - ball1->position;
				glm::vec2 push = distanceVec * ((ball1->radius + ball2->radius) - distance) / distance;
				ball1->position -= push * 0.5f;
				ball2->position += push * 0.5f;

				if (ball1 == mousePointer->ball || ball2 == mousePointer->ball)
					continue;

				// Elastic collision

				glm::vec2 collisionNormal = glm::normalize(distanceVec);
				glm::vec2 collisionTangent = glm::vec2(-collisionNormal.y, collisionNormal.x);

				float v1n = glm::dot(collisionNormal, ball1->velocity);
				float v1t = glm::dot(collisionTangent, ball1->velocity);
				float v2n = glm::dot(collisionNormal, ball2->velocity);
				float v2t = glm::dot(collisionTangent, ball2->velocity);
					
				float mass1 = ball1->radius * ball1->radius;
				float mass2 = ball2->radius * ball2->radius;

				float u1n = (mass1 - mass2) / (mass1 + mass2) * v1n + (2 * mass2) / (mass1 + mass2) * v2n;
				float u2n = (2 * mass1) / (mass1 + mass2) * v1n + (mass2 - mass1) / (mass1 + mass2) * v2n;

				ball1->velocity = collisionNormal * u1n + collisionTangent * v1t;
				ball2->velocity = collisionNormal * u2n + collisionTangent * v2t;

			}
		}
	}

	void BallEngine::ApplyForces(BallStruct* ball) {
		if (ball == mousePointer->ball) 
			return;

		for (BallForce* force : forces) {
			ball->velocity += force->velocity;
		}

		/*  Air Drag equation
		 *                     |         |         Cd
		 *  F = v^2 * Cd' * S  |  S ~ r  |  Cd' = -----  * 10^-3
		 *                     |         |        avg(S)
		 */
		glm::vec2 vSquare = glm::vec2(ball->velocity.x * ball->velocity.x, ball->velocity.y * ball->velocity.y);
		glm::vec2 airDrag = -vSquare * (airDragCoefficient * .01f / ((minBallRadius + maxBallRadius) / 2)) * ball->radius;
		ball->velocity += airDrag;

		ball->position += ball->velocity;

	}

	void BallEngine::Update() {
		int index = 0;
		for (BallStruct* ball : balls) {
			CheckBoundingBox(ball);
			CheckCollision(ball, index++);
			ApplyForces(ball);
		}
	}
	void BallEngine::Draw() {
		for (BallStruct* ball : balls) {
			ci::gl::ScopedColor color(ball->color);
			gl::drawSolidCircle(ball->position, ball->radius);
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
		if (glm::distance(ball->position, (glm::vec2)event.getPos()) < ball->radius) {
			engine->mousePointer->ball = ball;
			engine->mousePointer->grabbingVec = (glm::vec2)event.getPos() - ball->position;
			ball->velocity = glm::vec2(.0f, .0f);
			break;
		}
	}
}

void BasicApp::mouseDrag( MouseEvent event ) {
	PhysicsEngine::BallStruct* ball = engine->mousePointer->ball;
	
	if (ball == nullptr) return;

    ball->position = (glm::vec2)event.getPos() - engine->mousePointer->grabbingVec;
	engine->mousePointer->dragging = true;
    
}

void BasicApp::mouseUp( MouseEvent event ) {
	PhysicsEngine::BallStruct* ball = engine->mousePointer->ball;

	if (ball == nullptr) return;

	if (!engine->mousePointer->dragging)
		ball->velocity = ((glm::vec2)event.getPos() - (ball->position)) * ((engine->maxBallRadius + engine->minBallRadius) * .5f / ball->radius);

	engine->mousePointer->ball = nullptr;
	engine->mousePointer->dragging = false;
	
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
			if (gui::Button("Add Ball")) {
				engine->AddBall();
			}
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
			gui::PushID(1);
			gui::TextDisabled("Air Drag");
			gui::DragFloat("Coefficient", &engine->airDragCoefficient);
			gui::PopID();
			gui::PushID(2);
			gui::TextDisabled("Ground Friction");
			gui::DragFloat("Coefficient", &engine->groundFrictionCoefficient);
			gui::PopID();
			gui::End();
		}
	}

}

// This line tells Cinder to actually create and run the application.
CINDER_APP( BasicApp, RendererGl, prepareSettings )
