#ifndef Minimap_QT_USER_FUNCTIONS_H
#define Minimap_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/ray3.h>
#include <source/Minimap/Minimap_loop_functions.h>

using namespace std;
using namespace argos;

class Minimap_loop_functions;

class Minimap_qt_user_functions : public argos::CQTOpenGLUserFunctions {

	public:

		Minimap_qt_user_functions();

		/* interface functions between QT and ARGoS */
		void DrawOnRobot(argos::CFootBotEntity& entity);
		void DrawOnArena(argos::CFloorEntity& entity);

	private:

		/* private helper drawing functions */
		void DrawNest();
		void DrawFood();
		void DrawFidelity();
		void DrawPheromones();
		void DrawTargetRays();
		void DrawVisitedLocations();

		Minimap_loop_functions& loopFunctions;

};

#endif /* Minimap_QT_USER_FUNCTIONS_H */