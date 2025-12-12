#include "Minimap_qt_user_functions.h"

/*****
 * Constructor: In order for drawing functions in this class to be used by
 * ARGoS it must be registered using the RegisterUserFunction function.
 *****/
Minimap_qt_user_functions::Minimap_qt_user_functions() :
	loopFunctions(dynamic_cast<Minimap_loop_functions&>(CSimulator::GetInstance().GetLoopFunctions()))
{
	RegisterUserFunction<Minimap_qt_user_functions, CFootBotEntity>(&Minimap_qt_user_functions::DrawOnRobot);
	RegisterUserFunction<Minimap_qt_user_functions, CFloorEntity>(&Minimap_qt_user_functions::DrawOnArena);
}

void Minimap_qt_user_functions::DrawOnRobot(CFootBotEntity& entity) {
	Minimap_controller& c = dynamic_cast<Minimap_controller&>(entity.GetControllableEntity().GetController());

	if(c.IsHoldingFood() == true) {
		DrawCylinder(CVector3(0.0, 0.0, 0.3), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::BLACK);
	}

	if(loopFunctions.DrawIDs == 1) {
		/* Disable lighting, so it does not interfere with the chosen text color */
		glDisable(GL_LIGHTING);
		/* Disable face culling to be sure the text is visible from anywhere */
		glDisable(GL_CULL_FACE);
		/* Set the text color */
		CColor cColor(CColor::BLACK);
		glColor3ub(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());

		/* The position of the text is expressed wrt the reference point of the footbot
		 * For a foot-bot, the reference point is the center of its base.
		 * See also the description in
		 * $ argos3 -q foot-bot
		 */
		
		// Disable for now
		//GetOpenGLWidget().renderText(0.0, 0.0, 0.5,             // position
		//			     entity.GetId().c_str()); // text
		
		/* Restore face culling */
		glEnable(GL_CULL_FACE);
		/* Restore lighting */
		glEnable(GL_LIGHTING);
	}
}
 
void Minimap_qt_user_functions::DrawOnArena(CFloorEntity& entity) {
	DrawFood();
	DrawFidelity();
	DrawPheromones();
	DrawNest();
	DrawVisitedLocations();

	if(loopFunctions.DrawTargetRays == 1) DrawTargetRays();
}

/*****
 * This function is called by the DrawOnArena(...) function. If the iAnt_data
 * object is not initialized this function should not be called.
 *****/
void Minimap_qt_user_functions::DrawNest() {
	/* 2d cartesian coordinates of the nest */
	Real x_coordinate = loopFunctions.NestPosition.GetX();
	Real y_coordinate = loopFunctions.NestPosition.GetX();

	/* required: leaving this 0.0 will draw the nest inside of the floor */
	Real elevation = loopFunctions.NestElevation;

	/* 3d cartesian coordinates of the nest */
	CVector3 nest_3d(x_coordinate, y_coordinate, elevation);

	/* Draw the nest on the arena. */
	DrawCircle(nest_3d, CQuaternion(), loopFunctions.NestRadius, CColor::GRAY50);
}

void Minimap_qt_user_functions::DrawFood() {

	Real x, y;

	for(size_t i = 0; i < loopFunctions.FoodList.size(); i++) {
		x = loopFunctions.FoodList[i].GetX();
		y = loopFunctions.FoodList[i].GetY();
		DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, loopFunctions.FoodColoringList[i]);
	}
}

void Minimap_qt_user_functions::DrawFidelity() {

	Real x, y;
        for(map<string, CVector2>::iterator it= loopFunctions.FidelityList.begin(); it!=loopFunctions.FidelityList.end(); ++it) {
            x = it->second.GetX();
            y = it->second.GetY();
		DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::CYAN);
	}
}

void Minimap_qt_user_functions::DrawPheromones() {

	Real x, y, weight;
	vector<CVector2> trail;
	CColor trailColor = CColor::GREEN, pColor = CColor::GREEN;

	for(size_t i = 0; i < loopFunctions.PheromoneList.size(); i++) {
		x = loopFunctions.PheromoneList[i].GetLocation().GetX();
		y = loopFunctions.PheromoneList[i].GetLocation().GetY();

		if(loopFunctions.DrawTrails == 1) {
			trail  = loopFunctions.PheromoneList[i].GetTrail();
			weight = loopFunctions.PheromoneList[i].GetWeight();

			if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
				pColor = trailColor = CColor::GREEN;
			else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
				pColor = trailColor = CColor::YELLOW;
			else                                      // [   5.0% ,  0.0% ]
				pColor = trailColor = CColor::RED;

			CRay3 ray;
			size_t j = 0;

			for(j = 1; j < trail.size(); j++) {
				ray = CRay3(CVector3(trail[j - 1].GetX(), trail[j - 1].GetY(), 0.01),
							CVector3(trail[j].GetX(), trail[j].GetY(), 0.01));
				DrawRay(ray, trailColor, 1.0);
			}

			DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, pColor);
		} else {
			weight = loopFunctions.PheromoneList[i].GetWeight();

			if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
				pColor = CColor::GREEN;
			else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
				pColor = CColor::YELLOW;
			else                                      // [   5.0% ,  0.0% ]
				pColor = CColor::RED;

			DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, pColor);
		}
	}
}

void Minimap_qt_user_functions::DrawTargetRays() {
	//size_t tick = loopFunctions.GetSpace().GetSimulationClock();
	//size_t tock = loopFunctions.GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick() / 8;

	//if(tock == 0) tock = 1;

	//if(tick % tock == 0) {
		for(size_t j = 0; j < loopFunctions.TargetRayList.size(); j++) {
			DrawRay(loopFunctions.TargetRayList[j], loopFunctions.TargetRayColorList[j]);
		}
	//}
}

void Minimap_qt_user_functions::DrawVisitedLocations() {
	// Draw merged clusters as larger magenta dots, or individual yellow dots
	Real x, y;
	
	// First, draw merged clusters (areas with >50% coverage)
	for(size_t i = 0; i < loopFunctions.VisitedClusters.size(); i++) {
		if(loopFunctions.VisitedClusters[i].isMerged) {
			x = loopFunctions.VisitedClusters[i].center.GetX();
			y = loopFunctions.VisitedClusters[i].center.GetY();
			
			// Draw as a larger magenta dot to indicate a merged/clustered area
			CColor clusterColor = CColor::MAGENTA;
			Real clusterRadius = 0.1; // Larger radius for clusters
			Real clusterHeight = 0.02; // Slightly taller
			
			DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), clusterRadius, clusterHeight, clusterColor);
		}
	}

	// Then, draw individual visited locations that aren't in merged clusters
	for(size_t i = 0; i < loopFunctions.VisitedLocations.size(); i++) {
		x = loopFunctions.VisitedLocations[i].GetX();
		y = loopFunctions.VisitedLocations[i].GetY();
		
		// Check if this location is part of a merged cluster
		bool isInMergedCluster = false;
		for(size_t j = 0; j < loopFunctions.VisitedClusters.size(); j++) {
			if(loopFunctions.VisitedClusters[j].isMerged) {
				CVector2 diff = loopFunctions.VisitedLocations[i] - loopFunctions.VisitedClusters[j].center;
				if(std::abs(diff.GetX()) <= loopFunctions.VisitedClusters[j].width/2.0 && 
				   std::abs(diff.GetY()) <= loopFunctions.VisitedClusters[j].height/2.0) {
					isInMergedCluster = true;
					break;
				}
			}
		}
		
		// Only draw individual dots for locations not in merged clusters
		if(!isInMergedCluster) {
			CColor dotColor = CColor::YELLOW;
			Real dotRadius = 0.02; // Small radius for the dots
			Real dotHeight = 0.01; // Very small height
			DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), dotRadius, dotHeight, dotColor);
		}
	}
}

/*
void Minimap_qt_user_functions::DrawTargetRays() {

	CColor c = CColor::BLUE;

	for(size_t j = 0; j < loopFunctions.TargetRayList.size(); j++) {
			DrawRay(loopFunctions.TargetRayList[j],c);
	}

	//if(loopFunctions.SimTime % (loopFunctions.TicksPerSecond * 10) == 0) {
		// comment out for DSA, uncomment for Minimap
		loopFunctions.TargetRayList.clear();
	//}
}
*/

REGISTER_QTOPENGL_USER_FUNCTIONS(Minimap_qt_user_functions, "Minimap_qt_user_functions")
