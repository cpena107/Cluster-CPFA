#include "Cluster_qt_user_functions.h"

/*****
 * Constructor: In order for drawing functions in this class to be used by
 * ARGoS it must be registered using the RegisterUserFunction function.
 *****/
Cluster_qt_user_functions::Cluster_qt_user_functions() :
	loopFunctions(dynamic_cast<Cluster_loop_functions&>(CSimulator::GetInstance().GetLoopFunctions()))
{
	RegisterUserFunction<Cluster_qt_user_functions, CFootBotEntity>(&Cluster_qt_user_functions::DrawOnRobot);
	RegisterUserFunction<Cluster_qt_user_functions, CFloorEntity>(&Cluster_qt_user_functions::DrawOnArena);
}

void Cluster_qt_user_functions::DrawOnRobot(CFootBotEntity& entity) {
	Cluster_controller& c = dynamic_cast<Cluster_controller&>(entity.GetControllableEntity().GetController());

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
 
void Cluster_qt_user_functions::DrawOnArena(CFloorEntity& entity) {
	DrawFood();
	DrawFidelity();
	DrawLowClusterTargets();
	DrawPheromones();
	DrawNest();
	DrawVisitedLocations();

	if(loopFunctions.DrawTargetRays == 1) DrawTargetRays();
}

/*****
 * This function is called by the DrawOnArena(...) function. If the iAnt_data
 * object is not initialized this function should not be called.
 *****/
void Cluster_qt_user_functions::DrawNest() {
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

void Cluster_qt_user_functions::DrawFood() {

	Real x, y;

	for(size_t i = 0; i < loopFunctions.FoodList.size(); i++) {
		x = loopFunctions.FoodList[i].GetX();
		y = loopFunctions.FoodList[i].GetY();
		DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, loopFunctions.FoodColoringList[i]);
	}
}

void Cluster_qt_user_functions::DrawFidelity() {

	Real x, y;
        for(map<string, CVector2>::iterator it= loopFunctions.FidelityList.begin(); it!=loopFunctions.FidelityList.end(); ++it) {
            x = it->second.GetX();
            y = it->second.GetY();
		DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::CYAN);
	}
}

void Cluster_qt_user_functions::DrawPheromones() {

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

void Cluster_qt_user_functions::DrawLowClusterTargets() {
	CSpace& space = CSimulator::GetInstance().GetSpace();
	CSpace::TMapPerType& footbots = space.GetEntitiesByType("foot-bot");

	for(map<string, CVector2>::iterator it = loopFunctions.LowClusterTargetList.begin();
	    it != loopFunctions.LowClusterTargetList.end(); ++it) {
		const std::string& robotID = it->first;
		const CVector2&    target  = it->second;

		// Orange marker at the chosen low-cluster target.
		DrawCylinder(CVector3(target.GetX(), target.GetY(), 0.0),
		             CQuaternion(),
		             loopFunctions.FoodRadius * 2.0, 0.05,
		             CColor::ORANGE);

		// Blue line from the robot's current position to the target.
		CSpace::TMapPerType::iterator fbIt = footbots.find(robotID);
		if(fbIt != footbots.end()) {
			CFootBotEntity& fb      = *any_cast<CFootBotEntity*>(fbIt->second);
			CVector3        robotPos = fb.GetEmbodiedEntity().GetOriginAnchor().Position;
			DrawRay(CRay3(CVector3(robotPos.GetX(), robotPos.GetY(), 0.01),
			              CVector3(target.GetX(),   target.GetY(),   0.01)),
			        CColor::BLUE);
		}
	}
}

void Cluster_qt_user_functions::DrawTargetRays() {
	// Draw trails for each robot
	for(std::map<std::string, std::vector<argos::CRay3>>::iterator it = loopFunctions.RobotTrails.begin(); it != loopFunctions.RobotTrails.end(); ++it) {
		std::string robotID = it->first;
		std::vector<argos::CRay3>& trails = it->second;
		
		CColor color = CColor::BLACK;
		std::map<std::string, CColor>::iterator colorIt = loopFunctions.RobotTrailColors.find(robotID);
		if(colorIt != loopFunctions.RobotTrailColors.end()) {
			color = colorIt->second;
		}
		
		for(size_t j = 0; j < trails.size(); j++) {
			DrawRay(trails[j], color);
		}
	}

    CColor c = CColor::BLUE;
	for(size_t j = 0; j < loopFunctions.SearchLocationRays.size(); j++) {
			DrawRay(loopFunctions.SearchLocationRays[j],c);
	}

    // Clear search rays periodically to avoid clutter (every 10 seconds)
    if((int)(loopFunctions.GetSpace().GetSimulationClock()) % ((int)(argos::CSimulator::GetInstance().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()) * 10) == 0 && loopFunctions.GetSpace().GetSimulationClock() > 0) {
		loopFunctions.SearchLocationRays.clear();
	}
}

void Cluster_qt_user_functions::DrawVisitedLocations() {
	// Draw merged clusters as larger magenta dots, or individual yellow dots
	Real x, y;
	
	// First, draw merged clusters (areas with >50% coverage)
	for(size_t i = 0; i < loopFunctions.VisitedClusters.size(); i++) {
		if(loopFunctions.VisitedClusters[i].isMerged) {
			x = loopFunctions.VisitedClusters[i].center.GetX();
			y = loopFunctions.VisitedClusters[i].center.GetY();
			
			// Draw as a larger magenta dot to indicate a merged/clustered area
			CColor clusterColor = CColor::MAGENTA;
			Real clusterRadius = loopFunctions.VisitedClusters[i].radius; // Use stored radius
			Real clusterHeight = 0.02; // Slightly taller
			
			if(loopFunctions.VisitedClusters[i].isFrozen) {
				clusterColor = CColor::BLUE;
				clusterHeight = 0.02; // Slightly taller for merged clusters
			}
			DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), 0.05, clusterHeight, clusterColor);
			DrawCircle(CVector3(x, y, 0.01), CQuaternion(), clusterRadius, clusterColor, false);

			// Draw cluster ID to the left of the center for debugging
			DrawText(CVector3(x - clusterRadius - 0.1, y, 0.05),
			         std::to_string(loopFunctions.VisitedClusters[i].clusterId)+", "+std::to_string(loopFunctions.VisitedClusters[i].radius));
		}
	}

	// Then, draw individual visited locations that aren't in merged clusters.
	// The exclusion radius is expanded by eps (0.5m) to cover compression chain
	// endpoints that may slightly overshoot the cluster's stored radius due to
	// ceil() rounding in the chain-step calculation.
	const Real exclusionBuffer = 0.5; // matches DBSCAN eps in Cluster_loop_functions
	for(size_t i = 0; i < loopFunctions.VisitedLocations.size(); i++) {
		x = loopFunctions.VisitedLocations[i].GetX();
		y = loopFunctions.VisitedLocations[i].GetY();
		
		// Check if this location is part of a merged cluster
		bool isInMergedCluster = false;
		for(size_t j = 0; j < loopFunctions.VisitedClusters.size(); j++) {
			if(loopFunctions.VisitedClusters[j].isMerged) {
				Real exclusionRadius = loopFunctions.VisitedClusters[j].radius + exclusionBuffer;
				CVector2 diff = loopFunctions.VisitedLocations[i] - loopFunctions.VisitedClusters[j].center;
				if(diff.SquareLength() <= exclusionRadius * exclusionRadius) {
					isInMergedCluster = true;
					break;
				}
			}
		}
		
		// Only draw individual dots for locations not in merged clusters
		if(!isInMergedCluster) {
			CColor dotColor = CColor::YELLOW;
			Real dotRadius = 0.05; // Small radius for the dots
			Real dotArea = 0.16; // Contour of the area covered by the dot (for visualization purposes)
			Real dotHeight = 0.01; // Very small height
			DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), dotRadius, dotHeight, dotColor);
			DrawCircle(CVector3(x, y, 0.01), CQuaternion(), dotArea, CColor::YELLOW, false); // Add a transparent circle to indicate coverage area
		}
	}

	// Draw real robot-visit points that contributed to a cluster in the last
	// DBSCAN run as small green cylinders for debugging.
	for(size_t i = 0; i < loopFunctions.ClusteredVisitedLocations.size(); i++) {
		x = loopFunctions.ClusteredVisitedLocations[i].GetX();
		y = loopFunctions.ClusteredVisitedLocations[i].GetY();
		DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), 0.04, 0.03, CColor::GREEN);
	}
}

/*
void Cluster_qt_user_functions::DrawTargetRays() {

	CColor c = CColor::BLUE;

	for(size_t j = 0; j < loopFunctions.TargetRayList.size(); j++) {
			DrawRay(loopFunctions.TargetRayList[j],c);
	}

	//if(loopFunctions.SimTime % (loopFunctions.TicksPerSecond * 10) == 0) {
		// comment out for DSA, uncomment for Cluster
		loopFunctions.TargetRayList.clear();
	//}
}
*/

REGISTER_QTOPENGL_USER_FUNCTIONS(Cluster_qt_user_functions, "Cluster_qt_user_functions")
