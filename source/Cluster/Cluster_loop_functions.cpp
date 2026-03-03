#include "Cluster_loop_functions.h"
#include <algorithm>

Cluster_loop_functions::Cluster_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
	MaxSimTime(3600 * GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()),
	ResourceDensityDelay(0),
	RandomSeed(GetSimulator().GetRandomSeed()),
	SimCounter(0),
	MaxSimCounter(1),
	VariableFoodPlacement(0),
	OutputData(0),
	DrawDensityRate(4),
	DrawIDs(1),
	DrawTrails(1),
	DrawTargetRays(1),
	FoodDistribution(2),
	FoodItemCount(256),
	NumberOfClusters(4),
	MaxClusterCount(0),
	ClusterWidthX(8),
	ClusterLengthY(8),
	PowerRank(4),
	ProbabilityOfSwitchingToSearching(0.0),
	ProbabilityOfReturningToNest(0.0),
	ProbabilityOfSearchingLowClusters(0.0),
	UninformedSearchVariation(0.0),
	RateOfInformedSearchDecay(0.0),
	RateOfSiteFidelity(0.0),
	RateOfLayingPheromone(0.0),
	RateOfPheromoneDecay(0.0),
	FoodRadius(0.05),
	FoodRadiusSquared(0.0025),
	NestRadius(0.25),
	NestRadiusSquared(0.0625),
	NestElevation(0.01),
	// We are looking at a 4 by 4 square (3 targets + 2*1/2 target gaps)
	SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
	NumDistributedFood(0),
	score(0),
	PrintFinalScore(0),
	SitesCommunicatedSum(0),
	SitesCommunicatedCount(0),
	MaxClusterRadius(1.0),
	numSyntheticPoints(0),
	nextClusterId(0)
{}

void Cluster_loop_functions::Init(argos::TConfigurationNode &node) {	
	argos::CDegrees USV_InDegrees;
	argos::TConfigurationNode Cluster_node = argos::GetNode(node, "Cluster");

	argos::GetNodeAttribute(Cluster_node, "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
	argos::GetNodeAttribute(Cluster_node, "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
	argos::GetNodeAttributeOrDefault(Cluster_node, "ProbabilityOfSearchingLowClusters", ProbabilityOfSearchingLowClusters, 0.3);
	argos::GetNodeAttribute(Cluster_node, "UninformedSearchVariation",         USV_InDegrees);
	argos::GetNodeAttribute(Cluster_node, "RateOfInformedSearchDecay",         RateOfInformedSearchDecay);
	argos::GetNodeAttribute(Cluster_node, "RateOfSiteFidelity",                RateOfSiteFidelity);
	argos::GetNodeAttribute(Cluster_node, "RateOfLayingPheromone",             RateOfLayingPheromone);
	argos::GetNodeAttribute(Cluster_node, "RateOfPheromoneDecay",              RateOfPheromoneDecay);
	argos::GetNodeAttribute(Cluster_node, "PrintFinalScore",                   PrintFinalScore);

	UninformedSearchVariation = ToRadians(USV_InDegrees);
	argos::TConfigurationNode settings_node = argos::GetNode(node, "settings");

	argos::GetNodeAttribute(settings_node, "MaxSimTimeInSeconds", MaxSimTime);

	MaxSimTime *= GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();

	argos::GetNodeAttribute(settings_node, "MaxSimCounter", MaxSimCounter);
	argos::GetNodeAttribute(settings_node, "VariableFoodPlacement", VariableFoodPlacement);
	argos::GetNodeAttribute(settings_node, "OutputData", OutputData);
	argos::GetNodeAttribute(settings_node, "DrawIDs", DrawIDs);
	argos::GetNodeAttribute(settings_node, "DrawTrails", DrawTrails);
	argos::GetNodeAttribute(settings_node, "DrawTargetRays", DrawTargetRays);
	argos::GetNodeAttribute(settings_node, "FoodDistribution", FoodDistribution);
	argos::GetNodeAttribute(settings_node, "FoodItemCount", FoodItemCount);
	argos::GetNodeAttribute(settings_node, "NumberOfClusters", NumberOfClusters);
	argos::GetNodeAttribute(settings_node, "ClusterWidthX", ClusterWidthX);
	argos::GetNodeAttribute(settings_node, "ClusterLengthY", ClusterLengthY);
	argos::GetNodeAttribute(settings_node, "FoodRadius", FoodRadius);
	argos::GetNodeAttribute(settings_node, "NestElevation", NestElevation);
	argos::GetNodeAttributeOrDefault(settings_node, "MaxClusterRadius", MaxClusterRadius, MaxClusterRadius);
	
	FoodRadiusSquared = FoodRadius*FoodRadius;

    //Number of distributed foods
    if (FoodDistribution == 1){
        NumDistributedFood = ClusterWidthX*ClusterLengthY*NumberOfClusters;
    }
    else
    NumDistributedFood = FoodItemCount;  
	// calculate the forage range and compensate for the robot's radius of 0.085m
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085;
	argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085;
	ForageRangeX.Set(-rangeX, rangeX);
	ForageRangeY.Set(-rangeY, rangeY);

	// Send a pointer to this loop functions object to each controller.
	argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	argos::CSpace::TMapPerType::iterator it;

    Num_robots = footbots.size();
    RobotsReturnedToNest = 0;
    LastProcessedLocationIndex = 0;
	for(it = footbots.begin(); it != footbots.end(); it++) {
		argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		Cluster_controller& c2 = dynamic_cast<Cluster_controller&>(c);

		c2.SetLoopFunctions(this);
	}

	SetFoodDistribution();
    ForageList.clear(); 
}

void Cluster_loop_functions::Reset() {
	if(VariableFoodPlacement == 0) {
		RNG->Reset();
	}

	GetSpace().Reset();
	GetSpace().GetFloorEntity().Reset();
	MaxSimCounter = SimCounter;
	SimCounter = 0;
  	score = 0.0;
	MaxClusterCount = 0;
  	RobotsReturnedToNest = 0;
  	LastProcessedLocationIndex = 0;
	SitesCommunicatedSum = 0;
	SitesCommunicatedCount = 0;

	FoodList.clear();
	FoodColoringList.clear();
	PheromoneList.clear();
	FidelityList.clear();
	LowClusterTargetList.clear();
	//TargetRayList.clear();
	//TargetRayColorList.clear();
    RobotTrails.clear();
    SearchLocationRays.clear();
    RobotTrailColors.clear();
	VisitedClusters.clear();
    FrozenClusters.clear();
	VisitedLocations.clear();
	ClusteredVisitedLocations.clear();
	clusteredLocationIndices.clear();
	ClusterMap.clear();
	nextClusterId = 0;
	numSyntheticPoints = 0;

	SetFoodDistribution();
	argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	argos::CSpace::TMapPerType::iterator it;

	for(it = footbots.begin(); it != footbots.end(); it++) {
		argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		Cluster_controller& c2 = dynamic_cast<Cluster_controller&>(c);

		MoveEntity(footBot.GetEmbodiedEntity(), c2.GetStartPosition(), argos::CQuaternion(), false);
    c2.Reset();
	}
}

void Cluster_loop_functions::PreStep() {
	UpdatePheromoneList();

	if(GetSpace().GetSimulationClock() > ResourceDensityDelay) {
		for(size_t i = 0; i < FoodColoringList.size(); i++) {
			FoodColoringList[i] = argos::CColor::BLACK;
		}
	}

	if(FoodList.size() == 0) {
		FidelityList.clear();
		// TargetRayList.clear();
        RobotTrails.clear();
        SearchLocationRays.clear();
		PheromoneList.clear();
		VisitedLocations.clear();
		// VisitedClusters.clear(); // Don't clear clusters when food runs out - preserve them
	}
}

void Cluster_loop_functions::PostStep() {
	// Update visited location clusters periodically (every 5 seconds)
	// This ensures clusters are created even if robots aren't returning with food
	size_t ticksPerUpdate = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick() * 5;
	if(GetSpace().GetSimulationClock() % ticksPerUpdate == 0 && !VisitedLocations.empty()) {
		UpdateVisitedClusters();
		if (VisitedClusters.size() > MaxClusterCount) {
			MaxClusterCount = VisitedClusters.size();
		}
	}
}

bool Cluster_loop_functions::IsExperimentFinished() {
	bool isFinished = false;

	if(FoodList.size() == 0 || GetSpace().GetSimulationClock() >= MaxSimTime) {
		isFinished = true;
	}

	if(isFinished == true && MaxSimCounter > 1) {
		size_t newSimCounter = SimCounter + 1;
		size_t newMaxSimCounter = MaxSimCounter - 1;

		PostExperiment();
		Reset();

		SimCounter    = newSimCounter;
		MaxSimCounter = newMaxSimCounter;
		isFinished    = false;
	}

	return isFinished;
}

void Cluster_loop_functions::PostExperiment() {
	if (PrintFinalScore == 1) {
		double avgSites = (SitesCommunicatedCount > 0)
			? static_cast<double>(SitesCommunicatedSum) / static_cast<double>(SitesCommunicatedCount)
			: 0.0;
		printf("%f, %f, %lu, %lu, %f\n", getSimTimeInSeconds(), score, MaxClusterCount, VisitedClusters.size(), avgSites);
	}
}

argos::CColor Cluster_loop_functions::GetFloorColor(const argos::CVector2 &c_pos_on_floor) {
	return argos::CColor::WHITE;
}

void Cluster_loop_functions::UpdatePheromoneList() {
	// Return if this is not a tick that lands on a 0.5 second interval
	if ((int)(GetSpace().GetSimulationClock()) % ((int)(GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()) / 2) != 0) return;
	
	std::vector<Pheromone> new_p_list; 

	argos::Real t = GetSpace().GetSimulationClock() / GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();

	//ofstream log_output_stream;
	//log_output_stream.open("time.txt", ios::app);
	//log_output_stream << t << ", " << GetSpace().GetSimulationClock() << ", " << GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick() << endl;
	//log_output_stream.close();

	for(size_t i = 0; i < PheromoneList.size(); i++) {

		PheromoneList[i].Update(t);

		if(PheromoneList[i].IsActive() == true) {
			new_p_list.push_back(PheromoneList[i]);
		}
	}

	PheromoneList = new_p_list;
}

void Cluster_loop_functions::SetFoodDistribution() {
	switch(FoodDistribution) {
		case 0:
			RandomFoodDistribution();
			break;
		case 1:
			ClusterFoodDistribution();
			break;
		case 2:
			PowerLawFoodDistribution();
			break;
		default:
			argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
	}
}

void Cluster_loop_functions::RandomFoodDistribution() {
	FoodList.clear();

	argos::CVector2 placementPosition;

	for(size_t i = 0; i < FoodItemCount; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, 1, 1)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		FoodList.push_back(placementPosition);
		FoodColoringList.push_back(argos::CColor::BLACK);
	}
}

void Cluster_loop_functions::ClusterFoodDistribution() {
        FoodList.clear();
	argos::Real     foodOffset  = 3.0 * FoodRadius;
	size_t          foodToPlace = NumberOfClusters * ClusterWidthX * ClusterLengthY;
	size_t          foodPlaced = 0;
	argos::CVector2 placementPosition;

	FoodItemCount = foodToPlace;

	for(size_t i = 0; i < NumberOfClusters; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, ClusterLengthY, ClusterWidthX)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		for(size_t j = 0; j < ClusterLengthY; j++) {
			for(size_t k = 0; k < ClusterWidthX; k++) {
				foodPlaced++;
				/*
				#include <argos3/plugins/simulator/entities/box_entity.h>

				string label("my_box_");
				label.push_back('0' + foodPlaced++);

				CBoxEntity *b = new CBoxEntity(label,
					CVector3(placementPosition.GetX(),
					placementPosition.GetY(), 0.0), CQuaternion(), true,
					CVector3(0.1, 0.1, 0.001), 1.0);
				AddEntity(*b);
				*/

				FoodList.push_back(placementPosition);
				FoodColoringList.push_back(argos::CColor::BLACK);
				placementPosition.SetX(placementPosition.GetX() + foodOffset);
			}

			placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
			placementPosition.SetY(placementPosition.GetY() + foodOffset);
		}
	}
}

void Cluster_loop_functions::PowerLawFoodDistribution() {
 FoodList.clear();
	argos::Real foodOffset     = 3.0 * FoodRadius;
	size_t      foodPlaced     = 0;
	size_t      powerLawLength = 1;
	size_t      maxTrials      = 200;
	size_t      trialCount     = 0;

	std::vector<size_t> powerLawClusters;
	std::vector<size_t> clusterSides;
	argos::CVector2     placementPosition;

    //-----Wayne: Dertermine PowerRank and food per PowerRank group
    size_t priorPowerRank = 0;
    size_t power4 = 0;
    size_t FoodCount = 0;
    size_t diffFoodCount = 0;
    size_t singleClusterCount = 0;
    size_t otherClusterCount = 0;
    size_t modDiff = 0;
    
    //Wayne: priorPowerRank is determined by what power of 4
    //plus a multiple of power4 increases the food count passed required count
    //this is how powerlaw works to divide up food into groups
    //the number of groups is the powerrank
    while (FoodCount < FoodItemCount){
        priorPowerRank++;
        power4 = pow (4.0, priorPowerRank);
        FoodCount = power4 + priorPowerRank * power4;
    }
    
    //Wayne: Actual powerRank is prior + 1
    PowerRank = priorPowerRank + 1;
    
    //Wayne: Equalizes out the amount of food in each group, with the 1 cluster group taking the
    //largest loss if not equal, when the powerrank is not a perfect fit with the amount of food.
    diffFoodCount = FoodCount - FoodItemCount;
    modDiff = diffFoodCount % PowerRank;
    
    if (FoodItemCount % PowerRank == 0){
        singleClusterCount = FoodItemCount / PowerRank;
        otherClusterCount = singleClusterCount;
    }
    else {
        otherClusterCount = FoodItemCount / PowerRank + 1;
        singleClusterCount = otherClusterCount - modDiff;
    }
    //-----Wayne: End of PowerRank and food per PowerRank group
    
	for(size_t i = 0; i < PowerRank; i++) {
		powerLawClusters.push_back(powerLawLength * powerLawLength);
		powerLawLength *= 2;
	}

	for(size_t i = 0; i < PowerRank; i++) {
		powerLawLength /= 2;
		clusterSides.push_back(powerLawLength);
	}

    /*Wayne: Modified to break from loops if food count reached.
     Provides support for unequal clusters and odd food numbers.
     Necessary for DustUp and Jumble Distribution changes. */
	for(size_t h = 0; h < powerLawClusters.size(); h++) {
		for(size_t i = 0; i < powerLawClusters[h]; i++) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

			while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
				trialCount++;
				placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

				if(trialCount > maxTrials) {
					argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
					break;
				}
			}

            trialCount = 0;
			for(size_t j = 0; j < clusterSides[h]; j++) {
				for(size_t k = 0; k < clusterSides[h]; k++) {
					foodPlaced++;
					FoodList.push_back(placementPosition);
					FoodColoringList.push_back(argos::CColor::BLACK);
					placementPosition.SetX(placementPosition.GetX() + foodOffset);
                    if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
				}

				placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
				placementPosition.SetY(placementPosition.GetY() + foodOffset);
                if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
            if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
		}
	}

	FoodItemCount = foodPlaced;
}

bool Cluster_loop_functions::IsOutOfBounds(argos::CVector2 p, size_t length, size_t width) {
	argos::CVector2 placementPosition = p;

	argos::Real foodOffset   = 3.0 * FoodRadius;
	argos::Real widthOffset  = 3.0 * FoodRadius * (argos::Real)width;
	argos::Real lengthOffset = 3.0 * FoodRadius * (argos::Real)length;

	argos::Real x_min = p.GetX() - FoodRadius;
	argos::Real x_max = p.GetX() + FoodRadius + widthOffset;

	argos::Real y_min = p.GetY() - FoodRadius;
	argos::Real y_max = p.GetY() + FoodRadius + lengthOffset;

	if((x_min < (ForageRangeX.GetMin() + FoodRadius))
			|| (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
			(y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
			(y_max > (ForageRangeY.GetMax() - FoodRadius)))
	{
		return true;
	}

	for(size_t j = 0; j < length; j++) {
		for(size_t k = 0; k < width; k++) {
			if(IsCollidingWithFood(placementPosition)) return true;
			if(IsCollidingWithNest(placementPosition)) return true;
			placementPosition.SetX(placementPosition.GetX() + foodOffset);
		}

		placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
		placementPosition.SetY(placementPosition.GetY() + foodOffset);
	}

	return false;
}

bool Cluster_loop_functions::IsCollidingWithNest(argos::CVector2 p) {
	argos::Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
	argos::Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

	return ((p - NestPosition).SquareLength() < NRPB_squared);
}

bool Cluster_loop_functions::IsCollidingWithFood(argos::CVector2 p) {
	argos::Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
	argos::Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

	for(size_t i = 0; i < FoodList.size(); i++) {
		if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
	}

	return false;
}

unsigned int Cluster_loop_functions::getNumberOfRobots() {
	return GetSpace().GetEntitiesByType("foot-bot").size();
}

double Cluster_loop_functions::getProbabilityOfSwitchingToSearching() {
	return ProbabilityOfSwitchingToSearching;
}

double Cluster_loop_functions::getProbabilityOfReturningToNest() {
	return ProbabilityOfReturningToNest;
}

// Value in Radians
double Cluster_loop_functions::getUninformedSearchVariation() {
	return UninformedSearchVariation.GetValue();
}

double Cluster_loop_functions::getRateOfInformedSearchDecay() {
	return RateOfInformedSearchDecay;
}

double Cluster_loop_functions::getRateOfSiteFidelity() {
	return RateOfSiteFidelity;
}

double Cluster_loop_functions::getRateOfLayingPheromone() {
	return RateOfLayingPheromone;
}

double Cluster_loop_functions::getRateOfPheromoneDecay() {
	return RateOfPheromoneDecay;
}

argos::Real Cluster_loop_functions::getSimTimeInSeconds() {
	int ticks_per_second = GetSimulator().GetPhysicsEngine("Default").GetInverseSimulationClockTick();
	float sim_time = GetSpace().GetSimulationClock();
	return sim_time/ticks_per_second;
}

void Cluster_loop_functions::SetTrial(unsigned int v) {
}

void Cluster_loop_functions::setScore(double s) {
	score = s;
	if (score >= FoodItemCount) {
		PostExperiment();
	}
}

double Cluster_loop_functions::Score() {	
	return score;
}

void Cluster_loop_functions::ConfigureFromGenome(Real* g) {
	// Assign genome generated by the GA to the appropriate internal variables.
	ProbabilityOfSwitchingToSearching = g[0];
	ProbabilityOfReturningToNest      = g[1];
	UninformedSearchVariation.SetValue(g[2]);
	RateOfInformedSearchDecay         = g[3];
	RateOfSiteFidelity                = g[4];
	RateOfLayingPheromone             = g[5];
	RateOfPheromoneDecay              = g[6];
}

/*
 * Record how many sites a robot communicated to the nest.
 * Maintains running average via sum and count.
 */
void Cluster_loop_functions::RecordSitesCommunicated(size_t siteCount) {
	SitesCommunicatedSum += siteCount;
	SitesCommunicatedCount += 1;
}

/*****
 * Update visited location clusters using DBSCAN algorithm.
 *
 * Design intent: VisitedLocations grows monotonically as robots report visits.
 * DBSCAN requires minPts=2, so any two points within eps of each other form a
 * cluster.  eps is set to MaxClusterRadius, which also acts as the cluster size
 * cap: once a cluster's bounding radius (centroid → farthest member) reaches or
 * exceeds MaxClusterRadius it is "frozen" — its constituent points are excluded
 * from all future DBSCAN passes so the cluster stops growing.  Frozen clusters
 * remain in VisitedClusters (with isFrozen=true) for counting and rendering.
 * All clusters with ≥2 members have isMerged=true so the renderer draws them
 * as magenta circles.
 *****/
void Cluster_loop_functions::UpdateVisitedClusters() {
	if(VisitedLocations.empty()) return;

	const size_t  n         = VisitedLocations.size();
	const argos::Real eps   = .4;
	const argos::Real epsSq = eps * eps;
	const size_t  minPts    = 2;

	// -----------------------------------------------------------------------
	// 1. Mark points owned by frozen clusters so DBSCAN skips them.
	// -----------------------------------------------------------------------
	std::vector<bool> frozenPoint(n, false);
	for(size_t i = 0; i < n; ++i) {
		for(const auto& fc : FrozenClusters) {
			if((VisitedLocations[i] - fc.center).SquareLength() <= fc.radius * fc.radius + epsSq) {
				frozenPoint[i] = true;
				break;
			}
		}
	}

	// -----------------------------------------------------------------------
	// 2. DBSCAN on non-frozen points.
	// -----------------------------------------------------------------------
	const size_t maxClusterSize = 6;
	std::vector<int>  labels(n, -1);
	std::vector<bool> visited(n, false);
	int clusterID = 0;

	auto getNeighbors = [&](size_t idx) -> std::vector<size_t> {
		std::vector<size_t> nb;
		const argos::CVector2& p = VisitedLocations[idx];
		for(size_t i = 0; i < n; ++i) {
			if(i == idx || frozenPoint[i]) continue;
			if((p - VisitedLocations[i]).SquareLength() <= epsSq) nb.push_back(i);
		}
		return nb;
	};

	for(size_t i = 0; i < n; ++i) {
		if(visited[i] || frozenPoint[i]) continue;
		visited[i] = true;
		std::vector<size_t> neighbors = getNeighbors(i);
		if(neighbors.size() < minPts - 1) continue;

		labels[i] = clusterID;
		size_t clusterSize = 1;
		std::vector<size_t> seeds(neighbors.begin(), neighbors.end());
		for(size_t si = 0; si < seeds.size(); ++si) {
			if(clusterSize >= maxClusterSize) break;
			size_t q = seeds[si];
			if(frozenPoint[q]) continue;
			if(!visited[q]) {
				visited[q] = true;
				if(clusterSize + 1 < maxClusterSize) {
					std::vector<size_t> qnb = getNeighbors(q);
					if(qnb.size() >= minPts - 1)
						seeds.insert(seeds.end(), qnb.begin(), qnb.end());
				}
			}
			if(labels[q] == -1) { labels[q] = clusterID; ++clusterSize; }
		}
		++clusterID;
	}

	// -----------------------------------------------------------------------
	// 3. Build per-cluster point lists.
	// -----------------------------------------------------------------------
	std::vector<std::vector<size_t>> clusterPoints(clusterID);
	for(size_t i = 0; i < n; ++i)
		if(labels[i] >= 0) clusterPoints[labels[i]].push_back(i);

	// -----------------------------------------------------------------------
	// 4. Update ClusterMap from DBSCAN output.
	//    Each DBSCAN cluster finds its closest entry in ClusterMap within eps
	//    and updates it in-place (ID preserved). Unmatched DBSCAN clusters get
	//    a fresh entry with a new ID.
	// -----------------------------------------------------------------------
	for(int ci = 0; ci < clusterID; ++ci) {
		const std::vector<size_t>& pts = clusterPoints[ci];
		if(pts.empty()) continue;

		// Centroid
		argos::CVector2 centroid(0.0, 0.0);
		for(size_t idx : pts) centroid += VisitedLocations[idx];
		centroid /= static_cast<argos::Real>(pts.size());

		// Bounding radius
		argos::Real radius = 0.0;
		for(size_t idx : pts) {
			argos::Real d = (centroid - VisitedLocations[idx]).Length();
			if(d > radius) radius = d;
		}
		radius = std::max(radius, FoodRadius); // Minimum radius to cover a single point

		// Accumulate clustered locations (no duplicates)
		for(size_t idx : pts)
			if(clusteredLocationIndices.insert(idx).second)
				ClusteredVisitedLocations.push_back(VisitedLocations[idx]);

		// Find closest ClusterMap entry whose bounding circle contains
		// (or overlaps) this DBSCAN sub-cluster's centroid. Using
		// (existing.radius + eps) as the match radius handles large clusters whose
		// sub-cluster centroids may be well inside the stored circle but far from
		// its center.
		int         bestId     = -1;
		argos::Real bestDistSq = std::numeric_limits<argos::Real>::max();
		for(auto& [id, vc] : ClusterMap) {
			if(vc.isFrozen) continue;
			argos::Real dSq       = (centroid - vc.center).SquareLength();
			argos::Real threshold = vc.radius + eps;
			if(dSq > threshold * threshold) continue; // centroid outside reach
			if(dSq < bestDistSq) { bestDistSq = dSq; bestId = id; }
		}

		if(bestId != -1) {
			// Expand existing entry to contain the new sub-cluster — never shrink.
			// Because DBSCAN is capped at maxClusterSize, a large cluster may be
			// split across several DBSCAN output clusters each call.  Overwriting
			// with the sub-cluster's centroid/radius would shrink it.  Instead we
			// grow the stored circle to the minimum bounding circle that covers both
			// the old circle and the new sub-cluster: keep the old center, and set
			// the radius to max(old_radius, dist(old_center, new_centroid) + new_radius).
			VisitedCluster& existing = ClusterMap.at(bestId);
			argos::Real reach = (existing.center - centroid).Length() + radius;
			existing.radius     = std::max(existing.radius, reach);
			existing.visitCount += pts.size();
			existing.isMerged   = true;
			if(existing.radius >= MaxClusterRadius && !existing.isFrozen) {
				existing.isFrozen = true;
				FrozenClusters.push_back(existing);
			}
		} else {
			// New cluster — insert with a fresh ID
			VisitedCluster vc(centroid, radius);
			vc.visitCount = pts.size();
			vc.isMerged   = true;
			vc.clusterId  = nextClusterId++;
			if(vc.radius >= MaxClusterRadius) {
				vc.isFrozen = true;
				FrozenClusters.push_back(vc);
			}
			ClusterMap.emplace(vc.clusterId, vc);
		}
	}

	// -----------------------------------------------------------------------
	// 5. Containment cleanup on ClusterMap.
	//    If one cluster is fully inside another, remove the smaller one.
	// -----------------------------------------------------------------------
	{
		auto eraseFrozenById = [&](int id) {
			FrozenClusters.erase(
				std::remove_if(FrozenClusters.begin(), FrozenClusters.end(),
					[id](const VisitedCluster& vc) { return vc.clusterId == id; }),
				FrozenClusters.end());
		};

		bool changed = true;
		while(changed) {
			changed = false;
			std::vector<int> ids;
			ids.reserve(ClusterMap.size());
			for(const auto& kv : ClusterMap) ids.push_back(kv.first);

			for(size_t i = 0; i < ids.size() && !changed; ++i) {
				auto ita = ClusterMap.find(ids[i]);
				if(ita == ClusterMap.end()) continue;
				for(size_t j = i + 1; j < ids.size(); ++j) {
					auto itb = ClusterMap.find(ids[j]);
					if(itb == ClusterMap.end()) continue;

					VisitedCluster& ca = ita->second;
					VisitedCluster& cb = itb->second;
					argos::Real d = (ca.center - cb.center).Length();

					const argos::Real tol = 1e-6;
					bool aContainsB = (ca.radius + tol >= d + cb.radius);
					bool bContainsA = (cb.radius + tol >= d + ca.radius);
					if(!aContainsB && !bContainsA) continue;

					if(aContainsB && bContainsA) {
						// Near-identical circles: keep the one with larger visit count
						if(ca.visitCount >= cb.visitCount) bContainsA = false;
						else aContainsB = false;
					}

					if(aContainsB) {
						ca.visitCount += cb.visitCount;
						ca.isMerged = true;
						eraseFrozenById(cb.clusterId);
						ClusterMap.erase(cb.clusterId);
					} else {
						cb.visitCount += ca.visitCount;
						cb.isMerged = true;
						eraseFrozenById(ca.clusterId);
						ClusterMap.erase(ca.clusterId);
					}

					changed = true;
					break;
				}
			}
		}
	}

	// -----------------------------------------------------------------------
	// 5. Merge pass directly on ClusterMap.
	//    For each cluster, find the closest overlapping neighbor, update ca
	//    in-place, and erase the consumed cb.  Single greedy pass.
	// -----------------------------------------------------------------------
	{
		std::vector<int> ids;
		ids.reserve(ClusterMap.size());
		for(auto& [id, vc] : ClusterMap) ids.push_back(id);

		std::unordered_set<int> erased;
		for(int idA : ids) {
			if(erased.count(idA)) continue;
			auto ita = ClusterMap.find(idA);
			if(ita == ClusterMap.end()) continue;
			VisitedCluster& ca = ita->second;
			if(ca.isFrozen) continue;

			// Find closest overlapping neighbor
			int         bestId     = -1;
			argos::Real bestDistSq = std::numeric_limits<argos::Real>::max();
			for(int idB : ids) {
				if(idB == idA || erased.count(idB)) continue;
				auto itb = ClusterMap.find(idB);
				if(itb == ClusterMap.end()) continue;
				const VisitedCluster& cb = itb->second;
				if(cb.isFrozen) continue;
				argos::Real distSq    = (ca.center - cb.center).SquareLength();
				argos::Real distSqPos = (ca.radius + cb.radius) * (ca.radius + cb.radius);
				if(distSq > distSqPos) continue; // no overlap
				if(distSq < bestDistSq) { bestDistSq = distSq; bestId = idB; }
			}
			if(bestId == -1) continue;

			auto itBest = ClusterMap.find(bestId);
			if(itBest == ClusterMap.end()) continue;
			const VisitedCluster& cb = itBest->second;
			argos::Real d = std::sqrt(bestDistSq);
			argos::CVector2 newCenter;
			argos::Real     newRadius;

			if(ca.radius >= d + cb.radius) {
				// ca fully contains cb
				newCenter = ca.center;
				newRadius = ca.radius;
			} else if(cb.radius >= d + ca.radius) {
				// cb fully contains ca
				newCenter = cb.center;
				newRadius = cb.radius;
			} else {
				newCenter = (ca.center + cb.center) / 2.0;
				argos::Real ra = (newCenter - ca.center).Length() + ca.radius;
				argos::Real rb = (newCenter - cb.center).Length() + cb.radius;
				newRadius = std::max({ra, rb, FoodRadius});
			}

			if(newRadius > MaxClusterRadius * 1.2) continue; // would blow cap — skip

			// Update ca in-place; erase cb
			ca.center      = newCenter;
			ca.radius      = newRadius;
			ca.visitCount += cb.visitCount;
			ca.isMerged    = true;
			if(newRadius >= MaxClusterRadius && !ca.isFrozen) {
				ca.isFrozen = true;
				FrozenClusters.push_back(ca);
			}
			erased.insert(bestId);
			ClusterMap.erase(bestId);
		}
	}

	// -----------------------------------------------------------------------
	// 6. Rebuild VisitedClusters vector for rendering from ClusterMap (O(C)).
	// -----------------------------------------------------------------------
	VisitedClusters.clear();
	for(const auto& fc : FrozenClusters)
		VisitedClusters.push_back(fc);
	for(auto& [id, vc] : ClusterMap)
		if(!vc.isFrozen) VisitedClusters.push_back(vc);
}

/*****
 * Find a location in an underexplored area (low cluster count/coverage)
 * Returns a target location in the arena that has minimal cluster coverage
 *****/
argos::CVector2 Cluster_loop_functions::GetLowClusterSearchLocation() {
	// If no clusters exist yet, return a random location.
	if(VisitedClusters.empty()) {
		return argos::CVector2(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
	}

	// --- Build a spatial hash grid of cluster centers ---
	// Cell size == searchRadius guarantees every cluster within searchRadius of
	// a query point falls in at most the immediately adjacent cells (3×3 window).
	const argos::Real searchRadius    = MaxClusterRadius;
	const argos::Real searchRadiusSq  = searchRadius * searchRadius;
	const argos::Real cellSize        = searchRadius;

	// Same compact key as DBSCAN: gx * 100000 + gy.
	// Works for arenas up to ±500 m on each axis.
	auto gridKey = [&](argos::Real x, argos::Real y) -> long {
		long gx = static_cast<long>(std::floor(x / cellSize));
		long gy = static_cast<long>(std::floor(y / cellSize));
		return (gx * 100000L) + gy;
	};

	std::unordered_map<long, std::vector<size_t>> clusterGrid;
	clusterGrid.reserve(VisitedClusters.size() * 2);
	for(size_t ci = 0; ci < VisitedClusters.size(); ++ci) {
		const argos::CVector2& c = VisitedClusters[ci].center;
		clusterGrid[gridKey(c.GetX(), c.GetY())].push_back(ci);
	}

	// --- Enhanced scoring function considering cluster size, count, distance, and visit frequency ---
	auto calculateExplorationScore = [&](const argos::CVector2& pt) -> argos::Real {
		long gx = static_cast<long>(std::floor(pt.GetX() / cellSize));
		long gy = static_cast<long>(std::floor(pt.GetY() / cellSize));
		
		argos::Real minDistanceSq = std::numeric_limits<argos::Real>::max();
		argos::Real coverageScore = 0.0;
		int clusterCount = 0;
		argos::Real totalVisitWeight = 0.0;
		
		// Scan 3×3 cell window
		for(long dx = -1; dx <= 1; ++dx) {
			for(long dy = -1; dy <= 1; ++dy) {
				auto it = clusterGrid.find(((gx + dx) * 100000L) + (gy + dy));
				if(it == clusterGrid.end()) continue;
				
				for(size_t ci : it->second) {
					const VisitedCluster& cluster = VisitedClusters[ci];
					argos::CVector2 toCluster = pt - cluster.center;
					argos::Real distSq = toCluster.SquareLength();
					
					// Track minimum distance to any cluster
					if(distSq < minDistanceSq) {
						minDistanceSq = distSq;
					}
					
					// Only consider clusters within search radius
					if(distSq <= searchRadiusSq) {
						++clusterCount;
						
						argos::Real dist = std::sqrt(distSq);
						
						// Weighted coverage: larger clusters and those closer to the point
						// contribute more to making this location "already visited"
						// Normalized by MaxClusterRadius so values are comparable
						argos::Real radiusWeight = cluster.radius / MaxClusterRadius;
						argos::Real distanceWeight = 1.0 - (dist / searchRadius); // closer = higher weight
						
						coverageScore += radiusWeight * distanceWeight;
						
						// Visit frequency penalty: areas visited more often are less desirable
						// Log scale to prevent extreme values from dominating
						totalVisitWeight += std::log(1.0 + cluster.visitCount) * radiusWeight;
					}
				}
			}
		}
		
		// --- Compute final exploration score (higher = better for exploration) ---
		// We want to maximize distance from visited areas and minimize coverage
		
		argos::Real distanceScore = std::sqrt(minDistanceSq) / MaxClusterRadius;
		// Normalize: max possible distance in arena diagonal
		argos::Real maxDist = std::sqrt(ForageRangeX.GetSpan() * ForageRangeX.GetSpan() + 
		                                 ForageRangeY.GetSpan() * ForageRangeY.GetSpan());
		distanceScore = std::min(distanceScore, maxDist) / maxDist;
		
		// Invert coverage and visit weights (lower is better → higher score)
		argos::Real coveragePenalty = 1.0 / (1.0 + coverageScore);
		argos::Real visitPenalty = 1.0 / (1.0 + totalVisitWeight);
		argos::Real countPenalty = 1.0 / (1.0 + clusterCount);
		
		// Weighted combination: prioritize distance and low coverage
		// Weights can be tuned based on exploration strategy
		argos::Real explorationScore = 
			0.1 * distanceScore +      // Distance to nearest cluster
			0.2 * coveragePenalty +    // Spatial coverage by nearby clusters
			0.3 * countPenalty +       // Number of nearby clusters
			0.4 * visitPenalty;        // Visit frequency
		
		return explorationScore;
	};

	// --- Sample candidate points and pick the highest-scoring one ---
	const int numSamples = 20; // Increased samples for better coverage
	std::vector<argos::CVector2> samplePoints;
	std::vector<argos::Real>     explorationScores;
	samplePoints.reserve(numSamples);
	explorationScores.reserve(numSamples);

	for(int i = 0; i < numSamples; ++i) {
		argos::CVector2 pt(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		samplePoints.push_back(pt);
		explorationScores.push_back(calculateExplorationScore(pt));
	}

	// Find maximum score (best for exploration)
	argos::Real maxScore = *std::max_element(explorationScores.begin(), explorationScores.end());

	// Collect all points within 5% of the best score for randomization
	std::vector<argos::CVector2> bestPoints;
	argos::Real threshold = maxScore * 0.95;
	for(size_t i = 0; i < samplePoints.size(); ++i) {
		if(explorationScores[i] >= threshold)
			bestPoints.push_back(samplePoints[i]);
	}

	if(!bestPoints.empty()) {
		size_t idx = RNG->Uniform(argos::CRange<argos::UInt32>(0, bestPoints.size()));
		ProbabilityOfSearchingLowClusters += 0.001; // Increment probability for next time
		return bestPoints[idx];
	}

	// Fallback — should never be reached.
	return argos::CVector2(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
}

double Cluster_loop_functions::getProbabilityOfSearchingLowClusters() {
	return ProbabilityOfSearchingLowClusters;
}

REGISTER_LOOP_FUNCTIONS(Cluster_loop_functions, "Cluster_loop_functions")
