#include "Cluster_loop_functions.h"

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
	numSyntheticPoints(0)
{}

void Cluster_loop_functions::Init(argos::TConfigurationNode &node) {	
	argos::CDegrees USV_InDegrees;
	argos::TConfigurationNode Cluster_node = argos::GetNode(node, "Cluster");

	argos::GetNodeAttribute(Cluster_node, "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
	argos::GetNodeAttribute(Cluster_node, "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
	argos::GetNodeAttributeOrDefault(Cluster_node, "ProbabilityOfSearchingLowClusters", ProbabilityOfSearchingLowClusters, 0.0);
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
 * DBSCAN is rerun on the full set each update so that previously isolated
 * "noise" points can join clusters as new neighbours accumulate nearby.
 * No points are ever deleted from VisitedLocations, and no cluster-freezing
 * logic is applied — the natural DBSCAN output is used directly each cycle.
 *****/
void Cluster_loop_functions::UpdateVisitedClusters() {
	if(VisitedLocations.empty()) return;

	// Save previous clusters so new ones can inherit and grow their tracked radius.
	std::vector<VisitedCluster> prevClusters = VisitedClusters;
	VisitedClusters.clear();

	// Growth parameters: clusters start small and grow 20% per update cycle.
	const argos::Real initialClusterRadius = 0.3;
	const argos::Real growthFactor        = 1.2;

	// DBSCAN parameters
	// eps should be broad enough that a few robot visits in an area connect.
	// 0.5 m works well for typical robot spacing and visit density.
	const argos::Real eps = 0.5;
	const argos::Real epsSq = eps * eps;
	// minPts = 2: two nearby visits form a cluster, keeping sensitivity high
	// so that even sparsely visited areas generate clusters quickly.
	const size_t minPts = 3;

	// Data structures for DBSCAN
	size_t N = VisitedLocations.size();
	std::vector<bool> visited(N, false);
	std::vector<int> clusterId(N, -1); // -1 = noise, >=0 = cluster ID
	int C = -1; // Current cluster ID

	// Grid-based spatial hashing — cell size == eps ensures all neighbours
	// fall within at most the 3x3 surrounding cells.
	argos::Real cellSize = eps;
	std::unordered_map<long, std::vector<size_t>> grid;

	auto getGridKey = [&](const argos::CVector2& pos) -> long {
		long gx = static_cast<long>(std::floor(pos.GetX() / cellSize));
		long gy = static_cast<long>(std::floor(pos.GetY() / cellSize));
		return (gx * 100000L) + gy;
	};

	for(size_t i = 0; i < N; ++i) {
		grid[getGridKey(VisitedLocations[i])].push_back(i);
	}

	auto getNeighbors = [&](size_t pIdx) -> std::vector<size_t> {
		std::vector<size_t> neighbors;
		argos::CVector2 pPos = VisitedLocations[pIdx];
		long gx = static_cast<long>(std::floor(pPos.GetX() / cellSize));
		long gy = static_cast<long>(std::floor(pPos.GetY() / cellSize));

		for(long dx = -1; dx <= 1; ++dx) {
			for(long dy = -1; dy <= 1; ++dy) {
				long key = ((gx + dx) * 100000L) + (gy + dy);
				auto it = grid.find(key);
				if(it != grid.end()) {
					for(size_t nIdx : it->second) {
						if((VisitedLocations[nIdx] - pPos).SquareLength() <= epsSq) {
							neighbors.push_back(nIdx);
						}
					}
				}
			}
		}
		return neighbors;
	};

	// DBSCAN main loop
	for(size_t i = 0; i < N; ++i) {
		if(visited[i]) continue;

		visited[i] = true;
		std::vector<size_t> neighbors = getNeighbors(i);

		if(neighbors.size() < minPts) {
			// Noise point — leave as -1 but do NOT remove from VisitedLocations.
			// It will be reconsidered on the next update cycle when more nearby
			// visits may have accumulated.
			clusterId[i] = -1;
		} else {
			C++;
			clusterId[i] = C;

			size_t k = 0;
			while(k < neighbors.size()) {
				size_t nIdx = neighbors[k++];

				if(!visited[nIdx]) {
					visited[nIdx] = true;
					std::vector<size_t> nNeighbors = getNeighbors(nIdx);
					if(nNeighbors.size() >= minPts) {
						neighbors.insert(neighbors.end(), nNeighbors.begin(), nNeighbors.end());
					}
				}

				if(clusterId[nIdx] == -1) {
					clusterId[nIdx] = C;
				}
			}
		}
	}

	// Convert DBSCAN clusters to VisitedCluster objects
	if(C >= 0) {
		std::vector<std::vector<size_t>> pointsPerCluster(C + 1);
		for(size_t i = 0; i < N; ++i) {
			if(clusterId[i] >= 0) {
				pointsPerCluster[clusterId[i]].push_back(i);
			}
		}

		for(const auto& points : pointsPerCluster) {
			if(points.empty()) continue;

			// Centroid
			argos::CVector2 sum(0.0, 0.0);
			for(size_t idx : points) {
				sum += VisitedLocations[idx];
			}
			argos::CVector2 center = sum / static_cast<argos::Real>(points.size());

			// --- Radius assignment ---
			// Find ALL previous clusters whose spatial extent overlaps this new
			// DBSCAN cluster's centroid (within prevRadius + eps for chain slop).
			std::vector<size_t> matchedPrev;
			for(size_t pi = 0; pi < prevClusters.size(); ++pi) {
				if((center - prevClusters[pi].center).Length()
				        <= prevClusters[pi].radius + eps) {
					matchedPrev.push_back(pi);
				}
			}

			// A cluster grew if it absorbed at least one real robot-visit point
			// (index >= numSyntheticPoints) — i.e., a new visit landed inside it.
			bool hasNewVisits = false;
			for(size_t idx : points) {
				if(idx >= numSyntheticPoints) { hasNewVisits = true; break; }
			}

			// A cluster merged if the new DBSCAN group spans chain points that
			// originally belonged to more than one previous cluster.
			bool hasMerged = (matchedPrev.size() > 1);

			argos::Real grownRadius;
			if(matchedPrev.empty()) {
				// Brand-new cluster — start small.
				grownRadius = initialClusterRadius;
			} else {
				// Inherit the largest radius from all matched previous clusters.
				argos::Real prevRadius = 0.0;
				for(size_t pi : matchedPrev) {
					prevRadius = std::max(prevRadius, prevClusters[pi].radius);
				}
				if(prevRadius >= MaxClusterRadius) {
					// Already at max — stay static regardless of new data.
					grownRadius = MaxClusterRadius;
				} else if(hasNewVisits || hasMerged) {
					// New visit absorbed or two clusters merged — grow by 20%.
					grownRadius = std::min(prevRadius * growthFactor, MaxClusterRadius);
				} else {
					// No new data — hold radius exactly.
					grownRadius = prevRadius;
				}
			}

			VisitedCluster vc(center, grownRadius);
			vc.visitCount = points.size();
			vc.isMerged   = true;
			VisitedClusters.push_back(vc);
		}
	}

	// Compress VisitedLocations. Chain points go first so their indices are
	// [0, numSyntheticPoints), and real robot visits follow at indices
	// [numSyntheticPoints, end). This lets the next cycle distinguish them.
	{
		const argos::Real chainSpacing = eps;

		std::vector<argos::CVector2> compressed;
		compressed.reserve(VisitedClusters.size() * 20 + N);

		for(const auto& vc : VisitedClusters) {
			int steps = std::max(1, (int)std::ceil(vc.radius / chainSpacing));
			for(int di = -steps; di <= steps; ++di) {
				argos::CVector2 pt(vc.center.GetX() + di * chainSpacing,
				                   vc.center.GetY());
				compressed.push_back(pt);
			}
		}

		// Record boundary between synthetic and real points.
		numSyntheticPoints = compressed.size();

		// Preserve all noise points (real robot visits with no cluster yet).
		for(size_t i = 0; i < N; ++i) {
			if(clusterId[i] == -1) {
				compressed.push_back(VisitedLocations[i]);
			}
		}

		VisitedLocations = std::move(compressed);
	}

	LastProcessedLocationIndex = VisitedLocations.size();
}

/*****
 * Find a location in an underexplored area (low cluster count/coverage)
 * Returns a target location in the arena that has minimal cluster coverage
 *****/
argos::CVector2 Cluster_loop_functions::GetLowClusterSearchLocation() {
	// If no clusters exist yet, return a random location
	if(VisitedClusters.empty()) {
		argos::Real x = RNG->Uniform(ForageRangeX);
		argos::Real y = RNG->Uniform(ForageRangeY);
		return argos::CVector2(x, y);
	}

	// Sample 10 random points and find the one with fewest clusters within 1 meter radius
	const int numSamples = 10;
	const argos::Real searchRadius = MaxClusterRadius; // 0.5 meter radius
	const argos::Real searchRadiusSquared = searchRadius * searchRadius;
	
	std::vector<argos::CVector2> samplePoints;
	std::vector<int> clusterCounts;
	
	// Generate 10 random sample points
	for(int i = 0; i < numSamples; ++i) {
		argos::Real x = RNG->Uniform(ForageRangeX);
		argos::Real y = RNG->Uniform(ForageRangeY);
		argos::CVector2 samplePoint(x, y);
		samplePoints.push_back(samplePoint);
		
		// Count clusters within 1 meter radius of this point
		int clusterCount = 0;
		for(const auto& cluster : VisitedClusters) {
			argos::Real distSquared = (samplePoint - cluster.center).SquareLength();
			if(distSquared <= searchRadiusSquared) {
				clusterCount++;
			}
		}
		clusterCounts.push_back(clusterCount);
	}
	
	// Find the minimum cluster count
	int minClusterCount = *std::min_element(clusterCounts.begin(), clusterCounts.end());
	
	// Collect all points with the minimum cluster count
	std::vector<argos::CVector2> bestPoints;
	for(size_t i = 0; i < samplePoints.size(); ++i) {
		if(clusterCounts[i] == minClusterCount) {
			bestPoints.push_back(samplePoints[i]);
		}
	}
	
	// Return a random point from the best points (lowest cluster count) (in case of a tie)
	if(!bestPoints.empty()) {
		size_t randomIndex = RNG->Uniform(argos::CRange<argos::UInt32>(0, bestPoints.size()));
		return bestPoints[randomIndex];
	}
	
	// Fallback to random location (should not reach here)
	argos::Real x = RNG->Uniform(ForageRangeX);
	argos::Real y = RNG->Uniform(ForageRangeY);
	return argos::CVector2(x, y);
}

double Cluster_loop_functions::getProbabilityOfSearchingLowClusters() {
	return ProbabilityOfSearchingLowClusters;
}

REGISTER_LOOP_FUNCTIONS(Cluster_loop_functions, "Cluster_loop_functions")
