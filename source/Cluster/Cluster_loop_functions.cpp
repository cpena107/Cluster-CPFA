#include "Cluster_loop_functions.h"
#include <algorithm>
#include <cmath>

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
	ProbabilityOfReturningToNest(0.05),
	ProbabilityOfSearchingLowClusters(0.0),
	InitialProbabilityOfSearchingLowClusters(0.0),
	UninformedSearchVariation(0.0),
	RateOfInformedSearchDecay(0.0),
	RateOfSiteFidelity(0.0),
	RateOfLayingPheromone(0.0),
	RateOfLowClusterSearch(0.0),
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
	LowClusterReturnTimeoutSeconds(60.0),
	LowClusterPriorityWarmupSeconds(120.0),
	numSyntheticPoints(0),
	nextClusterId(0),
	percentCollected(0.1),
	timeIntervalForRecording(0.0),
	VisitedLocationTolerance(0.5)
{}

void Cluster_loop_functions::Init(argos::TConfigurationNode &node) {	
	argos::CDegrees USV_InDegrees;
	argos::TConfigurationNode Cluster_node = argos::GetNode(node, "Cluster");

	argos::GetNodeAttribute(Cluster_node, "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
	argos::GetNodeAttribute(Cluster_node, "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
	argos::GetNodeAttributeOrDefault(Cluster_node, "ProbabilityOfSearchingLowClusters", ProbabilityOfSearchingLowClusters, 0.3);
	InitialProbabilityOfSearchingLowClusters = ProbabilityOfSearchingLowClusters;
	argos::GetNodeAttribute(Cluster_node, "UninformedSearchVariation",         USV_InDegrees);
	argos::GetNodeAttribute(Cluster_node, "RateOfInformedSearchDecay",         RateOfInformedSearchDecay);
	argos::GetNodeAttribute(Cluster_node, "RateOfSiteFidelity",                RateOfSiteFidelity);
	argos::GetNodeAttribute(Cluster_node, "RateOfLayingPheromone",             RateOfLayingPheromone);
	argos::GetNodeAttributeOrDefault(Cluster_node, "RateOfLowClusterSearch",   RateOfLowClusterSearch, ProbabilityOfSearchingLowClusters);
	argos::GetNodeAttribute(Cluster_node, "RateOfPheromoneDecay",              RateOfPheromoneDecay);
	argos::GetNodeAttribute(Cluster_node, "PrintFinalScore",                   PrintFinalScore);
	argos::GetNodeAttributeOrDefault(Cluster_node, "percentCollected",                  percentCollected, 0.1);
	argos::GetNodeAttributeOrDefault(Cluster_node, "TimeIntervalForRecording",         timeIntervalForRecording, 0.0);
	argos::GetNodeAttributeOrDefault(Cluster_node, "LowClusterReturnTimeoutSeconds",   LowClusterReturnTimeoutSeconds, LowClusterReturnTimeoutSeconds);
	argos::GetNodeAttributeOrDefault(Cluster_node, "LowClusterPriorityWarmupSeconds",  LowClusterPriorityWarmupSeconds, LowClusterPriorityWarmupSeconds);

	argos::GetNodeAttributeOrDefault(Cluster_node, "VisitedLocationTolerance", VisitedLocationTolerance, 0.5);

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
	percentCollected = 0.1;
	timeIntervalForRecording = 0.0;

	FoodList.clear();
	FoodColoringList.clear();
	PheromoneList.clear();
	FidelityList.clear();
	LowClusterTargetList.clear();
	LowClusterMissions.clear();
	//TargetRayList.clear();
	//TargetRayColorList.clear();
    RobotTrails.clear();
    SearchLocationRays.clear();
    RobotTrailColors.clear();
	VisitedClusters.clear();
    FrozenClusters.clear();
	VisitedLocations.clear();
	ExistingVisitedLocations.clear();
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
	ProcessLowClusterMissionTimeouts();

	// Update visited location clusters periodically (every 5 seconds)
	// This ensures clusters are created even if robots aren't returning with food
	size_t ticksPerUpdate = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick() * 5;
	if(GetSpace().GetSimulationClock() % ticksPerUpdate == 0 && !VisitedLocations.empty()) {
		UpdateVisitedClusters();
		if (VisitedClusters.size() > MaxClusterCount) {
			MaxClusterCount = VisitedClusters.size();
		}
	}
	// Record the current time when 10%, 20%, ..., 100% of the food has been collected
	size_t foodCollected = FoodItemCount - FoodList.size();
	if(foodCollected >= percentCollected * FoodItemCount) {
		double currentTime = getSimTimeInSeconds();
		double timeInSeconds = currentTime - timeIntervalForRecording; // Time since last recording
		// random_seed,milestone_percent,time_interval,cumulative_time,food_distribution,algorithm_mode,num_robots,total_food
		printf("%lu, %f, %f, %f, %ld, %d, %lu, %lu\n", RandomSeed, percentCollected*100, timeInSeconds, currentTime, FoodDistribution, 0, Num_robots, foodCollected);
		timeIntervalForRecording = currentTime;
		percentCollected += 0.1;
	}
}

void Cluster_loop_functions::RegisterLowClusterMission(const std::string& robotId,
		const argos::CVector2& targetLocation,
		size_t dispatchTick) {
	size_t ticksPerSecond = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();
	size_t timeoutTicks = static_cast<size_t>(std::max<argos::Real>(1.0, LowClusterReturnTimeoutSeconds * ticksPerSecond));

	LowClusterMission mission;
	mission.targetLocation = targetLocation;
	mission.dispatchTick = dispatchTick;
	mission.timeoutTick = dispatchTick + timeoutTicks;
	LowClusterMissions[robotId] = mission;
}

void Cluster_loop_functions::ResolveLowClusterMission(const std::string& robotId, int missionOutcome) {
	auto it = LowClusterMissions.find(robotId);
	if(it == LowClusterMissions.end()) {
		LowClusterTargetList.erase(robotId);
		return;
	}

	if(missionOutcome == 0) {
		AddMaxRadiusClusterAt(it->second.targetLocation);
	}

	LowClusterMissions.erase(it);
	LowClusterTargetList.erase(robotId);
}

void Cluster_loop_functions::ProcessLowClusterMissionTimeouts() {
	if(LowClusterMissions.empty()) {
		return;
	}

	const size_t currentTick = GetSpace().GetSimulationClock();
	for(auto it = LowClusterMissions.begin(); it != LowClusterMissions.end();) {
		if(currentTick >= it->second.timeoutTick) {
			LowClusterTargetList.erase(it->first);
			AddMaxRadiusClusterAt(it->second.targetLocation);
			it = LowClusterMissions.erase(it);
		} else {
			++it;
		}
	}
}

void Cluster_loop_functions::AddMaxRadiusClusterAt(const argos::CVector2& targetLocation) {
	for(const auto& cluster : VisitedClusters) {
		if((targetLocation - cluster.center).SquareLength() <= (cluster.radius * cluster.radius)) {
			return;
		}
	}

	VisitedCluster forcedCluster(targetLocation, MaxClusterRadius);
	forcedCluster.visitCount = 1;
	forcedCluster.clusterId = nextClusterId++;
	VisitedClusters.push_back(forcedCluster);

	if(VisitedClusters.size() > MaxClusterCount) {
		MaxClusterCount = VisitedClusters.size();
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

double Cluster_loop_functions::getRateOfLowClusterSearch() {
	return RateOfLowClusterSearch;
}

double Cluster_loop_functions::getRateOfPheromoneDecay() {
	return RateOfPheromoneDecay;
}

argos::Real Cluster_loop_functions::getLowClusterPriorityWeight() {
	if(LowClusterPriorityWarmupSeconds <= 0.0) {
		return 1.0;
	}

	const argos::Real ticksPerSecond = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();
	const argos::Real elapsedSeconds = static_cast<argos::Real>(GetSpace().GetSimulationClock()) / ticksPerSecond;
	const argos::Real weight = elapsedSeconds / LowClusterPriorityWarmupSeconds;
	return std::max<argos::Real>(0.0, std::min<argos::Real>(1.0, weight));
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
 * Memoized clustering pipeline:
 * 1) Build clusters from *new* VisitedLocations with DBSCAN.
 * 2) Try to merge only new items (new singleton points and new clusters)
 *    into existing memoized state (ExistingVisitedLocations + VisitedClusters).
 * 3) Run one consolidation pass on existing memoized state:
 *    - move existing singleton points into clusters when they fall in/near one
 *    - merge overlapping clusters
 * 4) Clear new buffers (VisitedLocations + temporary new clusters).
 *****/
void Cluster_loop_functions::UpdateVisitedClusters() {
	if(VisitedLocations.empty()) {
		return;
	}

	// Use full configured cluster radius as DBSCAN neighborhood distance.
	// Using MaxClusterRadius/10 was too restrictive and prevented clusters
	// from forming in typical runs, which made VisitedClusters appear empty.
	const argos::Real eps = VisitedLocationTolerance + 0.001; // Add a small epsilon to prevent numerical issues with points that are very close to the radius boundary
	const argos::Real epsSq = eps * eps;
	const argos::Real growthSlack = eps * 0.1; // Allow clusters to grow slightly beyond the strict radius when merging in new points/clusters, to prevent excessive fragmentation. This is necessary because the cluster radius can only grow when merging in new points/clusters, not shrink, so if a cluster grows too large due to an outlier point, it can never be repaired and will just keep absorbing nearby points/clusters.
 	const argos::Real maxRadiusTolerance = 1e-2;

	auto IsMaxSizedCluster = [&](const VisitedCluster& cluster) {
		return cluster.radius >= (MaxClusterRadius - maxRadiusTolerance);
	};

	auto DistanceSq = [](const argos::CVector2& a, const argos::CVector2& b) {
		return (a - b).SquareLength();
	};

	auto ComputeCentroid = [](const std::vector<argos::CVector2>& pts) {
		argos::CVector2 centroid(0.0f, 0.0f);
		if(pts.empty()) return centroid;
		for(const auto& p : pts) {
			centroid += p;
		}
		centroid /= static_cast<argos::Real>(pts.size());
		return centroid;
	};

	auto ComputeRadius = [&](const std::vector<argos::CVector2>& pts, const argos::CVector2& center) {
		argos::Real radius = 0.0;
		for(const auto& p : pts) {
			const argos::Real dist = (p - center).Length();
			if(dist > radius) {
				radius = dist;
			}
		}
		return radius;
	};

	auto UpdateClusterFromMembers = [&](VisitedCluster& cluster, const std::vector<argos::CVector2>& members) {
		if(members.empty()) return;
		const argos::Real previousRadius = cluster.radius;
		cluster.center = ComputeCentroid(members);
		const argos::Real recomputedRadius = ComputeRadius(members, cluster.center);
		cluster.radius = std::max(previousRadius, recomputedRadius);
	};

	auto MergeIntoCluster = [&](VisitedCluster& cluster,
	                           const argos::CVector2& incomingCenter,
	                           argos::Real incomingRadius,
	                           size_t incomingCount) {
		if(incomingCount == 0) return;

		const argos::CVector2 oldCenter = cluster.center;
		const argos::Real oldRadius = cluster.radius;
		const size_t oldCount = std::max<size_t>(1, cluster.visitCount);
		const size_t totalCount = oldCount + incomingCount;

		const argos::Real newX = (oldCenter.GetX() * oldCount + incomingCenter.GetX() * incomingCount) /
		                         static_cast<argos::Real>(totalCount);
		const argos::Real newY = (oldCenter.GetY() * oldCount + incomingCenter.GetY() * incomingCount) /
		                         static_cast<argos::Real>(totalCount);
		cluster.center.Set(newX, newY);

		const argos::Real oldEnvelope = (oldCenter - cluster.center).Length() + oldRadius;
		const argos::Real incomingEnvelope = (incomingCenter - cluster.center).Length() + incomingRadius;
		const argos::Real mergedRadius = std::max(oldEnvelope, incomingEnvelope);
		cluster.radius = std::max(oldRadius, mergedRadius);
		cluster.visitCount += incomingCount;
	};

	auto IsCenterInsideCluster = [&](const argos::CVector2& center, const VisitedCluster& cluster, argos::Real buffer) {
		const argos::Real r = cluster.radius + buffer;
		return DistanceSq(center, cluster.center) <= r * r;
	};

	auto IsClusterFullyInside = [&](const VisitedCluster& inner, const VisitedCluster& outer, argos::Real buffer) {
		const argos::Real centerDistance = (inner.center - outer.center).Length();
		return (centerDistance + inner.radius) <= (outer.radius + buffer);
	};

	auto AreClustersOverlapping = [&](const VisitedCluster& c1, const VisitedCluster& c2, argos::Real buffer) {
		const argos::Real centerDistance = (c1.center - c2.center).Length();
		return centerDistance <= (c1.radius + c2.radius + buffer);
	};

	struct TempCluster {
		argos::CVector2 center;
		argos::Real radius;
		std::vector<argos::CVector2> members;
		bool merged;
	};

	// -------------------------
	// Step 0: increment count of clusters with new visited locations inside its radius, and remove those locations from the "new" pool since they are now accounted for in the memoized state
	// -------------------------
	for(size_t i = 0; i < VisitedLocations.size();) {
		const argos::CVector2& point = VisitedLocations[i];
		bool merged = false;

		for(auto& cluster : VisitedClusters) {
			const argos::Real acceptR = IsMaxSizedCluster(cluster)
				? cluster.radius
				: (cluster.radius + growthSlack);
			if(DistanceSq(point, cluster.center) <= acceptR * acceptR) {
				if(IsMaxSizedCluster(cluster)) {
					cluster.visitCount += 1;
				} else {
					MergeIntoCluster(cluster, point, 0.0, 1);
				}
				ClusteredVisitedLocations.push_back(point);
				merged = true;
				break;
			}
		}

		if(merged) {
			VisitedLocations.erase(VisitedLocations.begin() + i);
		}
		else {
			i++;
		}
	}
	
	for(size_t i = 0; i < ExistingVisitedLocations.size();) {
		const argos::CVector2& point = ExistingVisitedLocations[i];
		bool merged = false;

		for(auto& cluster : VisitedClusters) {
			const argos::Real acceptR = IsMaxSizedCluster(cluster)
				? cluster.radius
				: (cluster.radius + growthSlack);
			if(DistanceSq(point, cluster.center) <= acceptR * acceptR) {
				if(IsMaxSizedCluster(cluster)) {
					cluster.visitCount += 1;
				} else {
					MergeIntoCluster(cluster, point, 0.0, 1);
				}
				ClusteredVisitedLocations.push_back(point);
				merged = true;
				break;
			}
		}

		if(merged) {
			ExistingVisitedLocations.erase(ExistingVisitedLocations.begin() + i);
		}
		else {
			i++;
		}
	}
	
	// ------------------------
	// Step 1: DBSCAN on new visited locations only
	// ------------------------
	std::vector<bool> pointVisited(VisitedLocations.size(), false);
	std::vector<bool> pointAssigned(VisitedLocations.size(), false);
	std::vector<bool> isNoise(VisitedLocations.size(), false);
	std::vector<TempCluster> newClusterLocations;
	const size_t minPts = 2;

	auto RegionQuery = [&](size_t idx) {
		std::vector<size_t> neighbors;
		neighbors.reserve(VisitedLocations.size());
		for(size_t j = 0; j < VisitedLocations.size(); ++j) {
			if(DistanceSq(VisitedLocations[idx], VisitedLocations[j]) <= epsSq) {
				neighbors.push_back(j);
				if(neighbors.size() >= 3) {
					break;
				}
			}
		}
		return neighbors;
	};

	auto RegionQueryExisting= [&](const argos::CVector2& point) {
		std::vector<size_t> neighbors;
		neighbors.reserve(ExistingVisitedLocations.size());
		for(size_t j = 0; j < ExistingVisitedLocations.size(); ++j) {
			if(DistanceSq(point, ExistingVisitedLocations[j]) <= epsSq) {
				neighbors.push_back(j);
				if(neighbors.size() >= 3) {
					break;
				}
			}
		}
		return neighbors;
	};

	for(size_t i = 0; i < VisitedLocations.size(); ++i) {
		if(pointVisited[i]) continue;
		pointVisited[i] = true;

		std::vector<size_t> neighbors = RegionQuery(i);
		if(neighbors.size() < minPts) {
			isNoise[i] = true;
			continue;
		}

		std::vector<size_t> seeds = neighbors;
		pointAssigned[i] = true;

		for(size_t s = 0; s < seeds.size(); ++s) {
			size_t idx = seeds[s];
			if(!pointVisited[idx]) {
				pointVisited[idx] = true;
				std::vector<size_t> n2 = RegionQuery(idx);
				if(n2.size() >= minPts) {
					seeds.insert(seeds.end(), n2.begin(), n2.end());
				}
			}
			pointAssigned[idx] = true;
		}

		std::sort(seeds.begin(), seeds.end());
		seeds.erase(std::unique(seeds.begin(), seeds.end()), seeds.end());

		std::vector<argos::CVector2> members;
		members.reserve(seeds.size());
		for(size_t idx : seeds) {
			members.push_back(VisitedLocations[idx]);
		}

		TempCluster cluster;
		cluster.members = members;
		cluster.center = ComputeCentroid(cluster.members);
		cluster.radius = ComputeRadius(cluster.members, cluster.center);
		cluster.merged = false;
		newClusterLocations.push_back(cluster);
	}

	// ------------------------
	// Step 2a: merge new singleton points into existing memoized state
	// ------------------------
	for(size_t i = 0; i < VisitedLocations.size(); ++i) {
		if(pointAssigned[i]) {
			ClusteredVisitedLocations.push_back(VisitedLocations[i]);
			continue;
		}

		const argos::CVector2& point = VisitedLocations[i];
		bool merged = false;

		for(auto& cluster : VisitedClusters) {
			const argos::Real acceptR = IsMaxSizedCluster(cluster)
				? cluster.radius
				: (cluster.radius + growthSlack);
			if(DistanceSq(point, cluster.center) <= acceptR * acceptR) {
				if(IsMaxSizedCluster(cluster)) {
					cluster.visitCount += 1;
				} else {
					MergeIntoCluster(cluster, point, 0.0, 1);
				}
				ClusteredVisitedLocations.push_back(point);
				merged = true;
				break;
			}
		}

		if(!merged) {
			for(size_t j = 0; j < ExistingVisitedLocations.size(); ++j)
				if(DistanceSq(point, ExistingVisitedLocations[j]) <= epsSq) {
					VisitedCluster newExistingCluster((point + ExistingVisitedLocations[j]) / 2.0, 0.0);
					newExistingCluster.clusterId = nextClusterId++;
					std::vector<argos::CVector2> members;
					members.push_back(point);
					members.push_back(ExistingVisitedLocations[j]);
					UpdateClusterFromMembers(newExistingCluster, members);
					newExistingCluster.visitCount = 2;
					VisitedClusters.push_back(newExistingCluster);
					ClusteredVisitedLocations.push_back(point);
					ClusteredVisitedLocations.push_back(ExistingVisitedLocations[j]);
					ExistingVisitedLocations.erase(ExistingVisitedLocations.begin() + j);
					merged = true;
					break;
				}
		}

		if(!merged) {
			ExistingVisitedLocations.push_back(point);
		}
	}

	// ------------------------
	// Step 2b: merge new clusters into existing memoized state
	// ------------------------
	
	for(auto& newCluster : newClusterLocations) {
		bool mergedToExistingCluster = false;

		for(auto& existingCluster : VisitedClusters) {
			if(IsMaxSizedCluster(existingCluster)) {
				VisitedCluster incoming(newCluster.center, newCluster.radius);
				if(IsClusterFullyInside(incoming, existingCluster, 0.0)) {
					existingCluster.visitCount += newCluster.members.size();
					mergedToExistingCluster = true;
					break;
				}
				continue;
			}
			const bool centerInsideExisting = IsCenterInsideCluster(newCluster.center, existingCluster, growthSlack * 0.25);
			const bool existingInsideNew = DistanceSq(existingCluster.center, newCluster.center) <=
				                          (newCluster.radius + growthSlack * 0.25) * (newCluster.radius + growthSlack * 0.25);
			if(centerInsideExisting || existingInsideNew) {
				MergeIntoCluster(existingCluster, newCluster.center, newCluster.radius, newCluster.members.size());
				mergedToExistingCluster = true;
				break;
			}
		}

		if(mergedToExistingCluster) {
			continue;
		}

		bool absorbedAnyExistingPoint = false;
		for(size_t i = 0; i < ExistingVisitedLocations.size();) {
			argos::Real acceptR = newCluster.radius + growthSlack;
			if(DistanceSq(ExistingVisitedLocations[i], newCluster.center) <= acceptR * acceptR) {
				newCluster.members.push_back(ExistingVisitedLocations[i]);
				ClusteredVisitedLocations.push_back(ExistingVisitedLocations[i]);
				ExistingVisitedLocations.erase(ExistingVisitedLocations.begin() + i);
				absorbedAnyExistingPoint = true;
			} else {
				++i;
			}
		}

		if(absorbedAnyExistingPoint) {
			newCluster.center = ComputeCentroid(newCluster.members);
			newCluster.radius = ComputeRadius(newCluster.members, newCluster.center);
		}

		VisitedCluster persistentCluster(newCluster.center, newCluster.radius);
		persistentCluster.clusterId = nextClusterId++;
		persistentCluster.visitCount = newCluster.members.size();
		VisitedClusters.push_back(persistentCluster);
	}
	
	// ------------------------
	// Step 3a: move existing singleton points that now fall into a cluster
	// ------------------------
	for(size_t i = 0; i < ExistingVisitedLocations.size();) {
		bool absorbed = false;
		for(auto& cluster : VisitedClusters) {
			argos::Real acceptR = IsMaxSizedCluster(cluster)
				? cluster.radius
				: (cluster.radius + growthSlack);
			if(DistanceSq(ExistingVisitedLocations[i], cluster.center) <= acceptR * acceptR) {
				if(IsMaxSizedCluster(cluster)) {
					cluster.visitCount += 1;
				} else {
					MergeIntoCluster(cluster, ExistingVisitedLocations[i], 0.0, 1);
				}
				ClusteredVisitedLocations.push_back(ExistingVisitedLocations[i]);
				ExistingVisitedLocations.erase(ExistingVisitedLocations.begin() + i);
				absorbed = true;
				break;
			}
		}

		if(!absorbed) {
			++i;
		}
	}

	// ------------------------
	// Step 3b: merge overlapping existing clusters
	// ------------------------
	for(size_t i = 0; i < VisitedClusters.size();) {
		bool erasedI = false;
		if(IsMaxSizedCluster(VisitedClusters[i])) {
			for(size_t j = i + 1; j < VisitedClusters.size();) {
				const argos::Real coverageBuffer = growthSlack * 0.1;
				const bool jFullyInsideI = IsClusterFullyInside(VisitedClusters[j], VisitedClusters[i], coverageBuffer);
				if(jFullyInsideI) {
					VisitedClusters[i].visitCount += VisitedClusters[j].visitCount;
					VisitedClusters.erase(VisitedClusters.begin() + j);
					continue;
				}
				++j;
			}
			++i;
			continue;
		}
		for(size_t j = i + 1; j < VisitedClusters.size();) {
			const argos::Real coverageBuffer = growthSlack * 0.1;
			const bool jFullyInsideI = IsClusterFullyInside(VisitedClusters[j], VisitedClusters[i], coverageBuffer);
			const bool iFullyInsideJ = IsClusterFullyInside(VisitedClusters[i], VisitedClusters[j], coverageBuffer);
			if(IsMaxSizedCluster(VisitedClusters[j]) && !jFullyInsideI) {
				j++;
				continue;
			}
			if(IsMaxSizedCluster(VisitedClusters[i]) && !iFullyInsideJ) {
				break;
			}
			const bool clustersOverlapping = AreClustersOverlapping(VisitedClusters[i], VisitedClusters[j], coverageBuffer);

			if(jFullyInsideI) {
				MergeIntoCluster(VisitedClusters[i], VisitedClusters[j].center, VisitedClusters[j].radius, VisitedClusters[j].visitCount);
				VisitedClusters.erase(VisitedClusters.begin() + j);
				continue;
			}

			if(iFullyInsideJ) {
				if(IsMaxSizedCluster(VisitedClusters[j])) {
					VisitedClusters[j].visitCount += VisitedClusters[i].visitCount;
				} else {
					MergeIntoCluster(VisitedClusters[j], VisitedClusters[i].center, VisitedClusters[i].radius, VisitedClusters[i].visitCount);
				}
				VisitedClusters.erase(VisitedClusters.begin() + i);
				erasedI = true;
				break;
			}

			if(clustersOverlapping) {
				MergeIntoCluster(VisitedClusters[i], VisitedClusters[j].center, VisitedClusters[j].radius, VisitedClusters[j].visitCount);
				VisitedClusters.erase(VisitedClusters.begin() + j);
				continue;
			}

			++j;
		}

		if(!erasedI) {
			++i;
		}
	}

	if(VisitedClusters.size() > MaxClusterCount) {
		MaxClusterCount = VisitedClusters.size();
	}

	// Step 3c: compare clusters with max radius and merge them if they are inside each other
	for(size_t i = 0; i < VisitedClusters.size();) {
		bool erasedI = false;
		if(VisitedClusters[i].radius < MaxClusterRadius - maxRadiusTolerance) {
			++i;
			continue;
		}
		for(size_t j = i + 1; j < VisitedClusters.size();) {
			if(VisitedClusters[j].radius < MaxClusterRadius - maxRadiusTolerance) {
				j++;
				continue;
			}
			const argos::Real coverageBuffer = growthSlack * 0.1;
			const bool jFullyInsideI = IsClusterFullyInside(VisitedClusters[j], VisitedClusters[i], coverageBuffer);
			const bool iFullyInsideJ = IsClusterFullyInside(VisitedClusters[i], VisitedClusters[j], coverageBuffer);

			if(jFullyInsideI) {
				MergeIntoCluster(VisitedClusters[i], VisitedClusters[j].center, VisitedClusters[j].radius, VisitedClusters[j].visitCount);
				VisitedClusters.erase(VisitedClusters.begin() + j);
				continue;
			}

			if(iFullyInsideJ) {
				MergeIntoCluster(VisitedClusters[j], VisitedClusters[i].center, VisitedClusters[i].radius, VisitedClusters[i].visitCount);
				VisitedClusters.erase(VisitedClusters.begin() + i);
				erasedI = true;
				break;
			}

			j++;
		}

		if(!erasedI) {
			i++;
		}
	}

	// Step 4: clear new temporary state
	VisitedLocations.clear();
	newClusterLocations.clear();
}

/*****
 * Find a location in an underexplored area (low cluster count/coverage)
 * Returns a target location in the arena that has minimal cluster coverage
 *****/
argos::CVector2 Cluster_loop_functions::GetLowClusterSearchLocation() {
	const size_t maxSamples = 1000;

	for(size_t sample = 0; sample < maxSamples; ++sample) {
		argos::CVector2 candidate(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		bool isInCluster = false;

		for(const auto& cluster : VisitedClusters) {
			const argos::Real radiusSq = cluster.radius * cluster.radius;
			if((candidate - cluster.center).SquareLength() <= radiusSq) {
				isInCluster = true;
				break;
			}
		}

		if(!isInCluster) {
			return candidate;
		}
		// If we failed to find a location, remove the cluster with the largest radius to visited count ratio, and reattempt. This is a fallback to prevent the algorithm from getting stuck if the cluster state becomes too fragmented and prevents finding any low-cluster areas, which can happen when the cluster radius is small and growth is disabled, causing many small clusters to form and fragment the space.
		if(sample == maxSamples-1){
			cout << "Warning: Failed to find low cluster search location after " << maxSamples << " samples. Removing worst cluster and retrying.\n";
			auto worstClusterIt = std::max_element(VisitedClusters.begin(), VisitedClusters.end(),
				[](const VisitedCluster& a, const VisitedCluster& b) {
					const argos::Real ratioA = a.visitCount > 0 ? (a.radius / static_cast<argos::Real>(a.visitCount)) : std::numeric_limits<argos::Real>::max();
					const argos::Real ratioB = b.visitCount > 0 ? (b.radius / static_cast<argos::Real>(b.visitCount)) : std::numeric_limits<argos::Real>::max();
					return ratioA < ratioB;
				});
			if(worstClusterIt != VisitedClusters.end()) {
				VisitedClusters.erase(worstClusterIt);
				cout << "Removed cluster with center: (" << worstClusterIt->center.GetX() << ", " << worstClusterIt->center.GetY() << "), radius: " << worstClusterIt->radius << ", visitCount: " << worstClusterIt->visitCount << "\n";
				cout << "Remaining cluster count: " << VisitedClusters.size() << "\n";
			}
			sample = 0;
		}
	}
	
	cout << "Warning: Failed to find low cluster search location after " << maxSamples << " samples. Returning random location.\n";
	return argos::CVector2(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
}

double Cluster_loop_functions::getProbabilityOfSearchingLowClusters() {
	const argos::Real forageWidth = ForageRangeX.GetMax() - ForageRangeX.GetMin();
	const argos::Real forageHeight = ForageRangeY.GetMax() - ForageRangeY.GetMin();
	const argos::Real arenaArea = std::max<argos::Real>(1e-6, forageWidth * forageHeight);

	size_t globalVisits = 0; //VisitedLocations.size() + ExistingVisitedLocations.size();
	for(const auto& cluster : VisitedClusters) {
		globalVisits += std::max<size_t>(cluster.visitCount, 1);
	}

	const argos::Real visitProgress = std::min<argos::Real>(
		1.0,
		static_cast<argos::Real>(globalVisits) / arenaArea
	);

	if(VisitedClusters.empty()) {
		ProbabilityOfSearchingLowClusters = InitialProbabilityOfSearchingLowClusters +
			(1.0 - InitialProbabilityOfSearchingLowClusters) * visitProgress;
		return ProbabilityOfSearchingLowClusters;
	}

	const size_t sampleCount = 200;
	argos::Real sumCoverCount = 0.0;

	for(size_t sample = 0; sample < sampleCount; ++sample) {
		argos::CVector2 candidate(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		size_t coverCount = 0;

		for(const auto& cluster : VisitedClusters) {
			const argos::Real radiusSq = cluster.radius * cluster.radius;
			if((candidate - cluster.center).SquareLength() <= radiusSq) {
				++coverCount;
			}
		}

		sumCoverCount += static_cast<argos::Real>(coverCount);
	}

	const argos::Real lambda = sumCoverCount / static_cast<argos::Real>(sampleCount);
	const argos::Real knownCoverage = 1.0 - std::exp(-lambda);
	const argos::Real explorationPressure = 1.0 - ((1.0 - knownCoverage) * (1.0 - visitProgress));

	ProbabilityOfSearchingLowClusters = std::max<argos::Real>(0.0,
		std::min<argos::Real>(1.0,
			InitialProbabilityOfSearchingLowClusters +
			(1.0 - InitialProbabilityOfSearchingLowClusters) * explorationPressure));

	return ProbabilityOfSearchingLowClusters;
}

REGISTER_LOOP_FUNCTIONS(Cluster_loop_functions, "Cluster_loop_functions")
