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
	PrintFinalScore(0)
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

	FoodList.clear();
	FoodColoringList.clear();
	PheromoneList.clear();
	FidelityList.clear();
	TargetRayList.clear();

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
		TargetRayList.clear();
		PheromoneList.clear();
		//VisitedLocations.clear();
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
	if (PrintFinalScore == 1) printf("%f, %f, %lu, %lu\n", getSimTimeInSeconds(), score, MaxClusterCount, VisitedClusters.size());
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

void Cluster_loop_functions::ConfigureFromGenome(Real* g)
{
	// Assign genome generated by the GA to the appropriate internal variables.
	ProbabilityOfSwitchingToSearching = g[0];
	ProbabilityOfReturningToNest      = g[1];
	UninformedSearchVariation.SetValue(g[2]);
	RateOfInformedSearchDecay         = g[3];
	RateOfSiteFidelity                = g[4];
	RateOfLayingPheromone             = g[5];
	RateOfPheromoneDecay              = g[6];
}

/*****
 * Update visited location clusters by detecting grid-based regions
 * and merging visited locations when coverage exceeds 50%
 *****/
void Cluster_loop_functions::UpdateVisitedClusters() {
	if(VisitedLocations.empty()) return;
	// Only process new locations that haven't been clustered yet
	//if(LastProcessedLocationIndex >= VisitedLocations.size()) return;

	// Proximity-based clustering: group nearby visited points and compute centroid
	// Don't clear existing clusters - preserve them across updates

	// Use food cluster geometry to set a reasonable clustering radius
	argos::Real foodOffset = 1.5 * FoodRadius;
	argos::Real clusterWidth = ClusterWidthX * foodOffset;
	argos::Real clusterHeight = ClusterLengthY * foodOffset;
	argos::Real clusterRadius = std::max(clusterWidth, clusterHeight) / 5.0;

	// Distance threshold to assign points to the same cluster (slightly less than radius)
	argos::Real mergeThreshold = clusterRadius * 0.6;
	argos::Real mergeThresholdSq = mergeThreshold * mergeThreshold;

	// Only process new locations starting from LastProcessedLocationIndex
	std::vector<bool> assigned(VisitedLocations.size() - LastProcessedLocationIndex, false);

	for(size_t i = LastProcessedLocationIndex; i < VisitedLocations.size(); ++i) {
		size_t localIdx = i - LastProcessedLocationIndex;
		if(assigned[localIdx]) continue; // Already assigned to a cluster

		// Start a new cluster with point i or merge with existing cluster
		bool mergedWithExisting = false;
		
		// Check if this location is close to any existing cluster
		for(size_t k = 0; k < VisitedClusters.size(); ++k) {
			
			if(VisitedClusters[k].isMerged) continue;

			argos::CVector2 diff = VisitedLocations[i] - VisitedClusters[k].center;
			if(diff.SquareLength() <= mergeThresholdSq) {
				// Merge into existing cluster by updating its centroid and visit count
				argos::Real oldWeight = static_cast<argos::Real>(VisitedClusters[k].visitCount);
				argos::Real newWeight = oldWeight + 1.0;
				VisitedClusters[k].center = (VisitedClusters[k].center * oldWeight + VisitedLocations[i]) / newWeight;
				VisitedClusters[k].visitCount++;
				mergedWithExisting = true;
				assigned[localIdx] = true;
				break;
			}
		}
		
		if(mergedWithExisting) continue;
		
		// Start a new cluster with point i
		std::vector<size_t> members;
		members.push_back(i);
		assigned[localIdx] = true;

		// Simple agglomerative pass: attach unassigned points within threshold to this cluster
		for(size_t j = i + 1; j < VisitedLocations.size(); ++j) {
			size_t localJdx = j - LastProcessedLocationIndex;
			if(localJdx >= assigned.size() || assigned[localJdx]) continue;
			argos::CVector2 diff = VisitedLocations[j] - VisitedLocations[i];
			if(diff.SquareLength() <= mergeThresholdSq) {
				members.push_back(j);
				assigned[localJdx] = true;
			}
		}

		// Compute centroid of this cluster
		argos::Real sumX = 0.0, sumY = 0.0;
		for(size_t idx : members) {
			sumX += VisitedLocations[idx].GetX();
			sumY += VisitedLocations[idx].GetY();
		}
		// Create VisitedCluster object 
		argos::CVector2 centroid(sumX / members.size(), sumY / members.size());

		VisitedCluster vc(centroid, clusterRadius);
		vc.visitCount = members.size();

		// Decide merged flag based on coverage
		argos::Real coverage = CalculateClusterCoverage(vc);
		vc.isMerged = (coverage > 0.5);

		VisitedClusters.push_back(vc);
	}
	
	// Update the index to mark these locations as processed
	// Keep all visited locations for visualization (yellow dots)
	LastProcessedLocationIndex = VisitedLocations.size();

	// Second-level clustering: merge clusters into super-clusters
	MergeClustersIntoSuperClusters();
}

/*****
 * Calculate the coverage percentage of a cluster based on visited locations
 * Returns a value between 0.0 and 1.0
 *****/
argos::Real Cluster_loop_functions::CalculateClusterCoverage(const VisitedCluster& cluster) {
	// Estimate coverage using a circular cluster radius and visit spacing
	argos::Real visitTolerance = 0.3; // Same default as controller memory spacing
	argos::Real clusterArea = argos::CRadians::PI.GetValue() * cluster.radius * cluster.radius;
	argos::Real visitCellArea = visitTolerance * visitTolerance;
	argos::Real maxPossibleVisits = clusterArea / visitCellArea;
	argos::Real coverage = static_cast<argos::Real>(cluster.visitCount) / std::max(1.0, maxPossibleVisits);
	return std::min<argos::Real>(1.0, std::max<argos::Real>(0.0, coverage));
}

/*****
 * Merge clusters into bigger super-clusters when combined area exceeds 70% of potential merged cluster
 * Limited to 2 iterations to prevent over-consolidation into a single cluster
 *****/
void Cluster_loop_functions::MergeClustersIntoSuperClusters() {
	if(VisitedClusters.size() < 2) return; // Nothing to merge

	// Iterate to allow merged clusters to propagate
	int maxIterations = 5;
	for(int iteration = 0; iteration < maxIterations && VisitedClusters.size() > 1; ++iteration) {
		bool mergedAny = false;
		std::vector<VisitedCluster> newClusters;
		std::vector<bool> merged(VisitedClusters.size(), false);

		for(size_t i = 0; i < VisitedClusters.size(); ++i) {
			if(merged[i]) continue;

			std::vector<size_t> members;
			members.push_back(i);

			for(size_t j = i + 1; j < VisitedClusters.size(); ++j) {
				if(merged[j]) continue;

				argos::Real r1 = VisitedClusters[i].radius;
				argos::Real r2 = VisitedClusters[j].radius;
				argos::CVector2 diff = VisitedClusters[j].center - VisitedClusters[i].center;
				argos::Real dist = diff.Length();

				// Merge clusters only when they overlap
				if(dist < (r1 + r2)) {
					members.push_back(j);
					merged[j] = true;
				}
			}

			if(members.size() == 1) {
				newClusters.push_back(VisitedClusters[i]);
			} else {
				mergedAny = true;
				
				argos::Real totalWeight = 0.0;
				argos::Real sumX = 0.0, sumY = 0.0;
				size_t totalVisits = 0;
				argos::Real maxRadius = 0.0;

				for(size_t idx : members) {
					// Weight center by visit count to stabilize position
					argos::Real weight = static_cast<argos::Real>(VisitedClusters[idx].visitCount);
					sumX += VisitedClusters[idx].center.GetX() * weight;
					sumY += VisitedClusters[idx].center.GetY() * weight;
					totalWeight += weight;
					totalVisits += VisitedClusters[idx].visitCount;
					
					if(VisitedClusters[idx].radius > maxRadius) {
						maxRadius = VisitedClusters[idx].radius;
					}
				}

				argos::CVector2 superCenter(sumX / totalWeight, sumY / totalWeight);
				
				// Apply new radius rule: 1.2x the radius of the original clusters (using max as reference)
				argos::Real superRadius = maxRadius * 1.2;
				if(superRadius > 1.0) superRadius = 1.0;

				VisitedCluster superCluster(superCenter, superRadius);
				superCluster.visitCount = totalVisits;
				superCluster.isMerged = true;

				newClusters.push_back(superCluster);
			}
		}

		VisitedClusters = newClusters;
		
		// Break if no merges occurred in this iteration
		if(!mergedAny) break;
	}
}

/*****
 * Merge overlapping super clusters into a bigger cluster that covers all of them.
 * Clusters are treated as circles for overlap detection.
 * The resulting cluster is the enclosing circle of the merged set.
 *****/
void Cluster_loop_functions::MergeTriangularSuperClusters() {
	if(VisitedClusters.size() < 2) return;

	std::vector<bool> processed(VisitedClusters.size(), false);
	std::vector<VisitedCluster> newClusters;

	for(size_t i = 0; i < VisitedClusters.size(); ++i) {
		if(processed[i]) continue;

		if(!VisitedClusters[i].isMerged) {
			newClusters.push_back(VisitedClusters[i]);
			processed[i] = true;
			continue;
		}

		// Start a group with cluster i
		std::vector<size_t> group;
		std::vector<size_t> queue;
		queue.push_back(i);
		processed[i] = true;
		group.push_back(i);

		// BFS to find all connected (overlapping) clusters
		size_t head = 0;
		while(head < queue.size()) {
			size_t curr = queue[head++];
			
			argos::Real r1 = VisitedClusters[curr].radius;

			for(size_t j = 0; j < VisitedClusters.size(); ++j) {
				if(processed[j]) continue;
				if(!VisitedClusters[j].isMerged) continue;

				argos::Real r2 = VisitedClusters[j].radius;
				argos::Real dist = (VisitedClusters[curr].center - VisitedClusters[j].center).Length();

				// Check overlap
				if(dist < (r1 + r2)) {
					processed[j] = true;
					queue.push_back(j);
					group.push_back(j);
				}
			}
		}

		if(group.size() >= 2) {
			// Merge the group
			argos::Real minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
			size_t totalVisits = 0;

			for(size_t idx : group) {
				argos::Real r = VisitedClusters[idx].radius;
				argos::Real cx = VisitedClusters[idx].center.GetX();
				argos::Real cy = VisitedClusters[idx].center.GetY();

				minX = std::min(minX, cx - r);
				maxX = std::max(maxX, cx + r);
				minY = std::min(minY, cy - r);
				maxY = std::max(maxY, cy + r);

				totalVisits += VisitedClusters[idx].visitCount;
			}

			argos::Real newW = maxX - minX;
			argos::Real newH = maxY - minY;
			argos::CVector2 newCenter(minX + newW/2.0, minY + newH/2.0);

			// Calculate new enclosing radius
			argos::Real maxDist = 0.0;
			for(size_t idx : group) {
				argos::Real dist = (VisitedClusters[idx].center - newCenter).Length() + VisitedClusters[idx].radius;
				if(dist > maxDist) maxDist = dist;
			}
			
			VisitedCluster mergedCluster(newCenter, maxDist);
			mergedCluster.visitCount = totalVisits;
			mergedCluster.isMerged = true;
			newClusters.push_back(mergedCluster);
		} else {
			// Keep original
			newClusters.push_back(VisitedClusters[i]);
		}
	}

	VisitedClusters = newClusters;
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
	const argos::Real searchRadius = 0.5; // 0.5 meter radius
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
	
	// Return a random point from the best points (lowest cluster count)
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
