#ifndef Cluster_LOOP_FUNCTIONS_H
#define Cluster_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/Cluster/Cluster_controller.h>
#include <unordered_set>
#include <unordered_map>

using namespace argos;
using namespace std;

static const size_t GENOME_SIZE = 7; // There are 7 parameters to evolve

class Cluster_loop_functions : public argos::CLoopFunctions
{

	friend class Cluster_controller;
	friend class Cluster_qt_user_functions;

	public:

		Cluster_loop_functions();
	   
		void Init(argos::TConfigurationNode &t_tree);
		void Reset();
		void PreStep();
		void PostStep();
		bool IsExperimentFinished();
		void PostExperiment();
		argos::CColor GetFloorColor(const argos::CVector2 &c_pos_on_floor);

		// GA Functions
		
		/* Configures the robot controller from the genome */
		void ConfigureFromGenome(Real* pf_genome);
		/* Calculates the performance of the robot in a trial */
		Real Score();
	
		/**
		 * Returns the current trial.
		 */
		UInt32 GetTrial() const;
	
		/**
		 * Sets the current trial.
		 * @param un_trial The trial number.
		 */
		void SetTrial(UInt32 un_trial);
	
	/* public helper functions */
	void UpdatePheromoneList();
	void SetFoodDistribution();
	void UpdateVisitedClusters();

	argos::Real getSimTimeInSeconds();		std::vector<argos::CColor>   TargetRayColorList;

	unsigned int getNumberOfRobots();
	double getProbabilityOfSwitchingToSearching();
	double getProbabilityOfReturningToNest();
	double getProbabilityOfSearchingLowClusters();
	double getUninformedSearchVariation();
	double getRateOfInformedSearchDecay();
	double getRateOfSiteFidelity();
	double getRateOfLayingPheromone();
	double getRateOfPheromoneDecay();
	argos::CVector2 GetLowClusterSearchLocation();	protected:

		/* Record how many sites a robot communicated to the nest */
		void RecordSitesCommunicated(size_t siteCount);

		void setScore(double s);

		argos::CRandom::CRNG* RNG;
        size_t NumDistributedFood; 
		size_t MaxSimTime;
		size_t ResourceDensityDelay;
		size_t RandomSeed;
		size_t SimCounter;
		size_t MaxSimCounter;
		size_t VariableFoodPlacement;
		size_t OutputData;
		size_t DrawDensityRate;
		size_t DrawIDs;
		size_t DrawTrails;
		size_t DrawTargetRays;
		size_t FoodDistribution;
		size_t FoodItemCount;
		size_t NumberOfClusters;
		size_t MaxClusterCount;
		size_t ClusterWidthX;
		size_t ClusterLengthY;
		size_t PowerRank;

		/* Running average counters for sites communicated to nest */
		size_t SitesCommunicatedSum;
		size_t SitesCommunicatedCount;

	/* Cluster variables */
	argos::Real ProbabilityOfSwitchingToSearching;
	argos::Real ProbabilityOfReturningToNest;
	argos::Real ProbabilityOfSearchingLowClusters;
	argos::CRadians UninformedSearchVariation;
	argos::Real RateOfInformedSearchDecay;
	argos::Real RateOfSiteFidelity;
	argos::Real RateOfLayingPheromone;
	argos::Real RateOfPheromoneDecay;		/* physical robot & world variables */
		argos::Real FoodRadius;
		argos::Real FoodRadiusSquared;
		argos::Real NestRadius;
		argos::Real NestRadiusSquared;
		argos::Real NestElevation;
		argos::Real SearchRadiusSquared;

		/* list variables for food & pheromones */
		std::vector<argos::CVector2> FoodList;
		std::vector<argos::CColor>   FoodColoringList;
        map<string, argos::CVector2> FidelityList; 
		map<string, argos::CVector2> LowClusterTargetList;
		std::vector<Pheromone>   PheromoneList;
		//std::vector<argos::CRay3>    TargetRayList;
		std::vector<argos::CRay3>    SearchLocationRays;
        std::map<std::string, std::vector<argos::CRay3>> RobotTrails;
        std::map<std::string, CColor> RobotTrailColors;
		std::vector<argos::CVector2> VisitedLocations;
		// Snapshot of real robot-visit points that contributed to a cluster in the
		// most recent DBSCAN run.  Populated each UpdateVisitedClusters() call
		// (before compression) so the renderer can highlight them for debugging.
		std::vector<argos::CVector2> ClusteredVisitedLocations;
		size_t LastProcessedLocationIndex;
		// Number of synthetic chain points at the front of VisitedLocations after
		// each compression pass. Points at index >= numSyntheticPoints are real
		// robot visits added since the last update.
		size_t numSyntheticPoints;
		// Tracks which VisitedLocations indices have been permanently assigned
		// to a DBSCAN cluster, so ClusteredVisitedLocations accumulates without
		// duplicates across multiple UpdateVisitedClusters() calls.
		std::unordered_set<size_t> clusteredLocationIndices;

		/* Cluster structure for visited locations */
		struct VisitedCluster {
			argos::CVector2 center;
			// originalCenter is locked at formation and only replaced on a merge event.
			// It anchors the centroid so drift is bounded to maxDrift (0.3 m) between updates.
			argos::CVector2 originalCenter;
			argos::Real radius;
			size_t visitCount;
			bool isMerged;
            bool isFrozen;
			int clusterId; // unique ID for debugging
			
			VisitedCluster(argos::CVector2 c, argos::Real r) 
				: center(c), originalCenter(c), radius(r), visitCount(0), isMerged(false), isFrozen(false), clusterId(-1) {}
		};
		std::vector<VisitedCluster> VisitedClusters;
        std::vector<VisitedCluster> FrozenClusters; // Store clusters that reached max radius
		std::unordered_map<int, VisitedCluster> ClusterMap; // persistent map keyed by cluster ID
		int nextClusterId;

		argos::CRange<argos::Real>   ForageRangeX;
		argos::CRange<argos::Real>   ForageRangeY;

        size_t currNumCollectedFood;
        size_t Num_robots;
        size_t RobotsReturnedToNest;
        vector<size_t>			ForageList;
		argos::CVector2 NestPosition;
		argos::Real MaxClusterRadius;

	private:

		/* private helper functions */
		void RandomFoodDistribution();
		void ClusterFoodDistribution();
		void PowerLawFoodDistribution();
		bool IsOutOfBounds(argos::CVector2 p, size_t length, size_t width);
		bool IsCollidingWithNest(argos::CVector2 p);
		bool IsCollidingWithFood(argos::CVector2 p);
		void MergeClustersIntoSuperClusters();
		void MergeTriangularSuperClusters();
		argos::Real CalculateClusterCoverage(const VisitedCluster& cluster);
		double score;
		int PrintFinalScore;
};

#endif /* Cluster_LOOP_FUNCTIONS_H */
