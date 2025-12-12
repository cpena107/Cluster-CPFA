#ifndef Cluster_LOOP_FUNCTIONS_H
#define Cluster_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/Cluster/Cluster_controller.h>

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
		size_t ClusterWidthX;
		size_t ClusterLengthY;
		size_t PowerRank;

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
		std::vector<Pheromone>   PheromoneList;
		std::vector<argos::CRay3>    TargetRayList;
		std::vector<argos::CVector2> VisitedLocations;

		/* Cluster structure for visited locations */
		struct VisitedCluster {
			argos::CVector2 center;
			argos::Real width;
			argos::Real height;
			size_t visitCount;
			bool isMerged;
			
			VisitedCluster(argos::CVector2 c, argos::Real w, argos::Real h) 
				: center(c), width(w), height(h), visitCount(0), isMerged(false) {}
		};
		std::vector<VisitedCluster> VisitedClusters;

		argos::CRange<argos::Real>   ForageRangeX;
		argos::CRange<argos::Real>   ForageRangeY;

        size_t currNumCollectedFood;
        size_t Num_robots;
        vector<size_t>			ForageList;
		argos::CVector2 NestPosition;

	private:

		/* private helper functions */
		void RandomFoodDistribution();
		void ClusterFoodDistribution();
		void PowerLawFoodDistribution();
		bool IsOutOfBounds(argos::CVector2 p, size_t length, size_t width);
		bool IsCollidingWithNest(argos::CVector2 p);
		bool IsCollidingWithFood(argos::CVector2 p);
		void MergeClustersIntoSuperClusters();
		argos::Real CalculateClusterCoverage(const VisitedCluster& cluster);
		double score;
		int PrintFinalScore;
};

#endif /* Cluster_LOOP_FUNCTIONS_H */
