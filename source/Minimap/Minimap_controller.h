#ifndef Minimap_CONTROLLER_H
#define Minimap_CONTROLLER_H

#include <source/Base/BaseController.h>
#include <source/Base/Pheromone.h>
#include <source/Minimap/Minimap_loop_functions.h>

using namespace std;
using namespace argos;

static unsigned int num_targets_collected = 0;

class Minimap_loop_functions;

class Minimap_controller : public BaseController {

	public:

		Minimap_controller();

		// CCI_Controller inheritence functions
		void Init(argos::TConfigurationNode &node);
		void ControlStep();
		void Reset();

		bool IsHoldingFood();
		bool IsUsingSiteFidelity();
		bool IsInTheNest();

		Real FoodDistanceTolerance;

		void SetLoopFunctions(Minimap_loop_functions* lf);

	private:
        string controllerID;
		Minimap_loop_functions* LoopFunctions;
		argos::CRandom::CRNG* RNG;

		/* pheromone trail variables */
		std::vector<argos::CVector2> TrailToShare;
		std::vector<argos::CVector2> TrailToFollow;
		std::vector<argos::CRay3>    MyTrail;

		/* robot position variables */
		argos::CVector2 SiteFidelityPosition;
        bool	updateFidelity;
		vector<CRay3> myTrail;
		CColor        TrailColor;

		bool isInformed;
		bool isHoldingFood;
		bool isUsingSiteFidelity;
		bool isGivingUpSearch;

		size_t ResourceDensity;
		size_t MaxTrailSize;
		size_t SearchTime;

		/* iAnt Minimap state variable */
		enum Minimap_state {
			DEPARTING = 0,
			SEARCHING = 1,
			RETURNING = 2,
			SURVEYING = 3
		} Minimap_state;

		/* iAnt Minimap state functions */
		void Minimap();
		void Departing();
		void Searching();
		void Returning();
		void Surveying();

	/* Minimap helper functions */
	void SetRandomSearchLocation();
	void SetLowClusterSearchLocation();
	void SetHoldingFood();
	void SetLocalResourceDensity();
	void SetFidelityList(argos::CVector2 newFidelity);
	void SetFidelityList();
	bool SetTargetPheromone();	argos::Real GetExponentialDecay(argos::Real value, argos::Real time, argos::Real lambda);
	argos::Real GetBound(argos::Real value, argos::Real min, argos::Real max);
	argos::Real GetPoissonCDF(argos::Real k, argos::Real lambda);

	void UpdateTargetRayList();

	/* Memory-based search functions */
	void RecordVisitedLocation(argos::CVector2 location);
	bool HasVisitedLocation(argos::CVector2 location, argos::Real tolerance);
	void SetUnvisitedSearchLocation();
	void ShareVisitedLocationsWithNest();
	void ClearVisitedLocations();
	void DetectLostResource();

	CVector2 previous_position;

	string results_path;
	string results_full_path;
	bool isUsingPheromone;

	unsigned int survey_count;

	/* Memory of visited locations */
	std::vector<argos::CVector2> VisitedLocations;
	argos::Real VisitedLocationTolerance;
	size_t MaxVisitedLocations;
	bool isLostResource;
};

#endif /* Minimap_CONTROLLER_H */