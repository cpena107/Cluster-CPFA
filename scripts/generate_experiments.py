import argparse
import os

def generate_xml_content(dist_name, count, visited_tolerance, rec_freq, max_visited, max_radius, arena_x, arena_y):
    # Determine params based on dist and count
    food_dist_val = "0"
    num_clusters = "0"
    cluster_width = "8" # default random
    cluster_length = "8" # default random
    
    if dist_name == "clustered":
        food_dist_val = "1"
        num_clusters = str(count // 16)
        cluster_width = "4"
        cluster_length = "4"
    elif dist_name == "semi_cluster":
        food_dist_val = "2"
        num_clusters = str(count // 16)
        cluster_width = "4"
        cluster_length = "4"
    elif dist_name == "random":
        food_dist_val = "0"
        num_clusters = "4"
        cluster_width = "8"
        cluster_length = "8"

    xml_content = f"""<?xml version="1.0" encoding="utf-8"?>
<argos-configuration>

  <!-- ************************* -->
  <!-- * General configuration * -->
  <!-- ************************* -->
  <framework>
    <system threads="0" />
    <experiment length="16000"
                ticks_per_second="16"
		random_seed="0"/>
  </framework>

  <!-- *************** -->
  <!-- * Controllers * -->
  <!-- *************** -->
  <controllers>

    <Cluster_controller id="Cluster"
                     library="build/source/Cluster/libCluster_controller">
      <actuators>
        <differential_steering implementation = "default"/>
      </actuators>

      <sensors>
        <footbot_proximity    implementation = "default" show_rays = "false"/>

	<positioning          implementation = "default"
					/>	

        <footbot_motor_ground implementation = "rot_z_only"/>
      </sensors>

      <params>

        <settings TargetDistanceTolerance           = "0.05"
		  TargetAngleTolerance               = "0.1"
		  FoodDistanceTolerance              = "0.13"
                  NestDistanceTolerance = "0.05"  
                  NestAngleTolerance = "0.1"
                  SearchStepSize                    = "0.08"
                  RobotForwardSpeed                 = "16.0"
                  RobotRotationSpeed                = "8.0"
		  DestinationNoiseStdev             = "0.0"
		  PositionNoiseStdev             = "0.00"
		  VisitedLocationTolerance          = "{visited_tolerance}"
		  RecordingFrequency               = "{rec_freq}"
      MaxVisitedLocations              = "{max_visited}"
      MaxClusterRadius              = "{max_radius}"
		  ResultsDirectoryPath              = "results/"/>
      </params>

    </Cluster_controller>

  </controllers>

  <loop_functions library = "build/source/Cluster/libCluster_loop_functions"
                  label   = "Cluster_loop_functions">

    <!-- evolvable parameters -->
        <Cluster       ProbabilityOfSwitchingToSearching = "0.504"
                    ProbabilityOfReturningToNest      = "0.002"
                    ProbabilityOfSearchingLowClusters = "0.001"
                    UninformedSearchVariation         = "7.0"
                    RateOfInformedSearchDecay         = "0.28"
                    RateOfSiteFidelity                = "4.27"
                    RateOfLayingPheromone             = "3.75"
                    RateOfPheromoneDecay              = "0.03"
		                PrintFinalScore = "1"
		    />

        <settings
                    MaxSimTimeInSeconds = "8000"
                    MaxSimCounter = "1"
                    VariableFoodPlacement = "0"
                    OutputData = "0"
                    DrawDensityRate = "4"
                    DrawIDs = "1"
                    DrawTrails = "0"
                    DrawTargetRays = "0"
                    FoodDistribution = "{food_dist_val}"
                    FoodItemCount = "{count}"
                    NumberOfClusters = "{num_clusters}"
                    ClusterWidthX = "{cluster_width}"
                    ClusterLengthY = "{cluster_length}"
                    PowerRank = "4"
                    FoodRadius = "0.05"
                    NestRadius = "0.25"
                    NestElevation = "0.01"
          />

  </loop_functions>

  <!-- *********************** -->
  <!-- * Arena configuration * -->
  <!-- *********************** -->
  <arena size="{arena_x}, {arena_y}, 1" center="0,0,0.5">

    <floor id="floor" source="loop_functions" pixels_per_meter="10"/>

    <!-- Northwest Group -->
    <distribute>
      <position method="grid"
                center="-1.0, 0.5, 0.0"
                distances="0.2, 0.0, 0.0"
                layout="4, 1, 1" />
      <orientation method="constant" values="0.0, 0.0, 0.0" />
      <entity quantity="4" max_trials="100">
        <foot-bot id="Cluster_">
          <controller config="Cluster"/>
        </foot-bot>
      </entity>
    </distribute>

    <!-- Northeast Group -->
    <distribute>
      <position method="grid"
                center="1.0, 0.5, 0.0"
                distances="0.2, 0.0, 0.0"
                layout="4, 1, 1" />
      <orientation method="constant" values="0.0, 0.0, 0.0" />
      <entity quantity="4" max_trials="100">
        <foot-bot id="Cluster_NE_">
          <controller config="Cluster"/>
        </foot-bot>
      </entity>
    </distribute>

    <!-- Southwest Group -->
    <distribute>
      <position method="grid"
                center="-1.0, -0.5, 0.0"
                distances="0.2, 0.0, 0.0"
                layout="4, 1, 1" />
      <orientation method="constant" values="0.0, 0.0, 0.0" />
      <entity quantity="4" max_trials="100">
        <foot-bot id="Cluster_SW_">
          <controller config="Cluster"/>
        </foot-bot>
      </entity>
    </distribute>

    <!-- Southeast Group -->
    <distribute>
      <position method="grid"
                center="1.0, -0.5, 0.0"
                distances="0.2, 0.0, 0.0"
                layout="4, 1, 1" />
      <orientation method="constant" values="0.0, 0.0, 0.0" />
      <entity quantity="4" max_trials="100">
        <foot-bot id="Cluster_SE_">
          <controller config="Cluster"/>
        </foot-bot>
      </entity>
    </distribute>

  </arena>

  <!-- ******************* -->
  <!-- * Physics engines * -->
  <!-- ******************* -->
  <physics_engines>
    <dynamics2d id="dyn2d" />
  </physics_engines>

  <!-- ********* -->
  <!-- * Media * -->
  <!-- ********* -->
  <media />

  <!-- ****************** -->
  <!-- * Visualization * -->
  <!-- ****************** 
  <visualization>

    <qt-opengl>
        <camera>
          <placements>
            <placement index="0" position="0,0,13" look_at="0,0,0" up="0,1,0" lens_focal_length="35"/>
            <placement index="1" position="0,-9,5" look_at="0,0,0" up="0,1,0" lens_focal_length="35"/>
            <placement index="2" position="0,9,5" look_at="0,0,0" up="0,-1,0" lens_focal_length="35"/>
            <placement index="3" position="4,0,5" look_at="0,0,0" up="-1,0,0" lens_focal_length="20"/>
            <placement index="4" position="-4,0,5" look_at="0,0,0" up="1,0,0" lens_focal_length="20"/>
            <placement index="5" position="-3,0,2" look_at="1,0,0" up="1,0,0" lens_focal_length="35"/>
          </placements>
        </camera>
        <user_functions label="Cluster_qt_user_functions"/>
    </qt-opengl>

  </visualization>
-->
</argos-configuration>
"""
    return xml_content

def main():
    parser = argparse.ArgumentParser(description="Generate CPFA ClusterMap XML files.")
    parser.add_argument("--visited-tolerance", required=True, type=str, help="VisitedLocationTolerance value")
    parser.add_argument("--recording-freq", required=True, type=str, help="RecordingFrequency value")
    parser.add_argument("--max-visited", required=True, type=str, help="MaxVisitedLocations value")
    parser.add_argument("--max-radius", required=True, type=str, help="MaxClusterRadius value")
    parser.add_argument("--arena-x", required=True, type=str, help="Arena X dimension")
    parser.add_argument("--arena-y", required=True, type=str, help="Arena Y dimension")
    parser.add_argument("--output-dir", default=".", help="Directory to save generated files")

    args = parser.parse_args()

    distributions = ["clustered", "random", "semi_cluster"]
    counts = [16, 32, 48, 64, 80]
    
    # Create output directory if it doesn't exist
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    output_dir = args.output_dir

    for dist in distributions:
        for count in counts:
            content = generate_xml_content(dist, count, args.visited_tolerance, args.recording_freq, args.max_visited, args.max_radius, args.arena_x, args.arena_y)
            filename = f"CPFA_ClusterMap_{dist}_{count}.xml"
            filepath = os.path.join(output_dir, filename)
            with open(filepath, 'w') as f:
                f.write(content)
            print(f"Generated {filepath}")

if __name__ == "__main__":
    main()
