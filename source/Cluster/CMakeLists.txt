
###############################################
# define shared object files
###############################################

add_library(Cluster_loop_functions     SHARED      Cluster_loop_functions.h
                                                Cluster_loop_functions.cpp
                                                Cluster_qt_user_functions.h
                                                Cluster_qt_user_functions.cpp)

add_library(Cluster_controller         SHARED      Cluster_controller.h
                                                Cluster_controller.cpp
                                                Cluster_loop_functions.h
                                                Cluster_loop_functions.cpp)
###############################################
# link shared object files to dependencies
###############################################

target_link_libraries(  Cluster_controller
                        BaseController
                        Pheromone)

target_link_libraries(  Cluster_loop_functions
                        Cluster_controller
                        argos3core_simulator
                        argos3plugin_simulator_entities
                        argos3plugin_simulator_footbot
                        argos3plugin_simulator_genericrobot
                        argos3plugin_simulator_qtopengl
                        ${ARGOS_QTOPENGL_LIBRARIES})
