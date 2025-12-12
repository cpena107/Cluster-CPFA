
###############################################
# define shared object files
###############################################

add_library(Minimap_loop_functions     SHARED      Minimap_loop_functions.h
                                                Minimap_loop_functions.cpp
                                                Minimap_qt_user_functions.h
                                                Minimap_qt_user_functions.cpp)

add_library(Minimap_controller         SHARED      Minimap_controller.h
                                                Minimap_controller.cpp
                                                Minimap_loop_functions.h
                                                Minimap_loop_functions.cpp)
###############################################
# link shared object files to dependencies
###############################################

target_link_libraries(  Minimap_controller
                        BaseController
                        Pheromone)

target_link_libraries(  Minimap_loop_functions
                        Minimap_controller
                        argos3core_simulator
                        argos3plugin_simulator_entities
                        argos3plugin_simulator_footbot
                        argos3plugin_simulator_genericrobot
                        argos3plugin_simulator_qtopengl
                        ${ARGOS_QTOPENGL_LIBRARIES})
