cmake_minimum_required(VERSION 3.5)

project(ff_tests_maps)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  world/
  DESTINATION share/${PROJECT_NAME}/world)

install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models)




file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")

foreach(path ${traffic_editor_paths})

  # Get the output world name
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${CMAKE_CURRENT_BINARY_DIR}/world/${output_world_name}.world)
  set(output_model_dir ${output_dir}/models)

  ##############################################################################
  # download Models
  ##############################################################################

  if (NO_DOWNLOAD_MODELS)
    
  else()
    message("DOWNLOADING MODELS WITH COMMAND: ros2 run rmf_building_map_tools building_map_model_downloader ${map_path}")
    add_custom_command(
      DEPENDS ${map_path}
      COMMAND ros2 run rmf_building_map_tools building_map_model_downloader ${map_path} -e ~/.gazebo/models
      OUTPUT ${output_world_path}
    )
  endif()

  ##############################################################################
  # generate the navmesh and required files for crowd simulation for gz
  ##############################################################################
  set(crowd_sim_config_resource ${output_dir}/config_resource/)

  if (GENERATE_CROWD_SIM)
  add_custom_command(
    OUTPUT ${world_name}_crowdsim
    COMMAND ros2 run rmf_building_map_tools building_crowdsim ${map_path} ${crowd_sim_config_resource} ${output_world_path}
    DEPENDS ${output_world_path}
  )

  # This will initiate both custom commands: ${output_world_path} and ${world_name}_crowdsim
  add_custom_target(generate_${world_name}_crowdsim ALL
    DEPENDS ${world_name}_crowdsim
  )
  else()
  #do nothing
  endif()

  ##############################################################################
  # Generate the nav graphs
  ##############################################################################

  set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

endforeach()

ament_package()