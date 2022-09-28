# Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.

add_walking_controllers_application(
  NAME WalkingNavigationHelper
  SOURCES src/main.cpp src/Module.cpp
  HEADERS include/WalkingControllers/NavigationHelper/Module.h
  LINK_LIBRARIES WalkingControllers::YarpUtilities
  SUBDIRECTORIES app)
