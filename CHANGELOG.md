# Changelog
All notable changes to this project are documented in this file.

## [0.8.0] - 2023-11-15
### Added
- Add the configuration files to run the walking controller on `ergoCubGazeboV1_1` (https://github.com/robotology/walking-controllers/pull/152)
- Add the configuration files to run the walking controller on `ergoCubSN001` (https://github.com/robotology/walking-controllers/pull/153) 

### Changed
- Implement `populateData` lambda function to easily populate the `BipedalLocomotion::VectorsCollection` (https://github.com/robotology/walking-controllers/pull/147)
- Tune the parameters for `ergoCubSN000` (https://github.com/robotology/walking-controllers/pull/149 https://github.com/robotology/walking-controllers/pull/157)

### Fixed
- Fix non initialized variable used to remove the ZMP offset (https://github.com/robotology/walking-controllers/pull/149)

### Removed
- Removed deprecated IK solvers (https://github.com/robotology/walking-controllers/pull/150)
- Remove qpOASES dependency (https://github.com/robotology/walking-controllers/pull/156)

## [0.7.0] - 2023-03-07
### Added
- Now the IK problem can be solved by `BipedaLocomotion::IK::QPInverseKinematics` (https://github.com/robotology/walking-controllers/pull/118)
- Add the configuration files to run the walking controller on `ergoCubGazeboV1` (https://github.com/robotology/walking-controllers/pull/139)
- Add the possibility to remove the ZMP-CoM offset when the controller is started (https://github.com/robotology/walking-controllers/pull/137)
- Add the configuration files to run the walking controller (with and without reargeting) on `ergoCubSN000` (https://github.com/robotology/walking-controllers/pull/137 , https://github.com/robotology/walking-controllers/pull/140)

### Changed
- Add the possibility to control the root height instead of the CoM height (https://github.com/robotology/walking-controllers/pull/118)
- [iCubGenova09] Tune the gains (https://github.com/robotology/walking-controllers/pull/118)
- Print the timers status every 10 seconds (https://github.com/robotology/walking-controllers/pull/123)
- Modify the interface with the planner to allow the walking to run at 500Hz (https://github.com/robotology/walking-controllers/pull/127, https://github.com/robotology/walking-controllers/pull/136)
- Update vendored `AddUninstallTarget.cmake` to `YCM v0.15.1` version (https://github.com/robotology/walking-controllers/pull/141)

### Fixed
- Fix `WalkingModule::setPlannerInput()` when a new input is set from the joypad (https://github.com/robotology/walking-controllers/pull/129)

## [0.6.1] - 2022-09-14
### Added
- Enable the lateral walking (https://github.com/robotology/walking-controllers/pull/119)

### Changed
- Remove the `Loggermodule` and the `LoggerClient` if favor of `YarpRobotLoggerDevice` (https://github.com/robotology/walking-controllers/pull/120)
- Make all the dependencies required (https://github.com/robotology/walking-controllers/pull/120)

### Fixed
-  Fixes a compilation problem on Windows relative to the use of `std::min` (https://github.com/robotology/walking-controllers/pull/121)

## [0.5.2] - 2022-09-05
### Added
- Enable the minimum jerk trajectory for the first DS phase (https://github.com/robotology/walking-controllers/pull/117)

### Changed
- [iCubGenova09] Update the configuration files (https://github.com/robotology/walking-controllers/pull/114)
- Ask for `UnicyclePlanner v0.4.3` (https://github.com/robotology/walking-controllers/pull/117)

## [0.5.1] - 2022-05-01
### Fixed
- [iCubGenova09] Fix the name of the torso frame in the forwardKinematics.ini
- Remove logger from iCubGazeboV3 configuration (https://github.com/robotology/walking-controllers/pull/105)
- Fix yarp deprecation of methods like asDouble (https://github.com/robotology/walking-controllers/pull/112)

## [0.5.0] - 2022-02-01
### Added
- Add the possibility to use the walking-module with iCubGenova09 (https://github.com/robotology/walking-controllers/pull/80)
- Add the possibility to use the `FreeSpaceEllipse` in the `TrajectoryPlanner` (https://github.com/robotology/walking-controllers/pull/82)
- Enable the `iFeel` retargeting pipeline (https://github.com/robotology/walking-controllers/pull/87)
- Bipedal locomotion v0.6.0 is now required to compile the project (https://github.com/robotology/walking-controllers/pull/97)
- Add the possibility to control the root link height instead of the CoM height (https://github.com/robotology/walking-controllers/pull/93 and https://github.com/robotology/walking-controllers/pull/96)

### Changed
- Add the possibility to handle multiple wrenches in the RobotInterface/Helper class (https://github.com/robotology/walking-controllers/pull/80)
- Using the `feetYawDelta` and the `slowWhenBackwardFactor` in the `TrajectoryPlanner` (https://github.com/robotology/walking-controllers/pull/89)
- `WalkingModule` logs the data through bipedal locomotion framework logger (https://github.com/robotology/walking-controllers/pull/97)

### Fixed
- Fixed missing link library in `WholeBodyControllers` component  (https://github.com/robotology/walking-controllers/pull/81).

## [0.4.1] - 2020-02-04

### Added

### Changed
- Bugfix while resetting the hand smoother in the `RetargetingClient` (https://github.com/robotology/walking-controllers/pull/75)
- Fixed compilation if iDynTree 3 is used (https://github.com/robotology/walking-controllers/pull/77, https://github.com/robotology/walking-controllers/pull/78).
- Fix missing include in `ZMPController.h` (https://github.com/robotology/walking-controllers/pull/76)

## [0.4.0] - 2020-12-01
### Added
- Adding the possibility to use Gazebo base data inside the walking controller
- `TrajectoryGenerator` class of the `TrajectoryPlanner` library includes now
  the method `getWeightPercentage` to retrieve the amount of weight on each foot
  requested by the planner.

### Changed
- Adding the `use_external_robot_base` parameter inside the `dcm_walking_with_joypad.ini`
- Adding the Gazebo base data port inside the `robotControl.ini`
- Tunning the `zmpControllerParams.ini` and `dcmReactiveControllerParams.ini`
- Modifying the follwoing classes for geting and using Gazebo base data:
  - `/KinDynWrapper/Wrapper`
  - `RobotInterface/Helper`
  - `TrajectoryPlanner/TrajectoryGenerator`
  - `WalkingModule`
- Tune gains for iCubGenova04

## [0.3.3] - 2020-11-23
### Added

### Changed
- Including Eigen as a private dependency in all targets using EigenHelpers (https://github.com/robotology/walking-controllers/pull/62)
- Copied the parameters of hand_retargeting in joypad_control (https://github.com/robotology/walking-controllers/pull/63)

## [0.3.2] - 2020-03-21
### Changed
- Add missing includes in `TimeProfiler` (https://github.com/robotology/walking-controllers/pull/60)

## [0.3.1] - 2020-03-18
### Changed
- Fix the windows compilation (https://github.com/robotology/walking-controllers/pull/59)

## [0.3.0] - 2020-03-16
### Added
- The `CHANGELOG.md` file
- Implement the `WalkingControllersFindDepencies.cmake`
- Adding the possibility of selecting Stiff/Compliant mode in joint level.

### Changed
- General refactoring of the library. The WalkingModule is now split in several library. Namelly:
   - `YarpUtilities`: utilities for using `YARP`
   - `StdUtilities`: utilities related to `std` library
   - `iDynTreeUtilities`: utilities related to `iDynTree`
   - `TimeProfiler`: library for time profiling
   - `SimplifiedModelControllers` library related to simplified model controllers
   - `WholeBodyControllers` library related to controller based on the entire robot model
   - `TrajectoryPlanner` library related to trajectory planner
   - `KinDynWrapper` iDynTree `KinDynComputation` wrapper.
   - `RetargetingClient` client for the retargeting

## [0.2.0] - 2019-10-24
### Added
- Implement the first version of the hand retargeting (i.e. `RetargetingClient`)
- Is it possible running different mode of the `WalkingModules` by changing the main configuration file
- Add versioning to the project (0.2.0)

### Changed
- The `time profiler` is moved to the `utilities` library
- The required version of `osqp-eigen` the  is now `0.4.0` (before was `0.3.0`)
- Update the `MPCSolver` class to be compatible with `osqp-eigen v0.4.0`
- Update the `WalkingQPInverseKinematics` to take into account the hand retargeting (the
`setPhase` method is implemented)
- WalkingLogger is now in a separate library
- Parameters on `iCubGenova04` are tuned
- Parameters on `icubGazeboSim` are tuned
- Set default build type to `release`
- Avoid using `Eigen::VectorXd` in the evaluate gradient and bounds

###  Fixed
- Initialize the `zmpLeft` and `zmpRight` vectors in the `evaluateZMP` method
- Fix the initialization of the `inverseKinematicsQPSolverOptions` bottle in the `WalkingModule.cpp`

### Removed
- Remove `Stance` state from `WalkingFSM`

## [0.1.0] -  2019-10-21
### Added
- `cmake/FindEigen3.cmake` file in order to improve the compatibility with `Windows` operating system
- implement `getNumberFromSearchable` in `Utils.cpp` file

### Changed
- Improve the description of the errors in the `WalkingLoggerModule`
- Fix typos in `README.md`
- Implement the `stopWalking` and the `pauseWalking` commands
- Parameters on `iCubGazeboV2_5` are tuned
- Parameters on `iCubGenova04` are tuned
- The required version of the unicycle planner is now `0.1.2` (before was `0.1.102`)
- the `WalkingQPIK_qpOASES` and `WalkingQPIK_osqp` now inherits  from the `WalkingQPIK` class

###  Fixed
- Fix the `close` function in the `WalkingLoggerModule`
- Add missing includes in `TimeProfiler.cpp` and WaldkingDCMModelPredictiveController.cpp`
- The joypad control mapping is now fixed

### Removed
- `onTheFly` feature for the `WalkingModule` application
- `iCubGenova02` is no more supported

## [0.0.1] - 2018-12-18
### Added
- Implement the first version of the `WalkingModule`
- Implement the first version of the `WalkingLoggerModule`
- Implement the first version of the `WalkingJoypadModule`

[0.8.0]: https://github.com/robotology/walking-controllers/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/robotology/walking-controllers/compare/v0.6.1...v0.7.0
[0.6.1]: https://github.com/robotology/walking-controllers/compare/v0.5.2..v0.6.1
[0.5.2]: https://github.com/robotology/walking-controllers/compare/v0.5.1..v0.5.2
[0.5.1]: https://github.com/robotology/walking-controllers/compare/v0.5.0..v0.5.1
[0.5.0]: https://github.com/robotology/walking-controllers/compare/v0.4.1...v0.5.0
[0.4.1]: https://github.com/robotology/walking-controllers/compare/v0.4.0...v0.4.1
[0.4.0]: https://github.com/robotology/walking-controllers/compare/v0.3.3...v0.4.0
[0.3.3]: https://github.com/robotology/walking-controllers/compare/v0.3.2...v0.3.3
[0.3.2]: https://github.com/robotology/walking-controllers/compare/v0.3.1...v0.3.2
[0.3.1]: https://github.com/robotology/walking-controllers/compare/v0.3.0...v0.3.1
[0.3.0]: https://github.com/robotology/walking-controllers/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/robotology/walking-controllers/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/robotology/walking-controllers/compare/v0.0.1...v0.1.0
[0.0.1]: https://github.com/robotology/walking-controllers/releases/tag/v0.0.1
