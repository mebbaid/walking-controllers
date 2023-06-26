/**
 * @file NavigationHelper.h
 * @authors Simone Micheletti <simone.micheletti@iit.it>
 * @copyright 2023 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2023
 */

#ifndef NAVIGATION_HELPER__HPP
#define NAVIGATION_HELPER__HPP

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <atomic>
#include <thread>
#include <deque>
#include <mutex>

#include <WalkingControllers/KinDynWrapper/Wrapper.h>
#include <WalkingControllers/TrajectoryPlanner/StableDCMModel.h>
#include <WalkingControllers/TrajectoryPlanner/TrajectoryGenerator.h>

namespace WalkingControllers
{
    class NavigationHelper
    {
    private:
        yarp::os::BufferedPort<yarp::os::Bottle> m_unicyclePort; /**< Port that streams odometry info of the virtual unicycle. */
        yarp::os::BufferedPort<yarp::os::Bottle> m_replanningTriggerPort; /**< Publishes the flag triggering the navigation's global planner. */
        yarp::os::BufferedPort<yarp::os::Bottle> m_feetPort; /**< Feet port vector of feet positions (left, right). */
        std::atomic<bool> m_runThreads{false};      /**< Global flag that allows the looping of the threads. */
        std::deque<bool> m_leftInContact;           /**< Copy of the deques in Module of the left feet contacts status. */
        std::deque<bool> m_rightInContact;          /**< Copy of the deques in Module of the left feet contacts status. */
        double m_navigationReplanningDelay;         /**< Delay in seconds of how much to wait before sending the trigger to the navigation stack after exiting double support. */
        int m_navigationTriggerLoopRate;            /**< Loop rate for the thread computing the navigation trigger*/
        bool m_wasInDoubleSupport;
        bool m_publishInfo;                         /**< Flag to whether publish information. */

        std::thread m_virtualUnicyclePubliserThread; /**< Thread for publishing the state of the unicycle used in the TrajectoryGenerator. */
        std::thread m_navigationTriggerThread;      /**< Thread for publishing the flag triggering the navigation's global planner. */
        
        std::mutex m_updateFeetMutex;               /**< Mutex that regulates the access to m_leftInContact and m_rightInContact. */
        bool m_simulationMode{false};               /**< Flag that syncs the trigger delay with the external clock if in simulation. */  

        const std::string m_portPrefix = "/navigation_helper";
        
        /**
         * Function launched by the looping thread
         */
        void computeNavigationTrigger();

        /**
         * Close the Navigation Helper Threads
         * @return true/false in case of success/failure.
         */
        bool closeThreads();

    public:
        /**
         * Default constructor
         */
        NavigationHelper();

        /**
         * Default destructor
         */
        ~NavigationHelper();

        /**
         * Close the Navigation Helper Threads and ports
         * @return true/false in case of success/failure.
         */
        bool closeHelper();

        /**
         * Initialize the Navigation Helper
         * @param config yarp searchable object of the configuration.
         * @return true/false in case of success/failure.
         */
        bool init(const yarp::os::Searchable& config,
                    std::unique_ptr<WalkingFK> &FKSolver, 
                    std::unique_ptr<StableDCMModel> &stableDCMModel, 
                    std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator
                    );

        /**
         * Updates the internal variables containing the relative feet contacts status
         * @param left left feet contact status deque.
         * @param right right feet contact status deque.
         * @return true/false in case of success/failure.
         */
        bool updateFeetDeques(const std::deque<bool> &left, const std::deque<bool> &right);

        void computeVirtualUnicycleThread(std::unique_ptr<WalkingFK> &FKSolver, 
                                            std::unique_ptr<StableDCMModel> &stableDCMModel, 
                                            std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator
                                            );

        bool publishPlannedFootsteps(std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator);
    };
}

#endif