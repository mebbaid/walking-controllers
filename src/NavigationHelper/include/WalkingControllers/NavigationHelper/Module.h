/**
 * @file LoggerModule.h
 * @authors Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2022 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_NAVIGATION_HELPER_MODULE_HPP
#define WALKING_NAVIGATION_HELPER_MODULE_HPP

// std
#include <fstream>
#include <deque>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

namespace WalkingControllers
{

    /**
     * RFModule useful to collect data during an experiment.
     */
    class WalkingNavigationHelperModule : public yarp::os::RFModule
    {
        double m_dT; /**< RFModule period. */
        bool m_replan;  /**<  bool corresponding to rpc command to update path  */
        std::ofstream m_stream; /**< std stream. */

        yarp::os::BufferedPort<yarp::sig::Vector> m_inputPort; /**< Data port. */
        yarp::os::BufferedPort<yarp::os::Bottle> m_outputPort; /**< Data port. */
        yarp::os::RpcServer m_rpcServerPort; /**< RPC port. */
     
        // port names
        std::string m_rpcServerPortName;
        std::string m_rpcClientPortName;   //
        std::string nav_path_port_name;   // name of port opened by Navigation client to stream path
        std::string output_port_name;     // name of port opened by helper to stream goal data to walking-controller
        std::string robot_input_port_name;  // name of port recieving goal data in the walking module side
        std::string input_data_port_name;

        // buffer for path data
        std::deque<double> m_pathBuffer;  // buffer storing path data coming from navigation

    public:

        /**
         * Get the period of the RFModule.
         * @return the period of the module.
         */
        double getPeriod() override;

        /**
         * Main function of the RFModule.
         * @return true in case of success and false otherwise.
         */
        bool updateModule() override;

        /**
         * Configure the RFModule.
         * @param rf is the reference to a resource finder object.
         * @return true in case of success and false otherwise.
         */
        bool configure(yarp::os::ResourceFinder &rf) override;

        /**
         * Respond to a message from the RPC port.
         * @param command is the received message.
         * The following message has to be a bottle with the following structure:
         * 1. ("replan"); to provide a new path
         * 2. ("persist"). to continue with the current path
         * @param reply is the response of the server.
         * 1. 1 in case of success;
         * 2. 0 in case of failure.
         * @return true in case of success and false otherwise.
         */
        bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

        /**
         * Close the RFModule.
         * @return true in case of success and false otherwise.
         */
        bool close() override;
    };
};

#endif