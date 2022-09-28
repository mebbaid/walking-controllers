/**
 * @file NavigationHelper main
 * @authors Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2022 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2022
 */

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <WalkingControllers/NavigationHelper/Module.h>

using namespace WalkingControllers;

int main(int argc, char * argv[])
{
    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find YARP network";
        return EXIT_FAILURE;
    }

    // prepare and configure the resource finder
    yarp::os::ResourceFinder &rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.setDefaultConfigFile("navigation.ini");

    rf.configure(argc, argv);

    // create the producer module
    WalkingNavigationHelperModule module;

    return module.runModule(rf);
}

