/**
 * @file NavigationHelper.cpp
 * @authors Mohamed Elobaid <mohamed.elobaid@iit.it>
 * @copyright 2022 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2022
 */

// std
#include <iomanip>

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>

#include <WalkingControllers/NavigationHelper/Module.h>
#include <WalkingControllers/YarpUtilities/Helper.h>

using namespace WalkingControllers;

double WalkingNavigationHelperModule::getPeriod()
{
    return m_dT;
}

bool WalkingNavigationHelperModule::close()
{
    // clear deque
    m_pathBuffer.clear();
    // close the ports
    m_inputPort.close();
    m_outputPort.close();
    m_rpcServerPort.close();

    return true;
}

bool WalkingNavigationHelperModule::configure(yarp::os::ResourceFinder &rf)
{
    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << "[configure] Empty configuration file";
        return false;
    }

    // get the period
    m_dT = rf.check("period", yarp::os::Value(0.1)).asFloat64();

    // set the module name
    std::string name;
    if (!YarpUtilities::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    setName(name.c_str());

    // set rpc Server
    std::string portName;
    if (!YarpUtilities::getStringFromSearchable(rf, "rpcServerPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_rpcServerPortName = "/" + name + portName;
    if (!m_rpcServerPort.open(m_rpcServerPortName))
    {
        yError() << "[configure] failed to open the rpc port";
        return false;
    }

    // set data port name
    if (!YarpUtilities::getStringFromSearchable(rf, "modulePathInputPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    input_data_port_name = "/" + getName() + portName;
    m_inputPort.open(input_data_port_name);

    if (!YarpUtilities::getStringFromSearchable(rf, "data_port_name", nav_path_port_name))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    // yarp::os::Network::connect(input_data_port_name, nav_path_port_name);

    // set goal port name
    if (!YarpUtilities::getStringFromSearchable(rf, "robotGoalOutputPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    output_port_name = "/" + getName() + portName;
    m_outputPort.open(output_port_name);

    if (!YarpUtilities::getStringFromSearchable(rf, "robotGoalInputPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    robot_input_port_name = portName;
    // yarp::os::Network::connect(output_port_name, robot_input_port_name);

    m_replan = false;

    yInfo() << "[Configure] the Navigation helper module is successfuly configured";
    return true;
}

bool WalkingNavigationHelperModule::updateModule()
{

    yarp::os::Bottle cmd;
    m_rpcServerPort.read(cmd, true);
    // yInfo() << "[update] current status of m_replan is: " << m_replan;
    respond(cmd);

    // connect the ports
    if (!yarp::os::Network::isConnected(nav_path_port_name, input_data_port_name))
        yarp::os::Network::connect(nav_path_port_name, input_data_port_name);

    path = m_inputPort.read(false);
    if (path != nullptr)
    {
        yInfo() << "[update] got updated path ";
        auto size = path->size();
        if (!(size % 2 == 0))
        {
            yWarning() << "[update] path size is not a multiple of 2, rejecting last element";
            for (auto i = 0; i < size - 1; i++)
            {
                m_pathBuffer.push_back((*path)[i]);
            }
        }
        else
        {
            for (auto i = 0; i < size; i++)
            {
                m_pathBuffer.push_back((*path)[i]);
            }
        }
        for (double element : m_pathBuffer)
        {
            yInfo() << "[update] the buffer is updated: " << m_pathBuffer;
        }
    }
    else
    {
        // yWarning() << "[update] not recieving new path, attaching 0  to the end of the buffer";
    }

    if (!yarp::os::Network::isConnected(output_port_name, robot_input_port_name))
    {
        yarp::os::Network::connect(output_port_name, robot_input_port_name);
    }

    yarp::os::Bottle &goal = m_outputPort.prepare();
    goal.clear();

    if (!m_pathBuffer.empty() && !m_replan)
    {

        goal.addFloat64(m_pathBuffer.front());
        m_pathBuffer.pop_front();
        goal.addFloat64(m_pathBuffer.front());
        m_outputPort.write();
        m_pathBuffer.pop_front();
    }
    if (m_replan)
    {
        if (!m_pathBuffer.empty())
        {
            m_pathBuffer.clear();
        }
    }

    return true;
}

bool WalkingNavigationHelperModule::respond(const yarp::os::Bottle &command)
{
    yarp::os::Bottle reply;
    if (command.get(0).asString() == "persist")
    {

        m_replan = false;

        reply.addInt32(1);
        yInfo() << "[RPC Server] The path is not updated.";
        m_rpcServerPort.reply(reply);
        return true;
    }
    else if (command.get(0).asString() == "replan")
    {

        m_replan = true;
        yInfo() << "[RPC Server] The path is updated, replanning.";
        reply.addInt32(1);
        m_rpcServerPort.reply(reply);
        return true;
    }
    else
    {
        yError() << "[RPC Server] Unknown command.";
        reply.addInt32(0);
        m_rpcServerPort.reply(reply);

        return false;
    }
    return true;
}
