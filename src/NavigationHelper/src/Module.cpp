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

    return true;
}


bool WalkingNavigationHelperModule::configure(yarp::os::ResourceFinder &rf)
{
    // check if the configuration file is empty
    if(rf.isNull())
    {
        yError() << "[configure] Empty configuration file";
        return false;
    }

    // get the period
    m_dT = rf.check("period", yarp::os::Value(0.1)).asFloat64();

    // set the module name
    std::string name;
    if(!YarpUtilities::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    setName(name.c_str());

    // set rpc client 
    std::string portName;
    if(!YarpUtilities::getStringFromSearchable(rf, "rpcClientPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_rpcClientPortName = "/" + name + portName;
    m_rpcClientPort.open(m_rpcClientPortName);

    if(!YarpUtilities::getStringFromSearchable(rf, "rpcServerPort_name", m_rpcServerPortName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    yarp::os::Network::connect(m_rpcClientPortName, m_rpcServerPortName);

    

    // set data port name
    if(!YarpUtilities::getStringFromSearchable(rf, "modulePathInputPort_name", input_data_port_name))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_inputPort.open("/" + getName() + input_data_port_name);

    if(!YarpUtilities::getStringFromSearchable(rf, "data_port_name", nav_path_port_name))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    yarp::os::Network::connect(nav_path_port_name, input_data_port_name);

    // set goal port name
    if(!YarpUtilities::getStringFromSearchable(rf, "robotGoalOutputPort_name", output_port_name))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_outputPort.open("/" + getName() + output_port_name);
    
    if(!YarpUtilities::getStringFromSearchable(rf, "robotGoalInputPort_name", robot_input_port_name))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    yarp::os::Network::connect(output_port_name, robot_input_port_name);

    return true;
}


bool WalkingNavigationHelperModule::updateModule()
{

    // connect the ports
    if(!yarp::os::Network::isConnected(nav_path_port_name, input_data_port_name))
            yarp::os::Network::connect(nav_path_port_name, input_data_port_name);
    else {        
        // read navigation path from port
        yarp::sig::Vector* path = m_inputPort.read();
        if (path != nullptr) {
            yInfo() << "[update] got path data:" << path->toString().c_str();
            auto size = path->size();
            if (!(size % 2 ==0))
            {
                yWarning() << "[update] path size is not a multiple of 2, rejecting last element";
                for (auto i = 0; i < size-1; i++)
                {
                    
                    m_pathBuffer.push_back(*path[i].data());
                }
            }
            else {
                for (auto  i = 0; i < size; i++)
                {
                    m_pathBuffer.push_back(*path[i].data());
                }
            }
        }
        if(!yarp::os::Network::isConnected(output_port_name, robot_input_port_name))
        {
            yarp::os::Network::connect(output_port_name, robot_input_port_name);
        
            yarp::sig::Vector& goal= m_outputPort.prepare();
            goal.clear();

            goal.push_back(m_pathBuffer.front());
            m_pathBuffer.pop_front();
            goal.push_back(m_pathBuffer.front());
            m_outputPort.write();
        }
    }
    return true;
}

