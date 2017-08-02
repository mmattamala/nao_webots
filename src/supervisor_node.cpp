/**
 * @file main_node.cpp
 * Implementation of a node that publishes NAO data from Webots Simulation
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */


#include "NaoSupervisor.h"

// Main
int main(int argc, char** argv)
{
    // node initialization
    ros::init(argc, argv, "nao_supervisor_node");

    nao_webots::NaoSupervisor ns;

    ros::shutdown();
    return EXIT_SUCCESS;
}
