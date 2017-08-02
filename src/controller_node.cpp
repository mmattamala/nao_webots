/**
 * @file main_node.cpp
 * Implementation of a node that publishes NAO data from Webots Simulation
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */


#include "NaoController.h"

// Main
int main(int argc, char** argv)
{
    // node initialization
    ros::init(argc, argv, "nao_controller_node");

    nao_webots::NaoController nc;

    ros::shutdown();
    return EXIT_SUCCESS;
}
