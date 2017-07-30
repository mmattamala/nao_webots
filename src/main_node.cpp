/**
 * @file main_node.cpp
 * Implementation of a node that publishes NAO data from Webots Simulation
 *
 * @author <a href="mailto:mmattamala@ing.uchile.cl">Matias Mattamala</a>
 */


#include "NaoWebots.h"

// Main
int main(int argc, char** argv)
{
    // node initialization
    ros::init(argc, argv, "nao_webots");

    nao_webots::NaoWebots nw;

    ros::shutdown();
    return EXIT_SUCCESS;
}
