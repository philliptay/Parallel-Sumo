/*
Main program for running a parallel SUMO simulation. startSim() runs simulation.
Can comment out getFilePaths() and partitionNetwork() after these functions
have already been executed to run startSim() with same partitions.

Author: Phillip Taylor
*/

#include <iostream>
#include <cstdlib>
#include "ParallelSim.h"

int main(int argc, char* argv[]) {
    // params: host server, first port. sumo cfg file, gui option (true), number of threads
    ParallelSim client("localhost", 1337, "assets/corniche.sumocfg", true, 4);
  //  client.getFilePaths();
    // param: true for metis partitioning, false for grid partitioning (only works for 2 partitions currently)
  //  client.partitionNetwork(true);
    client.startSim();
}
