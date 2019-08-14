/**
ParallelSim.h

Class definition for ParallelSim.

Author: Phillip Taylor
*/

#ifndef PARALLELSIM_INCLUDED
#define PARALLELSIM_INCLUDED

#include <cstdlib>
#include "TraCIAPI.h"
#include "PartitionManager.h"


class ParallelSim {
  private:
    const char* SUMO_BINARY;
    const char* NETCONVERT_BINARY;
    TraCIAPI conn;
    std::string host;
    std::string path;
    const char* cfgFile;
    std::string netFile;
    std::string routeFile;
    int port;
    int numThreads;
    int endTime;
    // sets the border edges for all partitions
    void setBorderEdges(std::vector<border_edge_t>[], std::vector<PartitionManager*>&);

  public:
    // params: host, port, cfg file, gui (true), threads
    ParallelSim(const std::string&, int, const char*, bool, int);
    // gets network and route file paths
    void getFilePaths();
    // partition the SUMO network
    // param: true for metis partitioning, false for grid partitioning
    void partitionNetwork(bool);
    // execute parallel sumo simulations in created partitions
    void startSim();

};

#endif
