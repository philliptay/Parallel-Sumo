/**
PartitionManager.cpp

Manages partition's internal SUMO simulation and sychronizes
with other partitions running in parallel.

Author: Phillip Taylor
*/


#include <iostream>
#include <unistd.h>
#include <algorithm>
#include "TraCIAPI.h"
#include "PartitionManager.h"

PartitionManager::PartitionManager(const char* binary, int id, pthread_barrier_t* barr,
  pthread_mutex_t* lock, pthread_cond_t* cond, std::string& cfg, std::string& host, int port, int t) :
  SUMO_BINARY(binary),
  id(id),
  lockAddr(lock),
  barrierAddr(barr),
  condAddr(cond),
  cfg(cfg),
  host(host),
  port(port),
  endT(t) {}

void PartitionManager::setMyBorderEdges(std::vector<border_edge_t> borderEdges) {
  for(border_edge_t e : borderEdges) {
    if(e.to == this)
      toBorderEdges.push_back(e);
    else if(e.from == this)
      fromBorderEdges.push_back(e);
  }
}

bool PartitionManager::startPartition() {
  return (pthread_create(&myThread, NULL, internalSimFunc, this) == 0);
}

void PartitionManager::waitForPartition() {
  (void) pthread_join(myThread, NULL);
}

void PartitionManager::closePartition() {
  myConn.close();
  pthread_exit(NULL);
}

void PartitionManager::connect() {
  myConn.connect(host, port);
}

std::vector<std::string> PartitionManager::getEdgeVehicles(const std::string& edgeID) {
  return myConn.edge.getLastStepVehicleIDs(edgeID);
}

std::vector<std::string> PartitionManager::getRouteEdges(const std::string& routeID) {
  return myConn.route.getEdges(routeID);
}

void PartitionManager::add(const std::string& vehID, const std::string& routeID, const std::string& typeID,
 const std::string& laneInd, const std::string& depPos, const std::string& speed) {
  myConn.vehicle.add(vehID, routeID, typeID, "-1", laneInd, depPos, speed);
}

void PartitionManager::moveTo(const std::string& vehID, const std::string& laneID, double pos) {
  myConn.vehicle.moveTo(vehID, laneID, pos);
}

void PartitionManager::slowDown(const std::string& vehID, double speed) {
  myConn.vehicle.slowDown(vehID, speed, myConn.simulation.getDeltaT());
}

void PartitionManager::setSynching(bool b) {
  synching = b;
}

bool PartitionManager::isSynching() {
  return synching;
}

bool PartitionManager::isWaiting() {
  return waiting;
}

void PartitionManager::waitForSynch() {
  pthread_mutex_lock(lockAddr);
  waiting = true;
  while(synching) {
    pthread_cond_wait(condAddr, lockAddr);
  }
  waiting = false;
  pthread_mutex_unlock(lockAddr);
}

void PartitionManager::handleToEdges(int num, std::vector<std::string> prevToVehicles[]) {
  for(int i=0; i<num;i++) {
    pthread_mutex_lock(lockAddr);
    std::vector<std::string> currVehicles = getEdgeVehicles(toBorderEdges[i].id);
    pthread_mutex_unlock(lockAddr);

    if(!currVehicles.empty()) {
      for(std::string veh : currVehicles) {
        auto it = std::find(prevToVehicles[i].begin(), prevToVehicles[i].end(), veh);
        // vehicle speed is to be updated in previous partition
        if(it != prevToVehicles[i].end()) {
          PartitionManager* fromPart = toBorderEdges[i].from;

          // handle case where partitions update each other (e.g. two-way road)
          if(synching)
            waitForSynch();

          fromPart->setSynching(true);
          while(!fromPart->isWaiting()) {
            // make sure partitions aren't waiting for each other
            if(synching)
              break;
          }
          pthread_mutex_lock(lockAddr);

          // check if vehicle has been transferred out of partition
          std::vector<std::string> trans = fromPart->getEdgeVehicles(toBorderEdges[i].id);
          if(std::find(trans.begin(), trans.end(), veh) != trans.end()) {
            // set from partition vehicle speed to next partition vehicle speed
            try {
              fromPart->slowDown(veh, myConn.vehicle.getSpeed(veh));
            }
            catch(libsumo::TraCIException){}

          }

          fromPart->setSynching(false);
          pthread_mutex_unlock(lockAddr);
          pthread_cond_signal(condAddr);
        }
      }
      prevToVehicles[i] = currVehicles;
    }
  }
}

void PartitionManager::handleFromEdges(int num, std::vector<std::string> prevFromVehicles[]) {
  for(int i=0; i<num;i++) {
    pthread_mutex_lock(lockAddr);
    std::vector<std::string> currVehicles = getEdgeVehicles(fromBorderEdges[i].id);
    pthread_mutex_unlock(lockAddr);

    if(!currVehicles.empty()) {
      for(std::string veh : currVehicles) {
        auto it = std::find(prevFromVehicles[i].begin(), prevFromVehicles[i].end(), veh);
        // vehicle is to be inserted in next partition
        if(it == prevFromVehicles[i].end()) {
          PartitionManager* toPart = fromBorderEdges[i].to;

          // handle case where partitions update each other (e.g. two-way road)
          if(synching)
            waitForSynch();

          // make sure next partition is available to be updated
          toPart->setSynching(true);
          while(!toPart->isWaiting()) {
            // make sure partitions aren't waiting for each other
            if(synching)
              break;
          }

          pthread_mutex_lock(lockAddr);


          // check if vehicle not already on edge (if a vehicle starts on a border edge)
          std::vector<std::string> toVehs = toPart->getEdgeVehicles(fromBorderEdges[i].id);
          std::string route = myConn.vehicle.getRouteID(veh);

          if(std::find(toVehs.begin(), toVehs.end(), veh) == toVehs.end()) {

            // check if vehicle is on split route
            int pos = veh.find("_part");
            if(pos != std::string::npos) {
              int routePos = route.find("_part");
              std::string routeSub = route.substr(0,routePos+5);
              route = routeSub+"0";
              int routePart = 0;
              std::string firstEdge = (toPart->getRouteEdges(route))[0];
              while(firstEdge.compare(fromBorderEdges[i].id)) {
                routePart++;
                route = routeSub+std::to_string(routePart);
                firstEdge = (toPart->getRouteEdges(route))[0];
              }
            }
            try {
              // add vehicle to next partition
              toPart->add(veh, route, myConn.vehicle.getTypeID(veh),
              std::to_string(myConn.vehicle.getLaneIndex(veh)), std::to_string(myConn.vehicle.getLanePosition(veh)),
              std::to_string(myConn.vehicle.getSpeed(veh)));
              // move vehicle to proper lane position in next partition
              toPart->moveTo(veh, myConn.vehicle.getLaneID(veh), myConn.vehicle.getLanePosition(veh));
            }
            catch(libsumo::TraCIException){}
          }

          toPart->setSynching(false);
          pthread_mutex_unlock(lockAddr);
          pthread_cond_signal(condAddr);
        }
      }
      prevFromVehicles[i] = currVehicles;
    }
  }
}


void PartitionManager::internalSim() {
  pid_t pid;
  const char* args[7] = {SUMO_BINARY, "-c", cfg.c_str(), "--remote-port", std::to_string(port).c_str(), "--start", NULL};

  switch(pid = fork()){
    case -1:
      // fork() has failed
      perror("fork");
      break;
    case 0:
      // execute sumo simulation
      execv(args[0], (char*const*) args);
      std::cout << "execv() has failed" << std::endl;
      exit(EXIT_FAILURE);
      break;
  }
  // wait for server to startup (1 second)
  usleep(1000000);
  // ensure all servers have started before simulation begins
  pthread_barrier_wait(barrierAddr);
  connect();
  pthread_mutex_lock(lockAddr);
  std::cout << "partition " << id << " started in thread " << pthread_self() << std::endl;
  pthread_mutex_unlock(lockAddr);
  int numFromEdges = fromBorderEdges.size();
  int numToEdges = toBorderEdges.size();
  std::vector<std::string> prevToVehicles[numToEdges];
  std::vector<std::string> prevFromVehicles[numFromEdges];
  while(myConn.simulation.getTime() < endT) {
    waiting = false;
    pthread_mutex_lock(lockAddr);
    myConn.simulationStep();
    pthread_mutex_unlock(lockAddr);
    // synchronize border edges
    handleToEdges(numToEdges, prevToVehicles);
  //  waiting = true;
  //  pthread_barrier_wait(barrierAddr);
  //  waiting = false;
    handleFromEdges(numFromEdges, prevFromVehicles);

    // make sure every time step across partitions is synchronized
    waiting = true;
    pthread_barrier_wait(barrierAddr);
  }
  closePartition();
}
