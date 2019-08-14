/**
ParallelSim.cpp

Parallelizes a SUMO simulation. Partitions a SUMO network by number of threads,
and runs each parallel SUMO network partition in a PartitionManager.

Author: Phillip Taylor
*/

#include <iostream>
#include <pthread.h>
#include <ctime>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <iterator>
#include <unordered_map>
#include "Pthread_barrier.h"
#include "tinyxml2.h"
#include "ParallelSim.h"


typedef std::unordered_multimap<std::string, int>::iterator umit;

ParallelSim::ParallelSim(const std::string& host, int port, const char* cfg, bool gui, int threads) :
  host(host),
  port(port),
  cfgFile(cfg),
  numThreads(threads) {

  // set paths for sumo executable binaries
  const char* sumoExe;
  if(gui)
    sumoExe = "/bin/sumo-gui";
  else
    sumoExe = "/bin/sumo";

  char* sumoPath(getenv("SUMO_HOME"));
  if (sumoPath == NULL) {
    std::cout << "$SUMO_HOME is not set! Must set $SUMO_HOME." << std::endl;
    exit(EXIT_FAILURE);
  }
  else {
    std::cout << "$SUMO_HOME is set to '" << sumoPath << "'" << std::endl;
    int len = strlen(sumoPath);
    char* tmp1 = new char[len+14];
    char* tmp2 = new char[len+16];
    strcpy(tmp1, sumoPath);
    strcpy(tmp2, sumoPath);
    SUMO_BINARY = strcat(tmp1, sumoExe);
    NETCONVERT_BINARY = strcat(tmp2, "/bin/netconvert");

   }
   // get end time
   tinyxml2::XMLDocument cfgDoc;
   tinyxml2::XMLError e = cfgDoc.LoadFile(cfgFile);
   if(e) {
     std::cout << cfgDoc.ErrorIDToName(e) << std::endl;
     exit(EXIT_FAILURE);
   }
   tinyxml2::XMLElement* cfgEl = cfgDoc.FirstChildElement("configuration");
   if (cfgEl == nullptr) {
     std::cout << "sumo config error: no configuration" << std::endl;
     exit(EXIT_FAILURE);
   }
   tinyxml2::XMLElement* endTimeEl = cfgEl->FirstChildElement("time")->FirstChildElement("end");
   if (endTimeEl == nullptr) {
     std::cout << "No end time specified. Setting default end time at 1000 steps." << std::endl;
     endTime = 1000;
   }
   else {
     endTime = atoi(endTimeEl->Attribute("value"));
   }
}

void ParallelSim::getFilePaths(){
  // get paths for net and route files
  std::string cfgStr(cfgFile);
  int found = cfgStr.find_last_of("/\\");
  path = cfgStr.substr(0,found+1);
  // load sumo cfg file
  tinyxml2::XMLDocument cfgDoc;
  tinyxml2::XMLError e = cfgDoc.LoadFile(cfgFile);
  if(e) {
    std::cout << cfgDoc.ErrorIDToName(e) << std::endl;
    exit(EXIT_FAILURE);
  }

  tinyxml2::XMLElement* cfgEl = cfgDoc.FirstChildElement("configuration");
  if (cfgEl == nullptr) {
    std::cout << "sumo config error: no configuration" << std::endl;
    exit(EXIT_FAILURE);
  }
  // get net-file
  tinyxml2::XMLElement* netFileEl = cfgEl->FirstChildElement("input")->FirstChildElement("net-file");
  if (netFileEl == nullptr) {
    std::cout << "sumo config error: no net-file" << std::endl;
    exit(EXIT_FAILURE);
  }
  // get routes-file
  tinyxml2::XMLElement* routeFileEl = cfgEl->FirstChildElement("input")->FirstChildElement("route-files");
  if (netFileEl == nullptr) {
    std::cout << "sumo config error: no route-files" << std::endl;
    exit(EXIT_FAILURE);
  }

  // set net-file
  std::string netText(netFileEl->Attribute("value"));
  netText = path+netText;
  netFile.assign(netText);
  // set routes-file
  std::string routeText(routeFileEl->Attribute("value"));
  routeText= path+routeText;
  routeFile.assign(routeText);

}

void ParallelSim::partitionNetwork(bool metis){

  // load network xml
  tinyxml2::XMLDocument network;
  tinyxml2::XMLError e = network.LoadFile(netFile.c_str());
  if(e) {
    std::cout << network.ErrorIDToName(e) << std::endl;
    exit(EXIT_FAILURE);
  }
  // get net element
  tinyxml2::XMLElement* netEl = network.FirstChildElement("net");
  if (netEl == nullptr) {
    std::cout << "xml error: unable to find net element in net-file" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::vector<std::string> partBounds;
  std::string netconvertOption1;
  // partition network with metis
  if(metis) {
    pid_t pid;
    int status;
    const char* args[6] = {"python3", "convertToMetis.py", netFile.c_str(), std::to_string(numThreads).c_str(), NULL};
    switch(pid = fork()){
      case -1:
        // fork() has failed
        perror("fork");
        break;
      case 0:
        // execute metis for partition
        execvp(args[0], (char*const*) args);
        std::cout << "execvp() has failed" << std::endl;
        exit(EXIT_FAILURE);
        break;
      default:
        // waiting for convertToMetis.py
        pid = wait(&status);
        if(WEXITSTATUS(status)) {
          std::cout << "failed while converting to metis" << std::endl;
          exit(EXIT_FAILURE);
        }
        printf("metis partitioning successful with status: %d\n", WEXITSTATUS(status));
      }
      netconvertOption1 = "--keep-edges.input-file";
  }
  else {
    // partition network as grid
    tinyxml2::XMLElement* locEl = netEl->FirstChildElement("location");
    const char* boundText = nullptr;
    boundText = locEl->Attribute("convBoundary");
    std::stringstream ss(boundText);
    std::vector<int> bound;
    while(ss.good()){
      std::string substr;
      getline(ss, substr, ',');
      bound.push_back(stoi(substr));
    }
    int xCenter = (bound[0]+bound[2])/2;
    int yCenter = (bound[1]+bound[3])/2;
    int tmpXCenter = xCenter;
    int tmpYCenter = yCenter;
    if (numThreads == 2) {
      partBounds.push_back(std::to_string(bound[0])+","+std::to_string(bound[1])+","+std::to_string(xCenter)+","+std::to_string(bound[2]));
      partBounds.push_back(std::to_string(xCenter)+","+std::to_string(bound[1])+","+std::to_string(bound[2])+","+std::to_string(bound[2]));
    }
    netconvertOption1 = "--keep-edges.in-boundary";
  }

  // preprocess routes file for proper input to cutRoutes.py
  tinyxml2::XMLDocument routes;
  e = routes.LoadFile(routeFile.c_str());
  if(e) {
    std::cout << routes.ErrorIDToName(e) << std::endl;
    exit(EXIT_FAILURE);
  }

  // get routes element
  tinyxml2::XMLElement* routesEl = routes.FirstChildElement("routes");
  if (routesEl == nullptr) {
    std::cout << "xml error: unable to find routes element in routes-file" << std::endl;
    exit(EXIT_FAILURE);
  }
  int count = 0;
  // create route IDs for all routes defined within vehicles
  for(tinyxml2::XMLElement* el = routesEl->FirstChildElement("vehicle"); el != NULL; el = el->NextSiblingElement("vehicle")) {
    tinyxml2::XMLElement* routeEl = el->FirstChildElement("route");
    if(routeEl != nullptr) {
      std::string id = "custom_route"+std::to_string(count);
      tinyxml2::XMLElement* routeRefEl = routes.NewElement("route");
      routeRefEl->SetAttribute("id", id.c_str());
      routeRefEl->SetAttribute("edges", routeEl->Attribute("edges"));
      el->SetAttribute("route", id.c_str());
      routesEl->LinkEndChild(routeRefEl);
      el->DeleteChild(routeEl);
      count++;
    }
  }
  routes.SaveFile("processed_routes");


  for(int i=0; i<numThreads; i++){
    pid_t pid;
    int status;
    std::string charI = std::to_string(i);
    std::string netPart = "part"+charI+".net.xml";
    std::string rouPart = "part"+charI+".rou.xml";
    std::string cfgPart = "part"+charI+".sumocfg";

    std::string netconvertOption2;
    if(metis)
      netconvertOption2 = "edgesPart"+charI;

    else
      netconvertOption2 = partBounds[i];

    const char* partArgs[8] = {NETCONVERT_BINARY, netconvertOption1.c_str(), netconvertOption2.c_str(), "-s", netFile.c_str(), "-o", netPart.c_str(), NULL};
    const char* rouArgs[11] = {"python3", "cutRoutes.py", netPart.c_str(), "processed_routes", "--routes-output", rouPart.c_str(), "--orig-net", netFile.c_str(), "--disconnected-action", "keep", NULL};
    // create partition
    switch(pid = fork()){
      case -1:
        // fork() has failed
        perror("fork");
        break;
      case 0:
        // execute netconvert to create sumo network partition
        execv(partArgs[0], (char*const*) partArgs);
        std::cout << "execv() has failed" << std::endl;
        exit(EXIT_FAILURE);
        break;
      default:
        // waiting for partition to be created
        pid = wait(&status);
        if(WEXITSTATUS(status)) {
          std::cout << "Partition " << i << " failed to be created" << std::endl;
          exit(EXIT_FAILURE);
        }
        printf("partition %d successfully created with status: %d\n", i, WEXITSTATUS(status));

      }
      // create routes for partition
      switch(pid = fork()){
        case -1:
          // fork() has failed
          perror("fork");
          break;
        case 0:
          // execute cutRoutes.py to create routes
          execvp(rouArgs[0], (char*const*) rouArgs);
          std::cout << "execvp() has failed" << std::endl;
          exit(EXIT_FAILURE);
          break;
        default:
          // waiting for routes to be created
          pid = wait(&status);
          if(WEXITSTATUS(status)) {
            std::cout << "Routes " << i << " failed to be created" << std::endl;
            std::cout << "Routes must be specified as explicit edges" << std::endl;
            exit(EXIT_FAILURE);
          }
          printf("routes %d successfully created with status: %d\n", i, WEXITSTATUS(status));
        }
        // create sumo cfg file for partition
        char buf[BUFSIZ];
        std::size_t size;

        int source = open(cfgFile, O_RDONLY, 0);
        int dest = open(cfgPart.c_str(), O_WRONLY | O_CREAT, 0644);

        while((size = read(source, buf, BUFSIZ)) > 0){
          write(dest, buf, size);
        }
       close(source);
       close(dest);
       // set partition net-file and route-files in cfg file
       tinyxml2::XMLDocument cfgPartDoc;
       cfgPartDoc.LoadFile(cfgPart.c_str());
        tinyxml2::XMLElement* inputEl = cfgPartDoc.FirstChildElement("configuration")->FirstChildElement("input");
       tinyxml2::XMLElement* netFileEl = inputEl->FirstChildElement("net-file");
       tinyxml2::XMLElement* rouFileEl = inputEl->FirstChildElement("route-files");
       tinyxml2::XMLElement* guiFileEl = inputEl->FirstChildElement("gui-settings-file");
       netFileEl->SetAttribute("value", netPart.c_str());
       rouFileEl->SetAttribute("value", rouPart.c_str());
       if(guiFileEl != nullptr) {
         std::string newGuiVal = path+guiFileEl->Attribute("value");
         guiFileEl->SetAttribute("value", newGuiVal.c_str());
     }
       cfgPartDoc.SaveFile(cfgPart.c_str());
     }
}

void ParallelSim::setBorderEdges(std::vector<border_edge_t> borderEdges[], std::vector<PartitionManager*>& parts){
  std::unordered_multimap<std::string, int> allEdges;
  // add all edges to map, mapping edge ids to partition ids
  for(int i=0; i<parts.size(); i++) {
    std::string currNetFile = "part"+std::to_string(i)+".net.xml";
    tinyxml2::XMLDocument currNet;
    tinyxml2::XMLError e = currNet.LoadFile(currNetFile.c_str());
    tinyxml2::XMLElement* netEl = currNet.FirstChildElement("net");
    // insert all non-internal edges into map
    for(tinyxml2::XMLElement* el = netEl->FirstChildElement("edge"); el != NULL; el = el->NextSiblingElement("edge")) {
      if(el->Attribute("function") == nullptr || strcmp(el->Attribute("function"), "internal")!=0)
        allEdges.insert({el->Attribute("id"), i});
    }
  }
  // find border edges
  umit it = allEdges.begin();
  while(it != allEdges.end()) {
    std::string key = it->first;
    if(allEdges.count(key)>1){
      std::pair<umit, umit> edgePair = allEdges.equal_range(key);
      umit edgeIt1 = edgePair.first;
      umit edgeIt2 = ++edgePair.first;
      border_edge_t borderEdge1 = {};
      border_edge_t borderEdge2 = {};
      borderEdge1.id = key;
      borderEdge2.id = key;
      std::string currNetFile = "part"+std::to_string(edgeIt1->second)+".net.xml";
      tinyxml2::XMLDocument currNet;
      tinyxml2::XMLError e = currNet.LoadFile(currNetFile.c_str());
      tinyxml2::XMLElement* netEl = currNet.FirstChildElement("net");
      // find edge in net file to get attributes
      for(tinyxml2::XMLElement* el = netEl->FirstChildElement("edge"); el != NULL; el = el->NextSiblingElement("edge")) {
        if((edgeIt1->first).compare(el->Attribute("id"))==0) {
          // get lanes
          for(tinyxml2::XMLElement* laneEl = el->FirstChildElement("lane"); laneEl != NULL; laneEl = laneEl->NextSiblingElement("lane")) {
            (borderEdge1.lanes).push_back(laneEl->Attribute("id"));
            (borderEdge2.lanes).push_back(laneEl->Attribute("id"));
          }
          // determine from and to partitions -  find junction to determine if dead end
          const char* fromJunc = el->Attribute("from");
          PartitionManager* from;
          PartitionManager* to;
          for(tinyxml2::XMLElement* junEl = netEl->FirstChildElement("junction"); junEl != NULL; junEl = junEl->NextSiblingElement("junction")) {
            if(strcmp(fromJunc, junEl->Attribute("id"))==0) {
              if(strcmp(junEl->Attribute("type"), "dead_end")==0) {
                from = parts[edgeIt2->second];
                to = parts[edgeIt1->second];
              }
              else {
                from = parts[edgeIt1->second];
                to = parts[edgeIt2->second];
              }
              borderEdge1.from = from;
              borderEdge2.from = from;
              borderEdge1.to = to;
              borderEdge2.to = to;
              break;
          }
        }
        break;
      }
    }
    borderEdges[edgeIt1->second].push_back(borderEdge1);
    borderEdges[edgeIt2->second].push_back(borderEdge2);

    it = allEdges.erase(edgePair.first, edgePair.second);
  }
  else
    it = allEdges.erase(it);
  }
}

void ParallelSim::startSim(){
  std::string cfg;
  std::vector<PartitionManager*> parts;
  std::vector<border_edge_t> borderEdges[numThreads];
  pthread_mutex_t lock;
  pthread_barrier_t barrier;
  pthread_cond_t cond;

  // create partitions
  pthread_mutex_init(&lock, NULL);
  pthread_cond_init(&cond, NULL);
  pthread_barrier_init(&barrier, NULL, numThreads);
  for(int i=0; i<numThreads; i++) {
    cfg = "part"+std::to_string(i)+".sumocfg";
    PartitionManager* part = new PartitionManager(SUMO_BINARY, i, &barrier, &lock, &cond, cfg, host, port+i, endTime);
    parts.push_back(part);
  }

  setBorderEdges(borderEdges, parts);
  // start parallel simulations
  for(int i=0; i<numThreads; i++) {
    parts[i]->setMyBorderEdges(borderEdges[i]);
    if(!parts[i]->startPartition()){
      printf("Error creating partition %d", i);
      exit(EXIT_FAILURE);
    }
  }
  // join all threads when finished executing
  for (int i=0; i<numThreads; i++) {
    parts[i]->waitForPartition();
  }

  pthread_cond_destroy(&cond);
  pthread_mutex_destroy(&lock);
  pthread_barrier_destroy(&barrier);
  for(int i=0; i<numThreads; i++) {
    delete parts[i];
  }
}
