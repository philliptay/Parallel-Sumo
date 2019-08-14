#include <iostream>
#include "TraCIAPI.h"

class Client : public TraCIAPI {
public:
    Client() {};
    ~Client() {};
};

int main(int argc, char* argv[]) {
    Client client;
    client.connect("localhost", 1337);
    std::cout << "time in s: " << client.simulation.getTime() << "\n";
    std::cout << "run 5 steps ...\n";
    client.simulationStep(5);
    std::cout << "time in s: " << client.simulation.getTime() << "\n";
    client.close();
}
