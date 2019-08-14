CC=g++
CXXFLAGS= -std=c++11 -I.

all: main
clean:
	rm -f *.o

main: main.o ParallelSim.o PartitionManager.o TraCIAPI.o socket.o storage.o Pthread_barrier.o tinyxml2.o
#ParallelSim.o: ParallelSim.h
#PartitionManager.o: PartitionManager.h
