# convertToMetis.py
# Author: Phillip Taylor

"""
Convert SUMO network into proper format for METIS input, partition with METIS,
and write in one file per partition the SUMO network edges of that partition.

"""
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import codecs
import copy
import subprocess

from optparse import OptionParser
from collections import defaultdict

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(os.path.join(tools))
    from sumolib.output import parse, parse_fast
    from sumolib.net import readNet
    import sumolib
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def get_options(args=sys.argv[1:]):
    optParser = OptionParser()
    options, args = optParser.parse_args(args=args)
    options.network = args[0]
    options.parts = args[1]
    return options


def main(options):
    net = readNet(options.network)
    nodes = net.getNodes()
    nodesDict = {}
    neighbors = []
    numNodes = len(nodes)
    numUndirectedEdges = 0
    for i in range(numNodes):
        nodesDict.update({nodes[i] : i})
        neighs = nodes[i].getNeighboringNodes()
        for n in neighs:
            if n not in nodesDict:
                numUndirectedEdges+=1
        neighbors.append(neighs)

    # write metis input file
    with codecs.open("metisInputFile", 'w', encoding='utf8') as f:
        f.write("%s %s\n" % (numNodes, numUndirectedEdges))
        for neighs in neighbors:
            f.write("%s\n" % (" ".join([str(i+1) for i in [nodesDict[n] for n in neighs]])))

    # execute metis
    subprocess.call(["gpmetis", "-objtype=vol", "-contig", "metisInputFile", options.parts])

    # get edges corresponding to partitions
    edges = [set() for _ in range(int(options.parts))]
    curr = 0
    with codecs.open("metisInputFile.part."+options.parts, 'r', encoding='utf8') as f:
        for line in f:
            part = int(line)
            nodeEdges = nodes[curr].getIncoming() + nodes[curr].getOutgoing()
            for e in nodeEdges:
                if e.getID() not in edges[part]:
                    edges[part].add(e.getID())
            curr+=1

    # write edges of partitions in separate files
    for i in range(len(edges)):
        with codecs.open("edgesPart"+str(i), 'w', encoding='utf8') as f:
            for eID in edges[i]:
                f.write("%s\n" % (eID))



if __name__ == "__main__":
    main(get_options())
