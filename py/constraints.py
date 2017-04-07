import niGraphParser
import niGraph
import csv

import sys
import glob
import os
import random
import time
import re
from matplotlib import pyplot
import networkx as nx
import networkx.algorithms.shortest_paths.dense as dense
import xml.etree.ElementTree as ET
import networkx.algorithms.simple_paths as nxPaths

graphsDir = "..\DataSets"



def visit(node, sortedNodes, temp, perm, unmarked):
    if node in temp:
        raise Exception
    if node in unmarked:
        unmarked.remove(node)
        temp.add(node)
        for outEdge in (e for e in node.outEdges if not e.isFeedback):
            nextNode = outEdge.target
            visit(nextNode, sortedNodes, temp, perm, unmarked)
        temp.remove(node)
        perm.add(node)
#         sortedNodes.insert(0, node)
        sortedNodes.append(node)


def topologicalSort(graph):
    sortedNodes = []
    tempMarks = set()
    permMarks = set()
    unmarkedVertices = set(graph.getVertices())

    while len(unmarkedVertices):
        node = next(iter(unmarkedVertices))
        visit(node, sortedNodes, tempMarks, permMarks, unmarkedVertices)

    return sortedNodes

def longestPathFromTopolSort(sortedNodes):
    pathLength = {}
    for node in sortedNodes:
        pathLength[node] = 0

    for node in sortedNodes:
        for inEdge in (e for e in node.inEdges if not e.isFeedback):
            prevNode = inEdge.source

            pathLength[prevNode] = max(pathLength[prevNode], pathLength[node] + 1)

    return max(pathLength.values())

def createDOT(graph, filePath):
    fp = open(filePath, "w")
    fp.write("strict digraph {\n");
    for edge in graph.getEdges():
        fp.write("n" + str(edge.source.vertexId) + " -> n" + str(edge.target.vertexId) + "\n")
    fp.write("}\n")

def calculateDAGPolarPaths(graph, limit=None):
    sortedVertices = topologicalSort(graph)

    possiblePaths = {};
    totalPaths = 0;
    for node in sortedVertices:
        #print (node.vertexId)
        sum = 0;
        if len(node.outEdges) == 0:
            #print ("terminal")
            possiblePaths[node] = 1;
        else:
            #print("non-terminal");
            for edge in node.outEdges:
                if not edge.isFeedback:
                    sum = sum + possiblePaths[edge.target]
            possiblePaths[node] = sum;
        if len(node.inEdges) == 0:
            totalPaths = totalPaths + sum;
        if limit and totalPaths > limit:
            return -1
    return totalPaths

def processPath(path, targetPeriod):
    pass

def processCycle(cycle, targetPeriod):
    pass

def main():
    sys.setrecursionlimit(10000)
#     return

    #graphs = glob.glob(os.path.join(graphsDir, "*.graphml"))

    #graphs.sort(key = lambda x: int(re.match(".*DelayGraph_(\d+)\.graphml", x).group(1)))
    #graphs = graphs[0:500]

    #graph = niGraphParser.parseGraphMlFile(graphs[0])
    #createDOT(graph, "graph0.dot")

    graphs = []

    with open("feasability.csv", 'r') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            graphs.append(row[0])

    sizes = []
    times = []

    fp = open("constraints.csv","w")

    for graphPath in graphs:
        graph = niGraphParser.parseGraphMlFile(graphPath)

        pathMatch = re.match("(.*)DelayGraph(_\d+)\.graphml", graphPath)

        originalTargetPath = pathMatch.group(1) + "OriginalGoals" + \
                                pathMatch.group(2) + ".xml"

        print(originalTargetPath)

        tree = ET.parse(originalTargetPath)
        root = tree.getroot()

        targetPeriod = int(root.find('TargetClockPeriodInPicoSeconds').text)

        sourceNode = len(graph.vertices)
        sinkNode = sourceNode + 1

        D=nx.DiGraph()
        for edge in graph.getEdges():
            D.add_edge(edge.source.vertexId,edge.target.vertexId, weight=edge.delay)

        for vertex in graph.getVertices():
            if len(vertex.inEdges) == 0:
                D.add_edge(sourceNode,vertex.vertexId)
            if len(vertex.outEdges) == 0:
                D.add_edge(vertex.vertexId, sinkNode)

        startTime = time.time()

        # for all paths
        print("Paths:")
        pathsCount = 0
        for path in nx.all_simple_paths(D, sourceNode, sinkNode):
            pathsCount = pathsCount + 1
            processPath(path, targetPeriod)
            print(path)

        print("Count:" + str(pathsCount))

        cycleCount = 0
        cyclesLimit = 1000;
        limitReached = False;
        print("Cycles:")
        for cycle in nx.simple_cycles(D):
            processCycle(cycle, targetPeriod)
            print(cycle)
            cycleCount = cycleCount + 1;
            if cyclesLimit < cycleCount:
                limitReached = True
                break

        if limitReached:
            print ("Too many simple_cycles!")
            continue
        print ("Count" + str(cycleCount))

        totalPaths = pathsCount

        durationTime = time.time() - startTime

        sizes.append(len(graph.getVertices()) + len(graph.getEdges()))
        times.append(durationTime)

        #longestPath = longestPathFromTopolSort(sortedVertices)

        fp.write(graphPath + ",")
        fp.write(str(len(graph.vertices))+",")
        fp.write(str(len(graph.edges))+",")
        fp.write(str(len(graph.getVertices()) + len(graph.getEdges())) + ",")
        fp.write(str(durationTime) + ",")
        fp.write(str(totalPaths) + ",")
        fp.write(str(cycleCount))
        fp.write("\n")
        print (graphPath, len(graph.vertices), len(graph.edges), totalPaths)
        break
    fp.close()
#         print("Topological sort took " + str(durationTime) + " seconds")

#    pyplot.loglog(sizes,  times, "o")
#     pyplot.xscale("log")
#     pyplot.yscale("log")
#    pyplot.show()


if __name__ == "__main__":
    main()
