import niGraphParser
import niGraph

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

def calculateDAGPolarPaths(graph):
    sortedVertices = topologicalSort(graph)

    possiblePaths = {};
    totalPaths = 0;
    for node in sortedVertices:
        #print (node.vertexId)
        if len(node.outEdges) == 0:
            #print ("terminal")
            possiblePaths[node] = 1;
        else:
            #print("non-terminal");
            sum = 0;
            for edge in node.outEdges:
                if not edge.isFeedback:
                    sum = sum + possiblePaths[edge.target]
            possiblePaths[node] = sum;
        if len(node.inEdges) == 0:
            totalPaths = totalPaths + sum;
    return totalPaths

def countFeedbackCyclesRecursive(source, target, paths):
    if source == target:
        return 1
    if source in paths:
        return paths[source]
    else:
        sum = 0
        for edge in source.outEdges:
            if not edge.isFeedback:
                sum += countFeedbackCyclesRecursive(edge.target, target, paths)
        paths[source] = sum
        return sum

def countFeedbackCycles(source, target):
    paths = {}
    return countFeedbackCyclesRecursive(source, target, paths)

def main():
    sys.setrecursionlimit(10000)
#     return

    graphs = glob.glob(os.path.join(graphsDir, "*.graphml"))

    graphs.sort(key = lambda x: int(re.match(".*DelayGraph_(\d+)\.graphml", x).group(1)))
    graphs = graphs[23:24]

    graph = niGraphParser.parseGraphMlFile(graphs[0])
    createDOT(graph, "graph0.dot")

    sizes = []
    times = []

    fp = open("522r_asst1_feedback_count.csv","w")

    for graphPath in graphs:
        graph = niGraphParser.parseGraphMlFile(graphPath)

        pathMatch = re.match("(.*)DelayGraph(_\d+)\.graphml", graphPath)

        originalTargetPath = pathMatch.group(1) + "OriginalGoals" + \
                                pathMatch.group(2) + ".xml"

        print(originalTargetPath)

        tree = ET.parse(originalTargetPath)
        root = tree.getroot()

        targetPeriod = int(root.find('TargetClockPeriodInPicoSeconds').text)

        startTime = time.time()

        for edge in graph.getEdges():
            if edge.isFeedback:
                print (edge.source.vertexId, "->", edge.target.vertexId)
                print(countFeedbackCycles(edge.target, edge.source))
        #break
    fp.close()
#         print("Topological sort took " + str(durationTime) + " seconds")

#    pyplot.loglog(sizes,  times, "o")
#     pyplot.xscale("log")
#     pyplot.yscale("log")
#    pyplot.show()


if __name__ == "__main__":
    main()
