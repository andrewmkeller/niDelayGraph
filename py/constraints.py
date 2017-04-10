import niGraphParser
import niGraph
import csv
from collections import deque

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

from gurobipy import *

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

def processPath(G, graph, path, targetPeriod):
    # the first and last nodes in a path are the polar sinks and source of the
    # graph. Registering the second and second to last nodes will not solve
    # time-step boundaries.
    results = []
    scan = deque()
    totaldelay = 0
    totalNodes = len(path)
    # first source is the second node
    scan.append(path[1])
    for i in range(2,totalNodes - 1):
        source = scan[-1]
        sink = path[i]
        if sink == scan[0]:
            break
        delay = G.get_edge_data(source, sink)['weight']
        totaldelay = totaldelay + delay
        scan.append(sink)
        if totaldelay > targetPeriod:
            #print("Path longer than targetPeriod", targetPeriod)
            #print(scan)
            firstEdgeWeight = G.get_edge_data(scan[0],scan[1])['weight']
            #print("first edge weight =", firstEdgeWeight)
            while totaldelay - firstEdgeWeight > targetPeriod:
                #print("edge not needed, removing from path")
                totaldelay = totaldelay - firstEdgeWeight
                scan.popleft()
                firstEdgeWeight = G.get_edge_data(scan[0],scan[1])['weight']
                #print ("next edge weight =",firstEdgeWeight)
            result = []
            #print("Target path =", scan)
            scanlength = len(scan)
            for j in range(1, scanlength - 1):
                if not graph.vertices["n"+str(scan[j])].isRegistered and \
                    not graph.vertices["n"+str(scan[j])].disallowRegister:
                    result.append(scan[j])
            if len(result) > 0:
                #print(result)
                results.append(frozenset(result))
        if graph.vertices["n"+str(path[i])].isRegistered:

            scan.clear()
            scan.append(path[i])
            totaldelay = 0
    #print("Total constraints found in path =", len(results))
    return results

def processCycle(D, graph, cycle, targetPeriod):
    results = []
    # make sure there is atleast 1 register in the entire cycle
    result = []
    for i in cycle:
        if not graph.vertices["n"+str(i)].isRegistered and \
            not graph.vertices["n"+str(i)].disallowRegister:
            result.append(i)
    if len(result) > 0:
        results.append(frozenset(result))
    else:
        print("Cycle found that cannot be broken!")
    # make sure that any path along the cycle does not exceed cycle time
    pathswithin = processPath(D, graph, cycle + cycle, targetPeriod)
    for pathwithin in pathswithin:
        results.append(pathwithin)
    return results

def gurobiSolve(solutionPath, graph, constraintsSet):
    GRBVars = {}
    m = Model("reg")
    constraintCount = 0
    for constraint in constraintsSet:
        expr = LinExpr()
        for vertex in constraint:
            variable = None
            if vertex in GRBVars:
                variable = GRBVars[vertex]
            else:
                variable = m.addVar(vtype=GRB.BINARY, name="x"+str(vertex))
                GRBVars[vertex] = variable
            expr += 1.0*variable
            #(graph.verticies["n"+str(vertex)].registerCostIfRegistered)
        m.addConstr(expr, GRB.GREATER_EQUAL, 1.0, "c"+str(constraintCount))
        constraintCount += 1
    obj = LinExpr()
    for variable in GRBVars.values():
        obj += (graph.vertices["n"+str(vertex)].registerCostIfRegistered)*variable
    m.setObjective(obj, GRB.MINIMIZE)
    m.optimize()
    for v in m.getVars():
        print(v.varName, v.x)
    print('Obj:', m.objVal)


def main():
    sys.setrecursionlimit(10000)
#     return

    #graphs = glob.glob(os.path.join(graphsDir, "*.graphml"))

    #graphs.sort(key = lambda x: int(re.match(".*DelayGraph_(\d+)\.graphml", x).group(1)))


    #graph = niGraphParser.parseGraphMlFile(graphs[0])
    #createDOT(graph, "graph0.dot")

    graphs = []

    with open("feasability.csv", 'r') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            graphs.append(row[0])

    #graphs = graphs[0:3]

    sizes = []
    times = []

    fp = open("constraints.csv","w")

    for graphPath in graphs:
        graph = niGraphParser.parseGraphMlFile(graphPath)

        pathMatch = re.match("(.*)DelayGraph(_\d+)\.graphml", graphPath)

        originalTargetPath = pathMatch.group(1) + "OriginalGoals" + \
                                pathMatch.group(2) + ".xml"

        solutionPath = "..\\Solutions\\Solution_" + pathMatch.group(2) + ".afap.xml"

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
        constraintsSet = set()
        pathConstraintsCount = 0;
        # for all paths
        print("Paths:")
        pathsCount = 0
        for path in nx.all_simple_paths(D, sourceNode, sinkNode):
            pathsCount = pathsCount + 1
            #print(path)
            pathConstraints = processPath(D, graph, path, targetPeriod)
            for pathConstraint in pathConstraints:
                constraintsSet.add(pathConstraint)
                pathConstraintsCount = pathConstraintsCount + 1
        print("Count: " + str(pathsCount))
        for constraint in constraintsSet:
            print(constraint)
        print ("Constraints count =", len(constraintsSet), pathConstraintsCount)

        cycleCount = 0
        cyclesLimit = 1000;
        cycleConstraintsCount = 0
        limitReached = False;
        print("Cycles:")
        for cycle in nx.simple_cycles(D):
            #print(cycle)
            cycleConstraints = processCycle(D, graph, cycle, targetPeriod)
            cycleCount = cycleCount + 1;
            if cyclesLimit < cycleCount:
                limitReached = True
                break
            for cycleConstraint in cycleConstraints:
                constraintsSet.add(cycleConstraint)
                cycleConstraintsCount = cycleConstraintsCount + 1

        if limitReached:
            print ("Too many simple_cycles!")
            continue
        print ("Count" + str(cycleCount))
        for constraint in constraintsSet:
            print(constraint)
        print ("Constraints count =", len(constraintsSet), cycleConstraintsCount)

        totalPaths = pathsCount


        gurobiSolve(solutionPath, graph, constraintsSet)

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
        #break
    fp.close()
#         print("Topological sort took " + str(durationTime) + " seconds")

#    pyplot.loglog(sizes,  times, "o")
#     pyplot.xscale("log")
#     pyplot.yscale("log")
#    pyplot.show()


if __name__ == "__main__":
    main()
