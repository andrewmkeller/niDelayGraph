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
    # Registering the first or last node in a path will not solve
    # time-step boundaries.
    results = []
    scan = deque()
    totaldelay = 0
    totalNodes = len(path)
    # first source is the first node
    scan.append(path[0])
    for i in range(1,totalNodes):
        source = scan[-1]
        sink = path[i]
        # check all possible cylces within a cycle by only allow end nodes to
        # match. For example if you have cycle A-B-C-D-E-F-G, you can check
        # C-D-E-F-G-A-B-C for time step boundary, but don't check
        # C-D-E-F-G-A-B-C-D or beyond because these cycles do not exist
        if len(scan) > 1 and sink == scan[1]:
            #print("detected a cycle!")
            scan.popleft()
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
                #if not graph.vertices["n"+str(scan[j])].isRegistered and \
                #    not graph.vertices["n"+str(scan[j])].disallowRegister:
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
    abort = False
    for i in cycle:
        if graph.vertices["n"+str(i)].isRegistered:
            abort = True
            break
        if not graph.vertices["n"+str(i)].disallowRegister:
            result.append(i)
    if len(result) > 0 and not abort:
        results.append(frozenset(result))
    elif len(result) == 0 and not abort:
        print("Cycle found that cannot be broken!")
    # make sure that any path along the cycle does not exceed cycle time
    pathswithin = processPath(D, graph, cycle + cycle, targetPeriod)
    for pathwithin in pathswithin:
        results.append(pathwithin)
    return results

def gurobiSolve(solutionFile, graph, constraintsSet, cycles, paths):
    result = []
    if len(constraintsSet) == 0:
        return result
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

    # I should put this next step into its own function...
    # Taking care of siblings
    constrVars = list(GRBVars.keys())
    siblingSet = set()
    for constrVar in constrVars:
        if constrVar not in siblingSet:
            siblingSet.add(constrVar)
            currentVertex = graph.vertices["n"+str(constrVar)]
            nodeId = currentVertex.nodeUniqueId
            siblings = []
            for candidate in graph.getVertices():
                if candidate.nodeUniqueId != nodeId or candidate.isRegistered or \
                    candidate.isInputTerminal != currentVertex.isInputTerminal or \
                    candidate == currentVertex:
                    continue
                siblings.append(candidate.vertexId)
            if len(siblings) > 0:
                GRBVar = GRBVars[constrVar]
                for sibling in siblings:
                    siblingSet.add(sibling)
                    siblingVar = None
                    if sibling in GRBVars:
                        siblingVar = GRBVars[sibling]
                    else:
                        siblingVar = m.addVar(vtype=GRB.BINARY, name="x"+str(sibling))
                        GRBVars[sibling] = siblingVar
                        expr = LinExpr()
                        expr += 1.0*siblingVar
                        expr += -1.0*GRBVar
                        m.addConstr(expr, GRB.EQUAL, 0.0, "c"+str(constraintCount))
                        constraintCount += 1


    obj = LinExpr()
    default = False
    GRBVarsSet = set(GRBVars.keys())

    # optimizing throughput
    throuputConstrCount = 0
    throughputConstrVars = []
    for cycle in cycles:
        fixedCost = 0
        expr = LinExpr()
        expressionCount = 0
        for node in cycle:
            if node not in GRBVarsSet:
                if graph.vertices["n"+str(node)].isRegistered:
                    fixedCost += graph.vertices["n"+str(node)].throughputCostIfRegistered
            else:
                cost = graph.vertices["n"+str(node)].throughputCostIfRegistered
                if cost > 0:
                    expr += cost*GRBVars[node]
                    expressionCount += 1
        if expressionCount > 0 or fixedCost > 0:
            throughputConstr = m.addVar(name="t"+str(throuputConstrCount))
            expr += -1.0*throughputConstr
            m.addConstr(expr, GRB.EQUAL, -fixedCost, "tc"+str(throuputConstrCount))
            throughputConstrVars.append(throughputConstr)
            throuputConstrCount += 1
    tMax = m.addVar(name="tmax")
    if throuputConstrCount > 0:
        count = 0
        for throughputConstrVar in throughputConstrVars:
            expr = LinExpr()
            expr += 1.0*throughputConstrVar
            expr += -1.0*tMax
            m.addConstr(expr, GRB.LESS_EQUAL, 0.0, "tmaxc"+str(count))
            count += 1
        #obj += 1.0*tMax
    else:
        expr = LinExpr()
        expr += 1.0*tMax
        m.addConstr(expr, GRB.EQUAL, 0.0, "tmaxc")

    # Path restrictions
    latencyConstrCount = 0
    latencyConstrVars = []
    for path in paths:
        #print(path)
        fixedCost = 0
        expr = LinExpr()
        expressionCount = 0
        for node in path:
            if node not in GRBVarsSet:
                if graph.vertices["n"+str(node)].isRegistered:
                    fixedCost += graph.vertices["n"+str(node)].latencyCostIfRegistered
            else:
                cost = graph.vertices["n"+str(node)].latencyCostIfRegistered
                if cost > 0:
                    expr += cost*GRBVars[node]
                    expressionCount += 1
        if expressionCount > 0 or fixedCost > 0:
            latencyConstr = m.addVar(name="l"+str(latencyConstrCount))
            expr += -1.0*latencyConstr
            m.addConstr(expr, GRB.EQUAL, -fixedCost, "lc"+str(latencyConstrCount))
            latencyConstrVars.append(latencyConstr)
            latencyConstrCount += 1
    lMax = m.addVar(name="lmax")
    if latencyConstrCount > 0:
        count = 0
        for latencyConstrVar in latencyConstrVars:
            expr = LinExpr()
            expr += 1.0*latencyConstrVar
            expr += -1.0*lMax
            m.addConstr(expr, GRB.LESS_EQUAL, 0.0, "lmaxc"+str(count))
            count += 1
    else:
        expr = LinExpr()
        expr += 1.0*lMax
        m.addConstr(expr, GRB.EQUAL, 0.0, "lmaxc")

    m.NumObj = 3
    m.setParam(GRB.Param.ObjNumber, 0)
    m.ObjNName = "throughput"
    m.ObjNPriority = 2
    m.setAttr(GRB.Attr.ObjN,list([tMax]),[1])
    m.setParam(GRB.Param.ObjNumber, 1)
    m.ObjNName = "latency"
    m.ObjNPriority = 1
    m.setAttr(GRB.Attr.ObjN,list([lMax]),[1])
    m.setParam(GRB.Param.ObjNumber, 2)
    m.ObjNName = "registerCost"
    obj = LinExpr()

    registerVars = []
    registerCosts = []
    for vertex, variable in GRBVars.items():
        # obj += (graph.vertices["n"+str(vertex)].registerCostIfRegistered)*variable
        # total regeristers assigned optimization
        registerVars.append(variable)
        registerCosts.append(graph.vertices["n"+str(vertex)].registerCostIfRegistered)
    m.setAttr(GRB.Attr.ObjN,registerVars,registerCosts)
    m.setObjective(obj)
    m.ModelSense = GRB.MINIMIZE
    m.write(solutionFile)
    m.optimize()
    if m.status == GRB.Status.INFEASIBLE:
        # Turn presolve off to determine whether model is infeasible
        # or unbounded
        print('')
        print('Model is infeasible')
        m.computeIIS()
        m.write("model.ilp")
        print("IIS written to file 'model.ilp'")

    result = []
    for v in m.getVars():
        #print(v.varName, v.x)
        if v.varName[0:1] != "x":
            continue
        if v.x > 0.0:
            result.append(int(v.varName[1:]))
    #print('Obj:', m.objVal)
    return result

def saveResults(solutionPath, registerAssignments, durationTime):
    fi = open(solutionPath, "w")
    fi.write('<?xml version="1.0" encoding="utf-8" standalone="yes"?>\n')
    fi.write('<Root>\n')
    fi.write('  <processingTimeInMilliseconds>' + str(durationTime) + '</processingTimeInMilliseconds>\n')
    fi.write('  <RegisterAssignments>\n')
    for register in registerAssignments:
        fi.write('    <VertexIdToRegister>' + str(register) + '</VertexIdToRegister>\n')
    fi.write('  </RegisterAssignments>\n')
    fi.write('</Root>\n')
    fi.close()

def paths_from_node(G, source):
    visited = [source]
    stack = [iter(G[source])]
    #print("finding paths from", source)
    while stack:
        children = stack[-1]
        child = next(children, None)
        #print("at child", child)
        if child is None:
            stack.pop()
            visited.pop()
        else:
            if G.node[child]['registered'] or G.out_degree(child) == 0:
                yield visited + [child]
            elif child in visited:
                yield visited
            else:
                visited.append(child)
                stack.append(iter(G[child]))

def get_input_output_paths(G):
    for node in G.nodes_iter():
        if G.in_degree(node) > 0:
            continue
        visited = [node]
        stack = [iter(G[node])]
        #print("finding paths from", source)
        while stack:
            children = stack[-1]
            child = next(children, None)
            #print("at child", child)
            if child is None:
                stack.pop()
                visited.pop()
            else:
                if G.out_degree(child) == 0:
                    yield visited + [child]
                elif child not in visited:
                    visited.append(child)
                    stack.append(iter(G[child]))

def main():
    sys.setrecursionlimit(10000)
#     return

    #graphs = glob.glob(os.path.join(graphsDir, "*.graphml"))

    #graphs.sort(key = lambda x: int(re.match(".*DelayGraph_(\d+)\.graphml", x).group(1)))


    #graph = niGraphParser.parseGraphMlFile(graphs[0])
    #createDOT(graph, "graph0.dot")

    #graphs = []

    #with open("constraints_rebuild.csv", 'r') as csvfile:
    #    spamreader = csv.reader(csvfile)
    #    for row in spamreader:
    #        graphs.append(row[0])

    graphs = glob.glob(os.path.join(graphsDir, "*.graphml"))
    graphs.sort(key = lambda x: int(re.match(".*DelayGraph_(\d+)\.graphml", x).group(1)))

    graphPreviouslyProcessed = glob.glob(os.path.join("../oldAFAPsolutions","*.afap.xml"))
    graphPreviouslyProcessedSet = set()
    for graph in graphPreviouslyProcessed:
        graphPreviouslyProcessedSet.add(int(re.match(".*_(\d+)\.afap.xml", graph).group(1)))

    #graphs = graphs[int(sys.argv[1]):int(sys.argv[1])+1]
    #graphPreviouslyProcessedSet = set([1017])
    sizes = []
    times = []

    fp = open("constraints_rebuild.csv","a")
    #fp.write("Path,# vertices,# edges,# paths,# path constr,# unique path constr,path time,# cycles,# cycles constr,# unique cycle constr,cycle time,gurobi time,duration time\n")

    for graphPath in graphs:

        pathMatch = re.match("(.*)DelayGraph(_\d+)\.graphml", graphPath)

        originalTargetPath = pathMatch.group(1) + "OriginalGoals" + \
                                pathMatch.group(2) + ".xml"

        if int(pathMatch.group(2)[1:]) not in graphPreviouslyProcessedSet or \
            int(pathMatch.group(2)[1:]) < 1564:
            continue

        graph = niGraphParser.parseGraphMlFile(graphPath)

        solutionPath = "..\\Solutions\\Solution" + pathMatch.group(2) + ".afap.xml"

        gurobiFile = "Gurobi" + pathMatch.group(2) + ".lp"

        print(originalTargetPath)

        tree = ET.parse(originalTargetPath)
        root = tree.getroot()

        targetPeriod = int(root.find('TargetClockPeriodInPicoSeconds').text)

        maxDelay = 0
        for edge in graph.getEdges():
            if edge.delay > maxDelay:
                maxDelay = edge.delay

        if maxDelay > targetPeriod:
            targetPeriod = maxDelay

        D=nx.DiGraph()
        for edge in graph.getEdges():
            D.add_edge(edge.source.vertexId,edge.target.vertexId, weight=edge.delay)

        F=nx.DiGraph()
        for edge in graph.getEdges():
            if not edge.isFeedback:
                F.add_edge(edge.source.vertexId,edge.target.vertexId, weight=edge.delay)

        for vertex in graph.getVertices():
            D.add_node(vertex.vertexId,registered=vertex.isRegistered)
            F.add_node(vertex.vertexId,registered=vertex.isRegistered,isInputTerminal=vertex.isInputTerminal)

        startTimePaths = time.time()
        constraintsSet = set()
        pathConstraintsCount = 0;
        # for all paths
        pathsCount = 0
        abort = False
        for vertex in D.nodes(data=True):
            if (D.in_degree(vertex[0]) == 0 or vertex[1]['registered']):
                #print (vertex)
                for path in paths_from_node(D, vertex[0]):
                    pathsCount = pathsCount + 1
                    if pathsCount > 10000:
                        abort = True
                        break
                    #print(path)
                    pathConstraints = processPath(D, graph, path, targetPeriod)
                    for pathConstraint in pathConstraints:
                        #print ("C:", pathConstraint)
                        constraintsSet.add(pathConstraint)
                        pathConstraintsCount = pathConstraintsCount + 1
        if abort:
            print ("To many paths!")
            continue
        stopTimePaths = time.time()
        print("Paths:")
        print("Count: " + str(pathsCount))
        print("pathConstraintsCount =", pathConstraintsCount)
#        for constraint in constraintsSet:
#            print(constraint)
        print ("uniqueConstraints count =", len(constraintsSet))
        constraintsBeforeCycle = len(constraintsSet)
        startTimeCycles = time.time()
        cycleCount = 0
        cyclesLimit = 10000;
        cycleConstraintsCount = 0
        limitReached = False;
        cycles = []
        for cycle in nx.simple_cycles(D):
            #print(cycle)
            cycles.append(cycle)
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
        stopTimeCycles = time.time()
        print("Cycles:")
        print ("Count: " + str(cycleCount))
        print("CycleConstaints = ", cycleConstraintsCount)
        #for constraint in constraintsSet:
        #    print(constraint)
        print ("uniqueConstraints count =", len(constraintsSet) - constraintsBeforeCycle)

        input_output_paths_iter = get_input_output_paths(F)


        input_output_paths = []
        inout_count = 0

        for path in input_output_paths_iter:
            input_output_paths.append(path)
            inout_count += 1
            if inout_count > 10000:
                abort = True
        if abort:
            print ("To many in_out paths!")
            continue

        print("Number of input/output paths", len(input_output_paths))

        startTimeGurobi = time.time()
        registerAssignments = gurobiSolve(gurobiFile, graph, constraintsSet, cycles, input_output_paths)
        stopTimeGurobi = time.time()

        pathsTime = stopTimePaths - startTimePaths
        cyclesTime = stopTimeCycles - startTimeCycles
        gurobiTime = stopTimeGurobi - startTimeGurobi

        durationTime = pathsTime + cyclesTime + gurobiTime

        saveResults(solutionPath, registerAssignments, durationTime * 1000)

        sizes.append(len(graph.getVertices()) + len(graph.getEdges()))
        times.append(durationTime)

        #longestPath = longestPathFromTopolSort(sortedVertices)

        fp.write(graphPath + ",")

        fp.write(str(len(graph.vertices))+",")
        fp.write(str(len(graph.edges))+",")

        fp.write(str(pathsCount) + ",")
        fp.write(str(pathConstraintsCount) + ",")
        fp.write(str(constraintsBeforeCycle) + ",")
        fp.write(str(pathsTime * 1000) + ",")

        fp.write(str(cycleCount) + ",")
        fp.write(str(cycleConstraintsCount) + ",")
        fp.write(str(len(constraintsSet) - constraintsBeforeCycle) + ",")
        fp.write(str(cyclesTime * 1000) + ",")

        fp.write(str(gurobiTime * 1000) + ",")



        fp.write(str(durationTime * 1000) + ",")
        fp.write("\n")
        fp.flush()
        #print (graphPath, len(graph.vertices), len(graph.edges), totalPaths)
        #break
    fp.close()
#         print("Topological sort took " + str(durationTime) + " seconds")

#    pyplot.loglog(sizes,  times, "o")
#     pyplot.xscale("log")
#     pyplot.yscale("log")
#    pyplot.show()


if __name__ == "__main__":
    main()
