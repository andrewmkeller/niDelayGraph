#!/usr/bin/python3

import sys
import niGraphParser
import os
import re
import pickle
import matplotlib.pyplot as plt
import pylab as pl
import numpy as np

def getVertexCount(graphPath):
    g = niGraphParser.parseGraphMlFile(graphPath)
    return len(g.vertices)

def getRegisterCost(graphPath):
    g = niGraphParser.parseGraphMlFile(graphPath)
    return sum([v.registerCostIfRegistered for v in g.getVertices() if v.isRegistered])

def analyzeAllGraphs():
    assert(len(sys.argv) >= 2)    
    graphMlDirPath = sys.argv[1]
    
    graphMlFilePaths = [os.path.join(graphMlDirPath, f) for f in os.listdir(graphMlDirPath) if re.match("DelayGraph_\d+.graphml", f)]
     
#     graphMlFilePaths = graphMlFilePaths[:100]
     
    vertexCnt = []
    i = 1
    for graphPath in graphMlFilePaths:
        print ("Parsing " + str(i) + " of " + str(len(graphMlFilePaths)))
        vertexCnt.append(getVertexCount(graphPath))
        i += 1
        
    print(min(vertexCnt))
    print(sum(vertexCnt) / len(vertexCnt))
    print(max(vertexCnt))
    
    fp = open('vertexCnts.pkl', 'wb')
    pickle.dump(vertexCnt, fp)

def graphVertexCounts():
    fp = open("vertexCnts.pkl", 'rb')
    vertexCnt = pickle.load(fp)
    
    print(min(vertexCnt))
    print(sum(vertexCnt) / len(vertexCnt))
    print(max(vertexCnt))
    
    pl.hist(vertexCnt, bins=np.logspace(0, 6, 30))
    pl.gca().set_xscale("log")
    pl.ylabel("# Vertices")
    pl.show()
    
#     print(vertexCnt)
#     plt.hist(vertexCnt)
#     plt.show()
#     
 


def main():
    graphPath = sys.argv[1]
#     print (getVertexCount(graphPath))
    print(getRegisterCost(graphPath))
    #analyzeAllGraphs()
    
    #graphVertexCounts()
    
    
if __name__ == "__main__":
    main()