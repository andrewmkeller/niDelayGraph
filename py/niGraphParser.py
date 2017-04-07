import xml.etree.ElementTree
import os

import niGraph


def parseGraphMlFile(filePath):
    assert os.path.isfile(filePath)
    
    it = xml.etree.ElementTree.iterparse(filePath)
    for _, el in it:
        if '}' in el.tag:
            el.tag = el.tag.split('}', 1)[1]  # strip all namespaces
    root = it.root
    
    xmlGraph = root.find("graph")
    
    g = niGraph.NIGraph()
    
    for xmlVertex in xmlGraph.findall("node"):
        
        vId = xmlVertex.attrib["id"]
        assert vId not in g.vertices
        
        
        for dataNode in xmlVertex.findall("data"):
            s = dataNode.attrib["key"]
            if s == "VertexId":
                vertexId = int(dataNode.text)
            elif s == "NodeUniqueId":
                nodeUniqueId = int(dataNode.text)
            elif s == "ThroughputCostIfRegistered":
                throughputCostIfRegistered = int(dataNode.text)
            elif s == "LatencyCostIfRegistered":
                latencyCostIfRegistered = int(dataNode.text)
            elif s == "RegisterCostIfRegistered":
                registerCostIfRegistered = int(dataNode.text)
            elif s == "IsRegistered":
                isRegistered = (dataNode.text == "True")
            elif s == "IsInputTerminal":
                isInputTerminal = (dataNode.text == "True")
            elif s == "IsOutputTerminal":
                isOutputTerminal = (dataNode.text == "True")
            elif s == "DisallowRegister":
                disallowRegister = (dataNode.text == "True")
            elif s == "NodeType":
                nodeType = int(dataNode.text)
            else:
                assert False
        
        vertex = niGraph.NIGraphVertex(vertexId, nodeUniqueId, throughputCostIfRegistered, latencyCostIfRegistered, registerCostIfRegistered, isRegistered, isInputTerminal, isOutputTerminal, disallowRegister, nodeType)
        
        g.vertices[vId] = vertex
        
    for xmlEdge in xmlGraph.findall("edge"):
        sourceId = xmlEdge.attrib["source"]
        source = g.vertices[sourceId]
                
        targetId = xmlEdge.attrib["target"]
        target = g.vertices[targetId]
        
        for dataNode in xmlEdge.findall("data"):
            s = dataNode.attrib["key"]
            if s == "Delay":
                delay = int(dataNode.text)
            elif s == "IsFeedback":
                isFeedback = (dataNode.text == "True")
            else:
                print(s)
                assert False
    
        
        edge = niGraph.NIGraphEdge(source, target, delay, isFeedback)
        eId = xmlEdge.attrib["id"]
        assert eId not in g.edges
        g.edges[eId] = edge
        
        edge.source.outEdges.append(edge)
        edge.target.inEdges.append(edge)
        
    # Check consistency
    for xmlVertex in xmlGraph.findall("node"):
        vertex = g.vertices[xmlVertex.attrib["id"]]
        if len(vertex.inEdges) != int(xmlVertex.attrib["parse.indegree"]):
            print(xmlVertex.attrib["id"])
            print(len(vertex.inEdges))
            print(int(xmlVertex.attrib["parse.indegree"]))
        assert len(vertex.inEdges) == int(xmlVertex.attrib["parse.indegree"])
        assert len(vertex.outEdges) == int(xmlVertex.attrib["parse.outdegree"])
        
    return g


