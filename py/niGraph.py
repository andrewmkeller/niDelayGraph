class NIGraphEdge:
    def __init__(self, source, target, delay, isFeedback):
        self.source = source
        self.target = target        
        self.delay = delay
        self.isFeedback = isFeedback
        

class NIGraphVertex:
    def __init__(self, vertexId, nodeUniqueId, throughputCostIfRegistered, latencyCostIfRegistered, registerCostIfRegistered, isRegistered, isInputTerminal, isOutputTerminal, disallowRegister, nodeType):
        self.vertexId = vertexId
        self.nodeUniqueId = nodeUniqueId
        self.throughputCostIfRegistered = throughputCostIfRegistered
        self.latencyCostIfRegistered = latencyCostIfRegistered
        self.registerCostIfRegistered = registerCostIfRegistered
        self.isRegistered = isRegistered
        self.isInputTerminal = isInputTerminal
        self.isOutputTerminal = isOutputTerminal
        self.disallowRegister = disallowRegister
        self.nodeType = nodeType
        
        self.outEdges = []
        self.inEdges = []

        # Consistency checking        
        #assert not (self.isRegistered and self.disallowRegister)
        
class NIGraph:
    def __init__(self):
        self.vertices = {}
        self.edges = {}


    def printStats(self):
        print("Number of vertices: " + str(len(self.vertices)))
        
    def getNumVertices(self):
        return len(self.vertices)
    
    def getVertices(self):
        return self.vertices.values()
    
    def getEdges(self):
        return self.edges.values()