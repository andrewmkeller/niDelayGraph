using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using DelayGraph;
using RegisterPlacement.LatencyAssignment;

namespace RegisterPlacement
{
    public class Program
    {
        #region Public Methods

        public static void Main(string[] args)
        {
            Arguments parsedArguments;
            if (!TryParseCommandLine(args, out parsedArguments))
            {
                return;
            }

            var algorithms = new List<Tuple<string, ILatencyAssignment>>
            {
                new Tuple<string, ILatencyAssignment>("asap", new LatencyAssignmentAsap()),
                new Tuple<string, ILatencyAssignment>("greedy", new LatencyAssignmentGreedy()),
                // add your own latency assigner here
            };

            string dataSetsPath = Path.GetFullPath(parsedArguments.DataSetDirectory);
            List<DataSet> dataSets = new List<DataSet>();
            FindDataSetsRecursively(dataSetsPath, dataSets);

            if (!dataSets.Any())
            {
                Console.WriteLine("Warning: No data sets found. Make sure DelayGraph*.graphml and corresponding DelayGraphOriginalGoals*.xml files exist somewhere in this hierarchy: " + args[0]);
            }

            List<string> algorithmNames = algorithms.Select(kv => kv.Item1).ToList();
            Directory.CreateDirectory(parsedArguments.SolutionDirectory);

            var resultsSummary = InitializeResultsSummary(algorithmNames);
            Stopwatch sw = new Stopwatch();
            foreach (DataSet dataSet in dataSets)
            {
                foreach (var graphAndGoal in dataSet.GraphsAndGoalsFilePaths)
                {
                    DelayGraph.DelayGraph graph = DelayGraphGraphMlSerializer.DeserializeFromGraphMl(graphAndGoal.GraphPath);

                    XDocument doc = XDocument.Load(graphAndGoal.GoalPath);
                    XElement clockRateElement = doc.Root.Element("TargetClockPeriodInPicoSeconds");

                    int originalTargetClockRate = int.Parse(clockRateElement.Value);

                    if (DelayGraphSolution.PruneEdges(graph))
                    {
                        Console.WriteLine("Successfully removed redundant edges in graph");
                    }

                    int minClockPeriod = DelayGraphSolution.MinClockPeriod(graph);
                    if (minClockPeriod > originalTargetClockRate)
                    {
                        originalTargetClockRate = minClockPeriod;
                    }
                    foreach (var tuple in algorithms)
                    {
                        string algorithmName = tuple.Item1;
                        var algorithm = tuple.Item2;
                        sw.Restart();
                        HashSet<DelayGraphVertex> registeredTerminals = algorithm.Execute(graph, originalTargetClockRate);
                        double milliseconds = sw.Elapsed.TotalMilliseconds;
                        sw.Stop();

                        // writing solution XML
                        string uniqueGraphId = ParseOutUniquifier(Path.GetFileName(graphAndGoal.GraphPath));
                        string solutionSelectionFileName = "Solution" + uniqueGraphId + "." + algorithmName + ".xml";
                        string graphPathWithoutDataSetPath = Path.GetDirectoryName(graphAndGoal.GraphPath.Replace(dataSetsPath,""));
                        if (graphPathWithoutDataSetPath.StartsWith("\\"))
                            graphPathWithoutDataSetPath = graphPathWithoutDataSetPath.Replace("\\", "");
                        string destinationDirectory = Path.Combine(Path.GetFullPath(parsedArguments.SolutionDirectory), graphPathWithoutDataSetPath);
                        Directory.CreateDirectory(destinationDirectory);
                        string solutionSelectionPath = Path.Combine(destinationDirectory, solutionSelectionFileName);
                        using (var writer = new StreamWriter(path: solutionSelectionPath, append: false))
                        {
                            List<XElement> selectedNodes = new List<XElement>();
                            foreach (var registeredTerminal in registeredTerminals)
                            {
                                XElement el = new XElement("VertexIdToRegister", registeredTerminal.VertexId);
                                selectedNodes.Add(el);
                            }
                            XDocument solutionXML = new XDocument(
                                new XDeclaration("1.0", "utf-8", "yes"),
                                new XElement("Root",
                                    new XElement("processingTimeInMilliseconds", milliseconds.ToString("r")),
                                    new XElement("RegisterAssignments", selectedNodes.ToArray())
                                )
                                );
                            solutionXML.Save(writer);
                        }
                    }
                }
            }

            Console.WriteLine("Solutions written to: " + Path.GetFullPath(parsedArguments.SolutionDirectory));
            Debug.WriteLine("Solutions written to: " + Path.GetFullPath(parsedArguments.SolutionDirectory));
        }

        #endregion

        #region Private Methods

        private static string BuildExpectedGoalFileName(string uniquifierFromDelayGraphFileName)
        {
            return "OriginalGoals" + uniquifierFromDelayGraphFileName + ".xml";
        }

        private static void FindDataSetsRecursively(string root, List<DataSet> dataSets)
        {
            IList<string> graphFilePaths = Directory.EnumerateFiles(root, "DelayGraph*.graphml").ToList();

            if (graphFilePaths.Any())
            {
                List<GraphAndGoalPaths> graphsAndGoalsFilePaths = new List<GraphAndGoalPaths>();
                foreach (string graphFilePath in graphFilePaths)
                {
                    string graphFileName = Path.GetFileName(graphFilePath);
                    string uniquifier = ParseOutUniquifier(graphFileName);
                    string goalFileName = BuildExpectedGoalFileName(uniquifier);
                    IList<string> goalFilePaths = Directory.EnumerateFiles(root, goalFileName).ToList();
                    if (goalFilePaths.Count != 1)
                    {
                        Console.WriteLine("Error: Couldn't find DelayGraphOriginalGoals*.xml file for delay graph: " + graphFilePath);
                        continue;
                    }
                    GraphAndGoalPaths graghAndGoalPath = new GraphAndGoalPaths(graphFilePath, goalFilePaths.First());
                    graphsAndGoalsFilePaths.Add(graghAndGoalPath);
                }
                if (graphsAndGoalsFilePaths.Any())
                {
                    DataSet dataSet = new DataSet(root, graphsAndGoalsFilePaths);
                    dataSets.Add(dataSet);
                }
            }

            IEnumerable<string> subDirs = Directory.EnumerateDirectories(root);
            foreach (string subDir in subDirs)
            {
                FindDataSetsRecursively(subDir, dataSets);
            }
        }

        private static IDictionary<string, ResultInformation> InitializeResultsSummary(IList<string> algorithmNames)
        {
            var dictionary = new Dictionary<string, ResultInformation>();
            foreach (var algorithmName in algorithmNames)
            {
                dictionary[algorithmName] = new ResultInformation();
            }
            return dictionary;
        }

        private static string ParseOutUniquifier(string graphFileName)
        {
            string prependage = "DelayGraph";
            int dotLocationIndex = graphFileName.IndexOf('.');
            int midPieceIndex = prependage.Length;
            string middlePiece = graphFileName.Substring(midPieceIndex, dotLocationIndex - midPieceIndex);
            return middlePiece;
        }

        private static bool TryParseCommandLine(string[] args, out Arguments arguments)
        {
            arguments = new Arguments();
            if (args.Length != 2)
            {
                Console.WriteLine("Error: Expecting two arguments and not " + args.Length);
                Console.WriteLine("Usage: RegisterPlacement.exe <Data Set Directory Root> <Directory For Solutions>");
                Console.WriteLine(
                    "This program explores subdirectories of <Data Set Directory Root> to find delay graph data sets to try with register placement algorithms.");
                Console.WriteLine("Selection solutions are saved in the <Directory For Selection Solutions>");
                //Console.WriteLine("Various metrics are printed into the <Directory For Scorecard>.");
                return false;
            }
            if (!Directory.Exists(args[0]))
            {
                Console.WriteLine("Error: Directory does not exist: " + args[0]);
                return false;
            }
            arguments.SolutionDirectory = args[1];
            arguments.DataSetDirectory = args[0];
            return true;
        }

        #endregion

        #region Nested type: Arguments

        /// <summary>
        /// Simple container class for command line arguments after they're parsed to make it easier to keep track of.
        /// </summary>
        private class Arguments
        {
            #region Internal Properties

            /// <summary>
            /// The directory where the user has stored the data sets.
            /// </summary>
            internal string DataSetDirectory { get; set; }

            /// <summary>
            /// The directory where the user wants to save solutions from the run.
            /// </summary>
            internal string SolutionDirectory { get; set; }

            #endregion
        }

        #endregion

        #region Nested type: DataSet

        internal class DataSet
        {
            #region Constructors

            internal DataSet(string directory, List<GraphAndGoalPaths> graphsAndGoalsPaths)
            {
                Directory = directory;
                GraphsAndGoalsFilePaths = graphsAndGoalsPaths;
            }

            #endregion

            #region Internal Properties

            internal string Directory { get; private set; }
            internal List<GraphAndGoalPaths> GraphsAndGoalsFilePaths { get; }

            #endregion
        }

        #endregion

        #region Nested type: GraphAndGoalPaths

        internal class GraphAndGoalPaths
        {
            #region Constructors

            internal GraphAndGoalPaths(string graphPath, string goalPath)
            {
                GraphPath = graphPath;
                GoalPath = goalPath;
            }

            #endregion

            #region Internal Properties

            internal string GoalPath { get; }
            internal string GraphPath { get; }

            #endregion
        }

        #endregion

        #region Nested type: LatencyAssignerScoreCard

        internal class LatencyAssignerScoreCard
        {
            #region Constructors

            internal LatencyAssignerScoreCard()
            {
                TotalThroughputCosts = 0;
                TotalLatencyCosts = 0;
                TotalRegisterCosts = 0;
                TotalSumOfAchievedPeriods = 0;
                TotalNumberFailingPeriod = 0;
                TotalCases = 0;
                TotalExecutionTime = 0.0;
            }

            #endregion

            #region Internal Properties

            internal double PercentFailing => (double)TotalNumberFailingPeriod / TotalCases;
            internal long TotalCases { get; private set; }
            internal double TotalExecutionTime { get; private set; }
            internal long TotalLatencyCosts { get; private set; }
            internal long TotalNumberFailingPeriod { get; private set; }
            internal long TotalRegisterCosts { get; private set; }
            internal long TotalSumOfAchievedPeriods { get; private set; }

            internal long TotalThroughputCosts { get; private set; }

            #endregion

            #region Internal Methods

            internal void RegisterResult(
                long throughputCost,
                long latencyCost,
                long registerCost,
                int periodAchieved,
                int slack,
                double executionTime)
            {
                TotalThroughputCosts += throughputCost;
                TotalLatencyCosts += latencyCost;
                TotalRegisterCosts += registerCost;
                TotalSumOfAchievedPeriods += periodAchieved;
                TotalExecutionTime += executionTime;
                TotalCases++;
                if (slack < 0)
                {
                    TotalNumberFailingPeriod++;
                }
            }

            #endregion
        }

        #endregion

        #region Nested type: ResultInformation

        private class ResultInformation
        {
            #region Internal Properties

            internal int BestCount { get; set; }
            internal int BestOrTiedCount { get; set; }
            internal int FailedCount { get; set; }
            internal LatencyAssignerScoreCard OverallScoreCard { get; } = new LatencyAssignerScoreCard();

            #endregion
        }

        #endregion
    }
}
