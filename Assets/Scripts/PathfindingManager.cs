﻿using Assets.Scripts.IAJ.Unity.Pathfinding;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding;
using Assets.Scripts.IAJ.Unity.Pathfinding.GoalBounding;
using Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics;
using Assets.Scripts.IAJ.Unity.Pathfinding.Path;
using RAIN.Navigation;
using RAIN.Navigation.NavMesh;
using UnityEngine;
using Bounds = Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds;

namespace Assets.Scripts {
    public class PathfindingManager : MonoBehaviour {
        private const int NodesPerFrame = 1;

        //public fields to be set in Unity Editor
        public GameObject endDebugSphere;
        public GameObject startDebugSphere;
        public Camera camera;
        public GameObject p1;
        public GameObject p2;
        public GameObject p3;
        public GameObject p4;
        public GameObject p5;
        public GameObject p6;

        private KeyCode drawNavMeshKey = KeyCode.M;
        private bool drawNavMesh = false;

        //private fields for internal use only
        private Vector3 startPosition;
        private Vector3 endPosition;
        private NavMeshPathGraph navMesh;
        private int currentClickNumber;
        private GoalBoundsDijkstraMapFlooding mapFloodingAlgorithm;
        private GlobalPath currentSolution;
        private GlobalPath smoothedSolution;
        private bool draw = true;
        private int solutionIndex = 0;
        private int frameCount = 0;
        //public properties
        public AStarPathfinding PathFinding { get; private set; }

        private PathSmoothing pathSmoothing = new PathSmoothing();
        private AStarPathfinding aStarPathfinding;
        private NodeArrayAStarPathFinding nodeArrayPathFinding;
        private GoalBoundingPathfinding goalBoundingPathfinding;
        private readonly KeyCode NormalAStarKeyStart = KeyCode.A;
        private readonly KeyCode NodeArrayKeyStart = KeyCode.N;
        private readonly KeyCode GoalBoundKeyStart = KeyCode.G;
        private readonly KeyCode ClearKey = KeyCode.C;
        private GUIStyle guiStyle = new GUIStyle(); //to change font size
        private Bounds[] boundito;
        private Color[] c = new Color[] { Color.blue, Color.green, Color.red, Color.magenta, Color.yellow, Color.gray, Color.cyan };


        public void Initialize(NavMeshPathGraph navMeshGraph, AStarPathfinding pathfindingAlgorithm) {
            guiStyle.fontSize = 20;
            this.draw = true;
            this.navMesh = navMeshGraph;

            this.PathFinding = pathfindingAlgorithm;
            this.PathFinding.NodesPerFrame = NodesPerFrame;
        }

        // Use this for initialization
        void Awake() {
            this.currentClickNumber = 1;
            mapFloodingAlgorithm = new GoalBoundsDijkstraMapFlooding(NavigationManager.Instance.NavMeshGraphs[0]);

            aStarPathfinding =
                new AStarPathfinding(NavigationManager.Instance.NavMeshGraphs[0], new SimpleUnorderedNodeList(), new HashMapNodeList(), new EuclidianHeuristic());
            nodeArrayPathFinding =
                new NodeArrayAStarPathFinding(NavigationManager.Instance.NavMeshGraphs[0], new EuclidianHeuristic());
            var goalBoundTable = Resources.Load<GoalBoundingTable>("GoalBoundingTable");
            Debug.Log("Node 0: " + goalBoundTable.table[0]);
            goalBoundingPathfinding =
                new GoalBoundingPathfinding(NavigationManager.Instance.NavMeshGraphs[0], new EuclidianHeuristic(), goalBoundTable);
            this.Initialize(NavigationManager.Instance.NavMeshGraphs[0], aStarPathfinding);
        }


        // Update is called once per frame
        void Update() {
            Vector3 position;

            if (Input.GetMouseButtonDown(0)) {
                //if there is a valid position
                if (this.MouseClickPosition(out position)) {
                    //if this is the first click we're setting the start point
                    if (this.currentClickNumber == 1) {
                        //show the start sphere, hide the end one
                        //this is just a small adjustment to better see the debug sphere
                        this.startDebugSphere.transform.position = position + Vector3.up;
                        this.startDebugSphere.SetActive(true);
                        this.endDebugSphere.SetActive(false);
                        this.currentClickNumber = 2;
                        this.startPosition = position;
                        this.currentSolution = null;
                        this.draw = false;
                    } else {
                        //we're setting the end point
                        //this is just a small adjustment to better see the debug sphere
                        this.endDebugSphere.transform.position = position + Vector3.up;
                        this.endDebugSphere.SetActive(true);
                        this.currentClickNumber = 1;
                        this.endPosition = position;
                        this.draw = true;
                        //initialize the search algorithm
                        this.PathFinding.InitializePathfindingSearch(this.startPosition, this.endPosition);
                    }
                }
            } else if (Input.GetKeyDown(KeyCode.Alpha1)) {
                this.startPosition = this.p5.transform.localPosition;
                this.endPosition = this.p6.transform.localPosition;
                this.InitializePathFinding(this.startPosition, endPosition);
            } else if (Input.GetKeyDown(KeyCode.Alpha2)) {
                this.startPosition = this.p1.transform.localPosition;
                this.endPosition = this.p2.transform.localPosition;
                this.InitializePathFinding(this.startPosition, endPosition);
            } else if (Input.GetKeyDown(KeyCode.Alpha3)) {
                this.startPosition = this.p2.transform.localPosition;
                this.endPosition = this.p4.transform.localPosition;
                this.InitializePathFinding(this.startPosition, endPosition);
            } else if (Input.GetKeyDown(KeyCode.Alpha4)) {
                this.startPosition = this.p2.transform.localPosition;
                this.endPosition = this.p5.transform.localPosition;
                this.InitializePathFinding(this.startPosition, endPosition);
            } else if (Input.GetKeyDown(KeyCode.Alpha5)) {
                this.startPosition = this.p1.transform.localPosition;
                this.endPosition = this.p3.transform.localPosition;
                this.InitializePathFinding(this.p1.transform.localPosition, this.p3.transform.localPosition);
            } else if (Input.GetKeyDown(KeyCode.Alpha6)) {
                this.startPosition = this.p3.transform.localPosition;
                this.endPosition = this.p4.transform.localPosition;
                this.InitializePathFinding(this.startPosition, endPosition);
            } else if (Input.GetKeyDown(drawNavMeshKey)) {
                this.drawNavMesh = !this.drawNavMesh;
            } else if (Input.GetKeyDown(NodeArrayKeyStart)) {
                this.PathFinding = nodeArrayPathFinding;
                this.InitializePathFinding(this.startPosition, endPosition);

            } else if (Input.GetKeyDown(GoalBoundKeyStart)) {
                this.PathFinding = goalBoundingPathfinding;
                this.InitializePathFinding(this.startPosition, endPosition);

            } else if (Input.GetKeyDown(NormalAStarKeyStart)) {
                this.PathFinding = aStarPathfinding;
                this.InitializePathFinding(this.startPosition, endPosition);
            } else if (Input.GetKeyDown(ClearKey)) {
                this.currentSolution = null;
                
            }

            //call the pathfinding method if the user specified a new goal
            if (this.PathFinding.InProgress) {
                var finished = this.PathFinding.Search(out this.currentSolution, true);
                if (finished && currentSolution != null) {
                    currentSolution.PathPositions.Insert(0, startPosition);
                    solutionIndex = 0;
                    //Smooth it
                    smoothedSolution = pathSmoothing.Smooth(currentSolution);

                    smoothedSolution = pathSmoothing.Smooth(smoothedSolution);
                    smoothedSolution = pathSmoothing.Smooth(smoothedSolution);
                }
            }

            if (Input.GetKeyDown(KeyCode.H)) {
                draw = true;
                var StartNode = NavigationManager.Instance.NavMeshGraphs[0].QuantizeToNode(this.startPosition, 1.0f);
                //if it is not possible to quantize the positions and find the corresponding nodes, then we cannot proceed
                if (StartNode == null) return;

                var a = new NodeGoalBounds();
                var outConnectionsStart = StartNode.OutEdgeCount;
                a.connectionBounds = new Bounds[outConnectionsStart];
                for (int i = 0; i < a.connectionBounds.Length; i++) {
                    a.connectionBounds[i] = new Bounds();
                }
                mapFloodingAlgorithm.Search(StartNode, a);
                this.boundito = a.connectionBounds;
                Debug.Log(a.connectionBounds.Length);

            }
            if (Input.GetKeyDown(KeyCode.J)) {
                draw = true;
                var StartNode = NavigationManager.Instance.NavMeshGraphs[0].QuantizeToNode(this.endPosition, 1.0f);
                //if it is not possible to quantize the positions and find the corresponding nodes, then we cannot proceed
                if (StartNode == null) return;

                var a = new NodeGoalBounds();
                var outConnectionsStart = StartNode.OutEdgeCount;
                a.connectionBounds = new Bounds[outConnectionsStart];
                for (int i = 0; i < a.connectionBounds.Length; i++) {
                    a.connectionBounds[i] = new Bounds();
                }
                mapFloodingAlgorithm.Search(StartNode, a);

                this.boundito = a.connectionBounds;
                Debug.Log(a.connectionBounds.Length);
            }
        }



        public void OnGUI() {

            var activePathFinding = PathFinding.AlgorithmName;
            guiStyle.normal.textColor = Color.blue;
            guiStyle.fontSize = 30;
            GUI.Label(new Rect(10, 10, 300, 20), activePathFinding, guiStyle);


            var alwaysOnText = "Normal A* -> " + NormalAStarKeyStart.ToString()
                                    + "\nNodeArray -> " + NodeArrayKeyStart.ToString()
                                    + "\nGoalBound -> " + GoalBoundKeyStart.ToString()
                                    + "\n\nUsage: "
                                    + "\n  1st Select Algorithm"
                                    + "\n  2nd Choose The points (1 - 6 alpha numbers OR click)"
                                    + "\n\n\nDraw All Nodes -> " + drawNavMeshKey.ToString()
                ;
            guiStyle.normal.textColor = Color.black;
            guiStyle.fontSize = 20;
            GUI.Label(new Rect(10, 40, 300, 250), alwaysOnText, guiStyle);





            if (this.currentSolution != null) {
                var time = this.PathFinding.TotalProcessingTime * 1000;
                float timePerNode;
                if (this.PathFinding.TotalExploredNodes > 0) {
                    timePerNode = time / this.PathFinding.TotalExploredNodes;
                } else {
                    timePerNode = 0;
                }


                var text = "NodesPerFrame: " + NodesPerFrame
                           + "\nNodes Visited: " + this.PathFinding.TotalExploredNodes
                           + "\nMaximum Open Size: " + this.PathFinding.MaxOpenNodes
                           + "\nProcessing time (ms): " + time.ToString("F")
                           + "\nReal Processing time (ms): " + (PathFinding.PureTotalTime * 1000).ToString("F")
                           + "\nTime per Node (ms):" + timePerNode.ToString("F4")
                           + "\n" + frameCount
                           ;

                guiStyle.normal.textColor = Color.black;
                guiStyle.fontSize = 20;
                GUI.Label(new Rect(10, 280, 300, 200), text, guiStyle);
            }
        }

        public void OnDrawGizmos() {
            frameCount++;
            var boundBoxAlgorithm = PathFinding as GoalBoundingPathfinding;
            if(boundBoxAlgorithm != null) {

                mapFloodingAlgorithm = boundBoxAlgorithm.digjstra;
                if(boundBoxAlgorithm.toPrintNodeGoulBounds != null) {
                    this.boundito = boundBoxAlgorithm.toPrintNodeGoulBounds.connectionBounds;
                }

            }

            if (this.boundito != null && mapFloodingAlgorithm != null && mapFloodingAlgorithm.Closed != null) {
                var colorrrr = Color.black;
                foreach (var node in mapFloodingAlgorithm.Closed.All()) {
                    var boundIndex = node.StartNodeOutConnectionIndex;
                    if (boundIndex >= c.Length) {
                        colorrrr = Color.black;
                    } else {
                        colorrrr = c[boundIndex];
                    }
                    Gizmos.color = colorrrr;
                    Gizmos.DrawSphere(node.node.LocalPosition, 1f);
                }
                for (int i = 0; i < mapFloodingAlgorithm.StartNode.OutEdgeCount;i++) {
                    var rainNode = mapFloodingAlgorithm.StartNode.EdgeOut(i).ToNode;

                    var nodeToPrint = mapFloodingAlgorithm.NodeRecordArray.GetNodeRecord(rainNode);
                    var boundIndex = nodeToPrint.StartNodeOutConnectionIndex;
                    if (boundIndex >= c.Length) {
                        colorrrr = Color.black;
                    } else {
                        colorrrr = c[boundIndex];
                    }
                    Gizmos.color = colorrrr;
                    Gizmos.DrawSphere(nodeToPrint.node.LocalPosition, 0.5f);
                }
                Gizmos.color = Color.black;
                Gizmos.DrawSphere(mapFloodingAlgorithm.NodeRecordArray.GetNodeRecord(mapFloodingAlgorithm.StartNode).node.LocalPosition, 0.5f);


                var nrOfPoints = 0;
                var index = 0;
                colorrrr = Color.black;
                string boxes = "";
                foreach (var bound in boundito) {
                    if (index >= c.Length) {
                        colorrrr = Color.black;
                    } else {
                        colorrrr = c[index];
                    }
                    boxes += "\n" + index + " : " + bound.minx + " -> " + bound.maxx + " _z_ " + +bound.minz + " -> " + bound.maxz;
                    if (bound.minx == bound.maxx && bound.minz == bound.maxz) {
                        nrOfPoints++;
                        Gizmos.color = Color.cyan;
                        Gizmos.DrawSphere(new Vector3(bound.minx, 0, bound.minz), 0.5f);
                    } else {
                        Debug.DrawLine(new Vector3(bound.minx, 0, bound.minz), new Vector3(bound.minx, 0, bound.maxz), colorrrr);
                        Debug.DrawLine(new Vector3(bound.minx, 0, bound.minz), new Vector3(bound.maxx, 0, bound.minz), colorrrr);
                        Debug.DrawLine(new Vector3(bound.minx, 0, bound.maxz), new Vector3(bound.maxx, 0, bound.maxz), colorrrr);
                        Debug.DrawLine(new Vector3(bound.maxx, 0, bound.maxz), new Vector3(bound.maxx, 0, bound.minz), colorrrr);
                    }
                    index++;
                }
                //Debug.Log(boxes);
                //Debug.Log("nrOfPoints: " + nrOfPoints);

            }
            if (this.drawNavMesh) {
                var navMesh = this.PathFinding.NavMeshGraph;
                Gizmos.color = Color.cyan;
                for (int index = 0; index < navMesh.Size; index++) {
                    var node = navMesh.GetNode(index);
                    Gizmos.DrawSphere(node.LocalPosition, 0.4f);
                }
            }
            if (this.draw) {
                //draw the current Solution Path if any (for debug purposes)
                if (this.currentSolution != null) {
                    var colorLine = Color.red;
                    var previousPosition = this.startPosition;
                    foreach (var pathPosition in this.currentSolution.PathPositions) {
                        Debug.DrawLine(previousPosition, pathPosition, colorLine);
                        previousPosition = pathPosition;
                    }


                    if (this.currentSolution.Smoothed) {
                        GlobalPath pathToPrint = smoothedSolution;
                        colorLine = Color.yellow;
                        if (frameCount % 10 == 0) {
                            solutionIndex++;
                            if (solutionIndex == pathToPrint.PathPositions.Count) {
                                solutionIndex = 0;
                            }
                        }
                        Gizmos.DrawSphere(pathToPrint.PathPositions[solutionIndex], 10.0f);

                        previousPosition = this.startPosition;
                        foreach (var pathPosition in this.smoothedSolution.PathPositions) {
                            Debug.DrawLine(previousPosition, pathPosition, colorLine);
                            previousPosition = pathPosition;
                        }
                    }
                }
                //draw the nodes in Open and Closed Sets
                if (this.PathFinding != null && this.currentSolution != null) {
                    Gizmos.color = Color.magenta;
                    if (this.PathFinding.Open != null) {
                        foreach (var nodeRecord in this.PathFinding.Open.All()) {
                            Gizmos.DrawSphere(nodeRecord.node.LocalPosition, 2.0f);
                        }
                    }
                    Gizmos.color = Color.blue;

                    if (this.PathFinding.Closed != null) {
                        foreach (var nodeRecord in this.PathFinding.Closed.All()) {
                            Gizmos.DrawSphere(nodeRecord.node.LocalPosition, 1f);
                        }
                    }

                    Gizmos.color = Color.blue;

                    if (this.PathFinding.Closed != null)
                    {
                        foreach (var nodeRecord in this.PathFinding.Closed.All())
                        {
                            Gizmos.DrawSphere(nodeRecord.node.LocalPosition, 1f);
                        }
                    }
                }
            }
        }

        private bool MouseClickPosition(out Vector3 position) {
            RaycastHit hit;

            var ray = this.camera.ScreenPointToRay(Input.mousePosition);
            //test intersection with objects in the scene
            if (Physics.Raycast(ray, out hit)) {
                //if there is a collision, we will get the collision point
                position = hit.point;
                return true;
            }

            position = Vector3.zero;
            //if not the point is not valid
            return false;
        }

        private void InitializePathFinding(Vector3 p1, Vector3 p2) {

            //show the start sphere, hide the end one
            //this is just a small adjustment to better see the debug sphere
            this.startDebugSphere.transform.position = p1 + Vector3.up;
            this.startDebugSphere.SetActive(true);
            this.endDebugSphere.transform.position = p2 + Vector3.up;
            this.endDebugSphere.SetActive(true);
            this.currentClickNumber = 1;
            this.startPosition = p1;
            this.endPosition = p2;

            this.currentSolution = null;
            this.draw = true;

            this.PathFinding.InitializePathfindingSearch(this.startPosition, this.endPosition);
        }
    }
}
