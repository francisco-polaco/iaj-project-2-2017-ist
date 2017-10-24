using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding;
using Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics;
using RAIN.Navigation.NavMesh;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using RAIN.Navigation.Graph;
using UnityEngine;
using System.Collections.Generic;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.GoalBounding
{
    public class GoalBoundingPathfinding : NodeArrayAStarPathFinding
    {
        public GoalBoundingTable GoalBoundingTable { get; protected set;}
        
        public int DiscardedEdges { get; protected set; }
		public int TotalEdges { get; protected set; }

        private NodeGoalBounds startingNodeGoalBounds;

        private List<int> importantNodeGoalBoundsIndexes = new List<int>();

        public override string AlgorithmName {
            get {
                return "GoalBoundingPathfinding";
            }
        }


        public GoalBoundingPathfinding(NavMeshPathGraph graph, IHeuristic heuristic, GoalBoundingTable goalBoundsTable) : base(graph, heuristic)
        {
            this.GoalBoundingTable = goalBoundsTable;
        }

        public override void InitializePathfindingSearch(Vector3 startPosition, Vector3 goalPosition) {
            this.DiscardedEdges = 0;
            this.TotalEdges = 0;
            base.InitializePathfindingSearch(startPosition, goalPosition);

            //get GoalBoundingTable. NodeGoalBounds correspondente ao starting node
            for(int i = 0; i < startingNodeGoalBounds.connectionBounds.Length; i++) {
                if (startingNodeGoalBounds.connectionBounds[i].PositionInsideBounds(goalPosition)){
                    importantNodeGoalBoundsIndexes.Add(i);
                }

            }
           


        }

        protected override void ProcessChildNode(NodeRecord parentNode, NavigationGraphEdge connectionEdge, int edgeIndex)
        {
            //TODO: Implement this method for the GoalBoundingPathfinding to Work. 
            // If you implemented the NodeArrayAStar properly, you wont need to change the search method.
            var childNode = connectionEdge.ToNode;
            var childNodeRecord = this.NodeRecordArray.GetNodeRecord(childNode);

            if (childNodeRecord == null) {
                //this piece of code is used just because of the special start nodes and goal nodes added to the RAIN Navigation graph when a new search is performed.
                //Since these special goals were not in the original navigation graph, they will not be stored in the NodeRecordArray and we will have to add them
                //to a special structure
                //it's ok if you don't understand this, this is a hack and not part of the NodeArrayA* algorithm, just do NOT CHANGE THIS, or your algorithm will not work
                childNodeRecord = new NodeRecord {
                    node = childNode,
                    parent = parentNode,
                    status = NodeStatus.Unvisited
                };
                this.NodeRecordArray.AddSpecialCaseNode(childNodeRecord);
            }

            TotalEdges++;
            foreach(var boundIndex in importantNodeGoalBoundsIndexes) {
                if(startingNodeGoalBounds.connectionBounds[boundIndex].PositionInsideBounds(childNodeRecord.node.LocalPosition)){
                    base.ProcessChildNode(parentNode,connectionEdge,edgeIndex);
                    return;
                }
            }
            DiscardedEdges++;

            //var childNodeStatus = childNodeRecord.status;
            //float g = bestNode.gValue + (childNode.LocalPosition - bestNode.node.LocalPosition).magnitude;
            //float h = this.Heuristic.H(childNode, this.GoalNode);
            //float f = F(g, h);

            ////We can only update inside the ifs because otherwise we might be making the node worse

            //if (childNodeStatus == NodeStatus.Unvisited) {
            //    UpdateNode(bestNode, childNodeRecord, g, h, f);
            //    Open.AddToOpen(childNodeRecord);
            //} else if (childNodeStatus == NodeStatus.Open && childNodeRecord.fValue > f) {
            //    UpdateNode(bestNode, childNodeRecord, g, h, f);
            //    Open.Replace(childNodeRecord, childNodeRecord);
            //} else if (childNodeStatus == NodeStatus.Closed && childNodeRecord.fValue > f) {
            //    UpdateNode(bestNode, childNodeRecord, g, h, f);
            //    Closed.RemoveFromClosed(childNodeRecord);
            //    Open.AddToOpen(childNodeRecord);
            //}
        }
    }
}
