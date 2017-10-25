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

        }

        protected override void ProcessChildNode(NodeRecord parentNode, NavigationGraphEdge connectionEdge, int edgeIndex)
        {
            //TODO: Implement this method for the GoalBoundingPathfinding to Work. 
            // If you implemented the NodeArrayAStar properly, you wont need to change the search method.
            var childNode = connectionEdge.ToNode;
            var childNodeRecord = this.NodeRecordArray.GetNodeRecord(childNode);
            TotalEdges++;
            if(GoalBoundingTable.table[parentNode.node.NodeIndex].connectionBounds[edgeIndex].PositionInsideBounds(childNodeRecord.node.LocalPosition)){
                    base.ProcessChildNode(parentNode,connectionEdge,edgeIndex);
                    return;
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
