using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding;
using Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics;
using RAIN.Navigation.NavMesh;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using RAIN.Navigation.Graph;
using UnityEngine;
using System.Collections.Generic;
using Assets.Scripts.IAJ.Unity.Pathfinding.Path;
using Bounds = Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds;


namespace Assets.Scripts.IAJ.Unity.Pathfinding.GoalBounding
{
    public class GoalBoundingPathfinding : NodeArrayAStarPathFinding
    {
        public GoalBoundingTable GoalBoundingTable { get; protected set;}
        
        //public int DiscardedEdges { get; protected set; }
		public int TotalEdges { get; protected set; }

        private NodeGoalBounds startingNodeGoalBounds;

        private GoalBoundsDijkstraMapFlooding digjstra;


        public override string AlgorithmName {
            get {
                return "GoalBoundingPathfinding";
            }
        }


        public GoalBoundingPathfinding(NavMeshPathGraph graph, IHeuristic heuristic, GoalBoundingTable goalBoundsTable) : base(graph, heuristic)
        {
            this.GoalBoundingTable = goalBoundsTable;
            digjstra = new GoalBoundsDijkstraMapFlooding(graph);

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
            var table = GoalBoundingTable;
            var xD = table.table;
            var b = xD;
            NodeGoalBounds pls = GoalBoundingTable.table[0] as NodeGoalBounds;
            //Debug.Log(parentNode.node.NodeIndex);
            //Debug.Log(pls);
            var a = pls;
            
            NodeGoalBounds ngb  = (NodeGoalBounds) ScriptableObject.CreateInstance(typeof(NodeGoalBounds));
            var outConnectionsStart = parentNode.node.OutEdgeCount;
            ngb.connectionBounds = new Bounds[outConnectionsStart];
            for (int i = 0; i < ngb.connectionBounds.Length; i++)
            {
                ngb.connectionBounds[i] = (Bounds)ScriptableObject.CreateInstance(typeof(Bounds));
            }
            digjstra.Search(parentNode.node, ngb);

            //Debug.Log("1-"+ngb.connectionBounds[edgeIndex].minx);
            //Debug.Log("2-" + ngb.connectionBounds[edgeIndex].maxx);
            //Debug.Log("3-" + ngb.connectionBounds[edgeIndex].maxz);
            //Debug.Log("4-" + ngb.connectionBounds[edgeIndex].minz);
            if (ngb.connectionBounds[edgeIndex].PositionInsideBounds(GoalPosition))
            {
                base.ProcessChildNode(parentNode, connectionEdge, edgeIndex);
                       return;
            }
            //if (GoalBoundingTable.table[parentNode.node.NodeIndex].connectionBounds[edgeIndex].PositionInsideBounds(childNodeRecord.node.LocalPosition)){
            //       base.ProcessChildNode(parentNode,connectionEdge,edgeIndex);
            //       return;
            //}
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
