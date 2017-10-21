﻿
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics;
using Assets.Scripts.IAJ.Unity.Pathfinding.Path;
using RAIN.Navigation.Graph;
using RAIN.Navigation.NavMesh;
using UnityEngine;

namespace Assets.Scripts.IAJ.Unity.Pathfinding
{
    public class AStarPathfinding
    {
        public NavMeshPathGraph NavMeshGraph { get; protected set; }
        //how many nodes do we process on each call to the search method (this method will be called every frame when there is a pathfinding process active
        public uint NodesPerFrame { get; set; }

        public uint TotalExploredNodes { get; protected set; }
        public int MaxOpenNodes { get; protected set; }
        public float TotalProcessingTime { get; protected set; }
        public bool InProgress { get; protected set; }

        public IOpenSet Open { get; protected set; }
        public IClosedSet Closed { get; protected set; }

        public NavigationGraphNode GoalNode { get; protected set; }
        public NavigationGraphNode StartNode { get; protected set; }
        public Vector3 StartPosition { get; protected set; }
        public Vector3 GoalPosition { get; protected set; }

        //heuristic function
        public IHeuristic Heuristic { get; protected set; }

        public AStarPathfinding(NavMeshPathGraph graph, IOpenSet open, IClosedSet closed, IHeuristic heuristic)
        {
            this.NavMeshGraph = graph;
            this.Open = open;
            this.Closed = closed;
            this.NodesPerFrame = uint.MaxValue; //by default we process all nodes in a single request
            this.InProgress = false;
            this.Heuristic = heuristic;
        }

        public virtual void InitializePathfindingSearch(Vector3 startPosition, Vector3 goalPosition)
        {
            this.StartPosition = startPosition;
            this.GoalPosition = goalPosition;
            this.StartNode = this.Quantize(this.StartPosition);
            this.GoalNode = this.Quantize(this.GoalPosition);

            //if it is not possible to quantize the positions and find the corresponding nodes, then we cannot proceed
            if (this.StartNode == null || this.GoalNode == null) return;

            //I need to do this because in Recast NavMesh graph, the edges of polygons are considered to be nodes and not the connections.
            //Theoretically the Quantize method should then return the appropriate edge, but instead it returns a polygon
            //Therefore, we need to create one explicit connection between the polygon and each edge of the corresponding polygon for the search algorithm to work
            ((NavMeshPoly)this.StartNode).AddConnectedPoly(this.StartPosition);
            ((NavMeshPoly)this.GoalNode).AddConnectedPoly(this.GoalPosition);

            this.InProgress = true;
            this.TotalExploredNodes = 0;
            this.TotalProcessingTime = 0.0f;
            this.MaxOpenNodes = 0;
            this.PureTotalTime = 0;
            this.BegginingOfSearchRealTime = Time.realtimeSinceStartup;

            var initialNode = new NodeRecord
            {
                gValue = 0,
                hValue = this.Heuristic.H(this.StartNode, this.GoalNode),
                node = this.StartNode
            };

            initialNode.fValue = AStarPathfinding.F(initialNode);

            this.Open.Initialize(); 
            this.Open.AddToOpen(initialNode);
            this.Closed.Initialize();
        }

        public float BegginingOfSearchRealTime { get; set; }

        public float PureTotalTime { get; set; }

        protected virtual void ProcessChildNode(NodeRecord parentNode, NavigationGraphEdge connectionEdge)
        {
            //this is where you process a child node 
            var childNode = GenerateChildNodeRecord(parentNode, connectionEdge);
            var childNodeStatus = childNode.status;

            var open = Open.SearchInOpen(childNode);
            var close = Closed.SearchInClosed(childNode);
            if (open == null && close == null)
            {
                Open.AddToOpen(childNode);
            }
            else if (open != null)
            {
                //var searchInOpen = Open.SearchInOpen(childNode);
                if (open.fValue > childNode.fValue)
                {
                    Open.Replace(open, childNode);
                }

            }
            else if (close != null)
            {
                //var searchInClosed = Closed.SearchInClosed(childNode);
                if (close.fValue > childNode.fValue)
                {
                    Closed.RemoveFromClosed(close);
                    Open.AddToOpen(childNode);
                }
            }

            //if (childNodeStatus != NodeStatus.Open && childNodeStatus != NodeStatus.Closed)
            //{
            //    Open.AddToOpen(childNode);
            //}
            //else if (childNodeStatus == NodeStatus.Open)
            //{
            //    var searchInOpen = Open.SearchInOpen(childNode);
            //    if (searchInOpen.fValue > childNode.fValue)
            //    {
            //        Open.Replace(searchInOpen, childNode);
            //    }
            //}
            //else if (childNodeStatus == NodeStatus.Closed)
            //{
            //    var searchInClosed = Closed.SearchInClosed(childNode);
            //    if (searchInClosed.fValue > childNode.fValue)
            //    {
            //        Closed.RemoveFromClosed(searchInClosed);
            //        Open.AddToOpen(childNode);
            //    }
            //}
        }

        public bool Search(out GlobalPath solution, bool returnPartialSolution = false)
        {
            var initialFrameTime = Time.realtimeSinceStartup;
            uint count = 0;
            while (Open.CountOpen() > 0)
            {
                var bestNode = Open.GetBestAndRemove();
                count++;
                if (this.GoalNode.Equals(bestNode.node))
                {
                    InProgress = false;
                    solution = CalculateSolution(bestNode, false);
                    UpdateInfo(initialFrameTime);

                    return true;
                }
                Closed.AddToClosed(bestNode);
                var outConnections = bestNode.node.OutEdgeCount;
                for (int i = 0; i < outConnections; i++)
                {
                    ProcessChildNode(bestNode, bestNode.node.EdgeOut(i));
                }

                if (returnPartialSolution && count == NodesPerFrame)
                {
                    solution = CalculateSolution(bestNode, true);
                    UpdateInfo(initialFrameTime);
                    return false;
                }
            }

            UpdateInfo(initialFrameTime);
            InProgress = false;
            solution = null;
            return true;
        }

        private void UpdateInfo(float initialFrameTime)
        {
            PartialTime = Time.realtimeSinceStartup - initialFrameTime;
            TotalProcessingTime += PartialTime;
            TotalExploredNodes = (uint) Closed.All().Count;
            if (Open.CountOpen() > MaxOpenNodes)
            {
                MaxOpenNodes = Open.CountOpen();
            }
            PureTotalTime = Time.realtimeSinceStartup - BegginingOfSearchRealTime;
        }

        public float PartialTime { get; set; }

        protected NavigationGraphNode Quantize(Vector3 position)
        {
            return this.NavMeshGraph.QuantizeToNode(position, 1.0f);
        }

        protected void CleanUp()
        {
            //I need to remove the connections created in the initialization process
            if (this.StartNode != null)
            {
                ((NavMeshPoly)this.StartNode).RemoveConnectedPoly();
            }

            if (this.GoalNode != null)
            {
                ((NavMeshPoly)this.GoalNode).RemoveConnectedPoly();    
            }
        }

        protected virtual NodeRecord GenerateChildNodeRecord(NodeRecord parent, NavigationGraphEdge connectionEdge)
        {
            var childNode = connectionEdge.ToNode;
            var childNodeRecord = new NodeRecord
            {
                node = childNode,
                parent = parent,
                gValue = parent.gValue + (childNode.LocalPosition-parent.node.LocalPosition).magnitude,
                hValue = this.Heuristic.H(childNode, this.GoalNode)
            };

            childNodeRecord.fValue = F(childNodeRecord);

            return childNodeRecord;
        }

        protected GlobalPath CalculateSolution(NodeRecord node, bool partial)
        {
            var path = new GlobalPath
            {
                IsPartial = partial,
                Length = node.gValue
            };
            var currentNode = node;

            //
            //

            //I need to remove the first Node and the last Node because they correspond to the dummy first and last Polygons that were created by the initialization.
            //And we don't want to be forced to go to the center of the initial polygon before starting to move towards my destination.

            //skip the last node, but only if the solution is not partial (if the solution is partial, the last node does not correspond to the dummy goal polygon)
            if (!partial && currentNode.parent != null)
            {
                
                currentNode = currentNode.parent;
            }


            //Add target polygon to path
            if(!partial) {
                path.PathPositions.Add(this.GoalPosition);
            }
            
            while (currentNode.parent != null)
            {
                path.PathNodes.Add(currentNode.node); //we need to reverse the list because this operator add elements to the end of the list
                path.PathPositions.Add(currentNode.node.LocalPosition);

                if (currentNode.parent.parent == null) break; //this skips the first node
                currentNode = currentNode.parent;
            }

            path.PathNodes.Reverse();
            path.PathPositions.Reverse();
            return path;

        }

        public static float F(NodeRecord node)
        {
            return F(node.gValue,node.hValue);
        }

        public static float F(float g, float h)
        {
            return g + h;
        }

    }
}