﻿using System;
using System.Collections.Generic;
using System.Linq;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures
{
    public class LeftPriorityList : IOpenSet{
        private List<NodeRecord> Open { get; set; }

        public LeftPriorityList()
        {
            this.Open = new List<NodeRecord>();    
        }
        public void Initialize()
        {
            this.Open.Clear();
        }

        public void Replace(NodeRecord nodeToBeReplaced, NodeRecord nodeToReplace)
        {
            this.RemoveFromOpen(nodeToBeReplaced);
            this.AddToOpen(nodeToReplace);
        }

        public NodeRecord GetBestAndRemove()
        {
            var best = this.PeekBest();
            this.Open.RemoveAt(0);
            return best;
        }

        public NodeRecord PeekBest()
        {
            return this.Open[0];
        }

        public void AddToOpen(NodeRecord nodeRecord)
        {
            //a little help here
            //is very nice that the List<T> already implements a binary search method
            int index = this.Open.BinarySearch(nodeRecord);
            if (index < 0)
            {
                this.Open.Insert(~index, nodeRecord);
            }
        }

        public void RemoveFromOpen(NodeRecord nodeRecord)
        {
            this.Open.Remove(nodeRecord);
        }

        public NodeRecord SearchInOpen(NodeRecord nodeRecord)
        {
            return this.Open.FirstOrDefault(n => n.Equals(nodeRecord));
        }

        public ICollection<NodeRecord> All()
        {
            return new List<NodeRecord>(this.Open);
        }

        public int CountOpen()
        {
            return this.Open.Count;
        }
    }
}
