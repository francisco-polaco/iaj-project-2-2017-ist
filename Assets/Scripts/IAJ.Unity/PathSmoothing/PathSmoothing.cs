﻿using UnityEngine;
using UnityEditor;
using Assets.Scripts.IAJ.Unity.Pathfinding.Path;
using System.Threading;

public class PathSmoothing {
    //private readonly Collider _collider;

    //public PathSmoothing() {
    //    _collider = new Collider();
    //}

    public GlobalPath Smooth(GlobalPath globalPath) {
        globalPath.Smoothed = true;
        GlobalPath toReturn = new GlobalPath();

        if(globalPath.PathPositions.Count < 2) {
            return globalPath;
        }

        var previousSavedNode = globalPath.PathPositions[0];
        toReturn.PathPositions.Add(previousSavedNode);

        var lastIgnoredNode = globalPath.PathPositions[1];

        for (int index = 2; index < globalPath.PathPositions.Count; index++) {
            var nodeCurrentlyBeingConsidered = globalPath.PathPositions[index];
            if (IsThereCollisionBetween(previousSavedNode, nodeCurrentlyBeingConsidered)) {
                toReturn.PathPositions.Add(lastIgnoredNode);
                previousSavedNode = lastIgnoredNode;
            }
            lastIgnoredNode = nodeCurrentlyBeingConsidered;
        }
        toReturn.PathPositions.Add(globalPath.PathPositions[globalPath.PathPositions.Count-1]);


        return toReturn;
    }

    private bool IsThereCollisionBetween(Vector3 start, Vector3 end) {
        var direction = end - start;
        float maxDistance = direction.magnitude;
        return Physics.Raycast(start, direction.normalized, maxDistance);
    }
}