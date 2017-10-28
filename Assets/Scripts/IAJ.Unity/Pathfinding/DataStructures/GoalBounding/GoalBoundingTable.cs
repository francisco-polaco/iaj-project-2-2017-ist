using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using UnityEditor;
using UnityEngine;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding
{
    public class GoalBoundingTable : ScriptableObject
    {
        public NodeGoalBounds[] table;

        public void SaveToAssetDatabase()
        {
            string path = AssetDatabase.GetAssetPath(Selection.activeObject);
            if (path == "")
            {
                path = "Assets";
            }
            else if (System.IO.Path.GetExtension(path) != "")
            {
                path = path.Replace(System.IO.Path.GetFileName(AssetDatabase.GetAssetPath(Selection.activeObject)), "");
            }

            string assetPathAndName = 
                AssetDatabase.GenerateUniqueAssetPath(path + "/" + typeof(GoalBoundingTable).Name.ToString() + ".asset");

            AssetDatabase.CreateAsset(this, assetPathAndName);
            EditorUtility.SetDirty(this);

            int total = this.table.Length;
            int index = 0;
            float progress = 0;
            //save the GoalBounds Table
            foreach (var nodeGoalBounds in this.table)
            {
                if(index%10==0)
                {
                    progress = (float)index / (float)total;
                    EditorUtility.DisplayProgressBar("GoalBounding precomputation progress", "Saving GoalBoundsTable to an Asset file", progress);
                }

                if(nodeGoalBounds != null)
                {
                    AssetDatabase.AddObjectToAsset(nodeGoalBounds, assetPathAndName);

                    //save goalBounds for each edge of the node
                    foreach (var goalBounds in nodeGoalBounds.connectionBounds)
                    {
                        AssetDatabase.AddObjectToAsset(goalBounds, assetPathAndName);
                    }
                }
                
                index++;
            }


            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();
            EditorUtility.FocusProjectWindow();
            Selection.activeObject = this;
            
        }

        public void SaveToAssetDatabaseOptimized()
        {

            string assetPathAndName =
                AssetDatabase.GenerateUniqueAssetPath(Path() + "/" + typeof(GoalBoundingTable).Name.ToString() + ".bin");

            EditorUtility.DisplayProgressBar("GoalBounding precomputation progress", "Saving GoalBoundsTable to an Asset file", 0);


            IFormatter formatter = new BinaryFormatter();
            using (Stream stream = new FileStream(assetPathAndName, FileMode.Create, FileAccess.Write, FileShare.None))
            {
                formatter.Serialize(stream, table.Length);

                foreach (var nodeGoalBoundse in table)
                {
                    if (nodeGoalBoundse == null)
                    {
                        formatter.Serialize(stream, "null");
                    }
                    else
                    {
                        formatter.Serialize(stream, nodeGoalBoundse.connectionBounds.Length);

                        foreach (var nodeGoalBound in nodeGoalBoundse.connectionBounds)
                        {
                            if (nodeGoalBound == null)
                            {
                                formatter.Serialize(stream, "null");
                            }
                            else
                            {
                                formatter.Serialize(stream, nodeGoalBound.minx);
                                formatter.Serialize(stream, nodeGoalBound.maxx);
                                formatter.Serialize(stream, nodeGoalBound.minz);
                                formatter.Serialize(stream, nodeGoalBound.maxz);
                            }
                        }
                    }
                }
            }

            EditorUtility.DisplayProgressBar("GoalBounding precomputation progress", "Saving GoalBoundsTable to an Asset file", 100);

            EditorUtility.FocusProjectWindow();
            Selection.activeObject = this;

        }

        public void SaveToAssetDatabaseOptimized2()
        {

            string assetPathAndName =
                AssetDatabase.GenerateUniqueAssetPath(Path() + "/" + typeof(GoalBoundingTable).Name.ToString() + ".bin");

            EditorUtility.DisplayProgressBar("GoalBounding precomputation progress", "Saving GoalBoundsTable to an Asset file", 0);


            IFormatter formatter = new BinaryFormatter();
            using (Stream stream = new FileStream(assetPathAndName, FileMode.Create, FileAccess.Write, FileShare.None))
            {
                var matrix = new List<List<SerializableBounds>>(table.Length);

                for (var i = 0; i < table.Length; i++)
                {
                    if (table[i] != null)
                    {
                        for (var j = 0; j < table[i].connectionBounds.Length; j++)
                        {
                            var connectionBound = table[i].connectionBounds[j];
                            if (connectionBound != null)
                            {
                                if(i == matrix.Count)
                                    matrix.Insert(i, new List<SerializableBounds>(table[i].connectionBounds.Length));
                                matrix[i].Insert(j, new SerializableBounds(connectionBound.minx, connectionBound.maxx,
                                        connectionBound.minz,
                                        connectionBound.maxz));
                            }
                            else
                            {
                                matrix[i].Insert(j, null);
                            }
                        }
                    }
                    else
                    {
                        matrix.Insert(i, null);
                    }
                }
                formatter.Serialize(stream, matrix);

            }

            EditorUtility.DisplayProgressBar("GoalBounding precomputation progress", "Saving GoalBoundsTable to an Asset file", 1);

            EditorUtility.FocusProjectWindow();
            Selection.activeObject = this;

        }

        public void LoadOptimized2()
        {
            // We will assume a default name here

            string assetPathAndName =
                Path() + "/" + typeof(GoalBoundingTable).Name.ToString() + ".bin";

            IFormatter formatter = new BinaryFormatter();
            using (Stream stream =
                new FileStream(assetPathAndName, FileMode.Open, FileAccess.Read, FileShare.None))
            {
            	var matrix = (List<List<SerializableBounds>>) formatter.Deserialize(stream);
            	table = new NodeGoalBounds[matrix.Count];
                for (int i = 0; i < matrix.Count; i++)
                {
                    if (matrix[i] != null)
                    {
                        table[i] = new NodeGoalBounds(table.Length);
                        for (int j = 0; j < matrix[i].Count; j++)
                        {
                            if (matrix[i][j] != null)
                            {
                                var bounds = new Bounds
                                {
                                    minx = matrix[i][j].minx,
                                    maxx = matrix[i][j].maxx,
                                    minz = matrix[i][j].minz,
                                    maxz = matrix[i][j].maxz
                                };
                                table[i].connectionBounds[j] = bounds;
                            }
                            else
                            {
                                table[i].connectionBounds[j] = null;
                            }
                        }
                    }
                    else
                    {
                        table[i] = null;
                    }
                }
            }
        }

        public void LoadOptimized()
        {
            // We will assume a default name here
            string assetPathAndName =
                Path() + "/" + typeof(GoalBoundingTable).Name.ToString() + ".bin";

            IFormatter formatter = new BinaryFormatter();
            using (Stream stream = 
                new FileStream(assetPathAndName, FileMode.Open, FileAccess.Read, FileShare.None))
            {
                int sizeOfTable = (int) formatter.Deserialize(stream);
                table = new NodeGoalBounds[sizeOfTable];

                for (int i = 0; i < sizeOfTable; i++)
                {
                    object tableValue = formatter.Deserialize(stream);

                    if (tableValue.Equals("null"))
                    {
                        continue;
                    }
                    else
                    {
                        int sizeOfBounds = (int) tableValue;
                        table[i] = new NodeGoalBounds(sizeOfBounds);
                        for (int j = 0; j < sizeOfBounds; j++)
                        {
                            object boundValue = formatter.Deserialize(stream);
                            if (boundValue.Equals("null"))
                            {
                                continue;
                            }
                            else
                            {
                                float minx = (float) boundValue;
                                float maxx = (float) formatter.Deserialize(stream);
                                float minz = (float) formatter.Deserialize(stream);
                                float maxz = (float) formatter.Deserialize(stream);

                                var bounds = new Bounds();
                                bounds.minx = minx;
                                bounds.maxx = maxx;
                                bounds.minz = minz;
                                bounds.maxz = maxz;

                                table[i].connectionBounds[j] = bounds;
                            }
                        }
                    }
                }
            }
        }

        private static string Path()
        {
            string path = AssetDatabase.GetAssetPath(Selection.activeObject);
            if (path == "")
            {
                path = "Assets";
            }
            else if (System.IO.Path.GetExtension(path) != "")
            {
                path = path.Replace(System.IO.Path.GetFileName(AssetDatabase.GetAssetPath(Selection.activeObject)),
                    newValue: "");
            }
            return path;
        }

        [Serializable]
        private class SerializableBounds
        {
            public float minx;
            public float maxx;
            public float minz;
            public float maxz;

            public SerializableBounds(float minx, float maxx, float minz, float maxz)
            {
                this.minx = minx;
                this.maxx = maxx;
                this.minz = minz;
                this.maxz = maxz;
            }

            public SerializableBounds()
            {
                this.minx = 0f;
                this.maxx = 0f;
                this.minz = 0f;
                this.maxz = 0f;
            }
        }
        /* Para o array principal guardar o seu tamaanho
         * para cada array secundario, guardar o seu tamanho e os 4 floats de cada objecto
         * se em algum sitio houver um null a string "null" será escrita
         * */
    }
}
