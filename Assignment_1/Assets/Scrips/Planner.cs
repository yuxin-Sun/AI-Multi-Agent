using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using Priority_Queue;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    [RequireComponent(typeof(DroneController))]

    public class Planner
    {
        private Node startNode;
        private Node goalNode;
        private float[] steering;
        public float x_size = 0;
        public float z_size = 0;
        public float x_res = 0;
        public float z_res = 0;
        public float x_low = 0;
        public float x_high = 0;
        public float z_high = 0;
        public float z_low = 0;
        public float arc = 0;
        public Vector3 draw0;
        public Vector3 draw1;
        public Vector3 draw2;
        public List<Vector3> draw_list = new List<Vector3>();
        private Node n;
        private Node sucessor;
        private float maxSteerAngle = 0;
        private float[] orentations = new float[] { 0, (float)Math.PI / 4, (float)Math.PI / 2, (float)Math.PI * 3 / 4, (float)Math.PI, -(float)Math.PI * 3 / 4, -(float)Math.PI / 2, -(float)Math.PI / 4 };

        private float calculateEuclidean(float x1, float z1, float x2, float z2)
        {
            return (float)Math.Sqrt(Math.Pow( (x1-x2) , 2) + Math.Pow( (z1-z2), 2 ));
        }

        public int calculateGridIndex(float x, float z)
        {
            int xIdx = (int)Math.Round((x - x_low) / x_res);
            int zIdx = (int)Math.Round((z - z_low) / z_res);
            int gridIdx = zIdx*( (int)x_size + 1) + xIdx;

            return gridIdx;
        }

        public Node HybridAStar(TerrainManager terrain_manager, CarController m_Car, Vector3 start_pos, float start_angle, Vector3 goal_pos, float[,] obstacle_map, float[,] distance_map, int MAX_SIZE = 10000)
        {   
            Debug.Log("In HybridAStar");
            
            // Control list
            maxSteerAngle = m_Car.m_MaximumSteerAngle*(float)Math.PI/180;
            //Debug.Log("Maximum steering angle: " + maxSteerAngle);
            steering = new float[]{0, -(float)Math.PI/8, (float)Math.PI/8};
            //orentations2 = f(steering);
            
            // Getting map info
            x_size = obstacle_map.GetLength(0);
            z_size = obstacle_map.GetLength(1);
            x_low = terrain_manager.myInfo.x_low;
            x_high = terrain_manager.myInfo.x_high;
            z_low = terrain_manager.myInfo.z_low;
            z_high = terrain_manager.myInfo.z_high;
            x_res = (x_high - x_low)/x_size;
            z_res = (z_high - z_low)/z_size;
            arc = (float)Math.Sqrt(Math.Pow(x_res, 2) + Math.Pow(z_res, 2));

            // Create the startnode
            startNode = new Node(start_pos.x, start_pos.z, start_angle, calculateGridIndex(start_pos.x, start_pos.z), 0, 0, 0, null);
            //Debug.Log(calculateGridIndex(start_pos.x, start_pos.z));

            // Create the goalnode
            goalNode = new Node(goal_pos.x, goal_pos.z, 0, calculateGridIndex(goal_pos.x, goal_pos.z), 0, 0, 0, null);
            //Debug.Log(calculateGridIndex(goal_pos.x, goal_pos.z));

            // Creating the open set (Priority queue for guided search of map)
            FastPriorityQueue<Node> openSet = new FastPriorityQueue<Node>(MAX_SIZE);

            // Creating the closed set (Dictionary for book keeping of expanded nodes)
            Dictionary<float,Node> closedSet = new Dictionary<float, Node>();
            MultiDictionary<float, float, Node> closedSet2 = new MultiDictionary<float, float, Node>();

            // Starting the algorithm
            openSet.Enqueue(startNode, startNode.f);

            while(openSet.Count > 0)
            {   

                if(openSet.Count >= MAX_SIZE)
                {
                    Debug.Log("Open set is full");
                    return null;
                }

                // Getting node with highest priority from open set
                n = openSet.Dequeue();

                // If the node is in the vicinity of the goal, assign it as parent to the goalnode and return the goalnode
                if(calculateEuclidean(n.x, n.z, goalNode.x, goalNode.z) < 5)
                {
                    Node finalNode = new Node(goalNode.x, goalNode.z, goalNode.theta, calculateGridIndex(goalNode.x, goalNode.z), 0, 0, 0, n);
                    goalNode = finalNode;
                    return goalNode;
                }

                // Expand node
                foreach(float steerAngle in steering)
                {   
                    float actualAngle = n.theta + steerAngle;
                    float dx = (float)Math.Cos(actualAngle)*arc;
                    float dz = (float)Math.Sin(actualAngle)*arc;
                    sucessor = new Node(n.x + dx, n.z + dz, actualAngle, calculateGridIndex(n.x + dx, n.z + dz), 0, 0, 0, n);

                    // Check traversability
                    try
                    {
                        if(obstacle_map[(int)Math.Round((sucessor.x - x_low) / x_res - 0.5),(int)Math.Round((sucessor.z - z_low) / z_res - 0.5)] == 1)
                        {   
                            draw1 = new Vector3(n.x, 0, n.z);
                            draw2 = new Vector3(sucessor.x, 0, sucessor.z);
                            Debug.DrawLine(draw1, draw2, Color.yellow, 100f);
                            continue;
                        }
                    }
                    catch(IndexOutOfRangeException e) 
                    {
                        Debug.Log("x: " + (sucessor.x - x_low) / 1);
                        Debug.Log("x index: " + (int)Math.Round((sucessor.x - x_low) / 1));
                        Debug.Log("z: " + (sucessor.z - z_low) / 1);
                        Debug.Log("z index: " + (int)Math.Round((sucessor.z - z_low) / 1));
                        Debug.Log(e);
                        return null;
                    }

                    // Check if the sucessor is expanded  !closedSet.ContainsKey(sucessor.gridIdx)
                    try 
                    {
                        float orentation = (float)Math.PI;
                        foreach (float direc in orentations)
                        {
                            if ((float)Math.Abs(sucessor.theta - direc)<=(float)Math.PI/8)
                            {
                                orentation = direc;
                                break;
                            }
                        }
                        //if one node per cell we use !closedSet.ContainsKey(sucessor.gridIdx) 
                        if (closedSet2.Get(sucessor.gridIdx, orentation) == null)
                        {
                            float steeringPenalty = 1 - steerAngle/maxSteerAngle; // (0,1)

                            if(distance_map[(int)Math.Round((sucessor.x - x_low) / x_res - 0.5), (int)Math.Round((sucessor.z - z_low) / z_res - 0.5)] > 5)
                            {
                                float obstaclePenalty = 0;
                                sucessor.g = n.g + (float)Math.Sqrt(2) + steeringPenalty + obstaclePenalty;
                            }
                            else
                            {
                                float obstaclePenalty = 5 - distance_map[(int)Math.Round((sucessor.x - x_low) / x_res - 0.5), (int)Math.Round((sucessor.z - z_low) / z_res - 0.5)];
                                sucessor.g = n.g + (float)Math.Sqrt(2) + (float)Math.Sqrt(2) * steeringPenalty + (float)Math.Sqrt(2)*obstaclePenalty;
                            }

                            bool flag = false;

                            foreach (Node one in openSet)
                            {
                                // Check if the sucessor has a neighbour in the same cell
                                if (one.gridIdx == sucessor.gridIdx && one.theta == sucessor.theta)
                                {
                                    // Remove the neighbour from the cell if the sucessor has lower cost
                                    if (sucessor.g < one.g)
                                    {
                                        flag = true;
                                        openSet.Remove(one);
                                        break;
                                    }
                                }
                            }

                            if (!openSet.Contains(sucessor) || flag)
                            {
                                // Add the sucessor to the open list if it is not there or it has lower cost than a currently existing node in the same cell
                                sucessor.h = calculateEuclidean(sucessor.x, sucessor.z, goalNode.x, goalNode.z);
                                sucessor.f = sucessor.g + sucessor.h;
                                openSet.Enqueue(sucessor, sucessor.f);

                                // Drawing some stuff
                                
                                draw1 = new Vector3(n.x, 0, n.z);
                                draw2 = new Vector3(sucessor.x, 0, sucessor.z);
                                Debug.DrawLine(draw1, draw2, Color.blue, 300f);

                                // Push node onto the set of expanded nodes
                                //closedSet.Add(sucessor.gridIdx, sucessor);
                                closedSet2.Set(sucessor.gridIdx, orentation, sucessor);
                            }
                        }
                    }
                    catch(Exception e)
                    {
                        Debug.Log(e);
                    }
                }
            }

            Debug.Log("No path found!");
            return null;
        }

        public Node AStar(TerrainManager terrain_manager, DroneController m_Drone, Vector3 start_pos, Vector3 goal_pos, float[,] obstacle_map, float[,] distance_map, int MAX_SIZE = 10000)
        {
            Debug.Log("In AStar");

            // Getting map info
            x_size = obstacle_map.GetLength(0);
            z_size = obstacle_map.GetLength(1);
            x_low = terrain_manager.myInfo.x_low;
            x_high = terrain_manager.myInfo.x_high;
            z_low = terrain_manager.myInfo.z_low;
            z_high = terrain_manager.myInfo.z_high;
            x_res = (x_high - x_low) / x_size;
            z_res = (z_high - z_low) / z_size;
            arc = (float)Math.Sqrt(Math.Pow(x_res, 2) + Math.Pow(z_res, 2));

            // Create the startnode
            startNode = new Node(start_pos.x, start_pos.z, 0, calculateGridIndex(start_pos.x, start_pos.z), 0, 0, 0, null);
            Debug.Log(calculateGridIndex(start_pos.x, start_pos.z));

            // Create the goalnode
            goalNode = new Node(goal_pos.x, goal_pos.z, 0, calculateGridIndex(goal_pos.x, goal_pos.z), 0, 0, 0, null);
            Debug.Log(calculateGridIndex(goal_pos.x, goal_pos.z));

            // Creating the open set (Priority queue for guided search of map)
            FastPriorityQueue<Node> openSet = new FastPriorityQueue<Node>(MAX_SIZE);

            // Creating the closed set (Dictionary for book keeping of expanded nodes)
            Dictionary<float, Node> closedSet = new Dictionary<float, Node>();
            MultiDictionary<float, float, Node> closedSet2 = new MultiDictionary<float, float, Node>();

            // Starting the algorithm
            openSet.Enqueue(startNode, startNode.f);

            //Explore new nodes
            while (openSet.Count > 0)
            {
                //If the nodes waiting for expand are too much, we fail to find the path
                if (openSet.Count >= MAX_SIZE)
                {
                    Debug.Log("Open set is full");
                    return null;
                }

                // Getting node with highest priority from open set
                n = openSet.Dequeue();

                // If the node is in the vicinity of the goal, assign it as parent to the goalnode and return the goalnode
                if (calculateEuclidean(n.x, n.z, goalNode.x, goalNode.z) < 5)
                {
                    Node finalNode = new Node(goalNode.x, goalNode.z, goalNode.theta, calculateGridIndex(goalNode.x, goalNode.z), 0, 0, 0, n);
                    goalNode = finalNode;
                    return goalNode;
                }

                // Expand node
                foreach (float Angle in orentations)
                {
                    float actualAngle = n.theta + Angle;
                    float dx = (float)Math.Cos(actualAngle) * arc;
                    float dz = (float)Math.Sin(actualAngle) * arc;
                    sucessor = new Node(n.x + dx, n.z + dz, actualAngle, calculateGridIndex(n.x + dx, n.z + dz), 0, 0, 0, n);

                    // Check traversability
                    try
                    {
                        if (obstacle_map[(int)Math.Round( (sucessor.x - x_low) / x_res - 0.5), (int)Math.Round((sucessor.z - z_low) / z_res - 0.5)] == 1)
                        {
                            Debug.Log("x index: " + (Math.Round((sucessor.x - x_low) / x_res - 0.5)));
                            draw1 = new Vector3(n.x, 0, n.z);
                            draw2 = new Vector3(sucessor.x, 0, sucessor.z);
                            Debug.DrawLine(draw1, draw2, Color.yellow, 100f);
                            continue;
                        }
                    }
                    catch (IndexOutOfRangeException e)
                    {
                        Debug.Log("x: " + (sucessor.x - x_low) / 1);
                        Debug.Log("x index: " + (int)Math.Round((sucessor.x - x_low) / 1));
                        Debug.Log("z: " + (sucessor.z - z_low) / 1);
                        Debug.Log("z index: " + (int)Math.Round((sucessor.z - z_low) / 1));
                        Debug.Log(e);
                        return null;
                    }

                    // Check if the sucessor is expanded  
                    try
                    {
                        float orentation = (float)Math.PI;
                        foreach (float direc in orentations)
                        {
                            if ((float)Math.Abs(sucessor.theta - direc) <= (float)Math.PI / 8)
                            {
                                orentation = direc;
                                break;
                            }
                        }
                        //Allow only one node per cell If allow many nodes per cell we use !closedSet.ContainsKey(sucessor.gridIdx) closedSet2.Get(sucessor.gridIdx, orentation) == null
                        if (closedSet2.Get(sucessor.gridIdx, orentation) == null)
                        {

                            if(distance_map[(int)Math.Round((sucessor.x - x_low) / x_res - 0.5), (int)Math.Round((sucessor.z - z_low) / z_res - 0.5)] > 5)
                            {
                                sucessor.g = n.g + arc;
                            }
                            else
                            {
                                float obstaclePenalty = 5 - distance_map[(int)Math.Round((sucessor.x - x_low) / x_res - 0.5), (int)Math.Round((sucessor.z - z_low) / z_res - 0.5)];
                                sucessor.g = n.g + arc + obstaclePenalty;
                            }

                            bool flag = false;

                            //Check and update the lower cost node in same cell
                            foreach (Node one in openSet)
                            {
                                // Check if the sucessor has a neighbour in the same cell
                                if (one.gridIdx == sucessor.gridIdx && one.theta == sucessor.theta)
                                {
                                    // Remove the neighbour from the cell if the sucessor has lower cost
                                    if (sucessor.g < one.g)
                                    {
                                        flag = true;
                                        openSet.Remove(one);
                                        break;
                                    }
                                }
                            }

                            //Add the new qualified sucessor node to closeSet
                            if (!openSet.Contains(sucessor) || flag)
                            {
                                // Add the sucessor to the open list if it is not there or it has lower cost than a currently existing node in the same cell
                                sucessor.h = calculateEuclidean(sucessor.x, sucessor.z, goalNode.x, goalNode.z);
                                sucessor.f = sucessor.g + sucessor.h;
                                openSet.Enqueue(sucessor, sucessor.f);

                                // Drawing some stuff
                                /*
                                draw1 = new Vector3(n.x, 0, n.z);
                                draw2 = new Vector3(sucessor.x, 0, sucessor.z);
                                Debug.DrawLine(draw1, draw2, Color.blue, 100f);
                                */
                                // Push node onto the set of expanded nodes
                                closedSet2.Set(sucessor.gridIdx, orentation, sucessor);
                            }
                        }
                    }
                    catch (Exception e)
                    {
                        Debug.Log(e);
                    }
                }
            }
            Debug.Log("No path found!");
            return null;
        }
    }

    public class Node : FastPriorityQueueNode
    {
        public float x { get; set; }
        public float z { get; set; }
        public float theta { get; set; }
        public float g { get; set; }
        public float h { get; set; }
        public float f { get; set; }
        public int gridIdx { get; set;}
        public Node parent { get; set; }

        public Node(float x, float z, float theta, int gridIdx, float g = 0, float h = 0, float f = 0, Node parent = null)
        {
            this.x = x;
            this.z = z;
            this.theta = theta;
            this.g = g;
            this.h = h;
            this.f = g+h;
            this.gridIdx = gridIdx;
            this.parent = parent;
        }
    }

    public class MultiDictionary<Key1, Key2, Value>
    {

        Dictionary<Key1, Dictionary<Key2, Value>> mDict1 = new Dictionary<Key1, Dictionary<Key2, Value>>();


        public void Set(Key1 key1, Key2 key2, Value value)
        {
            if (mDict1.ContainsKey(key1))
            {
                var dict2 = mDict1[key1];
                if (dict2.ContainsKey(key2))
                    dict2[key2] = value;
                else
                    dict2.Add(key2, value);
            }
            else
            {
                var dict2 = new Dictionary<Key2, Value>();
                dict2.Add(key2, value);
                mDict1.Add(key1, dict2);
            }
        }

        public Value Get(Key1 key1, Key2 key2, Value defaultValue = default(Value))
        {
            if (mDict1.ContainsKey(key1))
            {
                var dict2 = mDict1[key1];
                if (dict2.ContainsKey(key2))
                    return dict2[key2];
            }
            return defaultValue;
        }
    }
    }