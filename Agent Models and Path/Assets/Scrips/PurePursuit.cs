using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    [RequireComponent(typeof(DroneController))]


    public class PurePursuit
    {
        public float view = 2f;                 //look-ahead distance
        public float viewcontrol = 0.1f;       //look forward gain
        public float speedcontrol = 1;         //speed proportional gain
        public float timeslot = 0.1f;          // time tick
        public float axlesize = 2.9f;          //wheel base of vehicle

        public int ind = 0;
        public int old_nearest_point_index = -1;
        Node2 current;
        List<Vector3> my_path = new List<Vector3>();
        //List<Node2> track_Node = new List<Node2>();

    public PurePursuit( List<Vector3> my_path)
        {
            this.my_path = my_path;
 
        }


        public Node2 PurePursuitA(Node2 current)
        {
            //set the current node
            current.rear_x = (float)(current.x - ((axlesize / 2) * Math.Cos(current.theta)));
            current.rear_z = (float)(current.z - ((axlesize / 2) * Math.Sin(current.theta)));
            this.current = current;
            //track_Node.Add(current);

            //speed (m/s)
            float tager_speed = 1.8f / 3.6f;
            //float time = 100;  // max simulation time

            ind = Calc_Target_Index(current, my_path);

            float ai = Pcontrol(tager_speed, current.v);
            float di = PurePursuitControl(current, my_path, ind);
            current = Update(current, ai, di);
            return current;
            /* while (time > 0 && ind< my_path.Count)
             {
                 //Calculate control input
                 float ai = Pcontrol(tager_speed, current.v);
                 float di = PurePursuitControl(current, my_path, ind);

                 //Debug.Log("control input ready  : "+ai+"and"+di);
                 current = Update(current, ai, di);
                 track_Node.Add(current);

                 time = time -timeslot;
                 if (ind == my_path.Count)
                     break;
             }
             return track_Node;
            */
        }

        public Node2 Update(Node2 current,float vt,float delta)
        {
            Node2 newnode = new Node2();
            newnode.x = current.x + current.v * (float)Math.Cos(current.theta) * timeslot;
            newnode.z = current.z + current.v * (float)Math.Sin(current.theta) * timeslot;
            newnode.theta = current.theta + current.v / axlesize * (float)Math.Tan(delta) * timeslot;
            newnode.v = current.v + vt * timeslot;
            newnode.rear_x = (float)(current.x - ((axlesize / 2) * Math.Cos(current.theta)));
            newnode.rear_z = (float)(current.z - ((axlesize / 2) * Math.Sin(current.theta)));


            return newnode; 
        }
        public float Pcontrol(float targetv,float currentv)
        {
            float v_change = speedcontrol * (targetv - currentv);

            return v_change;
        }
        public float PurePursuitControl(Node2 current, List<Vector3> my_path, int pind)
        {
            int index = Calc_Target_Index(current, my_path);

            if (pind >= index)
                index = pind;

            float tx = 0;
            float tz = 0;
            if(index < my_path.Count)
            {
                tx = my_path[index].x;
                tz = my_path[index].z;

            }
            else  // toward goal
            {
                tx = my_path[my_path.Count-1].x;
                tz = my_path[my_path.Count - 1].z;
                index = my_path.Count - 1;
            }

            float alpha = (float)Math.Atan2(tz - current.rear_z, tx - current.rear_x) - current.theta;
            if (current.v < 0)
            {
                alpha = (float)Math.PI - alpha;

            }
            //# update look ahead distance
            float Lf = viewcontrol * current.v + view;

            float delta = (float)Math.Atan2(2.0 * axlesize * Math.Sin(alpha) / Lf, 1.0);

            ind = index;
            return delta;
        }
        private float CalculateEuclidean(float x1, float z1, float x2, float z2)
        {
            return (float)Math.Sqrt(Math.Pow((x1 - x2), 2) + Math.Pow((z1 - z2), 2));
        }
        private int Calc_Target_Index(Node2 current, List<Vector3> my_path)
        {
            // List<float> dx = new List<float>();
            //List<float> dz = new List<float>();
            int index = 0;
            List<float> dist = new List<float>();
            float store = 10000;
            if (old_nearest_point_index == -1)
            {
                foreach (Vector3 one in my_path)
                {
                    dist.Add(CalculateEuclidean(current.x, current.z, one.x, one.z));
                }
                for (int i = 0; i < dist.Count; i++)
                {
                    if (dist[i] < store)
                    {
                        store = dist[i];
                        index = i;
                    }
                }
                old_nearest_point_index = index;
            }
            else
            {
                index = old_nearest_point_index;
                float distance_this_index = CalculateEuclidean(current.rear_x, current.rear_z, my_path[index].x, my_path[index].z);
                while((index + 1) < my_path.Count)
                {
                    float distance_next_index = CalculateEuclidean(current.rear_x, current.rear_z, my_path[index + 1].x, my_path[index + 1].z);
                    if(distance_this_index < distance_next_index)
                    {
                        break;
                    }
                    index += 1;
                }
                old_nearest_point_index = index;
            }
   
            //Debug.Log("the nearest index is:" + index);
       
            float L = CalculateEuclidean(current.x,current.z,current.rear_x,current.rear_z);
            // update look ahead distanced
            float Lf = viewcontrol * current.v + view;

            // search look ahead target point index
            while ( Lf > CalculateEuclidean(current.rear_x, current.rear_z, my_path[index].x, my_path[index].z))
            {
                if((index + 1)>= my_path.Count)
                {
                    break;
                }
                index += 1;
            }

            //Debug.Log("the nearest(view added) index is:" + index);
            
            return index; 
        }

    }

    public class Node2 
    {
        public float x { get; set; }
        public float z { get; set; }
        public float rear_x { get; set; }
        public float rear_z { get; set; }
        public float theta { get; set; }   
        public float v { get; set; }
        public Node2()
        {


        }
        public Node2(float x, float z, float rear_x, float rear_z, float theta,  float v=0)
        {
            this.x = x;
            this.z = z;
            this.theta = theta;
            this.v = v;
            this.rear_x = rear_x;
            this.rear_z = rear_z;

        }
    }
}