using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {   
        //// Declaration of variables

        // Unity variables
        private CarController m_Car;
        Rigidbody my_rigidbody;
        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        // Planning variables
        Vector3 start_pos;
        Vector3 goal_pos;
        Node parent;
        List<Node> my_path = new List<Node>();
        List<Node> dp_path = new List<Node>();
        List<Node> expand_path = new List<Node>();

        // Tracking variables
        public float k_p = 2f, k_d = 0.5f;
        float to_path, to_target, distance, steering, acceleration, starting_timer = 0, stuck_timer = 0, reverse_timer = 0, break_timer = 0, old_acceleration = 0, new_acceleration, acceleration_change, my_speed = 0, old_angle, new_angle, angle_change, unstuck_error, old_unstuck_error = 100, unstuck_error_change, clearance, max_clearance = 0, nextAngle, angle;
        int to_path_idx, last_path_idx, to_target_idx, dummy_idx, dummy_idx2, path_node_count = 0, lookahead = 0, my_max_speed = 25, stuck_times = 0;
        bool starting_phase = true, is_stuck = false, is_breaking = false, counting = false, no_waypoint = true;
        Vector3 pos, difference, target_position, aheadOfTarget_pos, target_velocity, position_error, velocity_error, desired_acceleration, closest, null_vector = new Vector3(0,0,0);
        Node target, aheadOfTarget, closestNode;
        List<float> controls = new List<float>(), cum_angle_change = new List<float>();

        //// Definition of functions
        // Function that computes the appropriate maximum speed depending on the curvature ahead
        public int curvatureToSpeed(float curvature_ahead)
        {   
            if(curvature_ahead > 60)
            {
                return (int)5;
            }
            else if(curvature_ahead > 45)
            {
                return (int)7;
            }
            else if(curvature_ahead > 30)
            {
                return (int)10;
            }
            else if (curvature_ahead > 20)
            {
                return (int)12;
            }
            else if (curvature_ahead > 10)
            {
                return (int)20;
            }
            else
            {
                return (int)25;
            }
        }

        // Function that computes the appropriate lookahead depending on the current speed
        public int speedToLookahead(float my_speed)
        {
            lookahead = (int)(4 + Math.Sqrt(my_speed));
            return lookahead;
        }

        private float calculateEuclidean(float x1, float z1, float x2, float z2)
        {
            return (float)Math.Sqrt(Math.Pow((x1 - x2), 2) + Math.Pow((z1 - z2), 2));
        }


        // Start function is run once every time the CarAI script is invoked
        private void Start()
        {
            // Get stuff
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            my_rigidbody = GetComponent<Rigidbody>();
            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;
            
            // Create mapper and compute obstacle map
            Debug.Log("Creating obstacle map of current terrain");
            Mapper mapper = new Mapper(terrain_manager);
            float[,] obstacle_map = mapper.configure_obstacle_map(terrain_manager, 1);
            float[,] distance_map = mapper.configure_distance_map(obstacle_map);

            // Create planner and find path
            Debug.Log("Planning path");
            Planner planner = new Planner();
            Node goalNode = planner.HybridAStar(terrain_manager, m_Car, start_pos, (float)Math.PI/2, goal_pos, obstacle_map, distance_map, 10000);
            
            // Disable code if path not found
            if(goalNode == null)
            {
                Debug.Log("Pathing failed");
                this.enabled = false;
            }

            // Construct path
            my_path.Add(goalNode);
            Node parent = goalNode.parent;
            while(parent != null)
            {  
                my_path.Add(parent);
                parent = parent.parent;
            }
            my_path.Reverse();

            // Plot path
            Vector3 old_wp = start_pos;
            foreach (Node n in my_path)
            {   
                Vector3 wp = new Vector3(n.x, 0, n.z);
                Debug.DrawLine(old_wp, wp, Color.red, 100f);
                old_wp = wp;
            }

            // Removing abudant nodes from path
            Debug.Log("old Path size" + my_path.Count);
            DouglasPeucker dp = new DouglasPeucker();
            dp_path = dp.DouglasPeuckerReduction(my_path, 0.5);
            Debug.Log("new Path size" + dp_path.Count);

            // Plot path
            Vector3 old_wpp = start_pos;
            foreach (Node n in dp_path)
            {   
                if(dp_path.IndexOf(n) < dp_path.Count - 1)
                {             
                    Vector3 thisPoint = new Vector3(n.x, 0, n.z);
                    Node nextNode = dp_path[dp_path.IndexOf(n)+1];  
                    Vector3 nextPoint = new Vector3(nextNode.x, 0, nextNode.z);
                    Vector3 direction = nextPoint - thisPoint;
                    angle = Vector3.SignedAngle(Vector3.right, direction, Vector3.up);

                    if(angle > 0)
                    {
                        angle -= 360;
                    }

                    n.theta = angle;
                }
                else
                {
                    n.theta = angle;
                }
                /*
                Vector3 wp = new Vector3(n.x, 0, n.z);
                Debug.DrawLine(old_wpp, wp, Color.blue, 100f);
                old_wpp = wp;
                */
            }


            float dist;

            // Expand the path
            foreach (Node n in dp_path)
            {
                //the index of current dp path node
                int indx = dp_path.IndexOf(n);

                //if it is not the last node(closest to goal)
                if (indx < dp_path.Count - 1)
                {
                    dist = calculateEuclidean(n.x, n.z, dp_path[indx + 1].x, dp_path[indx + 1].z);
                    if (dist <= 1.5f)
                    {
                        expand_path.Add(n);
                        continue;
                    }
                    else
                    {
                        expand_path.Add(n);
                        //insert number
                        int number = (int)Math.Floor(dist / 1)-1;
                        parent = n;
                        for (int i = 1; i <= number; i++)
                        {
                            float theta = dp_path[indx].theta;
                            Vector3 nextnode = new Vector3(dp_path[indx + 1].x, 0, dp_path[indx + 1].z);
                            Vector3 cnode = new Vector3(n.x,0,n.z);
                            Vector3 direction = nextnode - cnode;
                            cnode = cnode + direction.normalized *i * 1f;
                            float x = cnode.x;
                            float z = cnode.z;
                            int gridIdx = planner.calculateGridIndex(x, z);
                            Node newnode = new Node(x, z, theta, gridIdx, 0, 0, 0, parent);
                            parent = newnode;
                            expand_path.Add(newnode);
                        }
                        continue;
                    }
                }
                expand_path.Add(n);
            }

            Debug.Log("expand Path size" + expand_path.Count);

            // Plot path
            Vector3 old_wppp = start_pos;
            foreach (Node node in expand_path)
            {
                Vector3 wp = new Vector3(node.x, 0, node.z);
                Debug.DrawLine(old_wppp, wp, Color.yellow, 300f);
                old_wppp = wp;
            }
            

            // Getting angle changes in path
            for (int i = 0; i < dp_path.Count; i++)
            {   
                if(i == 0)
                {   
                    old_angle = dp_path[i].theta;
                    controls.Add(0);
                    continue;
                }

                new_angle = dp_path[i].theta;
                angle_change = new_angle - old_angle;
                old_angle = new_angle;
                controls.Add((float)Math.Abs(angle_change));
            }
            
            // Going to FixedUpdate()
            Debug.Log("Tracking path");
        }
        private void FixedUpdate2()
        {

        }
        private void FixedUpdate()
        {   

            // Starting out phase means no checking for stuck and no checking for breaking
            if(starting_phase)
            {   
                if(starting_timer < Time.time && !counting)
                {
                    starting_timer = Time.time + 5;
                    counting = true;
                }

                if(Time.time >= starting_timer)
                {
                    counting = false;
                    starting_phase = false;
                }

            }

            // Tracks the path generated by planner

            // Decide on lookahead based on speed;
            lookahead = speedToLookahead(my_speed);

            // Tracks forward along the path if not stuck
            if(!is_stuck)
            {
                //Once run ok, reset the stuck checker
                stuck_times = 0;

                // Finding closest node on path
                to_path = 1000;
                to_path_idx = 0;
                dummy_idx = 0;
                foreach(Node node in expand_path)
                {
                    pos = new Vector3(node.x, 0, node.z);
                    difference = pos - transform.position;
                    distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );
                    
                    if(distance < to_path)
                    {
                        to_path = distance;
                        to_path_idx = dummy_idx;
                    }
                    dummy_idx += 1;
                }

                // Saving data about node on path closest to the car
                closestNode = expand_path[to_path_idx];
                closest = new Vector3(closestNode.x, 0, closestNode.z);
                difference = closest - transform.position;
                to_path = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                // Finding target node on path
                to_target = 100;
                to_target_idx = 0;
                dummy_idx2 = 0;
                foreach(Node node in expand_path)
                {
                    if(dummy_idx2 < to_path_idx)
                    {   
                        dummy_idx2 += 1;
                        continue;
                    }

                    pos = new Vector3(node.x, 0, node.z);
                    difference = pos - closest;
                    distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                    if(Math.Abs(distance + to_path - lookahead) < to_target)
                    {
                        to_target = Math.Abs(distance + to_path - lookahead);
                        to_target_idx = dummy_idx2;
                    }
                    dummy_idx2 += 1;
                }

                Debug.Log("target node" + to_target_idx);

                // Finding closest node on coarse path
                float to_course_path = 100;
                int to_course_path_idx = 0;
                int dummy_idx3 = 0;
                foreach(Node node in dp_path)
                {
                    pos = new Vector3(node.x, 0, node.z);
                    difference = pos - transform.position;
                    distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );
                    
                    if(distance < to_course_path)
                    {
                        to_course_path = distance;
                        to_course_path_idx = dummy_idx3;
                    }
                    dummy_idx3 += 1;
                }

                // Get information about next point on course path
                try
                {
                    nextAngle = controls[to_course_path_idx + 1];
                }
                catch
                {
                    Debug.Log("Next node out of coarse path");
                }

                // Break condition
                try
                {
                    target = expand_path[to_target_idx];
                    aheadOfTarget = expand_path[to_target_idx + 1];
                }
                catch(Exception e)
                {   
                    Debug.Log("Goal reached");
                    Debug.Log(e);
                    this.enabled = false;
                }

                // Keep track of target position and velocity
                target_position = new Vector3(target.x, 0, target.z);
                aheadOfTarget_pos = new Vector3(aheadOfTarget.x, 0, aheadOfTarget.z);
                target_velocity = aheadOfTarget_pos-target_position;
                Debug.DrawLine(transform.position, target_position, Color.white);
                Debug.DrawLine(transform.position, transform.position + 10*target_velocity, Color.black);

                // a PD-controller to get desired velocity
                position_error = target_position - transform.position;
                velocity_error = target_velocity - my_rigidbody.velocity;
                desired_acceleration = k_p * position_error + k_d * velocity_error;

                // Apply controls
                steering = Vector3.Dot(desired_acceleration, transform.right);
                acceleration = Vector3.Dot(desired_acceleration, transform.forward);
                
                if (!starting_phase)
                {   if(to_course_path_idx+1< dp_path.Count)
                    {
                        Debug.Log("Next node on course path: " + (to_course_path_idx + 1));
                        Debug.Log("Closest angle: " + (dp_path[to_course_path_idx].theta));
                        Debug.Log("Upcoming angle: " + (dp_path[to_course_path_idx + 1].theta));
                        Debug.Log("Upcoming angle change: " + nextAngle);
                    }
                    my_max_speed = curvatureToSpeed(nextAngle);
                }

                /*
                if(!is_breaking)
                {
                    break_timer = 0;
                }
                */
            
                if(my_speed > my_max_speed)
                {
                    Debug.Log("now breaking");
                    //is_breaking = true;

                    /*
                    if(break_timer == 0)
                    {
                        break_timer = Time.time + 1;
                    }
                    */
                    
                    m_Car.Move(steering, -acceleration, -acceleration, 0f);

                    /*
                    if(Time.time > break_timer)
                    {
                        is_breaking = false;
                    }

                    if (controls[to_course_path_idx + 1] > 30)
                    {   
                        break_timer = Time.time + 1;                     
                    }
                    */
                }
                else
                {
                    m_Car.Move(steering, acceleration, acceleration, 0f);
                }

                // State variables for stuck condition
                my_speed = (float) Math.Sqrt( Math.Pow(my_rigidbody.velocity.x, 2) + Math.Pow(my_rigidbody.velocity.z, 2) );
                new_acceleration = (float) Math.Sqrt( Math.Pow(desired_acceleration.x, 2) + Math.Pow(desired_acceleration.z, 2) );
                acceleration_change = new_acceleration - old_acceleration;
                old_acceleration = new_acceleration;

                // Check stuckness. If stuck -> go to else statement below
                if(my_speed < 0.3 && acceleration_change < 0.0001 && !starting_phase)
                {   
                    if(stuck_timer <= Time.time && !counting)
                    {   
                        counting = true;
                        stuck_timer = Time.time + 1;
                    }
       
                    if (Time.time > stuck_timer)
                    {
                        Debug.Log(m_Car.CurrentSpeed);
                        Debug.Log("Car stuck detected");
                        counting = false;
                        is_stuck = true;
                    }
                }
            }

            // We are stuck
            else
            {

                stuck_times += 1;
                // Decide on lookbehind based on speed;
                lookahead = speedToLookahead(my_speed)+ 0 ;
                System.Random ran = new System.Random();
                if (stuck_times == 200)
                {
                    to_target_idx = ran.Next(expand_path.Count);
                    //stuck_times = 0;
                    Debug.Log("Random movemwnt");
                    no_waypoint = false;
                }
                if (no_waypoint)
                {
                    // Finding closest node on path
                    to_path = 100;
                    to_path_idx = 0;
                    dummy_idx = 0;
                    foreach(Node node in expand_path)
                    {
                        pos = new Vector3(node.x, 0, node.z);
                        difference = transform.position - pos;
                        distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                        if(distance < to_path)
                        {
                            to_path = distance;
                            to_path_idx = dummy_idx;
                        }

                        dummy_idx += 1;
                    }
                  
                    stuck_times += 1;
                    // Saving data about node on path closest to the car
                    closestNode = expand_path[to_path_idx];
                    closest = new Vector3(closestNode.x, 0, closestNode.z);
                    difference = closest - transform.position;
                    to_path = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                    // Finding target node on path
                    to_target = 100;
                    to_target_idx = 0;
                    dummy_idx2 = 0;
                    foreach(Node node in expand_path)
                    {
                        if(dummy_idx2 > to_path_idx)
                        {   
                            break;
                        }

                        pos = new Vector3(node.x, 0, node.z);
                        difference = pos - closest;
                        distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );

                        if(Math.Abs(distance + to_path - lookahead) < to_target)
                        {
                            to_target = Math.Abs(distance + to_path - lookahead);
                            to_target_idx = dummy_idx2;
                        }
                        dummy_idx2 += 1;
                    }
              
                    no_waypoint = false;
                }

                // Break condition
                try
                {
                    target = expand_path[to_target_idx];
                }
                catch(Exception e)
                {
                    Debug.Log("whole size id" + expand_path.Count + "to_target_idx:" + to_target_idx);
                    Debug.Log(e);
                    Debug.Log("Break condition Error");
                }

                // Keep track of target position and velocity
                target_position = new Vector3(target.x, 0, target.z);
                target_velocity = null_vector;
                Debug.DrawLine(transform.position, target_position);

                // a PD-controller to get desired velocity
                position_error = target_position - transform.position;
                velocity_error = target_velocity - my_rigidbody.velocity;
                desired_acceleration = k_p * position_error + k_d * velocity_error;

                // Apply controls
                steering = Vector3.Dot(desired_acceleration, transform.right);
                acceleration = Vector3.Dot(desired_acceleration, transform.forward);
                m_Car.Move(steering, acceleration, acceleration, 0f);

                // State variables for unstuck condition
                unstuck_error = (float) Math.Sqrt( Math.Pow(position_error.x, 2) + Math.Pow(position_error.z, 2) );
                unstuck_error_change = (float)Math.Abs(unstuck_error - old_unstuck_error);
                old_unstuck_error = unstuck_error;
                
                if(unstuck_error_change < 0.0001)
                {   
                    old_unstuck_error = 100;
                    is_stuck = false;
                    no_waypoint = true;
                }
            }
        }
    }
}
