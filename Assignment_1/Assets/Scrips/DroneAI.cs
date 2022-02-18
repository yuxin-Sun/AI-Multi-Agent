using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    //// Declaration of variables

    // Unity variables
    private DroneController m_Drone;
    Rigidbody my_rigidbody;
    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    // Planning variables
    Vector3 start_pos;
    Vector3 goal_pos;
    List<Node> my_path = new List<Node>();
    List<Node> dp_path = new List<Node>();

    // Tracking variables
    public float k_p = 16f, k_d = 12f;
    float to_path, to_target, distance, steering, acceleration, starting_timer = 0, stuck_timer = 0, reverse_timer = 0, break_timer = 0, old_acceleration = 0, new_acceleration, acceleration_change, my_speed = 0, old_angle, new_angle, angle_change, unstuck_error, old_unstuck_error = 100, unstuck_error_change, upcoming_angle, break_distance;
    int to_path_idx, to_target_idx, dummy_idx, dummy_idx2, lookahead = 0, my_max_speed = 25;
    bool starting_phase = true, is_stuck = false, is_breaking = false, counting = false, no_waypoint = true;
    Vector3 pos, difference, target_position, aheadOfTarget_pos, target_velocity, position_error, velocity_error, desired_acceleration, closest, null_vector = new Vector3(0,0,0), previous, next, desired_direction, next_error, my_position, vector, nextnext, next_direction;
    Node target, aheadOfTarget, closestNode;
    List<float> controls = new List<float>(), cum_angle_change = new List<float>();

    //// Definition of functions

    // Function that computes the appropriate lookahead depending on the current speed
    public int speedToLookahead(float my_speed)
    {
        lookahead = (int)(4 + Math.Sqrt(my_speed));
        return lookahead;
    }

    // Function that computes the appropriate maximum speed at the next turn
    public int angleToSpeed(float upcoming_angle)
    {
        if(Math.Abs(upcoming_angle) > 60)
        {
            return (int)5;
        }
        else if(Math.Abs(upcoming_angle) > 30)
        {
            return (int)10;
        }
        else
        {
            return (int)10;
        }
    }

    // Function that approximates the break distance based on the speed we currently have and the speed we want to have
    public float computeBreakDistance(float my_speed, float my_max_speed)
    {
        return (float)( Math.Pow(my_speed,2) - Math.Pow(my_max_speed,2) ) / (2*15);
    }
    
    private void Start()
    {
        // Get the drone controller
        m_Drone = GetComponent<DroneController>();
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
        Node goalNode = planner.AStar(terrain_manager, m_Drone, start_pos, goal_pos, obstacle_map, distance_map, 10000);

        if (goalNode == null)
        {
            Debug.Log("Pathing failed");
            this.enabled = false;
        }

        // Construct path
        my_path.Add(goalNode);
        Node parent = goalNode.parent;
        while (parent != null)
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

        Debug.Log("Tracking path");

        // Removing abudant nodes from path
        DouglasPeucker dp = new DouglasPeucker();
        dp_path = dp.DouglasPeuckerReduction(my_path, 1.5);
        Vector3 old_wpp = start_pos;
        foreach (Node n in dp_path)
        {
            Vector3 wp = new Vector3(n.x, 0, n.z);
            Debug.DrawLine(old_wpp, wp, Color.green, 100f);
            old_wpp = wp;
        }
    }

    private void FixedUpdate()
    {   
        // Execute your path here
        if(no_waypoint)
        {   
            no_waypoint = false;

            // Finding closest node on path
            to_path = 100;
            Debug.Log("To_path: " + to_path);
            to_path_idx = 0;
            Debug.Log("To_path_idx: " + to_path_idx);
            dummy_idx = 0;
            Debug.Log("Dummy_idx: " + dummy_idx);
            foreach(Node node in dp_path)
            {   
                Debug.Log("Going through the nodes");
                pos = new Vector3(node.x, 0, node.z);
                difference = pos - transform.position;
                distance = (float) Math.Sqrt( Math.Pow(difference.x,2) + Math.Pow(difference.z, 2) );
                        
                if(distance < to_path)
                {
                    to_path = distance;
                    Debug.Log("To_path: " + to_path);
                    to_path_idx = dummy_idx;
                    Debug.Log("To_path_idx: " + to_path_idx);
                }
                dummy_idx += 1;
                Debug.Log("Dummy_idx: " + dummy_idx);
            }

            Debug.Log("Index of previous node on path: " + to_path_idx);

            // Setting the desired_direction
            previous = new Vector3(dp_path[to_path_idx].x, 0, dp_path[to_path_idx].z);
            next = new Vector3(dp_path[to_path_idx + 1].x, 0, dp_path[to_path_idx + 1].z);
            desired_direction = next-previous;

            if(to_path_idx < dp_path.Count - 2)
            {
                nextnext = new Vector3(dp_path[to_path_idx + 2].x, 0, dp_path[to_path_idx + 2].z);
                next_direction = nextnext-next;
                upcoming_angle = Vector3.SignedAngle(desired_direction, next_direction, Vector3.up);
                Debug.Log("Upcoming angle change: " + upcoming_angle);
                my_max_speed = angleToSpeed(upcoming_angle);
                Debug.Log("Recommended max speed at next turn: " + my_max_speed);
            }
        }

        Debug.DrawLine(transform.position, previous, Color.cyan);
        Debug.DrawLine(transform.position, next, Color.magenta);

        // Keep track of target position and velocity
        my_position = new Vector3(transform.position.x, 0, transform.position.z);
        vector = my_position - previous;
        target_position = previous + Vector3.Project(vector, desired_direction.normalized) + desired_direction.normalized + (Vector3.up)*(transform.position.y);
        target_velocity = my_max_speed*desired_direction.normalized;
        Debug.DrawLine(transform.position, target_position, Color.white);
        Debug.DrawLine(target_position, target_position + target_velocity, Color.blue);


        // a PD-controller to get desired velocity
        position_error = target_position - transform.position;
        velocity_error = target_velocity - m_Drone.velocity;
        desired_acceleration = k_p * position_error + k_d * velocity_error;
        Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.red);
        m_Drone.Move(desired_acceleration.x, desired_acceleration.z);
        
        // Variables for switch condition
        next_error = next - previous - Vector3.Project(vector, desired_direction.normalized);
        Debug.Log("Distance to end of arc: " + next_error.magnitude);
    
        if(next_error.magnitude < 2)
        {   
            Debug.Log("Switching waypoints");
            no_waypoint = true;
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
