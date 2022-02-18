using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]

    public class Tracker: MonoBehaviour
    {
        private float minDist = 100;
        private int minDistIdx = 0;
        private int Idx = 0;
        private Vector3 pos;
        private Vector3 difference;
        private float dist;
        private Node target;
        private Node aheadOfTarget;
        private Vector3 target_position;
        private Vector3 aheadOfTarget_pos;
        private Vector3 target_velocity;
        private Vector3 position_error;
        private Vector3 velocity_error;
        private Vector3 desired_acceleration;

        public Tracker()
        {
            ;
        }

        public float calculateAmplitude(float x, float z)
        {
            return (float)Math.Sqrt( Math.Pow(x,2) + Math.Pow(z, 2) );
        }

        public Vector3 get_control_input(List<Node> my_path, Rigidbody my_rigidbody, Vector3 my_position, float k_p, float k_d, int lookahead, bool is_stuck)
        {   
            Debug.Log("Within get_control_input");
            foreach(Node node in my_path)
            {
                pos = new Vector3(node.x, 0, node.z);
                difference = my_position - pos;
                Debug.Log("Difference: " + difference);
                dist = calculateAmplitude(difference.x, difference.z);

                if(dist < minDist)
                {
                    minDist = dist;
                    minDistIdx = Idx;
                }

                Idx += 1;
            }

            try
            {
                target = my_path[minDistIdx + lookahead];
                aheadOfTarget = my_path[minDistIdx + lookahead + 1];
            }
            catch(ArgumentOutOfRangeException e)
            {   
                Vector3 fail = new Vector3(0,0,0);
                return fail;
            }

            // Keep track of target position and velocity
            target_position = new Vector3(target.x, 0, target.z);
            aheadOfTarget_pos = new Vector3(aheadOfTarget.x, 0, aheadOfTarget.z);
            target_velocity = aheadOfTarget_pos-target_position;

            // a PD-controller to get desired velocity
            position_error = target_position - my_position;
            velocity_error = target_velocity - my_rigidbody.velocity;
            desired_acceleration = k_p * position_error + k_d * velocity_error;

            return desired_acceleration;
        }
    }
}
