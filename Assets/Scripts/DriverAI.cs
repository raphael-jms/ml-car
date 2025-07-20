using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityStandardAssets.CrossPlatformInput;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class DriverAI : Agent
    {
        // Learning environment setup
        [SerializeField] private CarController m_Car;
        [SerializeField] private Rigidbody rb;

        // ranges for random initialization
        private (float Min, float Max) xrange = (-5.0f, 5.0f); // position
        private (float Min, float Max) yrange = (-5.0f, 5.0f);
        private (float Min, float Max) xvel   = (-1.0f, 1.0f); // velocity
        private (float Min, float Max) yvel   = (-1.0f, 1.0f);
        private GridTrajectoryGenerator traj_gen;
        private List<Vector3> trajectory;
        private int trajectory_initial_length;
        private LayerMask triggerLayer;
        private LayerMask wallLayer;

        int decision_counter;
        float last_distance;
        int decisions_since_last;

        private int numNextTargets = 5;
        private int num_rays = 6;
        private Quaternion roation_rays;
        private List<Vector3> nextTargets;
        
        public override void Initialize()
        {
            base.Initialize();
            traj_gen = new GridTrajectoryGenerator(10, 10);
            triggerLayer = LayerMask.NameToLayer("Trigger");
            wallLayer = LayerMask.NameToLayer("Wall");
            roation_rays = Quaternion.Euler(0f, 360f/((float)num_rays), 0f);
            nextTargets = new List<Vector3>();
            trajectory = new List<Vector3>();
        }

        public override void OnEpisodeBegin()
        {
            decision_counter = 0;
            decisions_since_last = 0;

            int breaker = 0;
            while(trajectory.Count < 6) {
                if (breaker == 100) {break;}
                trajectory = TupleToVector(traj_gen.GenerateTrajectory());
                breaker++;
            }
            trajectory_initial_length = trajectory.Count;

            transform.localPosition = trajectory[0];
            rb.velocity = new Vector3(0f, 0f, 0f);
            transform.localRotation = Quaternion.LookRotation(trajectory[1] - trajectory[0]);
            rb.angularVelocity = new Vector3(0, 0, 0);

            // Create cubes
            traj_gen.CreateCorridorWalls(transform.parent);
            traj_gen.CreateColliders(transform.parent);

            // Debug.LogFormat("Reward at start: {0}", GetCumulativeReward());
            last_distance = (transform.localPosition - trajectory[0]).magnitude;

            nextTargets.Clear();
            for (int i = 0; i < numNextTargets; i++)
            {
                AddNextTargetIfExists();
            }
        }

        private void AddNextTargetIfExists()
        {
            if (trajectory.Count > 0)
            {
                nextTargets.Add(trajectory[0]);
                trajectory.RemoveAt(0);
            } else {
                nextTargets.Add(nextTargets.Last());
            }
        }

        private void RemoveReachedTarget()
        {
            if (nextTargets.Count != 0)
            {
                nextTargets.RemoveAt(0);
            } else {
                Debug.LogWarning("nextTargets was empty");
            }
        }

        private List<Vector3> TupleToVector(List<(float x, float z)> traj_tuple)
        {
            // float y_val = transform.localPosition.y;
            float y_val = 1.0f;
            List<Vector3> traj_vec = new List<Vector3>();

            foreach(var t in traj_tuple)
            {
                traj_vec.Add(new Vector3(t.x, y_val, t.z));
            }
            return traj_vec;
        }

        private float Normalize(float val, float minv, float maxv)
        {
            return (val - (maxv+minv)/2f) / (maxv - minv);
        }
        
        public override void CollectObservations(VectorSensor sensor)
        {
            List<float> measurements = new List<float>();

            Vector3 local_dir = new Vector3(1,0,0);
            for (int i = 0; i < num_rays; i++)
            {
                RaycastHit hit;
                Physics.Raycast(transform.position, transform.TransformDirection(local_dir), out hit, Mathf.Infinity);
                local_dir = roation_rays * local_dir;

                sensor.AddObservation(hit.distance/10);
                measurements.Add(hit.distance/10);
            }

            // velocity
            Vector3 localVelocity = transform.InverseTransformDirection(rb.velocity);
            sensor.AddObservation( Normalize( (float)localVelocity.x, 2.0f*xvel.Min, 2.0f*xvel.Max) );
            sensor.AddObservation( Normalize( (float)localVelocity.z, 2.0f*yvel.Min, 2.0f*yvel.Max) );

            // relative position of the goal

            List<float> n_tars = new List<float>();
            foreach (var tar in nextTargets)
            {
                var t = transform.InverseTransformDirection(tar - transform.localPosition);
                sensor.AddObservation( Normalize( t.x, 2.0f*xrange.Min, 2.0f*xrange.Max));
                sensor.AddObservation( Normalize( t.z, 2.0f*yrange.Min, 2.0f*yrange.Max));
                n_tars.Add(Normalize( t.x, 2.0f*xrange.Min, 2.0f*xrange.Max));
                n_tars.Add(Normalize( t.z, 2.0f*yrange.Min, 2.0f*yrange.Max));
            }

            // Debug.LogFormat("{0} == {1}, {2}, == {3}", string.Join(", ", measurements),
            //                                            Normalize( localVelocity.x, 2.0f*xvel.Min, 2.0f*xvel.Max), 
            //                                            Normalize( localVelocity.z, 2.0f*yvel.Min, 2.0f*yvel.Max), 
            //                                            string.Join(", ", n_tars) );
            
            decision_counter++;
            decisions_since_last++;
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            float steering = actions.ContinuousActions[0];
            float accellerationInput = actions.ContinuousActions[1];
            float handbrake = actions.ContinuousActions[2];

            m_Car.Move(steering,  accellerationInput, accellerationInput, handbrake);

            // Reward going to the goal
            float current_distance = (transform.localPosition - nextTargets[0]).magnitude;
            AddReward(0.005f * (current_distance - last_distance));
            last_distance = current_distance;

            if (decision_counter > 600) {
                // AddReward(-0.3f);
                Debug.LogFormat("Episode ended after {0} steps with reward {1}", decision_counter, GetCumulativeReward());
                EndEpisode();
            }

            // Penalize doing nothing
            AddReward(-0.0005f);
        }
            
        private void OnTriggerEnter(Collider collision)
        {
            var layerMask = collision.gameObject.layer;

            if (layerMask == triggerLayer)
            {
                // Debug.Log("Yay!!!");
                AddReward(1.0f);

                // Remove Trigger
                UnityEngine.Object.Destroy(collision.gameObject);

                // Update future targets
                AddNextTargetIfExists();
                RemoveReachedTarget();

                if (decisions_since_last != 0)
                {
                    // float speed_approx = ((float)decisions_since_last ) / ((float)trajectory_initial_length);
                    AddReward(0.75f * Mathf.Exp(-0.3f * decisions_since_last)); 
                    // Debug.LogFormat("Added another {0} bonus after {1} decisions", 0.75f * Mathf.Exp(-0.3f * decisions_since_last), decisions_since_last);
                }
                decisions_since_last = 0;

                // if (trajectory.Count == 0)
                // {
                //     AddReward(20);
                //     EndEpisode();
                // }
            }
            else if (layerMask == wallLayer)
            {
                // Debug.Log("Nay! :(");
                AddReward(-0.2f);
                // AddReward(-0.3f);
                Debug.LogFormat("Episode ended after {0} steps with reward {1}", decision_counter, GetCumulativeReward());
                EndEpisode();
            } else {
                Debug.Log("Unknown collision with " + collision.gameObject.name);
            }
        }

//         public override void Heuristic(in ActionBuffers actionsOut)
//         {
//             var continuousActionsOut = actionsOut.ContinuousActions;
            
// #if !MOBILE_INPUT
//             float handbrake = CrossPlatformInputManager.GetAxis("Jump");
// #else
//             float handbrake = 0f;
// #endif
            
//             continuousActionsOut[0] = CrossPlatformInputManager.GetAxis("Horizontal");  // steering
//             continuousActionsOut[1] = CrossPlatformInputManager.GetAxis("Vertical");    // acceleration
//             continuousActionsOut[2] = CrossPlatformInputManager.GetAxis("Vertical");    // brake
//             continuousActionsOut[3] = handbrake;                                        // handbrake
//         }
    }
}