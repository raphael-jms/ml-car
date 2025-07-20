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
    public class DriverAIEvaluation : Agent
    {
        // Learning environment setup
        [SerializeField] private CarController m_Car;
        [SerializeField] private Rigidbody rb;
        [SerializeField] private TerrainManager terrain_manager;

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
        private int num_rays = 9;
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
            trajectory.Clear(); 

            // get trajectory
            trajectory = GetTrajectory();
            AStarPathfinder.CreateColliders(trajectory);
            trajectory_initial_length = trajectory.Count;

            last_distance = (transform.position - trajectory[0]).magnitude;

            nextTargets.Clear();
            for (int i = 0; i < numNextTargets; i++)
            {
                AddNextTargetIfExists();
            }
        }

        private List<Vector3> GetTrajectory()
        {
            var goal_pos = terrain_manager.myInfo.goal_pos;
            var start = transform.position;
            var traversability = terrain_manager.myInfo.traversability;

            float x_low = terrain_manager.myInfo.x_low;
            float x_high = terrain_manager.myInfo.x_high;
            float z_low = terrain_manager.myInfo.z_low;
            float z_high = terrain_manager.myInfo.z_high;
            int x_N = terrain_manager.myInfo.x_N;
            int z_N = terrain_manager.myInfo.z_N;

            float grid_size_x = (float)(x_high - x_low) / x_N;
            float grid_size_z = (float)(z_high - z_low) / z_N;

            int start_x = Mathf.RoundToInt((start.x - x_low) / grid_size_x);
            int start_z = Mathf.RoundToInt((start.z - z_low) / grid_size_z);
            int goal_x = Mathf.RoundToInt((goal_pos.x - x_low) / grid_size_x);
            int goal_z = Mathf.RoundToInt((goal_pos.z - z_low) / grid_size_z);

            Debug.LogFormat("start: {0}, start idx: {1}, {2}, goal: {3}, goal idx {4}, {5}", start, start_x, start_z, goal_pos, goal_x, goal_z);
            Debug.LogFormat("array with {0}, {1}", traversability.GetLength(0), traversability.GetLength(1));

            var whole_path = AStarPathfinder.FindPath(traversability, start_x, start_z, goal_x, goal_z);
            var traj = whole_path.WorldPath;
            traj.RemoveAt(traj.Count - 1); // dirty fix, should better fix A Star end instead
            traj.Add((goal_pos+new Vector3(5f, 0f, 5f)+traj.Last())/2f);
            traj.Add(goal_pos);

            // traj.Insert(0, new Vector3(start_x * 10f, 0f, start_z * 10f));

            Vector3 offset = traj[0] - transform.position;
            for(int i = 0; i < traj.Count - 1; i++)
            {
                traj[i] -= offset;
                traj[i] += new Vector3(5f, 0, 5f);
            }

            Debug.Log(string.Join(", ", traj));

            AStarPathfinder.Plot(whole_path);

            return traj;
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
                var t = transform.InverseTransformDirection(tar - transform.position);
                sensor.AddObservation( Normalize( t.x, 2.0f*xrange.Min, 2.0f*xrange.Max));
                sensor.AddObservation( Normalize( t.z, 2.0f*yrange.Min, 2.0f*yrange.Max));
                n_tars.Add(Normalize( t.x, 2.0f*xrange.Min, 2.0f*xrange.Max));
                n_tars.Add(Normalize( t.z, 2.0f*yrange.Min, 2.0f*yrange.Max));
            }

            Debug.LogFormat("car pos {0} - target pos {1} - t {2} - ntars {3}, {4}", transform.position, nextTargets[0], transform.InverseTransformDirection(nextTargets[0] - transform.position), n_tars[0], n_tars[1]);

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
            float current_distance = (transform.position - nextTargets[0]).magnitude;
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