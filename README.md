# Autonomous Driving AI

Just-for-fun side project trying to teach a car driving using RL in Unity. The code uses a quite realistic car model that includes nonlinearities such as skidding etc. and is taken from Unity standard assets.

## The learning environment: 

16 cars in parallel on randomized trajectories. The episode ends when a wall is hit. Then, a new trajectory is generated and the process repeats.

<img width="619" height="608" alt="training_env" src="https://github.com/user-attachments/assets/bd0d9b23-e2c9-4b98-a930-973cab240a44" />

Result:

https://github.com/user-attachments/assets/2d2a41cd-5af9-4b27-bbc8-d322c6bfa0aa


## Driving with pathfinding

Training an agent to find a path itself is hard - implementing it with a pathfinding algorithm (here: A*) is much easier. Agent and setup are the same, but I retrained the agent in a less cluttered environment since it seemed to rely mostly on the wall information, and not so much on the waypoints. You can find the code in the pathfinding branch.

<img width="619" height="623" alt="training_env_cluttered" src="https://github.com/user-attachments/assets/45e3ffe5-1d6a-4ec2-b9e3-4611d9ba305c" />

Result:

https://github.com/user-attachments/assets/859d4a79-80f1-4fc1-bedb-b30bf4401954

## Usage

This project requires Unity Editor version 2021.3.15f1. To see the learning environment, navigate to `Assets/Scenes` and open `main (car).unity`. If you are using the pathfinding branch, go to `Scenes` to load either the `Learning` scene or the `Evaluation` scene.
