# Legged Hopping Robot
The spring-loaded inverted pendulum (SLIP) describes gait with a point mass rebounding on spring legs. The model captures the center of mass dynamics observed in running animals and has become a basic gait template in biomechanics and robotics for studying the dynamics and control of compliant-legged locomotion. 

To survey various contact detection methods, I used the SLIP model to generate a hopping gait for a single-legged robot. This was done using a state machine with 2 modes for flight and stance respectively. In flight mode, radial position control is applied to maintain a certain leg length and touchdown angle and in the stance mode, an active damping control is used to add energy to the system to make it hop. I use Raibert's formula<sup>1</sup> to calculate the new touchdown angle for each mode switch from stance to flight. I have also implemented joint space, and position control which can also be used to achieve the same gaits. 
The momentum observer applied here is in discrete time and the outputs are filtered with a given frequency. 


![**Vertical Hopping Only**](https://github.com/bosky2001/CourseWork/blob/main/mjcpy/leg/VerticalHop.gif)



## References & Special Thanks  :
#### This code is based on the following:

1. [Dynamically Stable Legged Locomotion](https://www.ri.cmu.edu/pub_files/pub3/raibert_marc_h_1983_1/raibert_marc_h_1983_1.pdf)
2. [Discrete Time filtered MBO for a simple pendulum](https://github.com/meghna30/gmm_obs_pendulum)
3. [Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains](https://ieeexplore.ieee.org/document/8460904)

