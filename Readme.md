# Joint level assist-as-needed controller for gait rehabilitation
Developed a assistive controller for the swing phase of gait. The novelty of the controller is that joint level assistive torques are provided in an assist-as-needed manner based on the deviations of the ankle joint from a normative trajectory. Besides, the assistance is only provided if the ankle deviates beyond a preset distance normal to the normative path.

## Modeling of lower limb dynamics
Used a double pendulum model with link and joint properties set to mimick the human lower extremity (thigh and shank segment; hip, knee and ankle joints). 

## Deriving normative gait trajectories
Obtained hip and knee joint trajectories for the swing phase of gait from the following gait database for healthy individuals. Processed the raw x-y-z data to obtain the hip angle and knee angle. The processed data is available in the csv file 'Gait desired Final'. 

[Article link](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5922232/)

## Simulations of desired gait, undesired gait and assisted gait

### Desired gait
An inverse dynamics controller was used to track the desired joint trajectories. 

### Undesired gait
Undesired gait was generated from the desired gait trajectories by adding a perturbing signal at each joint as implemented in the *obtain_trajectory_data.py* script. The inverse dynamic controller was used to track the undesired gait trajectories.

### Assisted gait
The undesired gait was simulated as described previously. An assistive controller was used on top of the inverse dyn controller.

*Assist-as-needed implementation* - A PD controller was designed to correct deviations from desired gait trajectory when the deviation of the ankle joint from the desired ankle path was greater than 5cm. The PD gains were tuned by trial and error.

## Results







