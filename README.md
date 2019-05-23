# Global Robot Localization
The goal of this project is to implement generalized global localization algorithms for FRC (although it could be used in other applications).
## General Assumptions
All agorithms assume assume the following:
* Sensors that both measure the robot's movement (e.g. imu) and field data are being used
* A map of data from the field that contains any appropriate data for all sensors used is known
## Current Algorithms
Here is an up to date list of all algorithns implemented / being implemented

### Monte Carlo Localization
Monte Carlo localization is a particle filter based approach. The basic idea of this approach is that the position of the robot can be represented by a distribution of likely states, or particles. Over time, particles are moved and resampled based on sensor data from the robot and the distribution of likely states converges to the real state of the robot.

The general algorithm is as follows:
1. Initialization: Poses are randomly scattered and position across a map of the field. These are the particles.
2. Control update: Particles are shifted and moved in accordance with motion sensor data.
3. Observation update: Each particle is assigned a weight which cooresponds with how likely it is that that state is the state of the robot. This weight is calculated based on observed sensor data of the robot's sorroundings. A new set of particles is randomly resampled from the current particles based on each particles' weight.
4. Repeat from step 2.


### Kalman Filter
Kalman filters offer a unique opportunity to integrate sensor data, odometry data, and uncertainty into a state prediction. It represents the prediction as a "Gaussian blob", which creates a multi-dimensional normal distribution by combining the variances in to a "covariance matrix". This covariance matrix allows the algorithm to incorporate the potential relationship between elements of the state. One of the drawbacks of the normal algorithm is that it only functions accurately in linear systems: there are some variants of the Kalman filter which improve on some of its limitations:
1. Extended Kalman Filters
2. Unscented Kalman Filters
3. Multi Modal Kalman Filters

In every time step, the algorithm has two main steps:
1. Prediction Update: In this update, the algorithm moves forward one time step—it first updates the state vector ignoring outside influence, and then uses a control vector (which might contain odometry data) to update the state further.
2. Measurement Update: After moving forward one time step, the algorithm now incorporates sensor data into its calculations, and finally returns the resulting predicted Gaussian blob.
