# Particle Filters
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only modified file is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods.

## Project Introduction
Our robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

The objective of this project is the implementation of a two-dimensional particle filter in C ++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Comunication between uWebSocketIO and the main.cpp

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

* **INPUT**: values provided by the simulator to the c++ program
  * // sense noisy position data from the simulator
    * ["sense_x"]
    * ["sense_y"]
    * ["sense_theta"]
  * // get the previous velocity and yaw rate to predict the particle's transitioned state
    * ["previous_velocity"]
    * ["previous_yawrate"]
  * // receive noisy observation data from the simulator, in a respective list of x/y values
    * ["sense_observations_x"]
    * ["sense_observations_y"]

* **OUTPUT**: values provided by the c++ program to the simulator
  * // best particle values used for calculating the error evaluation
    * ["best_particle_x"]
    * ["best_particle_y"]
    * ["best_particle_theta"]
  * //Optional message data used for debugging particle's sensing and associations
  * // for respective (x,y) sensed positions ID label
    * ["best_particle_associations"]
  * // for respective (x,y) sensed positions
    * ["best_particle_sense_x"] <= list of sensed x positions
    * ["best_particle_sense_y"] <= list of sensed y positions

### Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

---

## Theoretical concepts

Particle filters methods are a set of genetic algorithms used to solve filtering probles arising in signal processing and Bayesian statistical inference. The filtering problem consist of estimating the internal states in dynamical system when partial observations are made, and random perturbations are present in the sensors as well as in the dynamical system. The objective is to compute the posterior distribution of the state of some Markov process, given some noisy and partial observations.

Our code is composed of five main steps:
* Initialization
* Prediction
* Data association
* Update weights
* Resample

Now, let's explain each part independently

### Initialization

First of all, we must to initialize our particles. The most practical way to initialize our particles and generate real time output, is to make an initial estimate using GPS input. As with all sensor based operations, this step is impacted by noise.  
We set the number of particles to 500, however we can use a smaller number of particles and we will have a good resul too.

```
// Set the number of particles
num_particles = NUMBER_OF_PARTICLES;

// Resize weights vector
weights.resize(num_particles);

// Resize particles vector
particles.resize(num_particles);
```

We have initialized all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.

```
// Creates a normal (Gaussian) distribution for x, y and theta
normal_distribution<double> dist_x(x, std[0]);
normal_distribution<double> dist_y(y, std[1]);
normal_distribution<double> dist_theta(theta, std[2]);

// Initializes particles
for (int i = 0; i < num_particles; i++){
  particles[i].id = i;
  particles[i].x = dist_x(gen);
  particles[i].y = dist_y(gen);
  particles[i].theta = dist_theta(gen);
  particles[i].weight = 1.0;
}
```

### Prediction

Now that we have initialized our particles it's time to predict the vehicle's position. Here we will use the motion models to predict where the vehicle will be at the next time step, by updating based on yaw rate and velocity, while accounting for Gaussian sensor noise.

![PreditionStpe](.\predictionStep.png)

We must be careful with the value of yaw, because if yaw is equal to 0, then we calculate the prediction in a different way

```
// Make distribution for adding noise
normal_distribution<double> dist_x(0, std_pos[0]);
normal_distribution<double> dist_y(0, std_pos[1]);
normal_distribution<double> dist_theta(0, std_pos[2]);

// Different equations based on if yaw_rate is zero or not
for (int i = 0; i < num_particles; ++i) {
  // Add measurements to particles
  if (fabs(yaw_rate) > EPS) {
    particles[i].x += vel_yaw * (sin(particles[i].theta + yaw) - sin(particles[i].theta));
    particles[i].y += vel_yaw * (cos(particles[i].theta) - cos(particles[i].theta + yaw));
    particles[i].theta += yaw;
  }
  else {
    particles[i].x += dist * cos(particles[i].theta);
    particles[i].y += dist * sin(particles[i].theta);
    // Theta will stay the same due to no yaw_rate
  }

  // Add noise to the particles
  particles[i].x += dist_x(gen);
  particles[i].y += dist_y(gen);
  particles[i].theta += dist_theta(gen);
}
```

### Data association

Data assocition is the problem of matching landmark measurements to objects in the real world, like map landmarks. Oftentimes, you have some map landmarks that have multiple lidar measurements that could correspond to it. To pick the right correspondents we can use a simple technique called nearest neighbor. In this method, simply take the closest measurement as the correct correspondents.


| Advantages                    | Disadvantages             |
| :---------------------------- | :------------------------ |
| Easy to understand            | Not robust to high density of measurements or map landmarks       |
| Easy to implement             | Not robust to sensor noise |
| Works well in many situations | Not robust to errors in position estimates |
|                               | Inefficient to calculate |
|                               | Does not take different sensor uncertainties into account |

Taking into account the above, we will create the nearest neighbor algorithm implementation in c ++. For this, we calculate the distance between observations and predictions and we store the minimum distance.

```
double xDistance = observations[i].x - predicted[j].x;
double yDistance = observations[i].y - predicted[j].y;

double distance = xDistance * xDistance + yDistance * yDistance;

// If the "distance" is less than min, stored the id and update min.
if ( distance < minDistance ) {
  minDistance = distance;
  mapId = predicted[j].id;
}
```

### Update weights

Now that we have incorporated velocity and yaw rate measurement inputs into our filter, we must update particle weights based on LIDAR and RADAR readings of landmarks.

We update the weights of each particle using a [mult-variate Gaussian distribution](https://en.wikipedia.org/wiki/Multivariate_normal_distribution)

The observations are given in the VEHICLE'S coordinate system and the particles are located according to the MAP'S coordinate system, so we will need to transform between the two systems. This [transformation](https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm) requires both rotation AND translation (but no scaling).

```
// Transform observation coordinates.
vector<LandmarkObs> mapObservations;
for(unsigned int j = 0; j < observations.size(); j++) {
  double rot_x = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
  double rot_y = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
  mapObservations.push_back(LandmarkObs{ observations[j].id, rot_x, rot_y });
}
```

Once the particles of interest have been identified, we must update their weights taking into account the following expression

```
weight = exp(-1.0 * (dX*dX/(2*std_landmark[0]*std_landmark[0]) + (dY*dY/(2*std_landmark[1]*std_landmark[1]))) / (2*M_PI*std_landmark[0]*std_landmark[1]));
```

### Resample

What we do here is to resample the particles with replacement with probability proportional to their weight.

```
// Resampling Wheel
int index = distInt(gen);
double beta = 0.0;
vector<Particle> resampledParticles;
for(int i = 0; i < num_particles; i++) {
  beta += distDouble(gen) * 2.0;
  while( beta > weights[index]) {
    beta -= weights[index];
    index = (index + 1) % num_particles;
  }
  resampledParticles.push_back(particles[index]);
}
```

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

  ## Running the Code
  This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

  This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

  Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

  1. mkdir build
  2. cd build
  3. cmake ..
  4. make
  5. ./particle_filter

  Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

  1. ./clean.sh
  2. ./build.sh
  3. ./run.sh

  Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
