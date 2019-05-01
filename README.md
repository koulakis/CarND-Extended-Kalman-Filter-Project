# Extended Kalman Filter Project
This is a solution to the fifth project of the Self-Driving Car Engineer Nanodegree Program of Udacity.

## Description
The core of the project is an implementation of an extended Kalman filter. In order to make the model usable a server is defined in `main.cpp` which communicates using websockets with a simulator that one can setup. The simulator moves a car in two predefined trajectories and produced laser and radar readings. The readings are continuously fed to the Kalman filter and its output is displayed with green points on the simulator. Additionaly the RMSE of the model is displayed in the screen.

## Setup
To compile and launch the project (assuming a bash shell) run:

`(mkdir -p build && cd build && cmake .. && make && ./ExtendedKF)`

To setup the simulator follow the instruction from here: [here](https://github.com/udacity/self-driving-car-sim/releases).

When both the `./ExtendedKF` and the simulator are running, the module with print `Connected!` once selecting a level. When the movement starts it will begin printing the `x` coordinates and updating the simulator.

## Testing
A test suite has been added to test the Kalman filter logic using Google Test. To run the tests execute:

`(mkdir -p build && cd build && cmake && make && ./ExecuteTests)`

## RMSE scores on simulator
- Dataset 1: 
    - X: 0.097
    - Y: 0.086
    - VX: 0.45
    - VY: 0.44
- Dataset 2:
    - X: 0.073
    - Y: 0.097
    - VX: 0.456
    - VY: 0.497

## Project code structure
The source code is under the `src` directory. Here is a description of the files:
- `tools.h`: Basic tools used to compute rmse, the Jacobian for the conversion to polar coordinates and convert between polar and cartesian coordinates.
- `kalman_filter.h`: Functions which compute the location and covariance matrix transforms during the prediction and update steps. In particular, the are two update transforms, one for the laser data which uses an ordinary Kalman filter and the one for the radar data which uses an extended Kalman filter.
- `fusion_ekf.h`: A state machine which updates the location and covariance matrix on the input of new measurements. When receiving a measurement it decides which filter to call depending on the input sensor type.
- `main.cpp`: Runs a websockets server which communicates with the simulator. it holds an instance of the fusion EKF in memory and updates it whenever the `onMessage` callback is called with a new measurement package.

The tests for the above code is unde the `tests` directory. The directory mirrors the structure of the `src` directory.

## Improvements on the original code layout
Those are some improvements:
- The tools and kalman filter classes where converted to templates which implement stateless functions. This made both testing and reasoning about the code easier.
- The fusion class has a clear separation between its configuration and its state. This increased readability and made it easier to see what is updated in the Kalman filter and which parameters need to be set or tuned (in our case we have them a priori tuned).
- Tests have been added to make changes easy.

## Potential improvements which have not been implemented
- Wrap the state of the Kalman filter on a thread-safe object. This will ensure that if the frequency of measurements increases (thus also the chance of simultaneous calls) then the state will be sequencially accessed and not get corrupted.
- Use the provided data to build an integration test which checks that the rmse is below the given thresholds. This check was only performed manually via the simulator.
- Separate the tests from the source code in the `CMakeLists.txt`. That would be great, since the source code compilation should be independent from the compilation of the tests.