[![License: MIT](https://camo.githubusercontent.com/2ff6a06f2f6e08b17783133ca7ebc23ce1f8ac4415eee8e835647b57048a8f0d/68747470733a2f2f696d672e736869656c64732e696f2f6769746875622f6c6963656e73652f6d6173686170652f6170697374617475732e737667)](LICENSE)


# Dependencies
* Eigen3.
* Manif (required for the filters operating on Lie groups).

# In this repo
I'm implementing multiple filters using C++. Mainly,
* A standard Kalman filter
* A multiplicative Kalman filter on Lie groups (on SE(2)).

# To do
* Read `.yaml` configuration file.
* Implement L-InEKF.