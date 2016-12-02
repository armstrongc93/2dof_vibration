# 2 Degree of Freedom Vehicle Vibration Analysis
2 degree of freedom forced vibration analysis using C++ and verification in MATLAB.

Verification files for MATLAB simply require input of the desired design parameters of the vehicle (mass, inertia, stiffness, damping, etc.) and the script will output the max pitch and bounce values, as well as plot of the pitch and bounce with respect to time. Micro-oscillations are present in the plot due to the forced nature of the system.

The C++ script requires the Eigen library and the odeint library from Boost in order to function properly. Eigen is used to determine the natural frequency of the system, and odeint utilizes the Runge-Kutta Dormand-Prince 5 method to solve the coupled ODE. The C++ script allows for the user to select a vehicle class (from SAE passenger vehicle classifications) or define their own parameters. The script outputs the undamped natural frequencies, and writes the results from the integration to a text file.


