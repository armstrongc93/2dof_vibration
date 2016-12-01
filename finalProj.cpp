/* Vehicle Vibration Analysis
*	Vehicle dynamics can be modeled as a 2DoF spring-mass-damper system with two ouptuts:
*		1. Vehicle bounce (z motion felt by passengers)
*		2. Vehicle pitch (rotation felt be passengers along front to back)
*	These ouputs are based on vehicle characteristics
*/

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <Eigen>
#include <iomanip>
#include <math.h>

using namespace Eigen;
using namespace std;
using namespace boost::numeric::odeint;

typedef std::vector< double > state_type;
double long pi = acos(-1);

// Define road class
class Road
{
protected:
	double A;			// 1/2 amplitude of road variation, m
	double L;			// Wavelength of road variation, m
	double V;			// Vehicle velocity, m/s
	double radFreq;		// Radial frequency (rad/s)

public:
	void getRoadValues()
	{
		cout << "It is efficient to model road variation as a sinusoidal curve, using peak amplitude (bump height) and wavelength (roughness) as defining characteristics." << endl;

		cout << "Enter the peak amplitude of the road variation, A (m): "; cin >> A;
		cout << "Enter the wavelength of the road variation, L (m): "; cin >> L;
		cout << "Enter the velocity of the vehicle, V (m/s): "; cin >> V;	
		cout << "\n";
	}

	// Calculate radial frequency due to road conditions
	void calcRadialFreq()
	{
		radFreq = (2 * pi * V) / L;
	}

};

// Define vehicle class
class Vehicle
{
protected:
	double mass;			// Vehicle mass, lb
	double inertia;			// Mass moment of inertia, lb*sec^2*in
	double stiffness_f;		// Stiffness of front suspension, lb/in
	double stiffness_r;		// Stiffness of rear suspension, lb/in
	double damping_f;		// Damping of front suspension, lb*s/in
	double damping_r;		// Damping of rear suspension, lb*s/in
	double frontLength;		// Distance from CG to front suspension, in
	double rearLength;		// Distance from CG to rear suspension, in
	double w1;				// 1st natural frequency, rad/s
	double w2;				// 2nd natural frequency, rad/s
	double maxPitch;		// Maximum pitch of system, rad	
	double maxBounce;		// Maximum bounce of system, in
	int classVal;			// User defined class type

public:
	// Output general information and get user choice (determines vehicle paramenters) 
	int setUserChoice()
	{
		int selection;
		cout << "Before the vibration analysis can be completed, modeling parameters for the vehicle need to be defined." << endl;
		cout << "The pre-defined parameters are based on SAE passenger car classifications, or you can define your own parameters.\n" << endl;
		cout << "The SAE classification are based on the wheelbase (length from front to back axle) of the vehicles. They are defined as follows:" << endl;
		cout << "\tClass 1 - Wheelbase = 80.9-94.8\"; Includes sub-compact vehicles, like the Ford Fiesta and Kia Rio." << endl;
		cout << "\tClass 2 - Wheelbase = 94.8-101.6\"; Includes compact vehicles, like the Toyota Corolla and Mazda3" << endl;
		cout << "\tClass 3 - Wheelbase = 101.6-110.4\"; Includes mid-size vehicles, like the Hyundai Sonata and Honda Accord" << endl;
		cout << "\tClass 4 - Wheelbase = 110.4-117.5\"; Includes full-size vehicles, like the Infiniti Q70 and Jaguar XF" << endl;
		cout << "\tClass 5 - Wheelbase = 117.5\"+; Unusual for modern passenger cars to fit this class\n" << endl;

		cout << "Enter a value from 1-5 to select one of the SAE classed (enter 1 to select Class 1, 2 to select Class 2, etc.)\nOR\nEnter 0 to define your own parameters: ";
		cin >> selection;

		// Check validity of selection and assign if data is correct
		if (selection >= 0 && selection <= 5)
		{
			classVal = selection;
			if (classVal == 0)
			{
				cout << "\nYou have selected to input your own parameters.\nNote that this may result in failed calculations if not properly defined.\n" << endl;
			}
			else
			{
				cout << "You have selected SAE Class " << classVal << "\n" << endl;
			}
			return 1;
		}
		else
		{
			cout << "You have entered an invalid selection." << endl;
			return 0;
		}
	}

	// Set modeling parameters for the vehicle based on user choice
	void setVehicleParameters()
	{
		if (classVal == 0)
		{
			// User defined parameters
			cout << "Enter the mass, m (kg): "; cin >> mass;
			cout << "Enter the moment of inertia, J (N*s^2*in): "; cin >> inertia;
			cout << "Enter the front suspension stiffness, k_f (N/m): "; cin >> stiffness_f;
			cout << "Enter the rear suspension stiffness, k_r (N/m): "; cin >> stiffness_r;
			cout << "Enter the front damping coefficient, c_f (N*s/m): "; cin >> damping_f;
			cout << "Enter the rear damping coefficient, c_r (N*s/m): "; cin >> damping_r;
			cout << "Enter the length from the center of gravity to the front axle, L1 (m): "; cin >> frontLength;
			cout << "Enter the length from the center of gravity to the rear axle, L2 (m); "; cin >> rearLength;
			cout << "\n";
		}
		else if (classVal == 1)
		{
			// Parameters for SAE class 1
			mass = 68.44025;		
			inertia = 12383.0;
			stiffness_f = 180.25;
			stiffness_r = 172.5;
			damping_f = 9.9;
			damping_r = 7.99;
			frontLength = 27.55;
			rearLength = 27.30;
		}
		else if (classVal == 2)
		{
			// Parameters for SAE class 2
			mass = 94.89014;
			inertia = 15845.0;
			stiffness_f = 184.69;
			stiffness_r = 162.33;
			damping_f = 9.23;
			damping_r = 7.01;
			frontLength = 28.31;
			rearLength = 28.66;
		}
		else if (classVal == 3)
		{
			// Parameters for SAE class 3
			mass = 110.2441;
			inertia = 20001;
			stiffness_f = 206.64;
			stiffness_r = 189.62;
			damping_f = 8.69;
			damping_r = 6.93;
			frontLength = 29.36;
			rearLength = 28.92;
		}
		else if (classVal == 4)
		{
			// Parameters for SAE class 4
			mass = 1926.4068;
			inertia = 2822.3653;
			stiffness_f = 16680.8315;
			stiffness_r = 23204.3063;
			damping_f = 125.4000;
			damping_r = 125.3000;
			frontLength = 0.7605;
			rearLength = 0.7854;
		}
		else
		{
			// Parameters for SAE class 5
			mass = 151.2088;
			inertia = 23985;
			stiffness_f = 288.73;
			stiffness_r = 292.40;
			damping_f = 6.86;
			damping_r = 7.64;
			frontLength = 30.01;
			rearLength = 30;
		}
	}

	// Calculate the two natural frequencies
	void calcNatFreq()
	{

		cout << "-- Natural Frequencies --" << endl;
		// Create a 2x2 matrix using Eigen
		// Mass matrix of the system, from eqtns of motion
		Matrix2d M;
		M(0, 0) = mass;
		M(1, 0) = 0;
		M(0, 1) = 0;
		M(1, 1) = inertia;
		cout << "The mass matrix, M, of this system is: \n" << M << "\n" << endl;

		// Create a 2x2 matrix using Eigen
		// Stiffness matrix of the system, from eqtns of motion
		Matrix2d K;
		K(0, 0) = stiffness_f + stiffness_r;
		K(1, 0) = (stiffness_r*rearLength) - (stiffness_f*frontLength);
		K(0, 1) = K(1, 0);
		K(1, 1) = (stiffness_f*pow(frontLength, 2)) + (stiffness_r*pow(rearLength, 2));
		cout << "The stiffness matrix, K, is: \n" << K << "\n" << endl;

		// Find eigenvalues of M,K
		GeneralizedSelfAdjointEigenSolver<MatrixXd> es(K, M);
		double lambda1 = es.eigenvalues()[0];
		double lambda2 = es.eigenvalues()[1];

		// Natural frequencies equal the sqrt of the eigenvalues
		w1 = sqrt(lambda1);
		w2 = sqrt(lambda2);

		cout << setprecision(4) << "The undamped natural frequencies of the system are " << w1 << " rad/s and " << w2 << " rad/s.\n" << endl;
	}

};

// Coupled ODE struct used to solve equations (inherits data from Vehicle & Road class)
class analysisParam
{
	public:
		double t0, tf, dt;
		double x_init1, x_init2, x_init3, x_init4;

		void getInterval()
		{
			cout << "-- Forced Excitation --" << endl;
			cout << "Enter the starting time of the interval (s): "; cin >> t0;
			cout << "Enter the ending time of the interval (s): "; cin >> tf;
			cout << "Enter the desired time increment (s): "; cin >> dt;
		}

		void getIntitialCond()
		{
			cout << "\nEnter the initial value of the bounce, (m): "; cin >> x_init1;
			cout << "Enter the initial value of the rate of change of the bounce (m/s): "; cin >> x_init2;
			cout << "Enter the initial value of the pitch (rad): "; cin >> x_init3;
			cout << "Enter the initial value of the rate of change of the pitch (rad/s): "; cin >> x_init4;
		}
};

class coupledODE : public Vehicle, public Road
{
	public:
		void operator()(state_type &x, state_type &dxdt, double t)
		{
			dxdt[0] = x[1];
			dxdt[1] = (1 / mass)*(-(damping_f + damping_r)*x[1] - (stiffness_f + stiffness_r)*x[0] - (damping_r*rearLength - damping_f*frontLength)*x[3] - (stiffness_r*rearLength - stiffness_f*frontLength)*x[2] + stiffness_f*A*sin((radFreq)*t) + stiffness_r*A*sin((radFreq)*t - (2 * pi*(frontLength + rearLength)) / L));
			dxdt[2] = x[3];
			dxdt[3] = (1 / inertia)*(-(damping_f*pow(frontLength, 2) + damping_r*pow(rearLength, 2))*x[3] - (stiffness_f*pow(frontLength, 2) + stiffness_r*pow(rearLength, 2))*x[2] - (stiffness_r*rearLength - stiffness_f*frontLength)*x[1] - (stiffness_r*rearLength - stiffness_f*frontLength)*x[0] + stiffness_r*rearLength*A*sin((radFreq)*t - (2 * pi*(rearLength + frontLength)) / L) - stiffness_f*frontLength*A*sin((radFreq)*t));
		}
};

struct writeVals
{
	std::vector< state_type >& m_states;
	std::vector< double >& m_time;

	writeVals(std::vector< state_type > &states, std::vector< double > &time)
		: m_states(states), m_time(time) { }

	void operator()(const state_type &x, double t)
	{
		m_states.push_back(x);
		m_time.push_back(t);
	}
};

int main()
{
	analysisParam calcParams;
	Vehicle car;
	Road road;

	cout << "2-DoF Vehicle Vibration Analysis\n" << endl;
	cout << "\n------ Vehicle Properties ------\n" << endl;
	if (car.setUserChoice() == 0)
	{
		exit(0);
	}
	else
	{
		car.setVehicleParameters();
		cout << "\n------ Driving Conditions ------\n" << endl;
		road.getRoadValues();

		cout << "\n------ System Analysis ------\n" << endl;
		car.calcNatFreq();
		calcParams.getInterval();
		calcParams.getIntitialCond();

		// Redefine input from analysisParams to useable values for odeint evaluation
		state_type x(4);		// Define state vector to hold initial values
		x[0] = calcParams.x_init1;		// value of x[0] (bounce)
		x[1] = calcParams.x_init2;		// value of x[1] (rate of change of bounce, x')
		x[2] = calcParams.x_init3;		// value of x[2] (pitch)
		x[3] = calcParams.x_init4;		// value of x[3] (rate of change of pitch, theta')
		vector<state_type> x_vec;
		vector<double> times;

		const double timeStep = calcParams.dt;		// Time step as defined by user
		double tStart = calcParams.t0;				// Start time as defined by user
		double tEnd = calcParams.tf;				// End time as defined by user

		// Initialize odeint using standard rk5 integration
		typedef runge_kutta_dopri5<state_type> stepper_type;
		integrate_const(make_dense_output<stepper_type>( 1E-6, 1E-3 ), coupledODE(), x, tStart, tEnd, timeStep, writeVals(x_vec, times));

	}

	return 0;
}





