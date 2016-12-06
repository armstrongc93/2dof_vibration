/* Vehicle Vibration Analysis
*	Vehicle dynamics can be modeled as a 2DoF spring-mass-damper system with two ouptuts:
*		1. Vehicle bounce (z motion felt by passengers)
*		2. Vehicle pitch (rotation felt be passengers along front to back)
*	These ouputs are based on vehicle characteristics
*/

// Courtney Armstrong
// Rev 2, 12/06/16

#include <iostream>
#include <fstream>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <Eigen>
#include <iomanip>
#include <math.h>

using namespace Eigen;
using namespace std;
using namespace boost::numeric::odeint;

typedef std::vector< double > state_type;
double pi = acos(-1);

// Define road class
class Road
{
	friend class coupledODE;
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
			radFreq = (2 * pi * V) / L;
			cout << setprecision(4) << "The radial frequency is " << radFreq << " rad/s\n";
		}


};

// Define vehicle class
class Vehicle
{
	friend class coupledODE;
	private:
		double mass;			// Vehicle mass, kg
		double inertia;			// Mass moment of inertia, N*sec^2*m
		double stiffness_f;		// Stiffness of front suspension, N/m
		double stiffness_r;		// Stiffness of rear suspension, N/m
		double damping_f;		// Damping of front suspension, N*s/m
		double damping_r;		// Damping of rear suspension, N*s/m
		double frontLength;		// Distance from CG to front suspension, m
		double rearLength;		// Distance from CG to rear suspension, m
		double w1;				// 1st natural frequency, rad/s
		double w2;				// 2nd natural frequency, rad/s
		int classVal;			// User defined class type

	public:
		// Output general information and get user choice (determines vehicle paramenters) 
		int setUserChoice()
		{
			int selection;
			cout << "Before the vibration analysis can be completed, modeling parameters for the vehicle need to be defined." << endl;
			cout << "The pre-defined parameters are based on SAE passenger car classifications, or you can define your own parameters.\n" << endl;
			cout << "The SAE classification are based on the wheelbase (length from front to back axle) of the vehicles. They are defined as follows:" << endl;
			cout << "\tClass 1 - Wheelbase = 2.06-2.41 m (80.9-94.8\"); Includes sub-compact vehicles, like the Ford Fiesta and Kia Rio." << endl;
			cout << "\tClass 2 - Wheelbase = 2.41-2.58 m (94.8-101.6\"); Includes compact vehicles, like the Toyota Corolla and Mazda3" << endl;
			cout << "\tClass 3 - Wheelbase = 2.58-2.80 m (101.6-110.4\"); Includes mid-size vehicles, like the Hyundai Sonata and Honda Accord" << endl;
			cout << "\tClass 4 - Wheelbase = 2.80-2.98 m (110.4-117.5\"); Includes full-size vehicles, like the Infiniti Q70 and Jaguar XF" << endl;
			cout << "\tClass 5 - Wheelbase = 2.98+ m (117.5\"+); Unusual for modern passenger cars to fit this class\n" << endl;

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
				cout << "Enter the moment of inertia, J (N*s^2*m): "; cin >> inertia;
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
				mass = 998.8104;		
				inertia = 1399.0933;
				stiffness_f = 16836.6944;
				stiffness_r = 17616.0088;
				damping_f = 1733.7550;
				damping_r = 1399.2629;
				frontLength = 0.6998;
				rearLength = 0.6934;
			}
			else if (classVal == 2)
			{
				// Parameters for SAE class 2
				mass = 1384.8175;
				inertia = 1790.2473;
				stiffness_f = 17931.2371;
				stiffness_r = 17048.5979;
				damping_f = 1616.4201;
				damping_r = 1227.6387;
				frontLength = 0.7191;
				rearLength = 0.7280;
			}
			else if (classVal == 3)
			{
				// Parameters for SAE class 3
				mass = 1608.8921;
				inertia = 2259.8130;
				stiffness_f = 19465.3483;
				stiffness_r = 20032.7592;
				damping_f = 1521.8516;
				damping_r = 1213.6285;
				frontLength = 0.7457;
				rearLength = 0.7346;
			}
			else if (classVal == 4)
			{
				// Parameters for SAE class 4
				mass = 1926.4068;
				inertia = 2822.3653;
				stiffness_f = 16680.8315;
				stiffness_r = 23204.3063;
				damping_f = 1229.3899;
				damping_r = 1218.8823;
				frontLength = 0.7605;
				rearLength = 0.7854;
			}
			else
			{
				// Parameters for SAE class 5
				mass = 2206.7269;
				inertia = 2709.9452;
				stiffness_f = 16812.1766;
				stiffness_r = 30354.7352;
				damping_f = 1201.3697;
				damping_r = 1337.9685;
				frontLength = 0.7623;
				rearLength = 0.7620;
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


// ODE integration parameters class used to solve equations (inherits data from Vehicle & Road class)
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

class coupledODE
{
	//double m_param, J_param, kf_param, kr_param, cf_param, cr_param, l1_param, l2_param, A_param, radF_param, L_param;
	//coupledODE(double m, double J, double kf, double kr, double cf, double cr, double l1, double l2, double Amp, double radialF, double waveL) : m_param(m), J_param(J), kf_param(kf), kr_param(kr), cf_param(cf), cr_param(cr), l1_param(l1), l2_param(l2), A_param(Amp), radF_param(radialF), L_param(waveL) {};

	public:
		Vehicle car; Road road;
		coupledODE(const Vehicle& car1, const Road& road1) : car(car1), road(road1) {};


		void operator()(state_type &x, state_type &dxdt, double t)
		{
			double wave_f = car.stiffness_f*road.A*sin((road.radFreq)*t);
			double wave_r = car.stiffness_r*road.A*sin((road.radFreq)*t - (2 * pi*(car.frontLength + car.rearLength)) / road.L);

			double term1f = car.stiffness_f*x[0] + car.damping_f*x[1];
			double term1r = car.stiffness_r*x[0] + car.damping_r*x[1];
			double term2f = car.stiffness_f*x[2] + car.damping_f*x[3];
			double term2r = car.stiffness_r*x[2] + car.damping_r*x[3];
			double term3f = -term1f + term2f*car.frontLength + wave_f;
			double term3r = -term1r - term2r*car.rearLength + wave_r;

			dxdt[0] = x[1];
			dxdt[1] = (1 / car.mass)*(term3f + term3r);
			dxdt[2] = x[3];
			dxdt[3] = (1 / car.inertia)*(-term3f*car.frontLength + term3r*car.rearLength);
		}
};

// Structure to write values generated by integration to vectors
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


		// Initialize odeint using standard rk5 integration (DOES NOT WORK)
		//typedef runge_kutta_dopri5<state_type> rk5;
		//ntegrate_const(make_dense_output(1E-3, 1E-6, rk5()), coupledODE(car,road), x, tStart, tEnd, timeStep, writeVals(x_vec, times));

		// Initialize odeint using Burlisch Stoer
		bulirsch_stoer_dense_out< state_type > stepper(1E-8, 0.0, 0.0, 0.0);
		integrate_const(stepper, coupledODE(car, road), x, tStart, tEnd, timeStep, writeVals(x_vec, times));

		ofstream out("results.txt");
		if (out.is_open())
		{
			double n = (tEnd - tStart) / timeStep;
			out << "System Analysis Results\n";
			out << "Time" << "\t" << "Bounce" << "\t" << "Pitch" << endl;
			for (int i = 0; i <= n; i++)
			{
				out << setprecision(8) << times[i] << "\t" << x_vec[i][0] << "\t" << x_vec[i][2] << endl;
			}
			out.close();
		}

		cout << "\n------ System results written to file ------\n" << endl;
		
	
		
	}

	return 0;
}
