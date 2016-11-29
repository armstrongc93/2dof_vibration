/* Vehicle Vibration Analysis
*	Vehicle dynamics can be modeled as a 2DoF spring-mass-damper system with two ouptuts:
*		1. Vehicle bounce (z motion felt by passengers)
*		2. Vehicle pitch (rotation felt be passengers along front to back)
*	These ouputs are based on vehicle characteristics 
*/

#include <iostream>
#include <vector>
#include <Eigen>
#include <iomanip>
#include <math.h>

using namespace Eigen;
using namespace std;

double pi = acos(-1);

// Define road class
class Road
{
	protected:
		double A;			// 1/2 amplitude of road variation, in
		double L;			// Wavelength of road variation, in
		double V;			// Vehicle velocity, mph (convert to in/s)
		double radFreq;		// Radial frequency (rad/s)

	public:
		void getRoadValues()
		{
			double v_mph;
			cout << "It is efficient to model road variation as a sinusoidal curve, using peak amplitude and wavelength as defining characteristics." << endl;

			cout << "Enter the peak amplitude of the road variation, A (in): "; cin >> A;

			cout << "Enter the wavelength of the road variation, L (in): "; cin >> L;

			cout << "Enter the velocity of the vehicle, V (mph): "; cin >> v_mph;	// Enter velocity as dummy variable (mph)
			V = v_mph*17.6;	// Convert dummy velocity to in/s and save as object's velocity for calculations
			cout << "\n";
		}

		// Calculate radial frequency due to road conditions
		void calcRadialFreq()
		{
			radFreq = (2 * pi * V) / L;
		}

};

// Define vehicle class
class Vehicle: public Road
{
	private:
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
					cout << "You have selected SAE Class " << classVal <<"\n" << endl;
				}
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
				cout << "Enter the mass, m (lbs): "; cin >> mass;
				cout << "Enter the moment of inertia, J (lb*s^2*in): "; cin >> inertia;
				cout << "Enter the front suspension stiffness, k_f (lb/in): "; cin >> stiffness_f;
				cout << "Enter the rear suspension stiffness, k_r (lb/in): "; cin >> stiffness_r;
				cout << "Enter the front damping coefficient, c_f (lb*s/in): "; cin >> damping_f;
				cout << "Enter the rear damping coefficient, c_r (lb*s/in): "; cin >> damping_r;
				cout << "Enter the length from the center of gravity to the front axle, L1 (in): "; cin >> frontLength;
				cout << "Enter the length from the center of gravity to the rear axle, L2 (in); "; cin >> rearLength;
				cout << "\n";
			}
			else if (classVal == 1)
			{
				// Parameters for SAE class 1
				mass = 2202.0;
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
				mass = 3053.0;
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
				mass = 3547;
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
				mass = 4247;
				inertia = 24980;
				stiffness_f = 215.4;
				stiffness_r = 186;
				damping_f = 7.02;
				damping_r = 6.96;
				frontLength = 29.94;
				rearLength = 30.92;
			}
			else
			{
				// Parameters for SAE class 5
				mass = 4865;
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

		// Calculate the solution to the ODE
		void systemOutput()
		{
			cout << "-- Bounce and Pitch Output --" << endl;
			// Define the time vector for analysis
			double t0, tf, dt;
			cout << "Enter the starting time for the analysis, s: "; cin >> t0;
			cout << "Enter the ending time for the anaylsis, s: "; cin >> tf;

			// Define time step
			dt = (t0 + tf) / 1000;
		}


};



int main()
{
	Vehicle car;

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
		car.getRoadValues();

		cout << "\n------ System Analysis ------\n" << endl;
		car.calcNatFreq();
		car.systemOutput();
	}

	return 0;
}