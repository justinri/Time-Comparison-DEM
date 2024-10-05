/*
	Calculates and return the time step according to 
	Rayleigh wave speed of force propagation
*/

double ray_wave(double E, double R, double rho, double nu){
	/*
		Input Parameters
		----------------
		E - Young's modulus
		R - Radius of smallest particle
		Rho - Particle density
		nu - Poisson's Ratio

		Calculated Parameters
		---------------------
		G - Shear modulus
		
		Output (return value)
		----------------
		Tr - Time step 
		Note: Normally, for Discrete element methods you will want to take 20%-40% of this value.
	*/
		
	double pi = 3.141592653;
	double G = E/(2*(1+nu));

	double Tr = (pi*R*pow((rho/G),(1/(double)2)))/(0.1631*nu + 0.8766);

	return Tr;
	}
