"""
	Calculates and return the time step according to 
	Rayleigh wave speed of force propagation
"""

def ray_wave(E, R, rho, nu):
	"""
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
	"""
		
	pi = 3.141592653
	G = E/(2*(1+nu))

	Tr = (pi*R*pow((rho/G),(1/(double)2)))/(0.1631*nu + 0.8766)

	return Tr
	
