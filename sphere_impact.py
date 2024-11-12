from yade.gridpfacet import *
import datetime

### To view gui
#from yade import qt
#qt.View()

## Variables
## Number of impacts to create
numImpacts = 1
numSteps = 200000 ## Number of steps to run

## Spheres information, steel
r = .05		        ## the sphere's radius[m]
nu = .3
rho = 7700		    ## [kg/m^3]  Calibrated density
E = 2.15e+11		## [Pa (215 GPa)] 
friction		= 0.350
angle			= atan(friction)   #https://github.com/yade/trunk/blob/d9e02d9296ecbbb3ed76c5b62f342dc3ac294ab8/examples/capillaryLaplaceYoung/CapillaryPhys-example.py
e = 1.00		# coefficient of restitution
O.materials.append(FrictMat(frictionAngle=angle, young=E, density=rho, poisson=nu, label='material_spheres'))

for num in range(0, numImpacts):
	O.bodies.append(sphere(center=(Vector3(2*r*num*1.25, 2*r*num*1.25, 0.0)), radius=r, material='material_spheres', fixed=False)) ### Shere to fall
	O.bodies.append(sphere(center=(Vector3(2*r*num*1.25, 2*r*num*1.25, 0.15)), radius=r, material='material_spheres', fixed=True)) ### Fixed sphere

## Time step set to 1% of Rayleigh Wave
O.dt=.01*utils.RayleighWaveTimeStep()
startTime = datetime.datetime.now()

O.engines=[
		###Reset all forces stored in Scene::forces (O.forces in python). Typically, this is the first engine to be run at every step. In addition, reset those energies that should be reset, if energy tracing is enabled.
		## Resets forces and moments that act on bodies
		ForceResetter(),
		InsertionSortCollider([Bo1_Sphere_Aabb()]),
		InteractionLoop(
			[Ig2_Sphere_Sphere_ScGeom()], #    Ig2_Facet_Sphere_ScGeom()       # collision geometry
			[Ip2_FrictMat_FrictMat_MindlinCapillaryPhys(en=e, es=e, label='ContactModel')],#for hertz model only
			[Law2_ScGeom_MindlinPhys_Mindlin()]#for hertz model only

	),
	NewtonIntegrator(damping = 0.0, gravity=(0,0,9.81)),
	PyRunner(firstIterRun=numSteps,command='endTime()', label='clocksEnds'),
#	PyRunner(iterPeriod=1,firstIterRun=2,command='interInfo()'),
]

def endTime():
	"""Ending Time"""
	endTime = datetime.datetime.now()
	elapsed = endTime-startTime
#	print("{},{},{},{}".format(elapsed.total_seconds()*1000, maxForce, O.dt, numSteps))
	print("{},{},{}".format(elapsed.total_seconds()*1000, O.dt, numSteps))
	clocksEnds.dead = True
	
def interInfo():
	global maxForce
	if 'maxForce' not in globals():
		maxForce = 0
	try:
		if O.interactions[0,1].isReal:
			i=O.interactions[0,1]
#			print(O.interactions, i.phys.normalForce)
			if i.phys.normalForce[2] > maxForce:
#				print(i.phys.normalForce[0])
				maxForce = i.phys.normalForce[2]
	except IndexError:
		pass	

#O.stopAtIter=numSteps 
O.run(numSteps+1, True)			
