// =============================================================================
// Sphere impact test
// =============================================================================

// For open cascade
#include "chrono_cascade/ChBodyEasyCascade.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "ray_wave.cpp"

// Using for elased time
#include <chrono> 
using namespace std::chrono; 

// Use the namespaces of Chrono
using namespace chrono;

// Use the main namespaces of Irrlicht
#include "chrono_irrlicht/ChIrrApp.h"
#include <irrlicht.h>
using namespace chrono::irrlicht;
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// Variables
// Number of impacts to create
int numImpacts = 1;
int numSteps = 200000; // Number of steps to run

// Spheres information, steel
double r = .05; 		    // the sphere's radius[m]
double nu = .3; 
double rho = 7700; 		    // [kg/m^3]  Calibrated density
double E = 2.15e+11; 		// [Pa (215 GPa)] 
double pi = 3.14159;
double friction_coeff = .350;		
double restitution_coeff = 1.00;
double gravity = 9.81;
double mass = (4.0/3.0)*pi*pow(r, 3)*rho;	// [kg] (volume = (4/3)*pi*r^3)

// Start program
int main(int argc, char* argv[]) {
    // Create the system
    ChSystemSMC msystem;
	msystem.Set_G_acc(ChVector<>(0, 0, gravity));

    // The following two lines are optional, since they are the default options. They are added for future reference,
    // i.e. when needed to change those models.
    msystem.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
    msystem.SetSolverType(ChSolver::Type::MINRES);

    // Create a material (will be used by the spheres)
    auto material_spheres = std::make_shared<ChMaterialSurfaceSMC>();
 	material_spheres->SetYoungModulus(E);
 	material_spheres->SetPoissonRatio(nu);
    material_spheres->SetFriction(friction_coeff);
    material_spheres->SetRestitution(restitution_coeff);
	msystem.UseMaterialProperties(true);

	// Setting effective radius of curvature (Note, both spheres have the same size)
	double r_curv_eff = 1.0/(1.0/r + 1.0/r);
	collision::ChCollisionInfo::SetDefaultEffectiveCurvatureRadius(r_curv_eff);

	// Taking 1% of the rayleigh time step
	double time_step = 0.01*ray_wave(E, r, rho, nu);

	// Setting up spheres falling
    for (int num = 0; num < numImpacts; num++) {
//    int num = 0;

    // Create the falling ball
    auto ballFalling = std::make_shared<ChBody>();

    ballFalling->SetIdentifier(100+num);
    ballFalling->SetMass(mass);
    ballFalling->SetPos(ChVector<>(2*r*num*1.25, 2*r*num*1.25, 0.0));
    ballFalling->SetBodyFixed(false);
    ballFalling->SetCollide(true);

    ballFalling->GetCollisionModel()->ClearModel();
	ballFalling->GetCollisionModel()->AddSphere(material_spheres, r);
    ballFalling->GetCollisionModel()->BuildModel();

    ballFalling->SetInertiaXX(0.4 * mass * r * r * ChVector<>(1, 1, 1));

    auto sphereFalling = std::make_shared<ChSphereShape>();
    sphereFalling->GetSphereGeometry().rad = r;
    ballFalling->AddAsset(sphereFalling);

    msystem.AddBody(ballFalling);

///////////////////////////////////////////////////////////////

    // Create the fixed ball
    auto ballFixed = std::make_shared<ChBody>();

    ballFixed->SetIdentifier(102+num+numImpacts);
    ballFixed->SetMass(mass);
    ballFixed->SetPos(ChVector<>(2*r*num*1.25, 2*r*num*1.25, 0.15));
    ballFixed->SetBodyFixed(true);
    ballFixed->SetCollide(true);

    ballFixed->GetCollisionModel()->ClearModel();
	ballFixed->GetCollisionModel()->AddSphere(material_spheres, r);
    ballFixed->GetCollisionModel()->BuildModel();

    ballFixed->SetInertiaXX(0.4 * mass * r * r * ChVector<>(1, 1, 1));

    auto sphereFixed = std::make_shared<ChSphereShape>();
    sphereFixed->GetSphereGeometry().rad = r;
    ballFixed->AddAsset(sphereFixed);

    msystem.AddBody(ballFixed);
    }

    // Create the Irrlicht visualization
//    ChIrrApp application(&msystem, L"", core::dimension2d<u32>(800, 600), false, true);
//    application.AddTypicalLogo();
//    application.AddTypicalSky();
//    application.AddTypicalLights();
//    application.AddTypicalCamera(core::vector3df(0, 5, 0));
//	application.AssetBindAll();
//    application.AssetUpdateAll();

	// Getting a start time for the test
	double max_nor_force = 0;
	int current_step = 0;
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	
	// Running sim
    for (int num = 0; num < numSteps; num++) {
//	application.BeginScene();
//    application.DrawAll();

	// Simulating one time step
    msystem.DoStepDynamics(time_step);
	current_step++;
		
	// Getting the maximum normal force (Noting the contact is in the x-direction.)
//	ChVector<> ballForce = ballFixed->GetContactForce();
//	if (ballForce.z() > max_nor_force){
//		max_nor_force = ballForce.z();
//	}
//    application.EndScene();
	}

	// Printing information to compare with yade
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	int elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count(); 
	std::cout << elapsed_time; 
//	std::cout << "," << max_nor_force; 	
	std::cout << "," << time_step; 	
	std::cout << "," << current_step;
    return 0;
}
