## Document Summary - USIP ADCS Simulation Overview

	This documentation is intended to be a thorough
summary of the current functionality of the Attitude Determination
and Control Simulation (ADCS) for the NASA-Sponsored Undergraduate Student
Instrument Project (USIP). This project is currently being maintained by
the Virginia Tech ADCS USIP subteam. Team lead Ivan Ventura 
(vivan19@vt.edu) or faculty sponsor Prof. Kevin Shinpaugh (kashin@vbi.vt.edu)
can be contacted for further information about the simulation or overall
project status. The GitHub page, , also contains the most recent working version
of the simulation code and supporting files.
-------------------------------------------------------------------------------

##Simulation Engine Algorithm

OPAS – Orbit Propagator and Attitude Simulator

1. Scan in data from CPE2TLE.txt for orbital params
2. Initialize constants
3. Get final time from TLE start and hard coded duration
4. Set constant vernal equinox epoch time reference
5. Convert TLE to Keplerian elements
	a. Calculate velocity, position, angular velocity initial from Keplerian elements
	b. Set orbit propagation initial state vector (state0)
6. Currently overrides previous final time using orbital elements
7. Call ode45 to solve orbit parameters from function oDiffEq
8. Convert orbit to inertial frame, inertial to earth frame, earth to lat/long
9. Use IGRF model to get magnetic field values
	a. rlltBL gets magnetic field in local NED coordinates
	b. Convert to ECEF
	c. Convert to ECI
	d. Find magnitudes
10. Create a ground track
	a. Create figure window
	b. Get world map background
	c. Plot ground track
	d. Plot ground area that can see S/C
	e. Plot ground station (currently Cal Poly)
	f. Format plot
11. Set CubeSat mass and dimensions (hard coded)
	a. Length x, width y, height z
12. Set/Calculate principal moments of inertia
	a. Calculate K – intermediate inertial properties
13. Simulation Setup
	a. Set control and measurement timing parameters (hard coded)
	b. Set initial angular velocity (worst case deployment)
		i. Convert to quaternion
		ii. Initialize attitude state vector (w0 and q0)
	c. Choose desired attitude/state
	d. Choose orbital or inertial frame for desired state
	e. Choose magnetometer modeling method
		i. Set calibration data based on modeling method (need to revisit)
	f. Set stdev and resolution for gryo, attitude, and magnetometer feedback
	g. Characterize magnetorquer properties
	h. Set control parameters (gain factors)
14. Simulation Portion w/o control
	a. Initialize initial time, state
	b. Initialize loop indices mdx, idx, ndx
	c. Turn torquers off, and solve rigid body EOM for reading period
		i. Extract final states for next solver
	d. Sensor readings – Attitude, rate, field with error
	e. Store measured magnetic field and time of reading for later display
	f. Use interpolation to get mean motion and true anomaly at current time
	g. Tranform from inertial to orbital state vector
15. Simulation Portion w/ Control
	a. Find rate and attitude errors
	b. PD Control Law to get required torque and magnetic dipole vector
	c. Calculate B-Dot and Magnetic Dipole (in bits or Tesla)
	d. Select dipole requested based on control law

16... TO BE COMPLETED
-------------------------------------------------------------------------------

## Domain of Applicability

The USIP ADCS simulation tool should be used with caution and with an 
understanding of the limitations and underlying assumptions. Failure to do so 
may result in incorrect tuning of the simulation or misapplication of results.
Simulation data should be supported by hardware testing to justify mission
critical decisions.

The original simulation was designed primarily for 1U CubeSats in low earth orbit 
using magnetorquer actuated attitude control, and should be used with caution 
if simulating other types of bodies in orbit, particularly those with larger 
size, more complex geometry, or different types of control systems.

Some Assumptions
	- Magnetorquers distributed evenly across 6 orthogonal faces of satellite
	- Sensors including magnetometers suffer Gaussian measurement error
	- Magnetorquers turn on and off quickly enough so as to minimally bias 
		magnetometers during the measurement phase of control.

Some Limitations
	- Limited accuracy in atmospheric and drag models
	- Limited number of control laws (B-Dot & Cross-Product) implemented
	- Not all successful control schemes in simulation will plausible in 
		reality because of constraints such as computing power

*TBR: Add more discussion of any known limitations/fundamental constraints*
-------------------------------------------------------------------------------

## Simulation Parameters

Note: discussions on how to change parameters in "instructions.md" document
		as this section intended to give overview of simulation degrees of
		freedom

List of Adjustable Parameters and Descriptions if Applicable
	- Initial Attitude
	- Initial Angular Velocity
	- Keplerian Elements (low eccentricity in LEO)
	- S/C Mass, Moments of Inertia
	- Sensor
	- Control Gain Factors
	- More to Come

-------------------------------------------------------------------------------

## Simulation Modes

B-Dot Control Law: for passive stabilization
Cross-Product Control Law: 3-axis control for specified attitude

-------------------------------------------------------------------------------

## Simulation Performance

	Currently, the simulation is not streamlined and requires manual tuning of 
parameters by changing them in hard code. The simulation still displays some 
error messages and unsupressed data to the command window, but is able to
produce graphs and successfully simulate B-Dot control in the orbital 
environment.

*When available, information about approximate amount of real time per day/hour
in simulation should be added here*

*Also, any known major bugs can be put here*

-------------------------------------------------------------------------------

## Development Timeline

Planned Spring 2016: streamline OPAS further, add attitude determination with
	sun sensors, and integrate with STK

Fall Semester 2016: troubleshooted and adapted OPAS code from
	previous study into working simulation tool

**TO BE REVISITED BY TEAM
	
-------------------------------------------------------------------------------

## Last Update to Document:	2-11-2017
							Louis Rizzi