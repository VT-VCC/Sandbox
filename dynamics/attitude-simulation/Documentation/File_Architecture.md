## File Architecture

	This document contains a brief explanation of the supporting
files for the main MATLAB simulation, and how they are organized.
The central .m file is not a standalone product, and requires several
other data files and function libraries in the working directory.

-------------------------------------------------------------------------------

## OPASv_.m
	- located within Source file at highest level
	- called by user at runtime to execute a simulation
	
## datfiles folder
	- contains files with data to support IGRF magnetic field model
	- should be included in MATLAB path/directory before runtime
	- located in same folder as OPASv_.m

## TO BE COMPLETED
