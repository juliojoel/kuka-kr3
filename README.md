# Kuka KR3

Model and simulation in MATLAB of the Kuka KR3 R540 robot. 

A kinematic analysis was done as part of the Robotics course at UFSC (Federal University of Santa Catarina, Brazil), and is available in Portuguese in the Kuka_KR3.pdf file.

## Requirements

- MATLAB Mathworks Software
- Robotics Toolbox for MATLAB 10.2.1, available at http://petercorke.com/wordpress/toolboxes/robotics-toolbox

## How to use

The Example.m file contains the code needed to simulate a specific robot position.

### kr3Init - Robot startup

The function initializes the parameters of the Kuka KR3 R540 robot manipulator.

``` matlab
kr3Init(f)

%    Arguments:
% 	      f =  tool positioning vector in mm
%               * opcional parameter, default [0 0 0]
%
%    Action:
%           Creates a global variable named kr3 containing all parameters
```

The global variable created by the kr3Init function is a SerialLink object that can be manipulated through the Robotics Toolbox functions. To do this, you must declare it:

``` matlab
global kr3
```

### kr3FK - Forward kinematics of the robot

Calculation of the forward kinematics homogeneous matrix of the robot, given a position vector of the joints.

``` matlab
kr3FK(angulosDH)

%    Arguments:
% 	      angulosDH = joint angles vector in degrees (Denavit-Hartemberg)
%                       * opcional parameter, default [0 -90 90 80 0 0]
%
%    Return:
%           Homogeneous matrix of the forward kinematics
```

### kr3IK - Inverse kinematics of the robot

Calculates the inverse kinematics of the manipulator robot. An array with angles in degrees (Denavit-Hartemberg) is returned, where each line represents a possible solution.

``` matlab
kr3IK(mat)

%    Arguments:
% 	        mat = 4 x 4 homogeneous matrix
%
%    Return:
%           Joint angles vector in degrees (Denavit-Hartemberg)
```

### kr3Teach - Graphical teach pendant

The function allows the user to manipulate the robot using a graphical interface.

``` matlab
kr3Teach(q)

%    Arguments:
% 	      q = joint angles vector in degrees (Denavit-Hartemberg)
%                       * opcional parameter, default [0 -90 90 80 0 0]
%
%    Action:
%           Creates figure for robot manipulation
```

## License

GNU General Public License v3.0

## Authors

Júlio J. da Costa Neto

Luiz O. Limurci
