# Lab Session 4: Impedance Control

## 4.1. Cartesian impedance control
In this lab session we are going to implement a Cartesian Impedance Controller. 

The controller will be implemented according to the following scheme

![impedance_control](images/impedance_control.svg)

## 4.2. Controller implementation



This script computes the dynamic model of a RR manipulator according to the impedance model:

F_ext - k x_error - B x'_error = M x''

where x_error = x_d - x, and x'_error = x'_d - x'

then: x'' = M^(-1)[F_ext - k x_error - B x'_error]

- To compute x_error and x'_error, we need the current x and x'.
They can be computed with the forward kinematics model and first-order differential kinematics:

Forward kinematics: x  =  f(q)
First-order differential kinematics: x' = J(q) q'

- We assume F_ext is given from the measures of an F/T sensor in the EE.

- The computed x'' represents the desired dynamic behavior of the manipulator at the EE level, but it must be transformed to the
joint space. It can be done with the second order differential kinematics (deriving the first-order kinematics):

First-order differential kinematics:   q'  = J(q)^(-1) x'
Second-order differential kinematics:  q'' = J(q)^(-1)[x'' - J'(q',q)q']
