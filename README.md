# DeterministicDP
## Intro
This little codebase is currently being built as a generic Deterministic Dynamic Programming code that can produce optimal policies.

## Current Progress
The current code built is for handling classical optimal control problems that basically require a discretization of continuous state and action spaces.
The code has been tested on a few test problems thus far, from a very simple 1D control problem to an Inverted Pendulum control problem where the admissable control is too weak to bring the pendulum to the inverted position directly.

## Where it is headed
The code is headed in the direction of having an implementation that can handle graphically represented problems. 
This representation will allow for representing a problem using a custom graphical model of your problem, in turn allowing a new set of problems to be tackled using Dynamic Programming relative to the control styled problems mentioned above.
