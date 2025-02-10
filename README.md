# EarthReturn
# 2D Simple Rocket Landing Simulation

## Project Overview

Welcome to **EarthReturn**, a project designed to simulate the landing of a rocket in a 2D environment. This project is part of a series that gradually increases in complexity, eventually leading to a full 3D rocket landing simulation.

This branch focuses on the simplest 2D case where the rocket can move vertically under the influence of gravity and controlled thrust.

## Features

- **2D Rocket Dynamics**: Simulates vertical motion with gravity and thrust.
- **Basic Control**: Implements a simple Model Predictive Control (MPC) algorithm to guide the rocket to a safe landing.
- **Simulation Environment**: A basic physics simulation that models the rocket's vertical position and velocity.

## Document
Please have a look at the [documentation](docs/document.pdf) for more information, including the theory behind the project.

## Project Structure

```plaintext
├── README.md                   # This file
├── docs                        # Project documentation file
├── src/                        # Source code for the simulation and control algorithms
│   ├── rocket_sim.py           # The main simulation code
│   ├── mpc_controller.py       # MPC controller implementation
│   ├── dynamics.py             # Defines the rocket dynamics
|   ├── physics_simulator.py    # Simulates the physics of the system and provides state updates
│   └── utils.py                # Utility functions (e.g., plotting, logging)
└── data/
└── Dockerfile                  # Docker file
