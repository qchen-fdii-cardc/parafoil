#!/bin/bash

# Build and run NSGA-II
cmake --build build
./build/nsga2_demo

# Run visualization script
python3 scripts/plot_pareto.py 