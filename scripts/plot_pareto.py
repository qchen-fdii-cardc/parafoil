import numpy as np
import matplotlib.pyplot as plt
import os

# Create directory if it doesn't exist
os.makedirs('doc/static/nsgaii', exist_ok=True)

# Load data
data = np.loadtxt('build/pareto_front.dat')
f1 = data[:, 0]
f2 = data[:, 1]

# Create figure
plt.figure(figsize=(10, 8))
plt.scatter(f1, f2, c='blue', s=50, alpha=0.5, label='NSGA-II Solutions')

# Plot true Pareto front for ZDT1
x = np.linspace(0, 1, 100)
plt.plot(x, 1 - np.sqrt(x), 'r--', label='True Pareto Front')

plt.xlabel('f₁')
plt.ylabel('f₂')
plt.title('NSGA-II Results on ZDT1 Problem')
plt.legend()
plt.grid(True)

# Save figure
plt.savefig('doc/static/nsgaii/pareto_front.png', dpi=300, bbox_inches='tight')
plt.close()

# Calculate metrics
# 1. Spread along f1
f1_spread = np.max(f1) - np.min(f1)
# 2. Number of solutions
num_solutions = len(f1)
# 3. Average spacing between solutions
sorted_indices = np.argsort(f1)
f1_sorted = f1[sorted_indices]
f2_sorted = f2[sorted_indices]
distances = np.sqrt(np.diff(f1_sorted)**2 + np.diff(f2_sorted)**2)
avg_spacing = np.mean(distances)

# Write metrics to file
with open('doc/static/nsgaii/metrics.txt', 'w') as f:
    f.write(f'Number of Pareto optimal solutions: {num_solutions}\n')
    f.write(f'Spread along f1: {f1_spread:.6f}\n')
    f.write(f'Average spacing between solutions: {avg_spacing:.6f}\n') 