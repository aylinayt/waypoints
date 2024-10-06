import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import time
from scipy.spatial import KDTree


# Load waypoints data
waypoints_df = pd.read_csv('waypoints.csv')

# Load pure pursuit output data
output_df = pd.read_csv('output.csv')
# Ensure data is properly loaded

print(waypoints_df.head())
print(output_df.head())

# Calculate average speed
average_speed = output_df['speed'].mean()

# Calculate runtime
runtime = output_df['time'].iloc[-1] - output_df['time'].iloc[0]


# Calculate Cross Track Error (CTE)
waypoints_coords = waypoints_df[['x', 'y']].values
output_coords = output_df[['x', 'y']].values

# Create a KDTree for fast nearest-neighbor lookup
waypoints_tree = KDTree(waypoints_coords)

# Calculate the distance from each output point to the nearest waypoint
distances, _ = waypoints_tree.query(output_coords)
average_cte = np.mean(distances)


# Plot waypoints and pure pursuit output
plt.figure(figsize=(10, 6))
# Waypoints
plt.plot(waypoints_df['x'], waypoints_df['y'], label='Waypoints', linestyle='-',
marker='o')

# Pure pursuit output
plt.plot(output_df['x'], output_df['y'], label='Pure Pursuit Output', linestyle='-',
marker='x')

# Labels and title
plt.xlabel('X')
plt.ylabel('Y')
plt.title(f'Waypoints vs Pure Pursuit Output\nRuntime: {runtime:.2f} seconds, Average Speed: {average_speed:.2f} m/s, Average CTE: {average_cte:.2f} meters')
plt.legend()
plt.grid()
# Show plot
plt.show()