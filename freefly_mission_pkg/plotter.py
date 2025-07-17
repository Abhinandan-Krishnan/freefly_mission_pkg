import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_3d_data(filename):
    """
    Read 3D coordinate data from a file and create a 3D plot.
    
    Args:
        filename (str): Path to the data file with comma-separated x,y,z coordinates
    """
    try:
        # Read the data from file
        data = np.loadtxt(filename, delimiter=',')
        
        # Extract x, y, z coordinates
        x = data[:, 0]
        y = data[:, 1]
        z = -1*data[:, 2]
        
        # Create 3D plot
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot the data points
        scatter = ax.scatter(x, y, z, c=np.arange(len(x)), cmap='viridis', s=50, alpha=0.7)
        
        # Add a line connecting the points (useful for trajectory data)
        ax.plot(x, y, z, 'b-', alpha=0.5, linewidth=1)
        
        # Customize the plot
        ax.set_xlabel('X Coordinate')
        ax.set_ylabel('Y Coordinate')
        ax.set_zlabel('Z Coordinate')
        ax.set_title('3D Data Visualization')
        
        # Add colorbar to show progression through data points
        cbar = plt.colorbar(scatter, ax=ax, shrink=0.5)
        cbar.set_label('Data Point Index')
        
        # Make the plot look better
        ax.grid(True, alpha=0.3)
        
        # Print some statistics
        print(f"Data loaded successfully!")
        print(f"Number of points: {len(x)}")
        print(f"X range: {x.min():.6f} to {x.max():.6f}")
        print(f"Y range: {y.min():.6f} to {y.max():.6f}")
        print(f"Z range: {z.min():.6f} to {z.max():.6f}")
        
        plt.show()
        
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
    except Exception as e:
        print(f"Error reading file: {e}")

def plot_3d_data_advanced(filename):
    """
    Advanced 3D plotting with multiple visualization options.
    """
    try:
        data = np.loadtxt(filename, delimiter=',')
        x, y, z = data[:, 0], data[:, 1], -1*data[:, 2]
        
        # Create subplot with multiple views
        fig = plt.figure(figsize=(15, 10))
        
        # 3D scatter plot
        ax1 = fig.add_subplot(221, projection='3d')
        scatter = ax1.scatter(x, y, z, c=np.arange(len(x)), cmap='plasma', s=30)
        ax1.set_title('3D Scatter Plot')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        
        # 3D line plot
        ax2 = fig.add_subplot(222, projection='3d')
        ax2.plot(x, y, z, 'r-', linewidth=2)
        ax2.scatter(x[0], y[0], z[0], color='green', s=100, label='Start')
        ax2.scatter(x[-1], y[-1], z[-1], color='red', s=100, label='End')
        ax2.set_title('3D Trajectory')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_zlabel('Z')
        ax2.legend()
        
        # 2D projections
        ax3 = fig.add_subplot(223)
        ax3.plot(x, y, 'b-', alpha=0.7)
        ax3.scatter(x, y, c=np.arange(len(x)), cmap='viridis', s=20)
        ax3.set_title('XY Projection')
        ax3.set_xlabel('X')
        ax3.set_ylabel('Y')
        ax3.grid(True, alpha=0.3)
        
        ax4 = fig.add_subplot(224)
        ax4.plot(x, z, 'g-', alpha=0.7)
        ax4.scatter(x, z, c=np.arange(len(x)), cmap='viridis', s=20)
        ax4.set_title('XZ Projection')
        ax4.set_xlabel('X')
        ax4.set_ylabel('Z')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
    except Exception as e:
        print(f"Error: {e}")

# Example usage
if __name__ == "__main__":
    # Replace 'your_file.txt' with the actual filename
    filename = '/home/abhinandan/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/data/drone_position.txt'  # Change this to your file path
    
    print("Choose plotting option:")
    print("1. Simple 3D plot")
    print("2. Advanced multi-view plot")
    
    choice = input("Enter choice (1 or 2): ").strip()
    
    if choice == '1':
        plot_3d_data(filename)
    elif choice == '2':
        plot_3d_data_advanced(filename)
    else:
        print("Invalid choice. Running simple plot...")
        plot_3d_data(filename)