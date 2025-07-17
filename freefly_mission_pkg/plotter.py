import numpy as np
import plotly.graph_objects as go
import plotly.express as px

def plot_3d_data(filename):
    """
    Read 3D coordinate data from a file and create an interactive 3D plot.
    
    Args:
        filename (str): Path to the data file with comma-separated x,y,z coordinates
    """
    try:
        # Read the data from file
        data = np.loadtxt(filename, delimiter=',')
        
        # Extract x, y, z coordinates
        x = data[:, 0]
        y = data[:, 1]
        z = -1*data[:, 2]  # Convert NED to altitude
        
        # Create color scale based on point sequence
        colors = np.arange(len(x))
        
        # Create the 3D plot
        fig = go.Figure()
        
        # Add trajectory line
        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines',
            line=dict(color='blue', width=4),
            name='Trajectory',
            hovertemplate='<b>Trajectory</b><br>' +
                         'X: %{x:.3f}<br>' +
                         'Y: %{y:.3f}<br>' +
                         'Z: %{z:.3f}<extra></extra>'
        ))
        
        # Add scatter points with color progression
        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='markers',
            marker=dict(
                size=6,
                color=colors,
                colorscale='Viridis',
                showscale=True,
                colorbar=dict(title="Point Index"),
                opacity=0.8
            ),
            name='Data Points',
            hovertemplate='<b>Point %{marker.color}</b><br>' +
                         'X: %{x:.3f}<br>' +
                         'Y: %{y:.3f}<br>' +
                         'Z: %{z:.3f}<extra></extra>'
        ))
        
        # Add start and end markers
        if len(x) > 0:
            # Start point (green)
            fig.add_trace(go.Scatter3d(
                x=[x[0]], y=[y[0]], z=[z[0]],
                mode='markers',
                marker=dict(size=12, color='green', symbol='diamond'),
                name='Start',
                hovertemplate='<b>Start Point</b><br>' +
                             'X: %{x:.3f}<br>' +
                             'Y: %{y:.3f}<br>' +
                             'Z: %{z:.3f}<extra></extra>'
            ))
            
            # End point (red)
            fig.add_trace(go.Scatter3d(
                x=[x[-1]], y=[y[-1]], z=[z[-1]],
                mode='markers',
                marker=dict(size=12, color='red', symbol='diamond'),
                name='End',
                hovertemplate='<b>End Point</b><br>' +
                             'X: %{x:.3f}<br>' +
                             'Y: %{y:.3f}<br>' +
                             'Z: %{z:.3f}<extra></extra>'
            ))
        
        # Customize the layout
        fig.update_layout(
            title={
                'text': 'Drone 3D Trajectory Visualization',
                'x': 0.5,
                'xanchor': 'center',
                'font': {'size': 20}
            },
            scene=dict(
                xaxis_title='X Coordinate (m)',
                yaxis_title='Y Coordinate (m)',
                zaxis_title='Z Coordinate (m)',
                xaxis=dict(range=[-40, 40]),
                yaxis=dict(range=[-40, 40]),
                zaxis=dict(range=[0, 40]),
                aspectmode='cube',
                camera=dict(
                    eye=dict(x=1.5, y=1.5, z=1.5)
                )
            ),
            legend=dict(
                x=0.02,
                y=0.98,
                bgcolor='rgba(255, 255, 255, 0.8)'
            ),
            width=1200,
            height=800
        )
        
        # Print some statistics
        print(f"Data loaded successfully!")
        print(f"Number of points: {len(x)}")
        print(f"X range: {x.min():.6f} to {x.max():.6f}")
        print(f"Y range: {y.min():.6f} to {y.max():.6f}")
        print(f"Z range: {z.min():.6f} to {z.max():.6f}")
        
        # Show the interactive plot
        fig.show()
        
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
    except Exception as e:
        print(f"Error reading file: {e}")


# Example usage
if __name__ == "__main__":
    # Replace with the actual filename
    filename = '/home/abhinandan/Downloads/Freefly/freefly_ws/src/freefly_mission_pkg/data/drone_position.txt'
    
    plot_3d_data(filename)