import socket
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

# Server configuration
HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 8002       # Port to match STM32 client

# Data buffers for plotting (using deques for efficient append/pop)
MAX_DATA_POINTS = 500  # Maximum data points to keep in the plot
raw_x = deque(maxlen=MAX_DATA_POINTS)
raw_y = deque(maxlen=MAX_DATA_POINTS)
raw_z = deque(maxlen=MAX_DATA_POINTS)
filtered_x = deque(maxlen=MAX_DATA_POINTS)
filtered_y = deque(maxlen=MAX_DATA_POINTS)
filtered_z = deque(maxlen=MAX_DATA_POINTS)

# Thread synchronization
data_lock = threading.Lock()
new_data_available = False

# Set up the figure with three subplots (one for each axis)
fig, axs = plt.subplots(3, 1, figsize=(12, 10))
fig.suptitle('Accelerometer Data Visualization', fontsize=16)

# Initialize line objects for each axis
# X-axis subplot (raw and filtered)
raw_line_x, = axs[0].plot([], [], 'r-', label='Raw')
filtered_line_x, = axs[0].plot([], [], 'b-', label='Filtered')
axs[0].set_title('X-axis Acceleration')
axs[0].set_ylabel('Acceleration')
axs[0].legend()
axs[0].grid(True)

# Y-axis subplot (raw and filtered)
raw_line_y, = axs[1].plot([], [], 'r-', label='Raw')
filtered_line_y, = axs[1].plot([], [], 'b-', label='Filtered')
axs[1].set_title('Y-axis Acceleration')
axs[1].set_ylabel('Acceleration')
axs[1].legend()
axs[1].grid(True)

# Z-axis subplot (raw and filtered)
raw_line_z, = axs[2].plot([], [], 'r-', label='Raw')
filtered_line_z, = axs[2].plot([], [], 'b-', label='Filtered')
axs[2].set_title('Z-axis Acceleration')
axs[2].set_xlabel('Sample')
axs[2].set_ylabel('Acceleration')
axs[2].legend()
axs[2].grid(True)

def receive_data():
    """Server function that receives data from the STM32"""
    global new_data_available
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        print(f"TCP Server listening on {HOST}:{PORT}...")
        
        while True:
            # Wait for a connection
            print("Waiting for a connection...")
            client_socket, client_address = server_socket.accept()
            print(f"Connection established with {client_address}")
            
            buffer = ""
            try:
                while True:
                    # Receive data in chunks
                    data = client_socket.recv(4096).decode('utf-8')
                    if not data:
                        print("Client disconnected")
                        break
                    
                    buffer += data
                    
                    # Process complete JSON objects (separated by newlines)
                    while '\n' in buffer:
                        line_end = buffer.find('\n')
                        json_str = buffer[:line_end].strip()
                        buffer = buffer[line_end + 1:]
                        
                        try:
                            # Parse the JSON data
                            sensor_data = json.loads(json_str)
                            
                            # Verify the data has the expected format
                            if 'raw' in sensor_data and 'filtered' in sensor_data:
                                with data_lock:
                                    # Add data points from the raw data block
                                    for point in sensor_data['raw']:
                                        raw_x.append(point['x'])
                                        raw_y.append(point['y'])
                                        raw_z.append(point['z'])
                                    
                                    # Add data points from the filtered data block
                                    for point in sensor_data['filtered']:
                                        filtered_x.append(point['x'])
                                        filtered_y.append(point['y'])
                                        filtered_z.append(point['z'])
                                    
                                    new_data_available = True
                            else:
                                print("Invalid data format: 'raw' or 'filtered' keys missing")
                        
                        except json.JSONDecodeError:
                            print(f"Failed to parse JSON: {json_str}")
                        except KeyError as e:
                            print(f"Key error: {e} missing in the data")
                        except Exception as e:
                            print(f"Error processing data: {e}")
            
            except Exception as e:
                print(f"Connection error: {e}")
            finally:
                client_socket.close()
                print("Connection closed")
    
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server_socket.close()
        print("Server socket closed")

def update_plot(frame):
    """Update function for animation"""
    global new_data_available
    
    if new_data_available:
        with data_lock:
            # Get array indices for x-axis
            x_indices = np.arange(len(raw_x))
            
            # Update X-axis subplot
            raw_line_x.set_data(x_indices, list(raw_x))
            filtered_line_x.set_data(x_indices, list(filtered_x))
            
            # Update Y-axis subplot
            raw_line_y.set_data(x_indices, list(raw_y))
            filtered_line_y.set_data(x_indices, list(filtered_y))
            
            # Update Z-axis subplot
            raw_line_z.set_data(x_indices, list(raw_z))
            filtered_line_z.set_data(x_indices, list(filtered_z))
            
            # Adjust axis limits for X-axis plot
            if len(raw_x) > 0:
                axs[0].set_xlim(0, len(raw_x))
                x_data = list(raw_x) + list(filtered_x)
                if x_data:
                    min_val = min(x_data)
                    max_val = max(x_data)
                    margin = max(1, (max_val - min_val) * 0.1)  # 10% margin
                    axs[0].set_ylim(min_val - margin, max_val + margin)
            
            # Adjust axis limits for Y-axis plot
            if len(raw_y) > 0:
                axs[1].set_xlim(0, len(raw_y))
                y_data = list(raw_y) + list(filtered_y)
                if y_data:
                    min_val = min(y_data)
                    max_val = max(y_data)
                    margin = max(1, (max_val - min_val) * 0.1)  # 10% margin
                    axs[1].set_ylim(min_val - margin, max_val + margin)
            
            # Adjust axis limits for Z-axis plot
            if len(raw_z) > 0:
                axs[2].set_xlim(0, len(raw_z))
                z_data = list(raw_z) + list(filtered_z)
                if z_data:
                    min_val = min(z_data)
                    max_val = max(z_data)
                    margin = max(1, (max_val - min_val) * 0.1)  # 10% margin
                    axs[2].set_ylim(min_val - margin, max_val + margin)
            
            new_data_available = False
    
    return raw_line_x, filtered_line_x, raw_line_y, filtered_line_y, raw_line_z, filtered_line_z

# Start the server in a separate thread
server_thread = threading.Thread(target=receive_data, daemon=True)
server_thread.start()

# Create animation
ani = animation.FuncAnimation(
    fig, update_plot, interval=100, blit=True)

plt.tight_layout()
plt.show()