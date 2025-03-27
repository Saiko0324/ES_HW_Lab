import socket
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time  # Import the time module

# Define the IP address and port to listen on
HOST = '0.0.0.0'  # Listen on all available interfaces
PORT = 8002       # Port number (must match the STM32 client's port)

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_socket.bind((HOST, PORT))

# Listen for incoming connections (max 1 connection in the queue)
server_socket.listen(1)

print(f"TCP Server listening on {HOST}:{PORT}...")

fig, ax = plt.subplots()
x = []
y = []
z = []
line, = ax.plot(x, y)  # Initialize for smoother updates (optional, but cleaner)

# Function to update the plot (used for animation, but also works in a loop)
def update_plot():
    ax.clear()
    ax.plot(x, label="X")
    ax.plot(y, label="Y")
    ax.plot(z, label="Z")
    ax.legend()
    plt.pause(0.001)  # Small pause for display update

# --- Main Loop ---
while True:
    # Wait for a connection
    print("Waiting for a connection...")
    client_socket, client_address = server_socket.accept()
    print(f"Connection established with {client_address}")

    try:
        while True:
            data = client_socket.recv(1024).decode('utf-8').strip()
            if not data:
                print("Client disconnected.")
                break  # Exit the inner loop if the client disconnects

            try:
                # Parse the JSON data
                sensor_data = json.loads(data)
                print(f"Received sensor data: {sensor_data}")

                # Append new data
                x.append(sensor_data["x"])
                y.append(sensor_data["y"])
                z.append(sensor_data["z"])

                # Update the plot
                update_plot()


                # Send confirmation message
                confirmation_message = "Data received and processed"
                client_socket.sendall(confirmation_message.encode('utf-8'))

            except json.JSONDecodeError:
                print("Invalid JSON data received")
                # Send an error message back to the client
                error_message = "Error: Invalid JSON format"
                client_socket.sendall(error_message.encode('utf-8'))
            except KeyError as e:  #Handle the case if x, y, or z keys do not exist in JSON
                print(f"KeyError: {e} not found in JSON data")
                error_message = f"Error: Key {e} missing in JSON"
                client_socket.sendall(error_message.encode('utf-8'))


    except Exception as e:
        print(f"Error: {e}")
    finally:
        client_socket.close()  # Close the client socket when done or on error
        print("Client socket closed.")
    #plt.show()  # Removed plt.show() from the inner loop, as it blocks
                # Keep it at the very end if needed (see below)

# If you want the plot window to stay open after all connections are closed:
#plt.show()  # Keep the plot open (blocks until manually closed)