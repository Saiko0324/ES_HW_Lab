import asyncio
import struct
import sys
import argparse
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# --- Configuration ---
# UUIDs copied from the STM32 C code (formatted for Bleak)
ACCEL_SERVICE_UUID = "00000000-0000-00b0-0000-000000000000"
ACCEL_DATA_CHAR_UUID = "00000001-0000-00b0-0000-000000000000" # (characteristic_a) Notify, Read
ACCEL_FREQ_CHAR_UUID = "00000002-0000-00b0-0000-000000000000" # (characteristic_b) Read, Write

# Global flag to signal disconnection
disconnect_event = asyncio.Event()

# --- Notification Handler ---
def handle_accel_notification(sender: int, data: bytearray):
    """
    Callback function to handle incoming acceleration data notifications.
    STM32 sends 3 x int16_t (x, y, z) = 6 bytes, little-endian.
    """
    if len(data) == 6:
        # Unpack 3 signed 16-bit integers (short), little-endian
        x, y, z = struct.unpack('<hhh', data)
        print(f"[*] Accel Notification: X={x: 6d}, Y={y: 6d}, Z={z: 6d} (mg)")
        # You can add further processing here (e.g., logging, filtering)
    else:
        print(f"[!] Received unexpected data length: {len(data)} bytes. Data: {data.hex()}")

# --- Main BLE Logic ---
async def run_ble_client(device_address: str, new_frequency: int):
    """
    Connects to the STM32, sets frequency, and listens for notifications.
    """
    print(f"[*] Attempting to connect to device: {device_address}")
    async with BleakClient(device_address, timeout=20.0) as client:
        if not client.is_connected:
            print(f"[!] Failed to connect to {device_address}")
            return

        print(f"[+] Connected to {device_address}")

        try:
            # --- 1. Read Current Frequency ---
            print(f"[*] Reading current sampling frequency (UUID: {ACCEL_FREQ_CHAR_UUID})...")
            freq_data_bytes = await client.read_gatt_char(ACCEL_FREQ_CHAR_UUID)
            # Unpack 1 unsigned 16-bit integer (unsigned short), little-endian
            current_frequency = struct.unpack('<H', freq_data_bytes)[0]
            print(f"[+] Current frequency: {current_frequency} Hz")

            # --- 2. Write New Frequency ---
            if new_frequency is not None:
                print(f"[*] Writing new sampling frequency: {new_frequency} Hz (UUID: {ACCEL_FREQ_CHAR_UUID})...")
                # Pack 1 unsigned 16-bit integer, little-endian
                new_freq_bytes = struct.pack('<H', new_frequency)
                await client.write_gatt_char(ACCEL_FREQ_CHAR_UUID, new_freq_bytes, response=True) # Use response=True for reliable write
                print(f"[+] Frequency set to {new_frequency} Hz")

                # Optional: Read back to verify
                await asyncio.sleep(0.5) # Give STM32 time to process
                freq_data_bytes = await client.read_gatt_char(ACCEL_FREQ_CHAR_UUID)
                verify_frequency = struct.unpack('<H', freq_data_bytes)[0]
                print(f"[+] Verified frequency: {verify_frequency} Hz")
                if verify_frequency != new_frequency:
                     print(f"[!] Warning: Verification failed. Expected {new_frequency}, got {verify_frequency}")
            else:
                print("[*] Skipping frequency write as no new frequency was specified.")


            # --- 3. Enable Notifications for Acceleration Data ---
            print(f"[*] Enabling notifications for acceleration data (UUID: {ACCEL_DATA_CHAR_UUID})...")
            await client.start_notify(ACCEL_DATA_CHAR_UUID, handle_accel_notification)
            print("[+] Notifications enabled. Waiting for data...")

            # --- 4. Keep Running ---
            # Stay connected and wait for notifications until disconnect_event is set
            await disconnect_event.wait()

        except BleakError as e:
            print(f"[!] BLE Error: {e}")
        except Exception as e:
            print(f"[!] An unexpected error occurred: {e}")
        finally:
            if client.is_connected:
                try:
                    print("[*] Disabling notifications...")
                    await client.stop_notify(ACCEL_DATA_CHAR_UUID)
                except BleakError as e:
                     print(f"[!] Error disabling notifications: {e}")
                print("[*] Disconnecting...")
            # The 'async with' block handles the actual disconnection

    print("[-] Client disconnected.")

# --- Main Execution ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="BLE Client for STM32 Accelerometer Service.")
    parser.add_argument("address", help="MAC Address of the STM32 BLE Server.")
    parser.add_argument("-f", "--frequency", type=int, help="New sampling frequency (Hz) to set. If omitted, frequency is only read.", default=None)

    args = parser.parse_args()

    loop = asyncio.get_event_loop()
    try:
        task = loop.create_task(run_ble_client(args.address, args.frequency))
        loop.run_until_complete(task)
    except KeyboardInterrupt:
        print("\n[*] KeyboardInterrupt received. Stopping client...")
        disconnect_event.set() # Signal the client loop to exit gracefully
        # Wait for the task to finish cleanup
        loop.run_until_complete(task)
    except BleakError as e:
         print(f"[!] Bleak Error during startup/shutdown: {e}")
    finally:
        print("[*] Program finished.")
