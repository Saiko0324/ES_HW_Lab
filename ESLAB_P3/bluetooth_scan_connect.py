#!/usr/bin/env python3

import asyncio
import sys
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError

# --- Configuration ---
# !! IMPORTANT !! Replace these with the actual UUIDs from your BLE Tool App
# Example UUIDs (replace with your actual service/characteristic UUIDs)
# These often look like: "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
TARGET_SERVICE_UUID = "0000fff0-0000-1000-8000-00805f9b34fb" # Example: HM-10 service
TARGET_CHAR_UUID_INDICATE = "0000fff2-0000-1000-8000-00805f9b34fb" # Example: A characteristic supporting indications

# Standard UUID for the Client Characteristic Configuration Descriptor (CCCD)
CCCD_UUID = "00002904-0000-1000-8000-00805f9b34fb"

# --- Globals ---
discovered_devices = []

# --- Notification/Indication Handler ---
def indication_handler(sender: int, data: bytearray):
    """Simple handler for BLE indications (and notifications)."""
    print(f"\n[Indication] Handle: {sender}, Data: {data.hex()} ({data.decode(errors='ignore')})")

async def scan_devices():
    """Scans for BLE devices."""
    global discovered_devices
    discovered_devices = []
    print("Scanning for BLE devices for 5 seconds...")
    try:
        # Scan for 5 seconds
        devices = await BleakScanner.discover(timeout=5.0)

        if not devices:
            print("No BLE devices found.")
            return False

        print("\n--- Discovered Devices ---")
        i = 1
        for device in devices:
            # Filter out devices without a name unless they have interesting data?
            # For simplicity, list all connectable devices found.
            if device.connectable:
                 # Store device object along with its display name/address
                 name = device.name if device.name else "Unknown"
                 discovered_devices.append({"device": device, "display": f"{name} ({device.address})"})
                 print(f"{i}. {name} ({device.address}) RSSI: {device.rssi}")
                 i += 1

        if not discovered_devices:
            print("No *connectable* BLE devices found.")
            return False

        return True

    except BleakError as e:
        print(f"Error during scanning: {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred during scanning: {e}")
        return False


async def connect_and_interact(device_info):
    """Connects to the selected device and interacts with it."""
    address = device_info["device"].address
    print(f"\nAttempting to connect to {device_info['display']}...")

    # Use 'async with' for automatic connection management (connect/disconnect)
    async with BleakClient(address, timeout=15.0) as client:
        try:
            if client.is_connected:
                print(f"Successfully connected to {address}")

                # --- Find the target service and characteristic ---
                target_service = None
                target_char = None
                target_cccd = None

                print(f"Looking for service: {TARGET_SERVICE_UUID}")
                for service in client.services:
                    # print(f"  Service Found: {service.uuid}") # Uncomment for debugging
                    if service.uuid.lower() == TARGET_SERVICE_UUID.lower():
                        target_service = service
                        print(f"Found target service: {service.uuid}")
                        print(f"  Looking for characteristic: {TARGET_CHAR_UUID_INDICATE}")
                        for char in service.characteristics:
                            # print(f"    Characteristic Found: {char.uuid}") # Uncomment for debugging
                            if char.uuid.lower() == TARGET_CHAR_UUID_INDICATE.lower():
                                target_char = char
                                print(f"Found target characteristic: {char.uuid}")
                                print(f"    Properties: {', '.join(char.properties)}")

                                # Check if characteristic supports indication
                                if "indicate" not in char.properties:
                                     print(f"      Warning: Characteristic {char.uuid} may not support indications!")

                                # Find the CCCD descriptor for this characteristic
                                print(f"      Looking for CCCD ({CCCD_UUID})...")
                                for descriptor in char.descriptors:
                                     # print(f"        Descriptor Found: {descriptor.uuid}") # Uncomment for debugging
                                     if descriptor.uuid.lower() == CCCD_UUID.lower():
                                         target_cccd = descriptor
                                         print(f"Found CCCD: Handle {target_cccd.handle}")
                                         break # Found CCCD
                                break # Found characteristic
                        break # Found service

                if not target_service:
                    print(f"Error: Service {TARGET_SERVICE_UUID} not found on the device.")
                    return
                if not target_char:
                    print(f"Error: Characteristic {TARGET_CHAR_UUID_INDICATE} not found within the service.")
                    return
                if not target_cccd:
                    print(f"Error: CCCD ({CCCD_UUID}) not found for the characteristic.")
                    print("       Cannot enable indications/notifications.")
                    return

                # --- Enable Indications by writing to the CCCD ---
                # Value 0x0001 is for Notifications
                # Value 0x0002 is for Indications
                indication_value = b'\x02\x00' # Little-endian 16-bit value 0x0002

                print(f"\nWriting {indication_value.hex()} (0x0002) to CCCD handle {target_cccd.handle} to enable indications...")
                try:
                    await client.write_gatt_descriptor(target_cccd.handle, indication_value)
                    print("Successfully wrote to CCCD.")

                    # Register the indication handler AFTER enabling indications via CCCD
                    print("Registering indication handler...")
                    await client.start_notify(target_char.uuid, indication_handler)
                    print("Indications enabled. Waiting for data...")

                    # Keep the connection alive for a while to receive indication
                    await asyncio.sleep(180) # Wait for 180 seconds

                    print("\nStopping indications...")
                    await client.stop_notify(target_char.uuid)
                    # Optionally write 0x0000 back to CCCD to disable
                    # await client.write_gatt_descriptor(target_cccd.handle, b'\x00\x00')

                except BleakError as e:
                    print(f"Error writing to CCCD or starting notifications: {e}")
                except Exception as e:
                    print(f"An unexpected error occurred during GATT operations: {e}")

            else:
                print(f"Failed to connect to {address}")

        except BleakError as e:
            print(f"Error during connection or communication: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        finally:
            # Disconnection is handled automatically by 'async with'
            if client.is_connected:
                 print("\nDisconnecting...") # Should technically be disconnected already

    print(f"Disconnected from {address}")


async def main():
    """Main function to run the BLE scan and connect process."""
    if await scan_devices():
        while True:
            try:
                selection = input(f"Enter the number of the device to connect (1-{len(discovered_devices)}), or 'q' to quit: ")
                if selection.lower() == 'q':
                    print("Exiting.")
                    break
                dev_index = int(selection) - 1 # Adjust for 0-based index
                if 0 <= dev_index < len(discovered_devices):
                    selected_device_info = discovered_devices[dev_index]
                    await connect_and_interact(selected_device_info)
                    break # Exit after interaction attempt
                else:
                    print("Invalid selection. Please try again.")
            except ValueError:
                print("Invalid input. Please enter a number or 'q'.")
            except KeyboardInterrupt:
                print("\nOperation cancelled by user.")
                break

if __name__ == "__main__":
    print("BLE Scan and Connect Tool")
    print("-------------------------")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting.")
    except BleakError as e:
         print(f"An unhandled BleakError occurred: {e}")
         print("Ensure Bluetooth is enabled and the script has permissions (e.g., run with sudo or add user to 'bluetooth' group).")
    except Exception as e:
         print(f"An unexpected error occurred in the main loop: {e}")
