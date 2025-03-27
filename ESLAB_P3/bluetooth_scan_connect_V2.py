#!/usr/bin/env python3

import asyncio
import sys
import time
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError

# --- Configuration ---
# !! IMPORTANT !! Replace these with the actual UUIDs from your BLE Tool App
TARGET_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb" # Example: HM-10 service
# Characteristic whose CCCD we want to monitor
TARGET_CHAR_UUID_FOR_CCCD = "0000ffe4-0000-1000-8000-00805f9b34fb" # Example: A characteristic that HAS a CCCD

# Standard UUID for the Client Characteristic Configuration Descriptor (CCCD)
CCCD_UUID = "00002902-0000-1000-8000-00805f9b34fb"

# --- Globals ---
discovered_devices = []
POLLING_INTERVAL_S = 2.0  # How often to read the CCCD value (in seconds)
MONITORING_DURATION_S = 30 # How long to monitor for changes

# --- Helper ---
def decode_cccd_value(value: bytes) -> str:
    """Decodes the CCCD value into a human-readable string."""
    val_int = int.from_bytes(value, byteorder='little')
    if val_int == 0x0000:
        return "0x0000 (Notifications/Indications Disabled)"
    elif val_int == 0x0001:
        return "0x0001 (Notifications Enabled)"
    elif val_int == 0x0002:
        return "0x0002 (Indications Enabled)"
    else:
        return f"Unknown value: {value.hex()}"

async def scan_devices():
    """Scans for BLE devices."""
    global discovered_devices
    discovered_devices = []
    print("Scanning for BLE devices for 5 seconds...")
    try:
        devices = await BleakScanner.discover(timeout=5.0)
        if not devices:
            print("No BLE devices found.")
            return False

        print("\n--- Discovered Devices ---")
        i = 1
        for device in devices:
            # if device.connectable:
            if device.name and i < 50:
                 name = device.name
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

async def connect_and_monitor_cccd(device_info):
    """Connects to the selected device and monitors the CCCD value."""
    address = device_info["device"].address
    print(f"\nAttempting to connect to {device_info['display']}...")

    last_cccd_value = None # Store the last known value

    try:
        async with BleakClient(address, timeout=15.0) as client:
            if client.is_connected:
                print(f"Successfully connected to {address}")

                # --- Find the target service, characteristic, and CCCD ---
                target_cccd = None
                print(f"Looking for service: {TARGET_SERVICE_UUID}")
                target_service = client.services.get_service(TARGET_SERVICE_UUID)

                if not target_service:
                    print(f"Error: Service {TARGET_SERVICE_UUID} not found.")
                    return # Exit if service not found

                print(f"Found target service: {target_service.uuid}")
                print(f"  Looking for characteristic: {TARGET_CHAR_UUID_FOR_CCCD}")
                target_char = target_service.get_characteristic(TARGET_CHAR_UUID_FOR_CCCD)

                if not target_char:
                     print(f"Error: Characteristic {TARGET_CHAR_UUID_FOR_CCCD} not found.")
                     return # Exit if characteristic not found

                print(f"Found target characteristic: {target_char.uuid}")
                print(f"    Properties: {', '.join(target_char.properties)}")
                print(f"      Looking for CCCD ({CCCD_UUID})...")
                for descriptor in target_char.descriptors:
                     if descriptor.uuid.lower() == CCCD_UUID.lower():
                         target_cccd = descriptor
                         print(f"Found CCCD: Handle {target_cccd.handle}")
                         break

                if not target_cccd:
                    print(f"Error: CCCD ({CCCD_UUID}) not found for characteristic {target_char.uuid}.")
                    print("       Cannot monitor its value.")
                    return # Exit if CCCD not found

                # --- Start monitoring loop ---
                print(f"\nMonitoring CCCD ({target_cccd.uuid}) on handle {target_cccd.handle} for {MONITORING_DURATION_S} seconds...")
                print(f"Polling every {POLLING_INTERVAL_S} seconds.")

                start_time = time.monotonic()
                while time.monotonic() - start_time < MONITORING_DURATION_S:
                    try:
                        # Read the current CCCD value
                        current_cccd_value = await client.read_gatt_descriptor(target_cccd.handle)

                        if last_cccd_value is None:
                            # First read
                            print(f"Initial CCCD value: {decode_cccd_value(current_cccd_value)} [{current_cccd_value.hex()}]")
                            last_cccd_value = current_cccd_value
                        elif current_cccd_value != last_cccd_value:
                            # Value has changed!
                            print("-" * 20)
                            print(f"!!! CCCD VALUE CHANGED !!!")
                            print(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}")
                            print(f"Previous value: {decode_cccd_value(last_cccd_value)} [{last_cccd_value.hex()}]")
                            print(f"New value     : {decode_cccd_value(current_cccd_value)} [{current_cccd_value.hex()}]")
                            print("-" * 20)
                            last_cccd_value = current_cccd_value
                        else:
                            # Value hasn't changed, maybe print periodic status? (Optional)
                            # print(f"CCCD value unchanged: {decode_cccd_value(current_cccd_value)}")
                            pass

                        # Wait before the next poll
                        await asyncio.sleep(POLLING_INTERVAL_S)

                    except BleakError as e:
                        print(f"\nError during CCCD read or while waiting: {e}")
                        print("Device may have disconnected. Stopping monitoring.")
                        break # Exit monitoring loop on error
                    except Exception as e:
                        print(f"\nAn unexpected error occurred during polling loop: {e}")
                        break # Exit monitoring loop on error

                print(f"\nFinished monitoring after {MONITORING_DURATION_S} seconds.")

            else:
                print(f"Failed to connect to {address}")

    except BleakError as e:
        print(f"Error during connection: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print(f"Disconnecting from {address} (if connected)...")
        # Disconnection happens automatically when exiting 'async with' block

async def main():
    """Main function to run the BLE scan and connect process."""
    if await scan_devices():
        while True:
            try:
                selection = input(f"Enter the number of the device to connect (1-{len(discovered_devices)}), or 'q' to quit: ")
                if selection.lower() == 'q':
                    print("Exiting.")
                    break
                dev_index = int(selection) - 1
                if 0 <= dev_index < len(discovered_devices):
                    selected_device_info = discovered_devices[dev_index]
                    await connect_and_monitor_cccd(selected_device_info)
                    break # Exit after interaction attempt
                else:
                    print("Invalid selection. Please try again.")
            except ValueError:
                print("Invalid input. Please enter a number or 'q'.")
            except KeyboardInterrupt:
                print("\nOperation cancelled by user.")
                break

if __name__ == "__main__":
    print("BLE Scan, Connect, and Monitor CCCD Tool")
    print("-----------------------------------------")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting.")
    except BleakError as e:
         print(f"\nAn unhandled BleakError occurred: {e}")
         print("Ensure Bluetooth is enabled and the script has permissions (e.g., run with sudo or add user to 'bluetooth' group).")
    except Exception as e:
         print(f"\nAn unexpected error occurred in the main loop: {e}")