#!/bin/bash

# Prompt user for interface and file name
read -p "Enter the network interface for tcpdump: " TCPDUMP_INTERFACE
read -p "Enter the output filename for tcpdump (e.g., capture.pcap): " TCPDUMP_OUTPUT

# Prompt user for the duration of the capture
read -p "Enter the number of seconds you want to record: " RECORD_DURATION

# Validate that the duration is a positive integer
if ! [[ "$RECORD_DURATION" =~ ^[0-9]+$ ]]; then
  echo "Error: Duration must be a positive integer."
  exit 1
fi

# Check if the interface exists
if ! ip link show "$TCPDUMP_INTERFACE" > /dev/null 2>&1; then
  echo "Error: Network interface $TCPDUMP_INTERFACE does not exist."
  exit 1
fi

# Start tcpdump in the background
echo "Starting tcpdump on interface $TCPDUMP_INTERFACE for $RECORD_DURATION seconds..."
tcpdump -i "$TCPDUMP_INTERFACE" -w "$TCPDUMP_OUTPUT" -s96 &
TCPDUMP_PID=$!

# Run a timer in the background and stop tcpdump after the duration
{
  sleep "$RECORD_DURATION"
  kill "$TCPDUMP_PID" 2>/dev/null
} &

echo "tcpdump is running in the background with PID $TCPDUMP_PID."
echo "You can continue with other tasks."