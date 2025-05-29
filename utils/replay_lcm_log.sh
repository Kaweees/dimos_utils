#!/bin/bash

# Function to cleanup on exit
cleanup() {
    echo "\nReceived interrupt signal. Shutting down..."
    
    # Kill any running lcm-logplayer processes started by this script
    if [ ! -z "$PLAYER_PID" ]; then
        kill $PLAYER_PID 2>/dev/null
    fi
    
    exit 0
}

# Set up trap to catch SIGINT (Ctrl+C) and SIGTERM
trap cleanup SIGINT SIGTERM

# Check if a filename was provided
if [ $# -eq 0 ]; then
    echo "Error: No filename provided"
    echo "Usage: $0 <lcm-log-file>"
    exit 1
fi

# Store the filename argument
LCM_LOG_FILE="$1"

# Check if the file exists
if [ ! -f "$LCM_LOG_FILE" ]; then
    echo "Error: File '$LCM_LOG_FILE' not found"
    exit 1
fi

echo "Starting continuous replay of '$LCM_LOG_FILE'"
echo "Press Ctrl+C to stop"

# Run in a loop until interrupted
while true; do
    echo "$(date): Running lcm-logplayer on '$LCM_LOG_FILE'"
    
    # Run lcm-logplayer in background so we can track its PID
    lcm-logplayer "$LCM_LOG_FILE" & 
    PLAYER_PID=$!
    
    # Wait for the process to complete
    wait $PLAYER_PID
    
    # Check the exit status
    PLAYER_STATUS=$?
    if [ $PLAYER_STATUS -ne 0 ] && [ $PLAYER_STATUS -ne 130 ]; then
        # Exit code 130 is from Ctrl+C, otherwise it's an error
        echo "lcm-logplayer exited with an error (status $PLAYER_STATUS)"
        sleep 1
    fi
    
    # If the script received a signal, cleanup may have already set PLAYER_PID to empty
    if [ -z "$PLAYER_PID" ]; then
        break
    fi
    
    echo "$(date): Restarting lcm-logplayer..."
done
