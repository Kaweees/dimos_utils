#!/bin/bash

# Script to run lcm-spy with the necessary Java classpath
# Works cross-platform by searching common LCM installation directories

# Function to find the LCM jar file
find_lcm_jar() {
    # Try common locations for macOS
    if [ "$(uname)" == "Darwin" ]; then
        # Check Homebrew location
        BREW_JAR=$(find /opt/homebrew/share/java -name "lcm*.jar" 2>/dev/null)
        if [ ! -z "$BREW_JAR" ]; then
            echo "$BREW_JAR"
            return 0
        fi
        
        # Check MacPorts location
        MAC_JAR=$(find /opt/local/share/java -name "lcm*.jar" 2>/dev/null)
        if [ ! -z "$MAC_JAR" ]; then
            echo "$MAC_JAR"
            return 0
        fi
    fi
    
    # Try common Linux locations
    LINUX_JAR=$(find /usr/local/share/java /usr/share/java -name "lcm*.jar" 2>/dev/null | head -1)
    if [ ! -z "$LINUX_JAR" ]; then
        echo "$LINUX_JAR"
        return 0
    fi
    
    # Try Java installation directories
    JAVA_JAR=$(find $JAVA_HOME/lib -name "lcm*.jar" 2>/dev/null | head -1)
    if [ ! -z "$JAVA_JAR" ]; then
        echo "$JAVA_JAR"
        return 0
    fi
    
    # Try current directory and standard locations
    LOCAL_JAR=$(find . -maxdepth 2 -name "lcm*.jar" 2>/dev/null | grep -v "lcm_types.jar" | head -1)
    if [ ! -z "$LOCAL_JAR" ]; then
        echo "$LOCAL_JAR"
        return 0
    fi
    
    # Not found
    return 1
}

# Find the LCM types jar
LCM_TYPES_JAR="java_lcm_msgs/lcm_types.jar"
if [ ! -f "$LCM_TYPES_JAR" ]; then
    echo "Error: lcm_types.jar not found in java_lcm_msgs directory."
    echo "Make sure to build it first using ./build_lcm_jar.sh and move it to the correct location."
    exit 1
fi

# Find the LCM jar
LCM_JAR=$(find_lcm_jar)
if [ -z "$LCM_JAR" ]; then
    echo "Error: Could not find LCM jar file on your system."
    echo "Please install LCM or specify the path to lcm.jar manually:"
    echo "CLASSPATH=/path/to/lcm.jar:lcm_types.jar lcm-spy"
    exit 1
fi

echo "Found LCM jar: $LCM_JAR"
echo "Found LCM types jar: $LCM_TYPES_JAR"

# Run lcm-spy with the classpath
echo "Running LCM Spy..."
CLASSPATH="$LCM_JAR:$LCM_TYPES_JAR" lcm-spy "$@"
