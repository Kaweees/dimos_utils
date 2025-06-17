#!/bin/bash

# Create build directory
mkdir -p build/classes

# Find LCM jar file
LCM_JAR=$(find /usr/local/share/java -name "lcm*.jar" 2>/dev/null || find /opt/homebrew/share/java -name "lcm*.jar" 2>/dev/null || echo "")

if [ -z "$LCM_JAR" ]; then
    echo "Error: LCM jar file not found. Please make sure LCM is installed."
    exit 1
fi

echo "Found LCM jar: $LCM_JAR"

# Compile all Java files
echo "Compiling Java files..."
find java_lcm_msgs -name "*.java" | xargs javac -cp "$LCM_JAR" -d build/classes

# Create JAR file
echo "Creating JAR file..."
jar cf lcm_types.jar -C build/classes .

echo "Done! Created lcm_types.jar"
echo "To use with lcm-spy, run: lcm-spy --lcm-url=YOURURL -cp $LCM_JAR:lcm_types.jar"
