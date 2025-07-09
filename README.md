# LCM Message Definitions and ROS msg to LCM Converter

This repository contains LCM message definitions and a tool to convert ROS messages to LCM messages.

## Converting ROS Messages to LCM

- The Python msg to lcm converter is used to convert the ROS messages to LCM messages.

```py
python3 ros_to_lcm.py ros_msgs lcm_files
```

- The lcm-gen command is used to generate the language bindings for the LCM message definitions.

```sh
./lcm_batch_processor.sh -p lcm_files -o python_lcm_msgs/lcm_msgs
```

- Then to make sure tha Python messages package is installable, run

```sh
cd python_lcm_msgs
python3 fix_imports.py
```

- The Foxglove bridge converts all LCM messages to JSON and sends them via WebSocket to Foxglove for visualization.

```py
python3 lcm_foxglove_bridge.py
```

## Using LCM-Spy with Custom Message Types

To use lcm-spy with your custom message types, you need to build a JAR file containing the Java bindings for your LCM message types.

### Building the JAR File

1. Make sure you have the Java LCM bindings generated in the `java_lcm_msgs` directory.
2. Run the build script to create the JAR file:

```sh
./build_lcm_jar.sh
```

This will compile all Java message definitions and create `lcm_types.jar` in the project directory.

### Running LCM-Spy

Use the provided script to run lcm-spy with your custom JAR file:

```sh
./run_lcm_spy.sh
```

You can also pass additional arguments to lcm-spy:

```sh
# For a specific LCM URL
./run_lcm_spy.sh --lcm-url=udpm://239.255.76.67:7667?ttl=1

# For a specific interface
./run_lcm_spy.sh --lcm-url=udpm://<multicast-address>:<port>
```

When running lcm-spy this way, all your custom message types will be automatically decoded and displayed in the lcm-spy interface.

### Manual Execution

If you prefer to run lcm-spy manually, use:

```sh
CLASSPATH=/path/to/lcm.jar:lcm_types.jar lcm-spy
```

Replace `/path/to/lcm.jar` with the actual path to the LCM Java library on your system.
