"""
dimos_utils - Utility tools for dimensional robotics

This package provides utility tools for dimensional robotics projects,
including LCM-Foxglove bridge functionality and LCM message definitions.
"""

__version__ = "0.1.0"

import os
import sys

# Setup lcm_msgs path for backwards compatibility with seperate python_lcm_msgs package
def _setup_lcm_msgs_path():
    """Setup lcm_msgs path so internal imports work correctly"""
    try:
        # Try to import lcm_msgs to see if it's already installed
        import lcm_msgs
    except ImportError:
        # If import fails, add the local python_lcm_msgs to path
        # ensures internal imports like "from lcm_msgs import std_msgs" work
        current_dir = os.path.dirname(os.path.abspath(__file__))
        lcm_module_dir = os.path.join(current_dir, "python_lcm_msgs")
        if os.path.exists(lcm_module_dir):
            if lcm_module_dir not in sys.path:
                sys.path.insert(0, lcm_module_dir)

_setup_lcm_msgs_path()

from . import lcm_foxglove_bridge

__all__ = ["lcm_foxglove_bridge"] 