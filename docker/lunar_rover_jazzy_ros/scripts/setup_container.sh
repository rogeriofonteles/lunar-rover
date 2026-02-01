#!/bin/bash

set -e

# Download external dependencies
vcs import ext/src < /dependencies.repos

source /opt/ros/jazzy/setup.bash

# rosdep
cd ext
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy

# Compile external packages
colcon build

echo "✅ Setup complete."

# Install fish ros2 plugin
if [ -t 1 ]; then
    echo "🟢 Interactive mode detected — launching shell..."
    exec fish
else
    echo "🟡 Non-interactive mode — sleeping to keep container alive..."
    exec sleep infinity
fi