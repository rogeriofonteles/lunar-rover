#!/bin/bash

set -e

cd ~
# Create virtual environment
python3 -m venv env
# Install pydrake
env/bin/pip install drake

echo "✅ Setup complete."

# Install fish ros2 plugin
if [ -t 1 ]; then
    echo "🟢 Interactive mode detected — launching shell..."
    exec fish
else
    echo "🟡 Non-interactive mode — sleeping to keep container alive..."
    exec sleep infinity
fi