#!/bin/bash

set -euo pipefail

echo "✅ Drake is ready: $(python -c 'import importlib.metadata; print(importlib.metadata.version("drake"))')"

# Install fish ros2 plugin
if [ -t 1 ]; then
    echo "🟢 Interactive mode detected — launching shell..."
    exec fish
else
    echo "🟡 Non-interactive mode — sleeping to keep container alive..."
    exec sleep infinity
fi
