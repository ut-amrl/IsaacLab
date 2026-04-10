#!/bin/bash
# Run a cobot_python script with system Python so rclpy works.
# The venv's Python often lacks the rclpy native module; system Python has it.
#
# Usage: run_cobot_script.sh <script_name> [args...]
# Example: run_cobot_script.sh cobot_pick_v2 --query "cup"
#
# Must be run after sourcing activate_openpi_env.sh (for ROS 2 and workspace),
# or in a shell where install/setup.bash and PYTHONPATH are already set.

COBOT_WS="${COBOT_WS:-/workspace/isaaclab/cobot_ws}"
COBOT_PYTHON="${COBOT_WS}/src/cobot/cobot_python"
SCRIPT_NAME="$1"
shift

# Source workspace install so PYTHONPATH includes amrl_msgs and other built packages.
if [ -f "${COBOT_WS}/install/setup.bash" ]; then
    set +u
    source "${COBOT_WS}/install/setup.bash"
    set -u
fi

if [ -z "$SCRIPT_NAME" ]; then
    echo "Usage: run_cobot_script.sh <script_name> [args...]"
    echo "Example: run_cobot_script.sh cobot_pick_v2 --query \"cup\""
    echo "Scripts live in: $COBOT_PYTHON"
    exit 1
fi

SCRIPT_PATH="${COBOT_PYTHON}/${SCRIPT_NAME}.py"
if [ ! -f "$SCRIPT_PATH" ]; then
    echo "Error: script not found: $SCRIPT_PATH"
    exit 1
fi

# System Python has rclpy; venv has kortex_api (from setup_venv_cobot.sh). Prepend
# venv site-packages so system Python can import kortex_api when running cobot scripts.
VENV_ROOT="${VENV_ROOT:-/root/.venv_cobot}"
if [ -d "$VENV_ROOT/lib" ]; then
    VENV_SITE=$(find "$VENV_ROOT/lib" -maxdepth 1 -type d -name "python*" 2>/dev/null | head -1)
    if [ -n "$VENV_SITE" ] && [ -d "${VENV_SITE}/site-packages" ]; then
        VENV_SITE="${VENV_SITE}/site-packages"
    else
        VENV_SITE=""
    fi
else
    VENV_SITE=""
fi
# Workspace install Python paths (amrl_msgs.GroundedSAM2Srv, etc.). Use find so we always
# pick up paths even when glob fails (e.g. in container).
INSTALL_PYTHON=""
if [ -d "${COBOT_WS}/install" ]; then
    INSTALL_PYTHON=$(find "${COBOT_WS}/install" -type d -path "*/local/lib/python*/dist-packages" 2>/dev/null | tr '\n' ':')
    INSTALL_PYTHON="${INSTALL_PYTHON%:}"  # strip trailing colon
fi

# Prepend venv, then install paths, then cobot; keep existing PYTHONPATH from setup.bash.
PREPEND="${VENV_SITE:+${VENV_SITE}:}${INSTALL_PYTHON:+${INSTALL_PYTHON}:}${COBOT_WS}:${COBOT_WS}/src/cobot"
export PYTHONPATH="${PREPEND}${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 "$SCRIPT_PATH" "$@"
