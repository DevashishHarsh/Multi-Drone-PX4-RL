#!/usr/bin/env bash
# setup.sh - Install PX4, QGroundControl, ROS2 Jazzy, Micro XRCE-DDS on Ubuntu 24.04 (WSL2)
set -euo pipefail
IFS=$'\n\t'

# Configurable variables
PX4_DIR="$HOME/PX4-Autopilot"
PX4_GIT="https://github.com/PX4/PX4-Autopilot.git"
PX4_COMMIT="38d67f5"
QGC_URL="https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage"
QGC_TARGET_DIR="$HOME/QGroundControl"
MICRO_XRCE_DIR="$HOME/Micro-XRCE-DDS-Agent"
MICRO_XRCE_BRANCH="v2.4.2"

if [ "$EUID" -eq 0 ]; then
  echo "Please run this script as a normal user (not root). Some parts will sudo when needed."
  exit 1
fi

echo "=== START: System update & required base packages ==="
sudo apt update -y
sudo apt upgrade -y || true

sudo apt install -y \
    build-essential git cmake ninja-build python3 python3-pip python3-venv \
    wget curl unzip pkg-config lsb-release ca-certificates gnupg \
    mesa-utils sudo jq

# Ensure python3-kconfiglib requested by you
echo "Installing python3-kconfiglib..."
sudo apt install -y python3-kconfiglib || true

echo "=== DONE: Base packages ==="

# --------------- PX4 clone & setup ---------------
echo "=== PX4: clone or update repository ==="
if [ -d "$PX4_DIR" ]; then
  echo "PX4 directory exists at $PX4_DIR. Fetching and resetting to requested commit..."
  pushd "$PX4_DIR" >/dev/null
  git fetch --all --tags --prune || true
  git checkout "$PX4_COMMIT" || git checkout --detach "$PX4_COMMIT"
  git reset --hard "$PX4_COMMIT" || true
  popd >/dev/null
else
  echo "Cloning PX4 into $PX4_DIR..."
  git clone "$PX4_GIT" --recursive "$PX4_DIR" || {
    echo "Initial clone failed. Trying shallow clone + recursive submodules..."
    git clone --recursive "$PX4_GIT" "$PX4_DIR"
  }
  pushd "$PX4_DIR" >/dev/null
  git checkout "$PX4_COMMIT" || git checkout --detach "$PX4_COMMIT"
  popd >/dev/null
fi

# Run PX4 setup script (your step 3)
echo "=== PX4: running Tools/setup/ubuntu.sh ==="
pushd "$PX4_DIR" >/dev/null
# Make sure the script is executable and run it. It will prompt for sudo when needed.
if [ -f Tools/setup/ubuntu.sh ]; then
  bash Tools/setup/ubuntu.sh || {
    echo "PX4 setup script failed (non-fatal). Continuing — you can inspect output above."
  }
else
  echo "ERROR: Tools/setup/ubuntu.sh not found in $PX4_DIR. Aborting PX4 setup."
fi
popd >/dev/null

# --------------- Build px4_sitl ---------------
echo "=== Building px4_sitl (make px4_sitl) ==="
pushd "$PX4_DIR" >/dev/null || true
set +e
make px4_sitl
RC=$?
set -e
if [ $RC -ne 0 ]; then
  echo "make px4_sitl failed. Running fallback: sync & update submodules then retry."
  git submodule sync --recursive || true
  git submodule update --init --recursive || true
  echo "Retrying make px4_sitl..."
  make px4_sitl || {
    echo "Second attempt to build px4_sitl failed. Continuing, but this is likely fatal for simulation."
  }
else
  echo "px4_sitl built successfully."
fi
popd >/dev/null || true

# If successful (or even if not), try the Gazebo run target per your step 9
echo "=== Attempting: make px4_sitl gz_x500 ==="
pushd "$PX4_DIR" >/dev/null || true
set +e
make px4_sitl gz_x500
RC2=$?
set -e
if [ $RC2 -ne 0 ]; then
  echo "make px4_sitl gz_x500 failed. Possible Gazebo Harmonic / dependencies issue. See PX4/Gazebo docs."
else
  echo "make px4_sitl gz_x500 completed (or at least attempted)."
fi
popd >/dev/null || true

# --------------- QGroundControl ---------------
echo "=== Installing QGroundControl AppImage ==="
mkdir -p "$QGC_TARGET_DIR"
QGC_FILE="$QGC_TARGET_DIR/QGroundControl-x86_64.AppImage"
if [ -f "$QGC_FILE" ]; then
  echo "QGroundControl AppImage already downloaded at $QGC_FILE"
else
  wget -O "$QGC_FILE" "$QGC_URL" || {
    echo "Failed to download QGroundControl AppImage from $QGC_URL. Exit code: $?"
  }
  chmod +x "$QGC_FILE" || true
fi
echo "QGroundControl placed at $QGC_FILE"

# Optional: create a small launcher in ~/bin
mkdir -p "$HOME/bin"
cat > "$HOME/bin/qgroundcontrol" <<'EOF'
#!/usr/bin/env bash
"$HOME/Applications/QGroundControl/QGroundControl-x86_64.AppImage" "$@"
EOF
chmod +x "$HOME/bin/qgroundcontrol"
echo 'export PATH="$HOME/bin:$PATH"' >> "$HOME/.bashrc" || true

# --------------- Graphics checks (your instructions) ---------------
echo "=== Graphics checks (glxinfo, nvidia-smi) ==="
if ! command -v glxinfo >/dev/null 2>&1; then
  echo "glxinfo not found: installing mesa-utils..."
  sudo apt install -y mesa-utils || true
fi

echo "glxinfo -B output (short):"
glxinfo -B || true

if command -v nvidia-smi >/dev/null 2>&1; then
  echo "nvidia-smi output:"
  nvidia-smi || true
  echo 'Adding MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA to ~/.bashrc as requested...'
  # Append only if not present
  if ! grep -q "MESA_D3D12_DEFAULT_ADAPTER_NAME" "$HOME/.bashrc"; then
    echo 'echo "export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA" >> ~/.bashrc' >> /tmp/mesa_add.sh
    # We add the export directly to .bashrc (not the echo wrapper) so it takes effect next shells
    echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> "$HOME/.bashrc"
    source "$HOME/.bashrc" || true
  else
    echo "Env var already present in ~/.bashrc"
  fi
else
  echo "nvidia-smi not found or no NVIDIA GPU detected. If you have an NVIDIA GPU, ensure drivers are installed on the host and available to WSL."
fi

# --------------- Install ROS2 Jazzy (binary packages) ---------------
# Using the official ROS 2 Jazzy instructions (debs). See: ROS 2 Jazzy doc.
# Citation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
echo "=== Installing ROS 2 Jazzy (binary debs) ==="
# Locale check / set
if ! locale | grep -q "UTF-8"; then
  echo "Setting up en_US.UTF-8 locale..."
  sudo apt install -y locales
  sudo locale-gen en_US en_US.UTF-8 || true
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true
  export LANG=en_US.UTF-8
fi

# Enable universe repository and install curl if missing
sudo apt update -y
sudo apt install -y software-properties-common gnupg2 curl lsb-release || true
sudo add-apt-repository universe || true

# Add ROS 2 apt repository and key (best-effort - may prompt)
ROS_LIST="/etc/apt/sources.list.d/ros2.list"
if [ ! -f "$ROS_LIST" ]; then
  echo "Adding ROS 2 Apt repository and keys..."
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.gpg || true
  # Add repository line (Ubuntu 24.04 = noble/numbat - ROS 'Jazzy' targets 24.04)
  echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

sudo apt update -y
# Install desktop (this will install many packages)
set +e
sudo apt install -y ros-jazzy-desktop
RC_ROS=$?
set -e
if [ $RC_ROS -ne 0 ]; then
  echo "ros-jazzy-desktop install failed. Check keyring or repo. You may need to update the archive key (see ROS docs)."
else
  echo 'source /opt/ros/jazzy/setup.bash' >> "$HOME/.bashrc"
  source "$HOME/.bashrc" || true
  echo "ROS 2 Jazzy installed and setup line appended to ~/.bashrc."
fi

# --------------- Micro XRCE-DDS Agent ---------------
echo "=== Installing Micro XRCE-DDS Agent (branch: $MICRO_XRCE_BRANCH) ==="
if [ -d "$MICRO_XRCE_DIR" ]; then
  echo "Micro-XRCE-DDS-Agent directory exists. Pulling updates..."
  pushd "$MICRO_XRCE_DIR" >/dev/null
  git fetch --all || true
  git checkout "$MICRO_XRCE_BRANCH" || true
  git pull --ff-only || true
  popd >/dev/null
else
  git clone -b "$MICRO_XRCE_BRANCH" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$MICRO_XRCE_DIR" || {
    echo "Failed to clone Micro-XRCE-DDS-Agent. Continuing."
  }
fi

# Build
pushd "$MICRO_XRCE_DIR" >/dev/null || true
mkdir -p build
cd build
set +e
cmake .. || true
make -j$(nproc) || RC_MAKE=$?
RC_MAKE=${RC_MAKE:-$?}
set -e
if [ -n "${RC_MAKE:-}" ] && [ "$RC_MAKE" -ne 0 ]; then
  echo "Make failed for Micro-XRCE-DDS-Agent. Attempting fastdds version fix in CMakeLists.txt as suggested..."
  popd >/dev/null || true
  # Try to patch CMakeLists.txt dependencies if present
  if [ -f "$MICRO_XRCE_DIR/CMakeLists.txt" ]; then
    echo "Patching CMakeLists.txt fastdds version variables (best-effort)."
    # The user suggested changing set(_fastdds_version 2.10) and tag; we'll make a conservative sed that makes no-op if not present
    sed -i.bak -E "s/set\(_fastdds_version[[:space:]]+[0-9]+\.[0-9]+(.[0-9]+)?\)/set(_fastdds_version 2.10)/g" "$MICRO_XRCE_DIR/CMakeLists.txt" || true
    sed -i.bak -E "s/set\(_fastdds_tag[[:space:]]+[^\)]+\)/set(_fastdds_tag 2.10.6)/g" "$MICRO_XRCE_DIR/CMakeLists.txt" || true
    pushd "$MICRO_XRCE_DIR/build" >/dev/null || true
    cmake .. || true
    make -j$(nproc) || true
    popd >/dev/null || true
  else
    echo "CMakeLists.txt not found — cannot auto-patch. Check repository manually."
  fi
fi

# After build attempt, try install
pushd "$MICRO_XRCE_DIR/build" >/dev/null || true
if [ -f Makefile ]; then
  set +e
  sudo make install || true
  set -e
  echo "Ran sudo make install (if built). Running ldconfig..."
  sudo ldconfig /usr/local/lib/ || true
else
  echo "Build did not produce Makefile; skipping sudo make install."
fi
popd >/dev/null || true

# --------------- Timesync tweak for uxrce_dds_client if requested ---------------
echo "=== Checking PX4 for uxrce_dds_client/module.yaml to set UXRCE_DDS_SYNCT default=0 if present ==="
UXRCE_MODULE_FILE="$PX4_DIR/src/modules/uxrce_dds_client/module.yaml"
if [ -f "$UXRCE_MODULE_FILE" ]; then
  echo "Found $UXRCE_MODULE_FILE — patching default value UXRCE_DDS_SYNCT -> 0 (best-effort)"
  # This will replace a line like '    UXRCE_DDS_SYNCT:' default: 1  -> default: 0
  # We'll do a safe sed replacement for 'UXRCE_DDS_SYNCT' occurrences
  cp "$UXRCE_MODULE_FILE" "${UXRCE_MODULE_FILE}.bak" || true
  sed -E -i "s/(UXRCE_DDS_SYNCT[[:space:]]*:[[:space:]]*).*([0-9]+)/\1 0/g" "$UXRCE_MODULE_FILE" || true
  # If there's a "default:" key style; try to handle that too
  sed -E -i "s/(name: UXRCE_DDS_SYNCT[[:space:]]*.*\n[[:space:]]*default:)[[:space:]]*[0-9]+/\1 0/g" "$UXRCE_MODULE_FILE" || true
  echo "Patched (backup in ${UXRCE_MODULE_FILE}.bak)"
else
  echo "module.yaml not found at $UXRCE_MODULE_FILE — skipping timesync tweak."
fi