#!/usr/bin/env bash
# SimFlake setup for Ubuntu 24.04 + ROS 2 Jazzy.
# Safe to re-run: every step checks first and skips what is already done.
#
#   ./setup.sh            interactive
#   ./setup.sh -y         say yes to everything (installs Unity Hub + the editor without asking)
set -euo pipefail

YES=0
[[ "${1:-}" == "-y" ]] && YES=1
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$ROOT/rover_sim"
VERSION_FILE="$PROJECT/ProjectSettings/ProjectVersion.txt"
UNITY_VERSION="$(awk '/m_EditorVersion:/ {print $2}' "$VERSION_FILE")"
UNITY_CHANGESET="$(sed -n 's/.*m_EditorVersionWithRevision: .* (\(.*\))/\1/p' "$VERSION_FILE")"

bold() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
ok()   { printf '  \033[32m✓\033[0m %s\n' "$*"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$*"; }
ask()  { [[ $YES == 1 ]] && return 0; read -r -p "  $1 [y/N] " r; [[ "$r" =~ ^[Yy] ]]; }

bold "System"
. /etc/os-release
if [[ "$VERSION_ID" == "24.04" ]]; then ok "Ubuntu $VERSION_ID"; else warn "Ubuntu $VERSION_ID detected; SimFlake targets 24.04 (Jazzy)."; fi

bold "ROS 2 Jazzy"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  ok "/opt/ros/jazzy found"
else
  warn "ROS 2 Jazzy is not installed. Install it first (RoverFlake2 setup, or https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then re-run this script."
  exit 1
fi

bold "rosbridge (the only ROS package the sim needs)"
if dpkg -s ros-jazzy-rosbridge-suite >/dev/null 2>&1; then
  ok "ros-jazzy-rosbridge-suite installed"
else
  echo "  Installing ros-jazzy-rosbridge-suite (needs sudo)..."
  sudo apt-get update -qq && sudo apt-get install -y ros-jazzy-rosbridge-suite
  ok "rosbridge installed"
fi

bold "URDF importer support"
# Ubuntu 24.04 no longer ships the libdl.so symlink. The URDF importer's Assimp plugin (used for COLLADA .dae meshes;
# .stl works without it) dlopen()s exactly "libdl.so".
LIBDL=/usr/lib/x86_64-linux-gnu/libdl.so
if [[ -e "$LIBDL" ]]; then
  ok "$LIBDL present"
elif ask "Create $LIBDL -> libdl.so.2 (needed to import URDFs with .dae meshes; needs sudo)?"; then
  sudo ln -s libdl.so.2 "$LIBDL" && ok "linked"
else
  warn "Skipped. Only matters for URDFs with .dae meshes."
fi

bold "Unity Hub"
if command -v unityhub >/dev/null 2>&1; then
  ok "unityhub found"
elif ask "Unity Hub is not installed. Install it from Unity's apt repository?"; then
  wget -qO - https://hub.unity3d.com/linux/keys/public | gpg --dearmor | sudo tee /usr/share/keyrings/Unity_Technologies_ApS.gpg >/dev/null
  echo "deb [signed-by=/usr/share/keyrings/Unity_Technologies_ApS.gpg] https://hub.unity3d.com/linux/repos/deb stable main" \
    | sudo tee /etc/apt/sources.list.d/unityhub.list >/dev/null
  sudo apt-get update -qq && sudo apt-get install -y unityhub
  ok "Unity Hub installed. Open it once and sign in (a free Personal license is fine)."
else
  warn "Skipped. Install Unity Hub: https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux"
fi

bold "Unity editor $UNITY_VERSION"
EDITOR="$HOME/Unity/Hub/Editor/$UNITY_VERSION/Editor/Unity"
if [[ -x "$EDITOR" ]]; then
  ok "installed at $EDITOR"
elif command -v unityhub >/dev/null 2>&1 && ask "Download Unity $UNITY_VERSION now through Unity Hub (several GB)?"; then
  unityhub -- --headless install --version "$UNITY_VERSION" --changeset "$UNITY_CHANGESET" || \
    warn "Hub CLI install failed. Open this link instead: unityhub://$UNITY_VERSION/$UNITY_CHANGESET"
else
  warn "Install it from Unity Hub > Installs, or open: unityhub://$UNITY_VERSION/$UNITY_CHANGESET"
fi

bold "Done"
cat <<EOF
  Next:
    1. Unity Hub > Projects > Add > Add project from disk > $PROJECT
    2. Open it, open a scene from Assets/Scenes (try TaskWorlds/WaypointTask), press Play.
       The sim starts rosbridge for you. Top-right of the Game view should say "ROS: connected".
    3. In a terminal:  python3 $ROOT/tools/sim_check.py --drive
       or drive it:    ros2 run teleop_twist_keyboard teleop_twist_keyboard
EOF
