#!/usr/bin/env bash
#
# setup_ursim.sh — start one (or more) URSim mock UR robot(s) on this host.
#
# Each invocation brings up ONE simulator. Run the script again with a
# different --ip / --model to launch additional simulators side by side.
# Ports are bound to the specific LAN alias IP, so any number of
# simulators can listen on the standard ports (29999, 30001-30004,
# 5900, 6080) at once without colliding.
#
# Usage:
#   sudo bash scripts/setup_ursim.sh --ip <IP> --model <MODEL> [options]
#
# Options:
#   --ip <IP>         LAN-facing IP for this simulator (required).
#   --model <MODEL>   UR model. Valid values:
#                       CB3 series:  ur3, ur5, ur10
#                       e-Series:    ur3e, ur5e, ur7e, ur10e, ur12e, ur16e
#                       Large e:     ur8long, ur15, ur18, ur20, ur30
#                     (required)
#   --name <NAME>     Container name. Default: ursim_<model>_<last_octet>.
#   --iface <IFACE>   LAN interface for the alias. Default: eno1.
#   --no-alias        Do NOT add a host IP alias; assume <IP> already
#                     resolves to this host (e.g. for 127.0.0.1).
#   --version <VER>   URSim image tag. Default: latest.
#   --urcap-dir <DIR> URCap mount directory. Default: $HOME/ursim_urcaps.
#   -h | --help       Show this help.
#
# Examples:
#   # One UR10e at 192.168.1.16
#   sudo bash scripts/setup_ursim.sh --ip 192.168.1.16 --model ur10e
#
#   # Two simulators side by side
#   sudo bash scripts/setup_ursim.sh --ip 192.168.1.16 --model ur10e
#   sudo bash scripts/setup_ursim.sh --ip 192.168.1.17 --model ur15
#
#   # Loopback (no alias needed)
#   sudo bash scripts/setup_ursim.sh --ip 127.0.0.1 --model ur5e --no-alias
#
# Teardown:
#   sudo bash scripts/teardown_ursim.sh --ip <IP>
#   sudo bash scripts/teardown_ursim.sh --name <NAME>
#   sudo bash scripts/teardown_ursim.sh --all

set -euo pipefail

# ----------------------------------------------------------------------
# Defaults
# ----------------------------------------------------------------------
TARGET_IP=""
URSIM_MODEL=""
CONTAINER_NAME=""
LAN_IFACE="eno1"
SKIP_ALIAS=false
URSIM_TAG="latest"
URCAP_DIR_OVERRIDE=""
START_URSIM_HELPER="/opt/ros/humble/lib/ur_client_library/start_ursim.sh"

# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------
log()  { printf "\033[1;36m[setup]\033[0m %s\n" "$*"; }
ok()   { printf "\033[1;32m[ ok ]\033[0m %s\n" "$*"; }
warn() { printf "\033[1;33m[warn]\033[0m %s\n" "$*"; }
err()  { printf "\033[1;31m[fail]\033[0m %s\n" "$*" >&2; }

usage() {
    sed -n '3,46p' "$0" | sed 's/^# \{0,1\}//'
}

# ----------------------------------------------------------------------
# Argument parsing
# ----------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ip)         TARGET_IP="${2:-}"; shift 2 ;;
        --model)      URSIM_MODEL="${2:-}"; shift 2 ;;
        --name)       CONTAINER_NAME="${2:-}"; shift 2 ;;
        --iface)      LAN_IFACE="${2:-}"; shift 2 ;;
        --no-alias)   SKIP_ALIAS=true; shift ;;
        --version)    URSIM_TAG="${2:-}"; shift 2 ;;
        --urcap-dir)  URCAP_DIR_OVERRIDE="${2:-}"; shift 2 ;;
        -h|--help)    usage; exit 0 ;;
        *)            err "unknown option: $1"; usage; exit 2 ;;
    esac
done

# ----------------------------------------------------------------------
# Validation
# ----------------------------------------------------------------------
if [[ $EUID -ne 0 ]]; then
    err "must be run with sudo"
    exit 1
fi
if [[ -z "$TARGET_IP" || -z "$URSIM_MODEL" ]]; then
    err "--ip and --model are required"
    usage
    exit 2
fi
if ! [[ "$TARGET_IP" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    err "invalid IPv4 address: $TARGET_IP"
    exit 2
fi
case "$URSIM_MODEL" in
    ur3|ur5|ur10|ur3e|ur5e|ur7e|ur10e|ur12e|ur16e|ur8long|ur15|ur18|ur20|ur30) ;;
    *) err "invalid --model: $URSIM_MODEL"
       err "valid: ur3 ur5 ur10 ur3e ur5e ur7e ur10e ur12e ur16e ur8long ur15 ur18 ur20 ur30"
       exit 2 ;;
esac
if [[ ! -x "$START_URSIM_HELPER" ]]; then
    err "URSim helper not found at $START_URSIM_HELPER"
    err "install ros-humble-ur-robot-driver / ros-humble-ur-client-library first"
    exit 1
fi

LAST_OCTET="${TARGET_IP##*.}"
if (( LAST_OCTET < 2 || LAST_OCTET > 254 )); then
    err "LAN IP last octet must be 2-254 (got $LAST_OCTET); cannot derive a unique internal docker IP"
    err "pick a different --ip or use --no-alias with a manual interface"
    exit 2
fi

# Default container name: ursim_<model>_<last_octet>  e.g. ursim_ur10e_16
if [[ -z "$CONTAINER_NAME" ]]; then
    CONTAINER_NAME="ursim_${URSIM_MODEL}_${LAST_OCTET}"
fi

# Internal docker network IP: 192.168.56.<last_octet>
# (start_ursim.sh creates ursim_net as 192.168.56.0/24, defaulting to .101 —
# we override so multiple containers don't collide.)
INTERNAL_IP="192.168.56.${LAST_OCTET}"

# URCap directory (per-user)
if [[ -n "$URCAP_DIR_OVERRIDE" ]]; then
    URCAP_DIR="$URCAP_DIR_OVERRIDE"
elif [[ -n "${SUDO_USER:-}" ]]; then
    URCAP_DIR="/home/${SUDO_USER}/ursim_urcaps"
else
    URCAP_DIR="${HOME}/ursim_urcaps"
fi

log "LAN IP            : $TARGET_IP"
log "Model             : $URSIM_MODEL"
log "Container name    : $CONTAINER_NAME"
log "Docker internal IP: $INTERNAL_IP"
log "LAN interface     : $LAN_IFACE"
log "URSim image tag   : $URSIM_TAG"
log "URCap mount dir   : $URCAP_DIR"

# ----------------------------------------------------------------------
# 1. Install docker if missing (system-wide, one-time)
# ----------------------------------------------------------------------
if command -v docker >/dev/null 2>&1; then
    ok "docker already installed: $(docker --version)"
else
    log "installing docker.io via apt-get …"
    apt-get update -y
    DEBIAN_FRONTEND=noninteractive apt-get install -y docker.io
    ok "docker installed: $(docker --version)"
fi

# ----------------------------------------------------------------------
# 2. Start docker daemon
# ----------------------------------------------------------------------
if systemctl is-active --quiet docker; then
    ok "docker daemon already running"
else
    log "starting docker daemon …"
    systemctl enable --now docker
    sleep 2
    systemctl is-active --quiet docker || { err "failed to start docker"; exit 1; }
    ok "docker daemon started"
fi

# ----------------------------------------------------------------------
# 3. Add real user to docker group (informational; affects future shells)
# ----------------------------------------------------------------------
if [[ -n "${SUDO_USER:-}" ]]; then
    if ! id -nG "$SUDO_USER" | tr ' ' '\n' | grep -qx docker; then
        usermod -aG docker "$SUDO_USER"
        warn "added '$SUDO_USER' to 'docker' group — log out & back in to pick it up"
    fi
fi

# ----------------------------------------------------------------------
# 4. Add IP alias (unless --no-alias)
# ----------------------------------------------------------------------
if $SKIP_ALIAS; then
    log "--no-alias given; skipping host IP alias step"
elif ip -4 -br addr show "$LAN_IFACE" 2>/dev/null | grep -qw "$TARGET_IP"; then
    ok "alias $TARGET_IP already present on $LAN_IFACE"
else
    if ! ip -4 -br addr show "$LAN_IFACE" >/dev/null 2>&1; then
        err "interface $LAN_IFACE not found; pass --iface <iface>"
        exit 1
    fi
    # Don't claim an IP that a real device on the LAN already owns.
    if ip -4 -br addr show | awk '{for(i=3;i<=NF;i++) print $i}' \
        | grep -qE "^${TARGET_IP}/" ; then
        log "$TARGET_IP is already on a local interface (skipping LAN reachability check)"
    elif ping -c 1 -W 1 -n -q "$TARGET_IP" >/dev/null 2>&1; then
        err "$TARGET_IP responds to ping — a real device is using it on the LAN"
        err "either unplug/reconfigure that device or pick a different --ip"
        exit 1
    fi
    log "adding $TARGET_IP/32 alias on $LAN_IFACE …"
    ip addr add "$TARGET_IP/32" dev "$LAN_IFACE"
    ok "alias added (non-persistent; survives until reboot)"
fi

# ----------------------------------------------------------------------
# 5. Stage External Control URCap (optional; for ur_robot_driver use)
# ----------------------------------------------------------------------
mkdir -p "$URCAP_DIR"
URCAP_SRC="/opt/ros/humble/share/ur_robot_driver/resources"
if compgen -G "$URCAP_DIR/external_control-*.urcap" >/dev/null; then
    ok "External Control URCap already in $URCAP_DIR"
elif compgen -G "$URCAP_SRC/external_control-*.urcap" >/dev/null; then
    cp "$URCAP_SRC"/external_control-*.urcap "$URCAP_DIR"/
    ok "staged External Control URCap → $URCAP_DIR"
else
    warn "no External Control URCap in $URCAP_SRC — URSim ships its own, ros2 control should still work"
fi
if [[ -n "${SUDO_USER:-}" ]]; then
    chown -R "$SUDO_USER":"$SUDO_USER" "$URCAP_DIR" 2>/dev/null || true
fi

# ----------------------------------------------------------------------
# 6. Pull URSim image (one-time per series/tag)
# ----------------------------------------------------------------------
# Series → image mapping (from start_ursim.sh internals):
#   cb3:        ur3, ur5, ur10                                → ursim_cb3
#   e-series:   ur3e, ur5e, ur7e, ur10e, ur12e, ur16e,
#               ur8long, ur15, ur18, ur20, ur30               → ursim_e-series
case "$URSIM_MODEL" in
    ur3|ur5|ur10)                                   IMAGE_REPO="universalrobots/ursim_cb3" ;;
    *)                                              IMAGE_REPO="universalrobots/ursim_e-series" ;;
esac
IMAGE_REF="${IMAGE_REPO}:${URSIM_TAG}"

if docker image inspect "$IMAGE_REF" >/dev/null 2>&1; then
    ok "URSim image already pulled: $IMAGE_REF"
else
    log "pulling $IMAGE_REF (this can take several minutes the first time) …"
    docker pull "$IMAGE_REF"
    ok "pulled $IMAGE_REF"
fi

# ----------------------------------------------------------------------
# 7. Start the URSim container (or reuse / restart existing)
# ----------------------------------------------------------------------
EXISTING_STATUS="$(docker ps -a -f name="^${CONTAINER_NAME}$" --format '{{.Status}}' || true)"
if [[ -n "$EXISTING_STATUS" ]]; then
    if [[ "$EXISTING_STATUS" == Up* ]]; then
        ok "container '$CONTAINER_NAME' is already running ($EXISTING_STATUS)"
    else
        log "container '$CONTAINER_NAME' exists but not running ($EXISTING_STATUS); starting …"
        docker start "$CONTAINER_NAME" >/dev/null
        ok "container started"
    fi
else
    # Bind every published port to the specific LAN IP, so multiple simulators
    # on different IPs can all listen on standard ports.
    PORT_FORWARDING="-p ${TARGET_IP}:29999:29999 \
        -p ${TARGET_IP}:30001-30004:30001-30004 \
        -p ${TARGET_IP}:5900:5900 \
        -p ${TARGET_IP}:6080:6080"

    log "starting URSim ($URSIM_MODEL) via official helper …"
    "$START_URSIM_HELPER" \
        -m "$URSIM_MODEL" \
        -v "$URSIM_TAG" \
        -i "$INTERNAL_IP" \
        -d \
        -n "$CONTAINER_NAME" \
        -u "$URCAP_DIR" \
        -f "$PORT_FORWARDING"
    ok "URSim container launched (detached)"
fi

# ----------------------------------------------------------------------
# 8. Wait for dashboard readiness
# ----------------------------------------------------------------------
# URSim cold-boot takes ~60–120 s before Polyscope is alive enough to
# answer dashboard queries. Poll until the server returns a real
# 'Robotmode:' reply, not just a TCP banner.
MAX_WAIT_SECS=180
log "waiting for dashboard server on ${TARGET_IP}:29999 (up to ${MAX_WAIT_SECS}s) …"

TCP_OK=false
for i in $(seq 1 "$MAX_WAIT_SECS"); do
    if timeout 1 bash -c "echo > /dev/tcp/${TARGET_IP}/29999" 2>/dev/null; then
        TCP_OK=true; break
    fi
    sleep 1
    if (( i % 15 == 0 )); then log "  TCP wait … ${i}s"; fi
done
if ! $TCP_OK; then
    err "TCP 29999 did not open within ${MAX_WAIT_SECS}s"
    err "check container logs: docker logs $CONTAINER_NAME --tail 50"
    exit 1
fi
ok "TCP port 29999 open on ${TARGET_IP}"

QUERY_DASHBOARD='
import socket, sys, time
try:
    s = socket.socket(); s.settimeout(3)
    s.connect(("'"$TARGET_IP"'", 29999))
    s.recv(4096)
    s.sendall(b"robotmode\n"); time.sleep(0.3)
    reply = s.recv(4096).decode(errors="replace").strip()
    s.sendall(b"polyscopeVersion\n"); time.sleep(0.3)
    ver = s.recv(4096).decode(errors="replace").strip()
    s.sendall(b"get robot model\n"); time.sleep(0.3)
    model = s.recv(4096).decode(errors="replace").strip()
    s.close()
    print(reply); print(ver); print(model)
    sys.exit(0 if reply.startswith("Robotmode:") else 1)
except Exception as e:
    print("err:", e); sys.exit(2)
'

DASH_OK=false
DASH_REPLY=""
for i in $(seq 1 "$MAX_WAIT_SECS"); do
    DASH_REPLY="$(python3 -c "$QUERY_DASHBOARD" 2>&1)" && DASH_OK=true && break
    sleep 1
    if (( i % 15 == 0 )); then log "  dashboard wait … ${i}s"; fi
done

echo "---- dashboard reply ----"
echo "${DASH_REPLY:-<empty>}"
echo "-------------------------"
if $DASH_OK; then
    ok "dashboard verified on ${TARGET_IP}:29999"
else
    err "dashboard never returned a 'Robotmode:' reply within ${MAX_WAIT_SECS}s"
    err "check container logs: docker logs $CONTAINER_NAME --tail 50"
    exit 1
fi

# ----------------------------------------------------------------------
# Summary
# ----------------------------------------------------------------------
cat <<EOF

╔══════════════════════════════════════════════════════════════════════╗
║              URSim ${URSIM_MODEL} is up at ${TARGET_IP}
╠══════════════════════════════════════════════════════════════════════╣
║ Container name          : ${CONTAINER_NAME}
║ Model                   : ${URSIM_MODEL}
║ LAN IP (host alias)     : ${TARGET_IP}
║ Docker internal IP      : ${INTERNAL_IP}
║ Polyscope GUI (browser) : http://${TARGET_IP}:6080/vnc.html
║ Dashboard server (TCP)  : ${TARGET_IP}:29999    (verified above)
║ Primary URScript (TCP)  : ${TARGET_IP}:30002
║ Secondary URScript (TCP): ${TARGET_IP}:30001
║ Realtime data (TCP)     : ${TARGET_IP}:30003
║ RTDE (TCP)              : ${TARGET_IP}:30004
║ Raw VNC                 : ${TARGET_IP}:5900
╠══════════════════════════════════════════════════════════════════════╣
║ Next steps                                                           ║
║   1. Open the Polyscope GUI in a browser.                            ║
║   2. Power on the robot, release brakes (or use the dashboard cmd).  ║
║   3. Test from Python:                                               ║
║        from ur15_robot_arm.ur15 import UR15Robot                     ║
║        r = UR15Robot("${TARGET_IP}", 30002); r.open()                       
║        r.popup("hi"); r.close()                                      ║
║                                                                      ║
║ Teardown                                                             ║
║   sudo bash scripts/teardown_ursim.sh --name ${CONTAINER_NAME}              
╚══════════════════════════════════════════════════════════════════════╝
EOF
