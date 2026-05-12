#!/usr/bin/env bash
#
# teardown_ursim.sh — stop & remove URSim mock robot(s) and their IP aliases.
#
# Usage:
#   sudo bash scripts/teardown_ursim.sh --name <NAME>
#   sudo bash scripts/teardown_ursim.sh --ip <IP>
#   sudo bash scripts/teardown_ursim.sh --all
#
# Options:
#   --name <NAME>   Remove a single container by name (e.g. ursim_ur10e_16).
#   --ip <IP>       Remove the container whose 29999 port is bound to <IP>.
#                   The matching host alias is also removed.
#   --all           Remove every container whose name starts with 'ursim_',
#                   and every /32 alias on the LAN interface that was added
#                   by the setup script (i.e. any alias whose port bindings
#                   reference it). Use with care.
#   --iface <IFACE> LAN interface for alias removal. Default: eno1.
#   -h | --help     Show this help.
#
# Does NOT remove:
#   - Pulled docker images (use 'docker rmi ...' to free disk).
#   - The docker.io package or the docker daemon.
#   - Any users from the 'docker' group.
#   - The staged External Control URCap in ~/ursim_urcaps.

set -euo pipefail

MODE=""
TARGET_NAME=""
TARGET_IP=""
LAN_IFACE="eno1"

log()  { printf "\033[1;36m[teardown]\033[0m %s\n" "$*"; }
ok()   { printf "\033[1;32m[  ok   ]\033[0m %s\n" "$*"; }
warn() { printf "\033[1;33m[ warn  ]\033[0m %s\n" "$*"; }
err()  { printf "\033[1;31m[ fail  ]\033[0m %s\n" "$*" >&2; }

usage() { sed -n '3,28p' "$0" | sed 's/^# \{0,1\}//'; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --name)   MODE=name; TARGET_NAME="${2:-}"; shift 2 ;;
        --ip)     MODE=ip;   TARGET_IP="${2:-}"; shift 2 ;;
        --all)    MODE=all; shift ;;
        --iface)  LAN_IFACE="${2:-}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) err "unknown option: $1"; usage; exit 2 ;;
    esac
done

if [[ $EUID -ne 0 ]]; then err "run with sudo"; exit 1; fi
if [[ -z "$MODE" ]]; then err "specify --name, --ip, or --all"; usage; exit 2; fi

# Return the host IP that container $1's port 29999 is bound to, or empty.
get_bound_ip() {
    docker inspect "$1" \
        --format '{{ with (index .NetworkSettings.Ports "29999/tcp") }}{{ (index . 0).HostIp }}{{ end }}' \
        2>/dev/null || true
}

remove_one() {
    local cname="$1"
    local bound_ip
    bound_ip="$(get_bound_ip "$cname")"

    if docker inspect "$cname" >/dev/null 2>&1; then
        log "stopping container '$cname' …"
        docker stop "$cname" >/dev/null 2>&1 || true
        log "removing container '$cname' …"
        docker rm "$cname" >/dev/null 2>&1 || true
        ok "container '$cname' removed"
    else
        warn "container '$cname' not found"
    fi

    if [[ -n "$bound_ip" && "$bound_ip" != "0.0.0.0" ]]; then
        # Skip alias removal if the IP doesn't look like a private LAN address
        # we'd have added (avoid wiping primary interface IPs).
        if ip -4 -br addr show "$LAN_IFACE" 2>/dev/null | grep -qw "${bound_ip}/32"; then
            log "removing $bound_ip/32 alias from $LAN_IFACE …"
            ip addr del "${bound_ip}/32" dev "$LAN_IFACE" 2>/dev/null \
                && ok "alias $bound_ip removed" \
                || warn "alias removal failed (already gone?)"
        else
            log "no /32 alias for $bound_ip on $LAN_IFACE (skipping)"
        fi
    fi
}

case "$MODE" in
    name)
        [[ -z "$TARGET_NAME" ]] && { err "--name requires a value"; exit 2; }
        remove_one "$TARGET_NAME"
        ;;
    ip)
        [[ -z "$TARGET_IP" ]] && { err "--ip requires a value"; exit 2; }
        # Find any ursim_* container whose 29999 is bound to that IP.
        mapfile -t MATCHES < <(
            for c in $(docker ps -a --filter "name=^ursim_" --format '{{.Names}}'); do
                if [[ "$(get_bound_ip "$c")" == "$TARGET_IP" ]]; then
                    echo "$c"
                fi
            done
        )
        if (( ${#MATCHES[@]} == 0 )); then
            warn "no ursim_* container is bound to $TARGET_IP"
            # Try alias removal anyway in case the container is already gone.
            if ip -4 -br addr show "$LAN_IFACE" 2>/dev/null | grep -qw "${TARGET_IP}/32"; then
                log "removing dangling $TARGET_IP/32 alias from $LAN_IFACE …"
                ip addr del "${TARGET_IP}/32" dev "$LAN_IFACE" && ok "alias removed"
            fi
        else
            for c in "${MATCHES[@]}"; do remove_one "$c"; done
        fi
        ;;
    all)
        mapfile -t ALL < <(docker ps -a --filter "name=^ursim_" --format '{{.Names}}')
        if (( ${#ALL[@]} == 0 )); then
            ok "no ursim_* containers to remove"
        else
            for c in "${ALL[@]}"; do remove_one "$c"; done
        fi
        ;;
esac

ok "teardown complete"
