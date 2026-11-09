#!/usr/bin/env bash
# Prepare a robot terminal without opening the servo bus or commanding motors.
# Execute to open a dedicated shell, or source to prepare the current shell.

_robot_connect_help() {
    cat <<'EOF'
Usage: ./connect.sh [options] [-- command args...]
       source ./connect.sh [options]

Find the adapter, fix port access if needed, prepare the servo venv, source ROS
and the workspace, and provide servo/chain/arm/calibrate commands. No motor
requests are sent by setup itself. Run as your normal user; sudo is used only
for missing setup packages or port access.

  --port PATH         Select a particular adapter instead of auto-detection
  --wait SECONDS      Wait for the adapter (default: 20; 0 for immediate check)
  --ros-distro NAME   ROS distribution (default: ROS_DISTRO or jazzy)
  --domain ID         ROS domain (default: 42, matching physical startup)
  --build             Rebuild the workspace; otherwise build only if unbuilt
  --no-shell          Prepare/check setup and exit; useful for diagnostics
  --no-port           Prepare ROS/tools without requiring USB (e.g. mock runs)
  -- command args...  Run a command in the prepared environment instead of a shell
  -h, --help          Show this help

Examples after setup:
  servo --id 1 inspect
  chain inspect --ids 1 2 3 4 5 6
  arm                         # Interactive physical zero/hold, then RViz/sliders
  calibrate --status

The servo venv is used by the helper commands, not activated globally, so ROS
continues to use its system Python. Exit the dedicated shell to return to your
original terminal. Re-run setup after USB reconnection if permissions changed.
EOF
}

_robot_connect_sudo() {
    if (( EUID == 0 )); then
        command "$@"
    else
        command sudo -- "$@"
    fi
}

_robot_connect_port() {
    local preferred="$1" explicit="$2" wait_seconds="$3" candidate
    local deadline=$((SECONDS + wait_seconds)) announced=0
    local -a candidates=()
    while :; do
        if [[ -n "$preferred" && -c "$preferred" ]]; then
            printf '%s\n' "$preferred"; return 0
        fi
        if [[ "$explicit" == 0 ]]; then
            candidates=()
            while IFS= read -r candidate; do
                [[ -c "$candidate" ]] && candidates+=("$candidate")
            done < <(compgen -G '/dev/serial/by-id/usb-1a86_USB_Single_Serial_*')
            if (( ${#candidates[@]} == 1 )); then
                printf '%s\n' "${candidates[0]}"; return 0
            elif (( ${#candidates[@]} > 1 )); then
                printf 'Multiple matching adapters; choose one with --port PATH:\n' >&2
                printf '  %s\n' "${candidates[@]}" >&2
                return 1
            fi
        fi
        if (( SECONDS >= deadline )); then
            printf 'Adapter not found. Connect USB and retry, or use --port PATH. Expected: %s\n' "$preferred" >&2
            return 1
        fi
        if (( !announced )); then
            printf 'Waiting up to %s seconds for the USB adapter...\n' "$wait_seconds" >&2
            announced=1
        fi
        sleep 1
    done
}

_robot_connect_dependencies_ready() {
    "$ROBOT_SOFTWARE_ROOT/.venv/bin/python" - "$ROBOT_SOFTWARE_ROOT/tools/requirements-servo.txt" <<'PY'
import importlib.metadata as metadata
from pathlib import Path
import sys
try:
    for line in Path(sys.argv[1]).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        name, separator, expected = line.partition('==')
        if not separator or metadata.version(name) != expected:
            raise RuntimeError('Requirements need installation')
    import scservo_sdk
    import serial
except Exception:
    raise SystemExit(1)
PY
}

_robot_connect_main() {
    local selected_port='' explicit=0 wait_seconds=20 build=0 no_shell=0 skip_port=0
    local distro="${ROS_DISTRO:-jazzy}" domain=42 calibration_port='' actual_port=''
    local path_entry clean_path='' old_venv="${VIRTUAL_ENV:-}"
    local -a run=() path_entries=()
    while (( $# )); do
        case "$1" in
            --port|--wait|--ros-distro|--domain)
                if (( $# < 2 )); then printf 'Missing value for %s\n' "$1" >&2; return 2; fi
                case "$1" in
                    --port) selected_port="$2"; explicit=1 ;;
                    --wait) wait_seconds="$2" ;;
                    --ros-distro) distro="$2" ;;
                    --domain) domain="$2" ;;
                esac
                shift 2 ;;
            --build) build=1; shift ;;
            --no-shell) no_shell=1; shift ;;
            --no-port) skip_port=1; shift ;;
            --) shift; run=("$@"); break ;;
            -h|--help) _robot_connect_help; return 0 ;;
            *) printf 'Unknown argument: %s\n' "$1" >&2; return 2 ;;
        esac
    done
    if [[ ! "$wait_seconds" =~ ^[0-9]{1,3}$ || ! "$domain" =~ ^[0-9]{1,3}$ ||
          ! "$distro" =~ ^[a-z][a-z0-9_]*$ ]]; then
        printf 'Require integer --wait/--domain and a valid ROS distribution name.\n' >&2; return 2
    fi
    wait_seconds=$((10#$wait_seconds)); domain=$((10#$domain))
    if (( wait_seconds > 300 || domain > 232 )); then
        printf 'Require --wait 0..300 and --domain 0..232.\n' >&2; return 2
    fi
    export ROBOT_SOFTWARE_ROOT
    ROBOT_SOFTWARE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)" || return
    if [[ ! -f "/opt/ros/$distro/setup.bash" ]]; then
        printf 'ROS setup missing: /opt/ros/%s/setup.bash. Install ROS or choose --ros-distro.\n' "$distro" >&2
        return 1
    fi
    if [[ -f "$ROBOT_SOFTWARE_ROOT/calibration/arm.json" ]]; then
        calibration_port="$(/usr/bin/python3 - "$ROBOT_SOFTWARE_ROOT/calibration/arm.json" <<'PY'
import json, sys
print(json.load(open(sys.argv[1]))['port'])
PY
)" || return
    fi
    if [[ -z "$selected_port" ]]; then
        selected_port="${PORT:-$calibration_port}"
    fi
    if (( !skip_port )); then
        selected_port="$(_robot_connect_port "$selected_port" "$explicit" "$wait_seconds")" || return
        actual_port="$(readlink -f -- "$selected_port")" || return
        if [[ ! -r "$actual_port" || ! -w "$actual_port" ]]; then
            printf 'Port access needs updating for %s; sudo may request your password.\n' "$actual_port"
            if ! command -v setfacl >/dev/null; then
                _robot_connect_sudo apt-get update || return
                _robot_connect_sudo apt-get install -y acl || return
            fi
            _robot_connect_sudo setfacl -m "u:$(id -u):rw" "$actual_port" || return
            if [[ ! -r "$actual_port" || ! -w "$actual_port" ]]; then
                printf 'Port is still inaccessible: %s\n' "$actual_port" >&2; return 1
            fi
        fi
        # Keep the spelling expected by calibration when both paths identify
        # the same device (e.g. /dev/ttyACM0 versus its persistent by-id link).
        if [[ -n "$calibration_port" && -c "$calibration_port" &&
              "$actual_port" == "$(readlink -f -- "$calibration_port")" ]]; then
            selected_port="$calibration_port"
        fi
    fi
    export PORT="$selected_port" ROS_DOMAIN_ID="$domain"

    if [[ ! -x "$ROBOT_SOFTWARE_ROOT/.venv/bin/python" ]]; then
        printf 'Creating servo virtual environment...\n'
        /usr/bin/python3 -m venv --without-pip "$ROBOT_SOFTWARE_ROOT/.venv" || return
    fi
    if ! _robot_connect_dependencies_ready; then
        if ! /usr/bin/python3 -m pip --version >/dev/null 2>&1; then
            _robot_connect_sudo apt-get update || return
            _robot_connect_sudo apt-get install -y python3-pip || return
        fi
        printf 'Installing pinned servo dependencies...\n'
        /usr/bin/python3 -m pip --python "$ROBOT_SOFTWARE_ROOT/.venv/bin/python" install \
            --disable-pip-version-check -r "$ROBOT_SOFTWARE_ROOT/tools/requirements-servo.txt" || return
        _robot_connect_dependencies_ready || return
    fi

    # Keep ROS executables and env-python scripts on system Python. Wrappers
    # below select the servo venv explicitly instead of changing PATH globally.
    if [[ -n "$old_venv" ]]; then
        if declare -F deactivate >/dev/null; then
            deactivate
        else
            IFS=: read -r -a path_entries <<< "$PATH"
            for path_entry in "${path_entries[@]}"; do
                if [[ "$path_entry" != "$old_venv/bin" ]]; then
                    clean_path+="${clean_path:+:}$path_entry"
                fi
            done
            export PATH="$clean_path"
            unset VIRTUAL_ENV
        fi
    fi
    source "/opt/ros/$distro/setup.bash" || return
    cd -- "$ROBOT_SOFTWARE_ROOT" || return
    if (( build )) || [[ ! -f "$ROBOT_SOFTWARE_ROOT/install/setup.bash" ]]; then
        if ! command -v colcon >/dev/null; then
            _robot_connect_sudo apt-get update || return
            _robot_connect_sudo apt-get install -y python3-colcon-common-extensions || return
        fi
        printf 'Building ROS workspace...\n'
        colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON || return
    fi
    source "$ROBOT_SOFTWARE_ROOT/install/setup.bash" || return

    unalias servo chain arm calibrate 2>/dev/null || true
    function servo { "$ROBOT_SOFTWARE_ROOT/.venv/bin/python" "$ROBOT_SOFTWARE_ROOT/tools/servo_setup.py" --hardware --port "$PORT" "$@"; }
    function chain { "$ROBOT_SOFTWARE_ROOT/.venv/bin/python" "$ROBOT_SOFTWARE_ROOT/tools/servo_chain.py" --hardware --port "$PORT" "$@"; }
    function arm { "$ROBOT_SOFTWARE_ROOT/.venv/bin/python" "$ROBOT_SOFTWARE_ROOT/tools/start_arm.py" --hardware --domain "$ROS_DOMAIN_ID" "$@"; }
    function calibrate { "$ROBOT_SOFTWARE_ROOT/.venv/bin/python" "$ROBOT_SOFTWARE_ROOT/tools/calibrate.py" --hardware --port "$PORT" "$@"; }
    export -f servo chain arm calibrate
    printf '\nRobot terminal ready: %s\nPort: %s\nROS: %s | domain %s\n' "$ROBOT_SOFTWARE_ROOT" "$PORT" "$ROS_DISTRO" "$ROS_DOMAIN_ID"
    printf 'Venv dependencies ready; ROS/workspace sourced. No motor requests sent.\n'
    if (( skip_port )); then printf 'USB discovery/access check skipped (--no-port).\n'; fi
    printf 'Commands: servo | chain | calibrate | arm\n'
    if (( !skip_port )) && [[ -n "$calibration_port" && "$PORT" != "$calibration_port" &&
          "$actual_port" != "$(readlink -f -- "$calibration_port")" ]]; then
        printf 'Adapter differs from calibration port %s. Bench commands use PORT; review the calibration adapter before arm startup.\n' "$calibration_port"
    fi
    if (( ${#run[@]} )); then
        "${run[@]}"; return $?
    fi
    if (( no_shell )) || [[ "${BASH_SOURCE[-1]}" != "$0" ]]; then return 0; fi
    if [[ ! -t 0 ]]; then
        printf 'No interactive terminal; use -- command, --no-shell, or source ./connect.sh.\n' >&2
        return 1
    fi
    printf 'Opening a prepared robot shell. Type exit to return to your original shell.\n'
    export PS1='(robot) \u@\h:\w\$ '
    exec /bin/bash --noprofile --norc -i
}

_robot_connect_main "$@"
_robot_connect_status=$?
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    return "$_robot_connect_status"
else
    exit "$_robot_connect_status"
fi
