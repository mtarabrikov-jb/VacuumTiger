#!/bin/sh
# ===========================================================================
# Inject avatap.so (the serial tap) into `ava` and run the relay.
#
#   !!! THIS RESTARTS ava, THE ROBOT'S NAVIGATION BRAIN. !!!
#   Run it only with the robot idle on the dock. The tap itself is read-only
#   (it mirrors serial traffic, it never writes to the MCU/LDS), but injecting
#   it restarts ava. Recover with `inject.sh remove` over SSH.
#
# Coexists with the camstream camera tap: if /data/camstream is installed, we
# build ONE wrapper that LD_PRELOADs both libcamtap.so and avatap.so over the
# same real ava, and keep a single bind-mount active (no mount stacking). This
# is a dev/RE tool and is intentionally NOT made boot-persistent.
#
#   inject.sh install   compose wrapper + restart ava + start relay (+ camera)
#   inject.sh remove     restore the camstream-only (or stock) ava + stop relay
#   inject.sh status     show tap + relay state
# ===========================================================================
set -u
DIR="${REMOTE_DIR:-/data/w10bridge}"
CAMSTREAM="/data/camstream"
WRAP="$DIR/ava.wrap"
RELAY="$DIR/avatap-relay"
RELAY_LOG="/data/log/avatap-relay.log"

# Reuse the camstream snapshot of the real ava if present, else snapshot now.
if [ -f "$CAMSTREAM/ava_real/ava" ]; then
    REAL="$CAMSTREAM/ava_real/ava"
else
    REAL="$DIR/ava_real/ava"
fi

ava_pid() { pidof ava 2>/dev/null; }
tap_active() { p=$(ava_pid); [ -n "$p" ] && grep -q avatap "/proc/$p/maps" 2>/dev/null; }
bound() { grep -q ' /usr/bin/ava ' /proc/mounts; }
restart_ava() { echo ">> restarting ava"; /etc/rc.d/ava.sh >/dev/null 2>&1 & sleep 3; }

snapshot_real() {
    [ -f "$REAL" ] && return
    # Snapshot before any bind mount can hide the real binary.
    if ! bound; then
        mkdir -p "$(dirname "$REAL")"
        cp -a /usr/bin/ava "$REAL"
        chmod +x "$REAL"
    else
        echo "ERROR: /usr/bin/ava is already bind-mounted and no real snapshot exists."
        echo "       Install camstream first, or remove that mount, then retry."
        exit 1
    fi
}

make_wrapper() {
    PRELOAD=""
    [ -f "$CAMSTREAM/libcamtap.so" ] && PRELOAD="$CAMSTREAM/libcamtap.so"
    if [ -n "$PRELOAD" ]; then PRELOAD="$PRELOAD:$DIR/avatap.so"; else PRELOAD="$DIR/avatap.so"; fi
    {
        echo "#!/bin/sh"
        echo "export LD_PRELOAD=$PRELOAD"
        echo "[ -f $CAMSTREAM/camtap.env ] && . $CAMSTREAM/camtap.env"
        echo "exec $REAL \"\$@\""
    } > "$WRAP"
    chmod +x "$WRAP"
}

start_relay() {
    killall avatap-relay 2>/dev/null
    [ -x "$RELAY" ] || { echo "ERROR: $RELAY missing (upload first)"; return 1; }
    mkdir -p /data/log
    setsid "$RELAY" 0.0.0.0 >"$RELAY_LOG" 2>&1 < /dev/null &
    sleep 1
    echo ">> relay started (ports 7701 mcu-rx / 7702 lds-rx / 7703 mcu-tx / 7704 lds-tx)"
}
start_camera() { [ -x "$CAMSTREAM/run_ir.sh" ] && REMOTE_DIR="$CAMSTREAM" sh "$CAMSTREAM/run_ir.sh" >/dev/null 2>&1; }

case "${1:-status}" in
    install)
        [ -f "$DIR/avatap.so" ] || { echo "ERROR: $DIR/avatap.so missing (upload first)"; exit 1; }
        snapshot_real
        make_wrapper
        # single active bind: drop whatever wrapper is mounted, mount ours
        bound && umount /usr/bin/ava
        mount --bind "$WRAP" /usr/bin/ava
        restart_ava
        start_relay
        start_camera
        if tap_active; then echo ">> OK: avatap active in ava; relay serving. Decode with: make decode"; else echo ">> WARN: avatap not seen in ava maps yet; check 'ps | grep ava' + $RELAY_LOG"; fi
        ;;
    remove)
        killall avatap-relay 2>/dev/null
        bound && umount /usr/bin/ava
        # restore the camstream camera wrapper if present, else stock ava
        if [ -x "$CAMSTREAM/inject-ava.sh" ]; then
            REMOTE_DIR="$CAMSTREAM" sh "$CAMSTREAM/inject-ava.sh" boot >/dev/null 2>&1
        fi
        restart_ava
        tap_active && echo ">> WARN: avatap still present" || echo ">> removed; avatap not in ava"
        ;;
    status)
        echo -n "bind mount : "; bound && echo yes || echo no
        echo -n "ava pid    : "; ava_pid || echo none
        echo -n "avatap in ava: "; tap_active && echo YES || echo no
        echo -n "relay proc : "; pidof avatap-relay >/dev/null 2>&1 && echo running || echo stopped
        echo -n "shm file   : "; [ -e /tmp/avatap.shm ] && echo present || echo none
        ;;
    *) echo "usage: inject.sh install|remove|status"; exit 1 ;;
esac
