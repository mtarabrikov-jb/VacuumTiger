#!/bin/sh
# ===========================================================================
# Enter/exit SangamIO DIRECT mode on the Dreame W10: replace `ava` at the MCU
# with the SangamIO daemon, which drives /dev/ttyS4 (MCU) + /dev/ttyS3 (LDS)
# itself. Run this ON the robot (upload sangamio + dreame_w10_direct.toml to
# $REMOTE_DIR first).
#
#   !!! This KILLS ava. The camera stream, Valetudo and ava's own cliff/bump
#       safety and docking go down while SangamIO drives the MCU directly.
#       SangamIO runs its own watchdog (500ms) + speed clamp (150mm/s, 1.5rad/s)
#       + cliff/bump gate. Reversible: `w10-direct.sh restore` brings ava back.
#
#   w10-direct.sh start     kill ava (pause its respawn), run sangamio (direct)
#   w10-direct.sh restore   stop sangamio, restart ava, resume the respawner
#   w10-direct.sh status    show ava / sangamio / respawner state
# ===========================================================================
set -u
DIR="${REMOTE_DIR:-/data/w10bridge}"
BIN="$DIR/sangamio"
CFG="${CFG:-$DIR/dreame_w10_direct.toml}"

sysmon_pid() { ps 2>/dev/null | grep '[s]ys_monitor.sh ava' | awk '{print $1}'; }

case "${1:-status}" in
    start)
        [ -x "$BIN" ] || { echo "ERROR: $BIN missing (upload first)"; exit 1; }
        echo ">> stop relay, pause ava respawn, kill ava"
        killall avatap-relay 2>/dev/null
        P=$(sysmon_pid); [ -n "$P" ] && kill -STOP $P   # freeze the respawner
        killall ava 2>/dev/null
        sleep 1
        mkdir -p /data/log
        echo ">> start sangamio (direct, ttyS4 + ttyS3)"
        RUST_LOG="${RUST_LOG:-info}" setsid "$BIN" --config "$CFG" \
            >/data/log/sangamio.log 2>&1 < /dev/null &
        sleep 2
        if pidof sangamio >/dev/null; then
            echo ">> sangamio running (commands TCP 5555). Restore: w10-direct.sh restore"
            tail -6 /data/log/sangamio.log 2>/dev/null
        else
            echo ">> WARN: sangamio not running -- see /data/log/sangamio.log"
            tail -8 /data/log/sangamio.log 2>/dev/null
        fi
        ;;
    restore)
        echo ">> stop sangamio, restart ava, resume respawn"
        killall sangamio 2>/dev/null
        sleep 1
        /etc/rc.d/ava.sh >/dev/null 2>&1 &   # restart ava (camera/Valetudo)
        sleep 3
        P=$(sysmon_pid); [ -n "$P" ] && kill -CONT $P   # resume the respawner
        sleep 1
        pidof ava >/dev/null && echo ">> ava back" || echo ">> WARN: ava not up yet"
        ;;
    status)
        echo -n "ava pid  : "; pidof ava || echo none
        echo -n "sangamio : "; pidof sangamio || echo none
        echo -n "sysmon   : "; sysmon_pid || echo none
        ;;
    *) echo "usage: w10-direct.sh start|restore|status"; exit 1 ;;
esac
