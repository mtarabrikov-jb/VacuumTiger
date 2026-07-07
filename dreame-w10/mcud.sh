#!/bin/sh
# ===========================================================================
# Enter/exit path-3 "mcud" mode: replace `ava` at the MCU with `w10-mcud`.
#
#   !!! This KILLS ava. The camera stream, Valetudo and ava's own cliff/bump
#       safety and docking go down while mcud drives the MCU directly. mcud runs
#       its own watchdog + speed clamp + cliff/bump gate. Reversible:
#       `mcud.sh restore` brings ava (and the read-only tap relay) back.
#
#   mcud.sh start     kill ava (pause its respawn), run w10-mcud
#   mcud.sh restore    stop mcud, restart ava + relay, resume the respawner
#   mcud.sh status     show ava / mcud / respawner state
# ===========================================================================
set -u
DIR="${REMOTE_DIR:-/data/w10bridge}"
MCUD="$DIR/w10-mcud"
RELAY="$DIR/avatap-relay"

sysmon_pid() { ps 2>/dev/null | grep '[s]ys_monitor.sh ava' | awk '{print $1}'; }

case "${1:-status}" in
    start)
        [ -x "$MCUD" ] || { echo "ERROR: $MCUD missing (upload first)"; exit 1; }
        echo ">> stop relay, pause ava respawn, kill ava"
        killall avatap-relay 2>/dev/null
        P=$(sysmon_pid); [ -n "$P" ] && kill -STOP $P   # freeze the respawner
        killall ava 2>/dev/null
        sleep 1
        mkdir -p /data/log
        echo ">> start w10-mcud"
        setsid "$MCUD" >/data/log/mcud.log 2>&1 < /dev/null &
        sleep 1
        if pidof w10-mcud >/dev/null; then
            echo ">> mcud running (control 7705, telem 7701). Restore with: mcud.sh restore"
        else
            echo ">> WARN: mcud not running — see /data/log/mcud.log"; tail -5 /data/log/mcud.log 2>/dev/null
        fi
        ;;
    restore)
        echo ">> stop mcud, restart ava + relay, resume respawn"
        killall w10-mcud 2>/dev/null
        sleep 1
        /etc/rc.d/ava.sh >/dev/null 2>&1 &   # fast ava restart (tap via bind-mount)
        sleep 3
        P=$(sysmon_pid); [ -n "$P" ] && kill -CONT $P   # resume the (now idle) respawner
        killall avatap-relay 2>/dev/null
        [ -x "$RELAY" ] && setsid "$RELAY" 0.0.0.0 >/data/log/avatap-relay.log 2>&1 < /dev/null &
        sleep 1
        pidof ava >/dev/null && echo ">> ava back; relay restarted" || echo ">> WARN: ava not up yet"
        ;;
    status)
        echo -n "ava pid  : "; pidof ava || echo none
        echo -n "mcud pid : "; pidof w10-mcud || echo none
        echo -n "sysmon   : "; sysmon_pid || echo none
        echo -n "relay    : "; pidof avatap-relay >/dev/null 2>&1 && echo running || echo stopped
        ;;
    *) echo "usage: mcud.sh start|restore|status"; exit 1 ;;
esac
