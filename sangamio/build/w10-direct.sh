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
#   w10-direct.sh start [rgb|tof]  kill ava, run sangamio (direct); with rgb/tof
#                                  also bring up the standalone camera (Phase 3)
#   w10-direct.sh restore          stop sangamio + camera, restart ava, resume
#   w10-direct.sh status           ava / sangamio / camera / respawner state
# ===========================================================================
set -u
DIR="${REMOTE_DIR:-/data/w10bridge}"
BIN="$DIR/sangamio"
CFG="${CFG:-$DIR/dreame_w10_direct.toml}"
# Optional Phase-3 standalone camera (from dreame-vacuum-livestream); needs ava
# stopped, which this script already does. Missing = camera step skipped.
CAMSH="${CAMSH:-/data/camstream/noava-cam.sh}"

# Two watchdogs restart ava: sys_monitor.sh (the respawner) and monitor.sh (a
# health check that runs `ava.sh` if ava stops responding, ~15s). Freeze BOTH,
# or ava comes back mid-session and fights sangamio for the MCU.
sysmon_pid() { ps 2>/dev/null | grep '[s]ys_monitor.sh ava' | awk '{print $1}'; }
mon_pid()    { ps 2>/dev/null | grep '[r]c.d/monitor.sh'     | awk '{print $1}'; }
freeze_ava_watchdogs() { for p in $(sysmon_pid) $(mon_pid); do kill -STOP "$p" 2>/dev/null; done; }
resume_ava_watchdogs() { for p in $(sysmon_pid) $(mon_pid); do kill -CONT "$p" 2>/dev/null; done; }

case "${1:-status}" in
    start)
        [ -x "$BIN" ] || { echo "ERROR: $BIN missing (upload first)"; exit 1; }
        echo ">> stop relay, freeze ava watchdogs, kill ava"
        killall avatap-relay 2>/dev/null
        freeze_ava_watchdogs
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
        case "${2:-}" in
            rgb) [ -x "$CAMSH" ] && { echo ">> start camera (RGB)"; sh "$CAMSH" start; } ;;
            tof) [ -x "$CAMSH" ] && { echo ">> start camera (IR/ToF)"; sh "$CAMSH" start tof; } ;;
        esac
        ;;
    restore)
        echo ">> stop camera + sangamio, restart ava, resume respawn"
        [ -x "$CAMSH" ] && sh "$CAMSH" stop >/dev/null 2>&1
        killall sangamio 2>/dev/null
        sleep 1
        /etc/rc.d/ava.sh >/dev/null 2>&1 &   # restart ava (camera/Valetudo)
        sleep 3
        resume_ava_watchdogs
        sleep 1
        pidof ava >/dev/null && echo ">> ava back" || echo ">> WARN: ava not up yet"
        ;;
    status)
        echo -n "ava pid  : "; pidof ava || echo none
        echo -n "sangamio : "; pidof sangamio || echo none
        echo -n "camera   : "; pidof w10-cam || echo none
        echo -n "sysmon   : "; sysmon_pid || echo none
        echo -n "monitor  : "; mon_pid || echo none
        ;;
    *) echo "usage: w10-direct.sh start [rgb|tof] | restore | status"; exit 1 ;;
esac
