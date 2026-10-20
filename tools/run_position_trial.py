import csv
import json
from datetime import datetime
from pathlib import Path
import sys,time,serial

log_root=Path(__file__).resolve().parents[1]/"data"/"logs"

if sys.argv[1:]==["--new-experiment"]:
    # Called once by JSL, before any trials. Never reuse an existing folder.
    log_root.mkdir(parents=True,exist_ok=True)
    stem="DiscoveryPID_"+datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix=0
    while True:
        experiment_dir=log_root/(stem+(f"_{suffix}" if suffix else ""))
        try:
            experiment_dir.mkdir()
            break
        except FileExistsError:
            suffix+=1
    print(experiment_dir)
    sys.exit(0)

port=sys.argv[1];kp,ki,kd=[round(float(x),3) for x in sys.argv[2:5]]
MAX_SETTLING_SECONDS=180.0
SETTLING_COUNTS=75
MAX_SETUP_SECONDS=20.0
settle,over,s=MAX_SETTLING_SECONDS,300.0,None
status,detail="error","Trial did not complete"
setup_seconds=return_seconds=None
final_counts_from_home=None

# One CSV per worker invocation; exclusive creation avoids overwriting a trial.
# JSL supplies the same experiment directory to every trial in its run.
# Direct worker invocations without this optional argument retain the old layout.
log_dir=Path(sys.argv[5]) if len(sys.argv)>5 else log_root
log_dir.mkdir(parents=True,exist_ok=True)
stem="DiscoveryPID_"+datetime.now().strftime("%Y%m%d_%H%M%S")
suffix=0
while True:
    log_path=log_dir/(stem+(f"_{suffix}" if suffix else "")+".csv")
    try:
        log=log_path.open("x",newline="",encoding="utf-8")
        break
    except FileExistsError:
        suffix+=1
writer=csv.writer(log)
writer.writerow(["PC Timestamp","Elapsed Time (s)","KP","KI","KD","Raw Serial Line"])
log.flush()
log_start=time.monotonic()
pending=b""

def record(raw):
    writer.writerow([datetime.now().isoformat(timespec="milliseconds"),
                     f"{time.monotonic()-log_start:.3f}",kp,ki,kd,
                     raw.decode("utf-8",errors="backslashreplace")])
    log.flush()

def line():
    # readline may return a fragment on timeout. Keep it until its newline.
    global pending
    pending+=s.readline()
    if not pending.endswith(b"\n"):
        return ""
    raw=pending[:-1]
    if raw.endswith(b"\r"):
        raw=raw[:-1]
    pending=b""
    record(raw)
    return raw.decode(errors="ignore").strip()

try:
    s=serial.Serial();s.port=port;s.baudrate=115200;s.timeout=.05;s.dtr=False;s.rts=False;s.open();time.sleep(.7)
    send=lambda x:(s.write((x+"\n").encode()),s.flush())
    while s.in_waiting:line()
    send("ZERO");send(f"KP={kp}");send(f"KI={ki}");send(f"KD={kd}");send("TEST")
    over=0.0;t0=time.monotonic();ret=last=None;stable=0
    return_start=last_data=None
    while True:
        now=time.monotonic()
        if ret is None and now-t0>=MAX_SETUP_SECONDS:
            status,detail="setup_timeout",f"No RETURN received within {MAX_SETUP_SECONDS:g} seconds"
            setup_seconds=now-t0
            break
        if ret is not None and now-return_start>=MAX_SETTLING_SECONDS+1:
            return_seconds=now-return_start
            if last_data is None or now-last_data>1:
                status,detail="telemetry_timeout","Return telemetry stopped before measurement completed"
            else:
                status,detail="not_settled",f"Return reached the {MAX_SETTLING_SECONDS:g} second limit"
            break
        z=line()
        if z.startswith("RETURN,"):
            if ret is None:
                ret=int(z.split(",")[1])
                return_start=time.monotonic()
                setup_seconds=return_start-t0
            continue
        if z.startswith("D,") and len(z.split(","))<8:
            status,detail="firmware_mismatch","Upload firmware with the relative_count serial field"
            break
        if not z.startswith("D,") or ret is None:
            continue
        a=z.split(",")
        if int(a[2])!=2:
            continue
        ms,pos,count=int(a[1]),float(a[3]),int(a[7]);t=((ms-ret) % (2**32))/1000
        final_counts_from_home=count
        last_data=time.monotonic();return_seconds=t
        over=max(over,max(0.0,-pos))
        # Do not count a late sample toward settling inside the allowed window.
        if t>=MAX_SETTLING_SECONDS:
            status,detail="not_settled",f"Return reached the {MAX_SETTLING_SECONDS:g} second limit"
            break
        stable=stable+1 if abs(count)<=SETTLING_COUNTS and count==last else 0;last=count
        if stable>=5:
            settle=t
            status,detail="settled",f"Stationary within {SETTLING_COUNTS} Home-relative counts"
            break
except Exception as exc:
    status,detail="error",f"{type(exc).__name__}: {exc}"
finally:
    if s:
        try:
            send("STOP");send("HOME");end=time.monotonic()+20
            while time.monotonic()<end:
                if line()=="HOME":
                    break
            send("STOP")
            # Read through the final STOP acknowledgment, including any queued data.
            end=time.monotonic()+1
            while time.monotonic()<end:
                if line()=="OK":
                    while s.in_waiting:line()
                    break
        except Exception:
            pass
        finally:
            try:
                s.close()
            except Exception:
                pass
    if pending:
        record(pending)
    log.close()
valid=status in ("settled","not_settled")
log_path.with_suffix(".json").write_text(json.dumps({
    "status":status,"detail":detail,"setup_seconds":setup_seconds,
    "return_seconds":return_seconds,"settling_time_s":settle if valid else None,
    "overshoot_deg":min(over,300) if valid else None,
    "final_counts_from_home":final_counts_from_home
},indent=2),encoding="utf-8")
if valid:
    print(f"{settle:.4f},{min(over,300):.4f}")
else:
    print(f"ERROR,{status}")
