import csv
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
settle,over,s=5.0,300.0,None

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
    while time.monotonic()-t0<50:
        z=line()
        if z.startswith("RETURN,"):
            ret=int(z.split(",")[1]);continue
        if not z.startswith("D,") or ret is None:
            continue
        a=z.split(",");ms,pos,count=int(a[1]),float(a[3]),int(a[5]);t=(ms-ret)/1000
        over=max(over,max(0.0,-pos));stable=stable+1 if abs(count)<=2 and count==last else 0;last=count
        if stable>=5:
            settle=min(5.0,t);break
        if t>=5:break
except Exception:
    settle,over=5.0,300.0
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
print(f"{settle:.4f},{min(over,300):.4f}")
