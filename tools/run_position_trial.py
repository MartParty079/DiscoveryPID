import sys,time,serial
port=sys.argv[1];kp,ki,kd=[round(float(x),3) for x in sys.argv[2:5]]
settle,over,s=5.0,300.0,None

try:
    s=serial.Serial();s.port=port;s.baudrate=115200;s.timeout=.05;s.dtr=False;s.rts=False;s.open();time.sleep(.7)
    send=lambda x:(s.write((x+"\n").encode()),s.flush())
    line=lambda:s.readline().decode(errors="ignore").strip()
    while s.in_waiting:line()
    send("ZERO");send(f"KP={kp}");send(f"KI={ki}");send(f"KD={kd}");send("TEST")
    over=0.0;t0=time.monotonic();ret=last=None;stable=0
    while time.monotonic()-t0<50:
        z=line()
        if z.startswith("RETURN,"):ret=int(z.split(",")[1]);continue
        if not z.startswith("D,") or ret is None:continue
        a=z.split(",");ms,pos,count=int(a[1]),float(a[3]),int(a[5]);t=(ms-ret)/1000
        over=max(over,max(0.0,-pos));stable=stable+1 if abs(count)<=2 and count==last else 0;last=count
        if stable>=5:settle=min(5.0,t);break
        if t>=5:break
except Exception:
    settle,over=5.0,300.0
finally:
    if s:
        try:
            send("STOP");send("HOME");end=time.monotonic()+20
            while time.monotonic()<end:
                if line()=="HOME":break
            send("STOP");s.close()
        except Exception:pass
print(f"{settle:.4f},{min(over,300):.4f}")
