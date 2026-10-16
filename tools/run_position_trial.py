import sys,time,serial

port,kp,ki,kd=sys.argv[1],*[round(float(x),3) for x in sys.argv[2:5]]
s=serial.Serial();s.port=port;s.baudrate=115200;s.timeout=.05;s.dtr=False;s.rts=False;s.open();time.sleep(.7)
def send(x): s.write((x+"\n").encode());s.flush()
def line(): return s.readline().decode(errors="ignore").strip()

settle,over=5.0,0.0
try:
    while s.in_waiting: line()
    send("ZERO");time.sleep(.1);send(f"P,{kp},{ki},{kd}");time.sleep(.1);send("TEST")
    t0=time.monotonic();ret=None;last=None;stable=0
    while time.monotonic()-t0<50:
        z=line()
        if not z: continue
        if z.startswith("RETURN,"): ret=int(z.split(",")[1]);continue
        if not z.startswith("D,") or ret is None: continue
        a=z.split(",");ms,pos,count=int(a[1]),float(a[3]),int(a[5]);t=(ms-ret)/1000
        over=max(over,max(0.0,-pos))
        stable=stable+1 if abs(count)<=2 and count==last else 0
        last=count
        if stable>=5: settle=min(5.0,t);break
        if t>=5: break
finally:
    send("STOP");time.sleep(.05);send("HOME")
    end=time.monotonic()+20
    while time.monotonic()<end:
        if line()=="HOME": break
    send("STOP");s.close()
print(f"{settle:.4f},{min(over,300.0):.4f}")
