# CP2102 per-request USB health test (run on the Pi with sudo, needs python3-usb).
# Sends each request type 100 times directly over USB, bypassing the cp210x driver.
# Read-only except SET_* (UART settings) and SET_MHS, which resets an attached ESP once.
import usb.core, usb.util, time
d = usb.core.find(idVendor=0x10c4, idProduct=0xea60)
assert d, "CP2102 not found"
if d.is_kernel_driver_active(0): d.detach_kernel_driver(0)
VIN, VOUT = 0xC1, 0x41
N = 100
def run(name, fn):
    ok = err = 0; last = ""; sample = None
    for _ in range(N):
        try: r = fn(); ok += 1; sample = r
        except Exception as e: err += 1; last = str(e)
        time.sleep(0.005)
    s = bytes(sample).hex() if sample is not None and not isinstance(sample, int) else sample
    print(f"{name:34s} ok {ok:3d}  fail {err:3d}  {('sample='+str(s)) if ok else last}")
print("== standard USB requests (handled by chip USB core)")
run("GET_DESCRIPTOR device",   lambda: d.ctrl_transfer(0x80, 6, 0x0100, 0, 18))
run("GET_DESCRIPTOR config",   lambda: d.ctrl_transfer(0x80, 6, 0x0200, 0, 32))
run("GET_DESCRIPTOR string 2", lambda: d.ctrl_transfer(0x80, 6, 0x0302, 0x0409, 255))
run("GET_STATUS device",       lambda: d.ctrl_transfer(0x80, 0, 0, 0, 2))
print("== CP210x vendor reads (chip firmware)")
run("IFC_ENABLE (out, no data)", lambda: d.ctrl_transfer(VOUT, 0x00, 1, 0, None))
run("GET_BAUDRATE 0x1D",       lambda: d.ctrl_transfer(VIN, 0x1D, 0, 0, 4))
run("GET_LINE_CTL 0x04",       lambda: d.ctrl_transfer(VIN, 0x04, 0, 0, 2))
run("GET_MDMSTS 0x08",         lambda: d.ctrl_transfer(VIN, 0x08, 0, 0, 1))
run("GET_PROPS 0x0F",          lambda: d.ctrl_transfer(VIN, 0x0F, 0, 0, 66))
run("GET_COMM_STATUS 0x10",    lambda: d.ctrl_transfer(VIN, 0x10, 0, 0, 19))
run("GET_FLOW 0x14",           lambda: d.ctrl_transfer(VIN, 0x14, 0, 0, 16))
run("VENDOR part num 0x370B",  lambda: d.ctrl_transfer(VIN, 0xFF, 0x370B, 0, 1))
print("== CP210x vendor writes (UART settings, safe)")
run("SET_BAUDRATE 115200",     lambda: d.ctrl_transfer(VOUT, 0x1E, 0, 0, (115200).to_bytes(4,"little")))
run("SET_LINE_CTL 8N1",        lambda: d.ctrl_transfer(VOUT, 0x03, 0x0800, 0, None))
print("== bulk IN (ESP serial data)")
d.ctrl_transfer(VOUT, 0x00, 1, 0, None)
tot = errs = 0; t = time.time()
while time.time() - t < 3:
    try: tot += len(d.read(0x81, 64, 200))
    except usb.core.USBTimeoutError: pass
    except Exception as e: errs += 1
print(f"bulk read 3s: {tot} bytes, {errs} errors")
print("== DTR/RTS (resets ESP once)")
run("SET_MHS DTR/RTS idle",    lambda: d.ctrl_transfer(VOUT, 0x07, 0x0300, 0, None))
usb.util.dispose_resources(d)
try: d.attach_kernel_driver(0)
except Exception as e: print("reattach:", e)
