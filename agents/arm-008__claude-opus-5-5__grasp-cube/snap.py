import time, camd_client as cc
def snap(cam, timeout=4.0):
    t0=time.monotonic()
    while True:
        try: return cc.read_frame(cam)[0]
        except cc.CamdError:
            if time.monotonic()-t0>timeout: return None
            time.sleep(0.1)
