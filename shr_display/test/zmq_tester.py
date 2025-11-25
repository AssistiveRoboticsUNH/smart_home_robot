import zmq, time, json, threading

# ─── CONFIG ───────────────────────────────────────────────────────────────────
# You already bind on the PC; keep it this way:
PUB_ENDPOINT  = "tcp://0.0.0.0:5556"  # Android SUB.connects here
PULL_ENDPOINT = "tcp://0.0.0.0:5557"  # Android PUSH.connects here

COMMANDS = [
    # "TURN_OFF",
    "TURN_ON",
    "protocols",
    "1",
    "0",
    "2",
    "3",
    "1",
    # "text:I am going to dock",
    # "TURN_OFF",
    # "TURN_ON",
    # "file:///storage/emulated/0/Download/maggie_coffee.mp4",
]
# COMMANDS = ["blink", "eye:0.35,-0.4", "eye:left", "eye:right", "eye:up", "eye:down", "eye:center", "exp:happy", "exp:confused" , "exp:sorry", "exp:astonished", "exp:sleeping", "exp:talking", "exp:neutral"]

with open('protocol_routines_test.json', 'r') as f:
    json_str = json.dumps(json.load(f))

# ─── CODE ─────────────────────────────────────────────────────────────────────
def recv_forever(pull_sock: zmq.Socket):
    """Print every message from the phone (ACK/HELLO/anything)."""
    poller = zmq.Poller()
    poller.register(pull_sock, zmq.POLLIN)
    while True:
        events = dict(poller.poll(timeout=500))
        if pull_sock in events and events[pull_sock] == zmq.POLLIN:
            try:
                msg = pull_sock.recv_string(flags=zmq.NOBLOCK)
                print(f"← FROM PHONE: {msg}")
            except zmq.Again:
                pass

def main():
    ctx = zmq.Context.instance()

    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 1000)
    pub.bind(PUB_ENDPOINT)

    pull = ctx.socket(zmq.PULL)
    pull.setsockopt(zmq.RCVHWM, 1000)
    pull.bind(PULL_ENDPOINT)

    # Start a background thread that prints all incoming messages
    threading.Thread(target=recv_forever, args=(pull,), daemon=True).start()

    # Give Android time to connect (PUB/SUB slow-joiner)
    print("Waiting 2s for phone to connect…")
    time.sleep(2.0)

    # Send JSON first
    print("→ Sending protocol_routines.json")
    pub.send_string(json_str)
    time.sleep(0.5)

    # Send commands, but don't stop listening in between
    for cmd in COMMANDS:
        print(f"\n→ CMD: {cmd!r}")
        pub.send_string(cmd)
        time.sleep(2)  # small gap is fine

    print("\nAll commands sent. This process will keep printing incoming messages.")
    print("Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        pub.close(0); pull.close(0); ctx.term()

if __name__ == "__main__":
    main()


# {"type":"protocol_confirm","index":1,"confirmed":true,"protocol":"Evening Medication at 7:00pm"}