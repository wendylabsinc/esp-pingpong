"""http://localhost:8093/mesh : the Anchor's live page without a token in the address.

The Anchor runs on the Dragonwing and is reached through `wendy cloud tunnel`
on 127.0.0.1:8091, which takes a second or two per request. This keeps a fresh
copy of /mesh.json (three overlapping askers in the background) and hands it
out at once, adds the Anchor's token itself, and passes on the page's buttons.
Listens on 127.0.0.1 only.
usage: live_proxy.py <port> <upstream> <key file>"""
import hashlib, hmac, http.server, json, sys, threading, time, urllib.error, urllib.request
port, upstream = int(sys.argv[1]), sys.argv[2].rstrip("/")
token = hmac.new(bytes.fromhex(open(sys.argv[3]).read().strip()), b"anchor-http", hashlib.sha256).hexdigest()[:16]
state = {"json": b'{"nodes": [], "edges": [], "anchor": null, "speed": 343.4, "temp_c": 20, "self_mm": 40}', "page": None, "at": 0.0, "sent": 0.0}
lock = threading.Lock()

def ask(path, method="GET", timeout=25):
    req = urllib.request.Request(upstream + path, method=method, headers={"Authorization": "Bearer " + token})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status, r.read()

def poll(wait):
    """One of several askers, started apart: an answer takes a second or two to come through the tunnel,
    so three overlapping ones bring a fresh copy about every 0.7 s where one brought it every 2.5 s."""
    time.sleep(wait)
    while True:
        sent = time.time()
        try:
            _, body = ask(f"/mesh.json?t={sent * 1000:.0f}")   # stamped: the Anchor learns what a quick trip looks like
            json.loads(body)
            with lock:
                if sent > state["sent"]:          # an answer to an older question never replaces a newer one
                    state["json"], state["sent"], state["at"] = body, sent, time.time()
        except Exception as e:
            print("upstream:", e, flush=True)
            time.sleep(2)
        time.sleep(0.1)

def press(path, full):
    """Pass one press of a button on to the Anchor. Each is stamped with this clock, so the Anchor can drop
    one that was held up on the way instead of acting late. A beep is given up on after 4 s. The other
    buttons are asked again, up to three times 6 s: a second copy does no harm (a run already going is left
    alone). What went wrong, if anything, is shown on the page for a few seconds."""
    why = "the Anchor did not answer: press again"
    for _ in range(1 if path == "/beep" else 3):
        try:
            ask(f"{full}{'&' if '?' in full else '?'}t={time.time() * 1000:.0f}", "POST", timeout=4 if path == "/beep" else 6)
            return
        except urllib.error.HTTPError as e:
            why = {409: "fewer than two listening boards heard lately", 408: "held up on the way and dropped: press again"}.get(e.code, f"the Anchor said {e.code}")
            break
        except Exception:
            pass
    print("press", full, "->", why, flush=True)
    with lock:
        state["note"], state["note_at"] = why, time.time()

class Handler(http.server.BaseHTTPRequestHandler):
    def send(self, code, body, kind="text/plain"):
        self.send_response(code); self.send_header("Content-Type", kind); self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
    def do_GET(self):
        path = self.path.split("?")[0].split("#")[0]
        if path in ("/", "/mesh"):
            try:      # the page from a local copy when there is one (a page change then needs no redeploy), else the Anchor's
                state["page"] = open(sys.argv[4], "rb").read() if len(sys.argv) > 4 else ask("/mesh")[1]
            except Exception:
                pass
            return self.send(200, state["page"] or b"The Anchor's page could not be fetched yet.", "text/html; charset=utf-8")
        if path == "/mesh.json":
            with lock:
                body, note = state["json"], state.get("note") if time.time() - state.get("note_at", 0) < 6 else None
            if note:      # a press that came to nothing, told to the page with the next few copies
                body = body.rstrip()[:-1] + b', "page_note": ' + json.dumps(note).encode() + b"}"
            return self.send(200, body, "application/json")
        self.send(404, b"not here\n")
    def do_POST(self):
        path = self.path.split("?")[0]
        if path not in ("/range", "/listen", "/dchirp", "/beep"):
            return self.send(404, b"not here\n")
        # The page gets its answer at once and the press goes on to the Anchor behind its back: the tunnel
        # holds about one request in twelve for many seconds, and a browser keeps only six requests open
        # to one address, so a few held presses used to stop the page's own refreshing (greyed buttons,
        # a dot that stayed large, presses that seemed to do nothing).
        threading.Thread(target=press, args=(path, self.path), daemon=True).start()
        self.send(202, b"asked\n")
    def log_message(self, *a):
        pass

for wait in (0.0, 0.7, 1.4):
    threading.Thread(target=poll, args=(wait,), daemon=True).start()
print(f"http://localhost:{port}/mesh", flush=True)
http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()
