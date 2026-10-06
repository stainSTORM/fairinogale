"""Bind a device app to an Arkitekt server from a small web page.

The same file lives in every device repo (fairinogale, OT2windy). Keep the copies identical.

    from arkitekt_bind import serve
    serve(app, "config.yaml")  # blocks

The page (http://<host>:8080/, or /fairino/ and /opentrons/ behind the Pi's Caddy) shows the
connection state. Enter the server and press Bind: the page shows a code and an approval link
(the device-code login). Approve it in any browser and the app starts providing its actions.
The login is stored, so the device reconnects by itself after a restart.

The page also edits config.yaml. Saving restarts the process so the new settings apply.

Everything the device keeps sits next to config.yaml, in .state/: the stored login (fakts), the
chosen server (bind.json) and the rekuest journal. Delete .state/ to start over.
"""

import asyncio
import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import yaml

DEFAULT_URL = "https://go.arkitekt.live"
RECONNECT_DELAY_S = 30.0


class StoredLoginInvalid(RuntimeError):
    """A reconnect hit the device-code login: the stored login expired or was revoked."""


def _normalise(url: str) -> str:
    url = (url or "").strip().rstrip("/")
    return url if "://" in url else f"https://{url}"


def _is_insecure(url: str) -> bool:
    """Plain http to another host: fakts refuses that unless explicitly allowed."""
    parsed = urlparse(url)
    return parsed.scheme == "http" and parsed.hostname not in ("localhost", "127.0.0.1", "::1")


def _friendly(e: BaseException, url: str) -> str:
    kind, text = type(e).__name__, str(e) or type(e).__name__
    if isinstance(e, StoredLoginInvalid):
        return text
    if kind == "InsecureTransportError":
        return f"{url} is plain http. Tick 'allow http' for a server without TLS."
    if kind in ("DeviceCodeExpiredError", "DeviceCodeTimeoutError"):
        return "The code expired before it was approved. Press Bind for a new one."
    if kind == "UserDeniedError":
        return "The login was declined in the browser."
    if "discover" in text.lower() or kind in ("ClientConnectorError", "DiscoveryError"):
        return f"Could not reach an Arkitekt server at {url}: {text}"
    return f"{kind}: {text}"


class Binder:
    """The connection of one app to one Arkitekt server, run on its own thread and loop."""

    def __init__(self, app, state_dir: Path):
        self.app = app
        self.state_dir = state_dir
        self.settings_file = state_dir / "bind.json"
        self.settings = {"url": DEFAULT_URL, "insecure": False}
        if self.settings_file.exists():
            self.settings.update(json.loads(self.settings_file.read_text()))
        self.session = {"state": "unbound", "message": None}
        self._lock = threading.RLock()
        self._thread = self._loop = self._task = self._retry = None
        self._stopping = self._forget = self._automatic = False

    def _set(self, **changes):
        with self._lock:
            self.session.update(changes)

    def stored_login(self) -> Path:
        from arkitekt.app.fakts import _cache_path

        return Path(_cache_path(self.app, self.settings["url"]))

    def status(self) -> dict:
        with self._lock:
            session = dict(self.session)
        return {**session, **self.settings, "app": self.app.identifier,
                "version": self.app.version, "hasStoredLogin": self.stored_login().exists()}

    def bind(self, url=None, insecure=None, automatic=False) -> dict:
        """Connect in the background: discovery, login (device code if needed), provide."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return {**self.status(), "error": "Already connecting or connected."}
            if url:
                self.settings["url"] = _normalise(url)
            if insecure is not None:
                self.settings["insecure"] = bool(insecure)
            self.state_dir.mkdir(parents=True, exist_ok=True)
            self.settings_file.write_text(json.dumps(self.settings))
            if self.settings["insecure"] and _is_insecure(self.settings["url"]):
                os.environ["FAKTS_ALLOW_INSECURE_TRANSPORT"] = "1"
            if self._retry is not None:
                self._retry.cancel()
            self._stopping = self._forget = False
            self._automatic = automatic
            self.session = {"state": "connecting", "message": f"Connecting to {self.settings['url']}"}
            self._thread = threading.Thread(target=self._run, args=(self.settings["url"],),
                                            name="arkitekt", daemon=True)
            self._thread.start()
        return self.status()

    def auto_connect(self) -> None:
        """At startup: reconnect if bound before. Never starts a login by itself."""
        if self.stored_login().exists():
            self.bind(automatic=True)

    def cancel(self, forget=False) -> dict:
        """Stop a pending login or disconnect. *forget* also removes the stored login."""
        with self._lock:
            if self._retry is not None:
                self._retry.cancel()
            thread, loop, task = self._thread, self._loop, self._task
            self._stopping, self._forget = True, forget
        if thread is not None and thread.is_alive() and loop is not None and task is not None:
            loop.call_soon_threadsafe(task.cancel)
            thread.join(10)
        if forget:
            self.stored_login().unlink(missing_ok=True)
        self._set(state="unbound", message="Unbound." if forget else "Disconnected.",
                  userCode=None, approveUrl=None)
        return self.status()

    def _run(self, url: str) -> None:
        error, retry = None, True
        try:
            asyncio.run(self._amain(url))
            if not self._stopping:
                error = "The Arkitekt agent stopped."
        except asyncio.CancelledError:
            pass
        except BaseException as e:  # noqa: BLE001 - shown on the page
            print(f"Arkitekt connection to {url} failed: {e!r}", flush=True)
            error = _friendly(e, url)
            retry = not isinstance(e, StoredLoginInvalid) and type(e).__name__ not in (
                "InsecureTransportError", "UserDeniedError")
        finally:
            self._loop = self._task = None
        if error is None:
            return
        self._set(state="error", message=error, userCode=None, approveUrl=None)
        # Once logged in, keep reconnecting (the server or the network may come back); a first
        # bind that fails is left to the person at the page.
        if retry and not self._stopping and self.stored_login().exists():
            self._retry = threading.Timer(RECONNECT_DELAY_S, self.bind, kwargs={"automatic": True})
            self._retry.daemon = True
            self._retry.start()
            self._set(message=f"{error} Retrying in {RECONNECT_DELAY_S:.0f} s.")

    async def _amain(self, url: str) -> None:
        from arkitekt import connect

        self._loop, self._task = asyncio.get_running_loop(), asyncio.current_task()
        runtime = connect(self.app, provide=True, url=url, headless=True,
                          device_code_hook=self._on_device_code, force=True)
        async with runtime:  # discovery, login, agent registration
            self._set(state="connected", message=None, userCode=None, approveUrl=None)
            print(f"Arkitekt: providing '{self.app.identifier}' at {url}", flush=True)
            try:
                await runtime.arun()
            finally:
                if self._forget and runtime.fakts is not None:
                    await runtime.fakts.alogout()

    async def _on_device_code(self, challenge) -> None:
        """fakts' device-code hook: show the code on the page instead of opening a browser."""
        if self._automatic:
            raise StoredLoginInvalid("The stored login is no longer valid. Press Bind to approve "
                                     "this device again.")
        approve = challenge.verification_uri_complete
        if urlparse(approve or "").scheme not in ("http", "https"):
            approve = None
        print(f"Arkitekt login: approve code {challenge.user_code} at {approve}", flush=True)
        self._set(state="awaiting_login", userCode=challenge.user_code, approveUrl=approve,
                  message="Approve this device in a browser to finish binding.")


PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Arkitekt binding</title>
<style>
body{font:15px system-ui,sans-serif;max-width:46rem;margin:1.5rem auto;padding:0 1rem}
input[type=text]{width:100%;box-sizing:border-box;padding:.4rem}
textarea{width:100%;box-sizing:border-box;height:24rem;font:13px monospace}
button{padding:.4rem .9rem;margin:.4rem .4rem 0 0}
#code{font:bold 2rem monospace;letter-spacing:.1em}
.error{color:#b00020}
</style></head><body>
<h1 id="title">Arkitekt</h1>
<p>State: <b id="state">…</b> <span id="message"></span></p>
<div id="login" hidden><p>Code: <span id="code"></span></p>
<p><a id="approve" target="_blank">Approve this device</a> and check that the code matches.</p></div>
<h2>Server</h2>
<input type="text" id="url" placeholder="https://go.arkitekt.live">
<label><input type="checkbox" id="insecure"> allow plain http to this server (no TLS)</label><br>
<button onclick="post('api/bind',{url:url.value,insecure:insecure.checked})">Bind</button>
<button onclick="post('api/cancel')">Disconnect</button>
<button onclick="if(confirm('Forget the stored login?'))post('api/unbind')">Unbind</button>
<h2>config.yaml</h2>
<p id="path"></p>
<textarea id="config" spellcheck="false" wrap="off"></textarea>
<button onclick="saveConfig()">Save and restart</button> <span id="saved"></span>
<script>
let typing = false;
url.oninput = () => typing = true;
async function post(path, body) {
  const r = await fetch(path, {method: 'POST', body: JSON.stringify(body || {})});
  show(await r.json()); typing = false;
}
function show(s) {
  title.textContent = s.app + ' — Arkitekt';
  state.textContent = s.state;
  message.textContent = s.error || s.message || '';
  message.className = (s.error || s.state === 'error') ? 'error' : '';
  login.hidden = s.state !== 'awaiting_login';
  code.textContent = s.userCode || '';
  approve.href = s.approveUrl || '#';
  if (!typing) { url.value = s.url; insecure.checked = s.insecure; }
}
async function poll() { show(await (await fetch('api/status')).json()); }
async function loadConfig() {
  const c = await (await fetch('api/config')).json();
  path.textContent = c.path; config.value = c.text;
}
async function saveConfig() {
  const r = await (await fetch('api/config', {method: 'POST', body: config.value})).json();
  saved.textContent = r.error || 'Saved. Restarting…';
  saved.className = r.error ? 'error' : '';
  if (!r.error) setTimeout(() => location.reload(), 4000);
}
poll(); loadConfig(); setInterval(poll, 2000);
</script></body></html>
"""


def serve(app, config_path, port: int = 8080) -> None:
    """Serve the binding page on *port* and reconnect if bound before. Blocks."""
    config_path = Path(config_path).resolve()
    state = config_path.parent / ".state"
    os.environ.setdefault("XDG_STATE_HOME", str(state))  # the stored login (Linux)
    os.environ.setdefault("REKUEST_JOURNAL_PATH", str(state / "rekuest_journal.db"))
    binder = Binder(app, state)

    class Handler(BaseHTTPRequestHandler):
        def _reply(self, body, kind="application/json"):
            data = body.encode() if isinstance(body, str) else json.dumps(body).encode()
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?")[0].rstrip("/").rsplit("/", 1)[-1]
            if path == "status":
                self._reply(binder.status())
            elif path == "config":
                self._reply({"path": str(config_path), "text": config_path.read_text()})
            else:
                self._reply(PAGE, "text/html; charset=utf-8")

        def do_POST(self):
            body = self.rfile.read(int(self.headers.get("Content-Length") or 0)).decode()
            path = self.path.rstrip("/").rsplit("/", 1)[-1]
            if path == "bind":
                args = json.loads(body or "{}")
                self._reply(binder.bind(args.get("url"), args.get("insecure")))
            elif path == "cancel":
                self._reply(binder.cancel())
            elif path == "unbind":
                self._reply(binder.cancel(forget=True))
            elif path == "config":
                try:
                    yaml.safe_load(body)
                except yaml.YAMLError as e:
                    return self._reply({"error": f"Not valid YAML: {e}"})
                config_path.write_text(body)
                self._reply({"saved": True})
                threading.Timer(0.5, _restart).start()
            else:
                self.send_error(404)

        def log_message(self, *args):  # keep the console for the app's own output
            pass

    def _restart():
        print("config.yaml changed: restarting", flush=True)
        binder.cancel()
        os.execv(sys.executable, [sys.executable, *sys.argv])

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"Binding page on http://0.0.0.0:{port}/ (config: {config_path})", flush=True)
    binder.auto_connect()
    server.serve_forever()

