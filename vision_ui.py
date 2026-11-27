"""The vision page: what the gripper camera sees at a station, and saving its reference.

    http://<host>:8093/      (vision.port in config.yaml, or VISION_PORT)

The page shows the live image with the tags found (green) and the tags of the saved reference
(orange), the station's move since teaching, and whether the arm would correct it, go in as
taught or stay out. "Save reference" stores the current view as the station's reference: press it
with the arm at the look point, right after teaching the station's points.

With vision.allow_motion the page can also drive the arm to the look point and back. It has no
login: leave that off where the network is not yours.

app.py starts it and passes itself in (start(port, app_module)), so this file imports neither
app.py nor Arkitekt.
"""

import base64
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2

import vision

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Station vision</title>
<style>
body{font:15px system-ui,sans-serif;max-width:76rem;margin:1.2rem auto;padding:0 1rem;color:#222}
select,input[type=text]{padding:.35rem;font:inherit}
button{padding:.4rem .9rem;margin:.4rem .4rem 0 0;font:inherit}
.views{display:flex;gap:1rem;flex-wrap:wrap;margin-top:.8rem}
.views figure{margin:0;flex:1 1 26rem}
.views img{width:100%;background:#ddd;min-height:8rem;display:block}
figcaption{font-size:13px;color:#555;margin-top:.2rem}
#verdict{font-weight:600;padding:.5rem .7rem;margin:.8rem 0;border-left:.4rem solid #999;background:#f3f3f3}
#verdict.ok{border-color:#2e7d32}#verdict.correct{border-color:#e08a00}#verdict.stop{border-color:#b00020}
table{border-collapse:collapse}td{padding:.15rem 1.2rem .15rem 0}td.n{text-align:right;font-variant-numeric:tabular-nums}
.green{color:#1a8a1a}.orange{color:#d97800}.error{color:#b00020}
</style></head><body>
<h1>Station vision</h1>
<p>
 <label>Station <select id="station"></select></label>
 <label id="samplebox" hidden>Sample <input type="text" id="sample" size="10" placeholder="Box1_A1"></label>
 <label>Look point <select id="look"></select></label>
</p>
<div id="verdict">…</div>
<div class="views">
 <figure><img id="live" alt="live image"><figcaption>Now. <span class="green">Green</span>: tags
 found. <span class="orange">Orange</span>: where the reference has them.</figcaption></figure>
 <figure><img id="saved" alt="no reference saved"><figcaption id="savedcap">Reference</figcaption></figure>
</div>
<table id="numbers"></table>
<p>
 <button id="save">Save reference</button>
 <span id="motion" hidden>
  <button id="goto">Drive to the look point</button>
  <button id="back">Drive back</button>
 </span>
 <span id="note"></span>
</p>
<p style="font-size:13px;color:#555">Save the reference with the arm at the look point, the station
as taught and one carrier on its seat. The arm corrects only the difference to that view.</p>
<script>
const $ = id => document.getElementById(id);
let stations = [], busy = false;
const query = () => 'station=' + encodeURIComponent($('station').value) + '&sample=' +
  encodeURIComponent($('sample').value) + '&look=' + encodeURIComponent($('look').value);
const body = extra => JSON.stringify({station: $('station').value, sample: $('sample').value,
  look: $('look').value, ...extra});
function pickStation() {
  const s = stations.find(s => s.name === $('station').value);
  $('look').innerHTML = s.looks.map(l => `<option>${l}</option>`).join('');
  $('samplebox').hidden = !s.sample;
  showSaved();
}
function showSaved() { $('saved').src = 'reference.jpg?' + query() + '&t=' + Date.now(); }
async function start() {
  const s = await (await fetch('api/stations')).json();
  stations = s.stations;
  if (!stations.length) { $('verdict').textContent = 'No station has a vision block in config.yaml.'; return; }
  $('station').innerHTML = stations.map(s => `<option>${s.name}</option>`).join('');
  $('motion').hidden = !s.allowMotion;
  pickStation();
  setInterval(poll, 700);
}
const fix = v => (Math.abs(v) < 0.005 ? 0 : v).toFixed(2);  // no "-0.00"
const num = (v, unit) => v == null ? '–' : fix(v) + ' ' + unit;
async function poll() {
  if (busy || document.hidden) return;
  busy = true;
  try {
    const r = await (await fetch('api/look?' + query())).json();
    if (r.image) $('live').src = r.image;
    const v = $('verdict'), m = r.measurement;
    if (r.error) { v.className = 'stop'; v.textContent = 'Stay out: ' + r.error; }
    else if (r.apply) { v.className = 'correct'; v.textContent = 'Would correct: the points move up to ' + r.travel_mm.toFixed(2) + ' mm.'; }
    else { v.className = 'ok'; v.textContent = 'As taught: the station is inside the deadband (' + r.travel_mm.toFixed(2) + ' mm).'; }
    const seat = r.on_seat == null ? 'not checked' : (r.on_seat ? 'on the seat' : 'not on the seat') +
      (m && m.carrier_mm != null ? ` (tag ${m.carrier_tag}, ${m.carrier_mm.toFixed(2)} mm off)` : ' (no carrier tag in view)');
    const rows = [['Tags in view', (r.tags || []).join(', ') || 'none'],
      ['Shift x / y / z', m ? m.shift.map(fix).join(' / ') + ' mm' : '–'],
      ['Yaw', num(m && m.yaw_deg, 'deg')], ['Lift', num(m && m.lift_mm, 'mm')],
      ['Tilt', num(m && m.tilt_deg, 'deg')], ['Fit error', num(m && m.error_px, 'px')],
      ['Carrier', seat]];
    $('numbers').innerHTML = rows.map(r => `<tr><td>${r[0]}</td><td class="n">${r[1]}</td></tr>`).join('');
    $('savedcap').textContent = r.saved ? 'Reference, saved ' + r.saved : 'No reference saved for this look point';
  } catch (e) { $('verdict').className = 'stop'; $('verdict').textContent = 'No answer from the app: ' + e; }
  busy = false;
}
async function post(path, extra) {
  $('note').className = ''; $('note').textContent = 'Working…';
  const r = await (await fetch(path, {method: 'POST', body: body(extra)})).json();
  $('note').className = r.error ? 'error' : ''; $('note').textContent = r.error || r.done || '';
  return r;
}
$('station').onchange = pickStation;
$('look').onchange = showSaved; $('sample').onchange = showSaved;
$('save').onclick = async () => {
  if (!confirm('Replace the reference of this look point by the current view?')) return;
  let r = await post('api/reference');
  if (r.error && r.canForce && confirm(r.error + '\\n\\nSave anyway?')) r = await post('api/reference', {force: true});
  showSaved();
};
$('goto').onclick = () => confirm('The arm will drive to the look point. Is its way clear?') && post('api/goto');
$('back').onclick = () => confirm('The arm will drive back from the look point. Is it standing there?') && post('api/return');
start();
</script></body></html>
"""


def start(port: int, api) -> None:
    """Serves the page in a background thread. api: the app module (observe, save_reference, ...)."""

    def stations() -> dict:
        found = [{"name": name, "looks": api.look_points(st), "sample": "{sample}" in st["file"]}
                 for name, st in api.cfg["stations"].items() if api.look_points(st)]
        return {"stations": found, "allowMotion": bool(api.VISION.get("allow_motion"))}

    def look(args: dict) -> dict:
        seen = api.observe(args["station"], args.get("sample", ""), args.get("look") or None)
        picture = vision.annotate(seen["image"], seen["now"], seen["reference"])
        jpeg = cv2.imencode(".jpg", picture, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]
        m = seen["measurement"]
        return {"image": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode(),
                "tags": sorted(seen["now"]), "error": str(seen["error"]) if seen["error"] else None,
                "apply": seen["apply"], "travel_mm": seen["travel_mm"], "on_seat": seen["on_seat"],
                "saved": (seen["reference"] or {}).get("saved"),
                "measurement": {k: v for k, v in m.as_dict().items() if k != "correction"} if m else None}

    def act(path: str, args: dict) -> dict:
        station, sample = args["station"], args.get("sample", "")
        if path == "reference":
            try:
                api.save_reference(station, sample, args.get("look") or None, bool(args.get("force")))
            except (api.NotAtLookPoint, ConnectionError) as e:
                # Not at the look point, or no arm to ask: the person at the page may know better.
                return {"error": str(e), "canForce": True}
            return {"done": "Reference saved."}
        if not api.VISION.get("allow_motion"):
            return {"error": "vision.allow_motion is off in config.yaml: the page does not drive the arm."}
        speed, acc = api.cfg["speed"], api.cfg["acceleration"]
        if path == "goto":
            api.look_at(station, sample, False, True, speed, acc, api.cfg["danger_speed"], api.ConsoleTask())
            return {"done": "The arm is at the look point."}
        api.leave_look(station, sample, speed, acc)
        return {"done": "The arm is back at the start of the route."}

    class Handler(BaseHTTPRequestHandler):
        def _reply(self, body, kind="application/json", status=200):
            data = body if isinstance(body, bytes) else (
                body.encode() if isinstance(body, str) else json.dumps(body).encode())
            self.send_response(status)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url = urlparse(self.path)
            path = url.path.rstrip("/").rsplit("/", 1)[-1]
            args = {k: v[0] for k, v in parse_qs(url.query).items()}
            try:
                if path == "stations":
                    self._reply(stations())
                elif path == "look":
                    self._reply(look(args))
                elif path == "reference.jpg":
                    file = vision.reference_file(api.station_points(args["station"], args.get("sample", ""))[1])
                    image = file.with_suffix(f".{args.get('look', '')}.jpg")
                    if image.exists():
                        self._reply(image.read_bytes(), "image/jpeg")
                    else:
                        self._reply(b"", "image/jpeg", 404)
                else:
                    self._reply(PAGE, "text/html; charset=utf-8")
            except Exception as e:  # noqa: BLE001 - shown on the page
                self._reply({"error": f"{type(e).__name__}: {e}"})

        def do_POST(self):
            body = self.rfile.read(int(self.headers.get("Content-Length") or 0)).decode()
            path = self.path.rstrip("/").rsplit("/", 1)[-1]
            if path not in ("reference", "goto", "return"):
                return self.send_error(404)
            try:
                self._reply(act(path, json.loads(body or "{}")))
            except Exception as e:  # noqa: BLE001 - shown on the page
                self._reply({"error": f"{type(e).__name__}: {e}"})

        def log_message(self, *args):  # keep the console for the app's own output
            pass

    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    threading.Thread(target=server.serve_forever, name="vision-page", daemon=True).start()
    print(f"Vision page on http://0.0.0.0:{port}/", flush=True)
