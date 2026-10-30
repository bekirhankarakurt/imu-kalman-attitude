#include "imu/http_server.hpp"

namespace imu {

std::string_view viewerHtml() noexcept {
  static constexpr char kPage[] = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>IMU attitude viewer</title>
<style>
  :root { color-scheme: dark; --bg:#10151c; --panel:#18202b; --fg:#e6edf3; --dim:#8b98a8; --acc:#4cc2ff; }
  * { box-sizing: border-box; }
  body { margin:0; font:14px/1.4 system-ui, sans-serif; background:var(--bg); color:var(--fg);
         display:flex; flex-wrap:wrap; gap:16px; padding:16px; min-height:100vh; }
  .panel { background:var(--panel); border-radius:10px; padding:16px; }
  #stage { flex:1 1 420px; min-height:420px; display:flex; align-items:center; justify-content:center;
           perspective:900px; overflow:hidden; }
  #view { transform-style:preserve-3d; transform:rotateX(-22deg) rotateY(28deg); }
  #box { position:relative; width:0; height:0; transform-style:preserve-3d; }
  .face { position:absolute; display:flex; align-items:center; justify-content:center; font-weight:600;
          color:#fff; border:1px solid rgba(255,255,255,.35); backface-visibility:visible; }
  #side { flex:0 0 300px; display:flex; flex-direction:column; gap:12px; }
  h1 { font-size:16px; margin:0; }
  select { width:100%; padding:6px; background:var(--bg); color:var(--fg); border:1px solid #2c3948; border-radius:6px; }
  table { width:100%; border-collapse:collapse; font-variant-numeric:tabular-nums; }
  td { padding:3px 0; } td:last-child { text-align:right; }
  .big td { font-size:20px; } .dim { color:var(--dim); font-size:12px; }
  #status { color:var(--dim); } #status.ok { color:#5ad17a; }
</style>
</head>
<body>
<div id="stage" class="panel"><div id="view"><div id="box"></div></div></div>
<div id="side">
  <div class="panel"><h1>IMU attitude</h1><div id="status">connecting...</div></div>
  <div class="panel">
    <select id="filter" aria-label="filter"></select>
    <table class="big">
      <tr><td>Roll</td><td id="roll">-</td></tr>
      <tr><td>Pitch</td><td id="pitch">-</td></tr>
      <tr><td>Yaw</td><td id="yaw">-</td></tr>
    </table>
    <div class="dim">degrees; world frame x = north, z = up</div>
  </div>
  <div class="panel"><table>
    <tr><td>Gyro bias (deg/s)</td><td id="bias">-</td></tr>
    <tr><td>Attitude error vs truth</td><td id="err">n/a</td></tr>
    <tr><td>Stream time</td><td id="time">-</td></tr>
    <tr><td>Sample rate</td><td id="rate">-</td></tr>
  </table></div>
</div>
<script>
(function () {
  var A = 160, B = 90, C = 24;           // box size along body x, y, z
  var faces = [
    { t: 'translateZ(' + C / 2 + 'px) scaleX(-1)', w: A, h: B, bg: '#1f6feb', label: '+Z (screen)' },
    { t: 'rotateY(180deg) translateZ(' + C / 2 + 'px)', w: A, h: B, bg: '#274b8f', label: '' },
    { t: 'rotateY(90deg) translateZ(' + A / 2 + 'px)', w: C, h: B, bg: '#d1453b', label: '' },
    { t: 'rotateY(-90deg) translateZ(' + A / 2 + 'px)', w: C, h: B, bg: '#8f2f28', label: '' },
    { t: 'rotateX(90deg) translateZ(' + B / 2 + 'px)', w: A, h: C, bg: '#2ea043', label: '' },
    { t: 'rotateX(-90deg) translateZ(' + B / 2 + 'px)', w: A, h: C, bg: '#1d6b2c', label: '' }
  ];
  var box = document.getElementById('box');
  faces.forEach(function (f) {
    var d = document.createElement('div');
    d.className = 'face';
    d.style.width = f.w + 'px';
    d.style.height = f.h + 'px';
    d.style.left = -f.w / 2 + 'px';
    d.style.top = -f.h / 2 + 'px';
    d.style.background = f.bg;
    d.style.transform = f.t;
    d.textContent = f.label;
    box.appendChild(d);
  });

  // Body -> world rotation matrix from quaternion [w, x, y, z].
  function matrix(q) {
    var w = q[0], x = q[1], y = q[2], z = q[3];
    var R = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
             [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
             [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]];
    // The matrix is a reflection (CSS is left-handed), so the label face is pre-mirrored above.
    // World (x north, y west, z up) -> screen (right, down, toward viewer); columns = images of body axes.
    var m = [];
    for (var j = 0; j < 3; j++) m.push(-R[1][j], -R[2][j], -R[0][j], 0);
    m.push(0, 0, 0, 1);
    return 'matrix3d(' + m.join(',') + ')';
  }

  var sel = document.getElementById('filter');
  var chosen = null;
  sel.addEventListener('change', function () { chosen = sel.value; });
  function $(id) { return document.getElementById(id); }
  function f2(v) { return (v >= 0 ? '+' : '') + v.toFixed(2); }

  function render(s) {
    if (sel.options.length !== s.filters.length) {
      sel.innerHTML = '';
      s.filters.forEach(function (f) { var o = document.createElement('option'); o.value = o.textContent = f.name; sel.appendChild(o); });
      chosen = chosen || s.selected;
      sel.value = chosen;
    }
    var cur = s.filters.filter(function (f) { return f.name === (chosen || s.selected); })[0] || s.filters[0];
    if (!cur) return;
    box.style.transform = matrix(cur.q);
    $('roll').textContent = f2(cur.roll);
    $('pitch').textContent = f2(cur.pitch);
    $('yaw').textContent = f2(cur.yaw);
    $('bias').textContent = cur.bias_dps.map(f2).join(' ');
    $('time').textContent = s.t.toFixed(1) + ' s';
    $('rate').textContent = s.rate_hz.toFixed(0) + ' Hz';
    if (s.truth) {
      var a = cur.q, b = s.truth.q;
      var d = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
      $('err').textContent = (2 * Math.acos(Math.min(1, d)) * 180 / Math.PI).toFixed(2) + ' deg';
    } else { $('err').textContent = 'n/a'; }
  }

  function poll() {
    fetch('/api/state', { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .then(function (s) { $('status').textContent = 'live'; $('status').className = 'ok'; render(s); })
      .catch(function () { $('status').textContent = 'disconnected'; $('status').className = ''; })
      .then(function () { setTimeout(poll, 50); });
  }
  poll();
})();
</script>
</body>
</html>
)HTML";
  return {kPage, sizeof(kPage) - 1};
}

}  // namespace imu
