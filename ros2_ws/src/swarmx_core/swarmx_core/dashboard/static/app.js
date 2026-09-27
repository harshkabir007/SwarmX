/* SwarmX fleet dashboard - vanilla JS, no build step.
 * Data arrives over a WebSocket from a *passive* P2P listener (FleetMonitor);
 * commands go back as JSON. All network-provided strings are inserted with
 * textContent, never innerHTML. */
'use strict';
(() => {
  const $ = (id) => document.getElementById(id);
  const SVGNS = 'http://www.w3.org/2000/svg';

  const S = {
    world: null, info: {}, snap: null, recvAt: 0, t0: null,
    selected: null, tool: 'inspect', hover: null, ws: null, colors: {},
    chartIdx: null, showTable: false, retry: 500,
  };

  // --------------------------------------------------------------- theme
  function readColors() {
    const cs = getComputedStyle(document.documentElement);
    const g = (n) => cs.getPropertyValue(n).trim();
    S.colors = {
      floor: g('--floor'), shelf: g('--shelf'), wall: g('--wall'),
      zoneHeld: g('--zone-held'), zoneClaim: g('--zone-claim'),
      work: g('--state-work'), carry: g('--state-carry'), charge: g('--state-charge'), idle: g('--state-idle'),
      critical: g('--critical'), warning: g('--warning'), good: g('--good'),
      ink1: g('--ink-1'), ink2: g('--ink-2'), ink3: g('--ink-3'), surface: g('--surface-1'),
      grid: g('--grid'), axis: g('--axis'),
    };
  }
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { readColors(); buildLegend(); draw(); renderChart(); });

  // ------------------------------------------------------------- states
  const STATE = {
    to_pickup: { cat: 'work', label: 'To pickup' }, picking: { cat: 'work', label: 'Picking' },
    to_dropoff: { cat: 'carry', label: 'Carrying' }, dropping: { cat: 'carry', label: 'Dropping off' },
    to_charger: { cat: 'charge', label: 'To charger' }, charging: { cat: 'charge', label: 'Charging' },
    idle: { cat: 'idle', label: 'Idle' }, parking: { cat: 'idle', label: 'Parking' },
    failed: { cat: 'failed', label: 'Failed' },
  };
  const catOf = (r) => (!r.alive && r.st !== 'failed') ? 'lost' : (STATE[r.st] || { cat: 'idle' }).cat;
  const colorOf = (cat) => ({ work: S.colors.work, carry: S.colors.carry, charge: S.colors.charge, idle: S.colors.idle,
    failed: S.colors.critical, lost: S.colors.ink3 })[cat];
  function stateLabel(r) {
    if (!r.alive && r.st !== 'failed') return 'No signal';
    return (STATE[r.st] || { label: String(r.st) }).label;
  }

  // -------------------------------------------------------------- icons
  const ICONS = {
    good: 'M8 1a7 7 0 1 0 0 14A7 7 0 0 0 8 1Zm3.3 5.2-4 4.3a.8.8 0 0 1-1.1 0L4.7 9a.8.8 0 1 1 1.1-1.1l1 1 3.4-3.7a.8.8 0 1 1 1.1 1Z',
    warning: 'M8.9 1.6a1 1 0 0 0-1.8 0L.6 13.4A1 1 0 0 0 1.5 15h13a1 1 0 0 0 .9-1.6L8.9 1.6ZM8 5a.8.8 0 0 1 .8.8v3.6a.8.8 0 0 1-1.6 0V5.8A.8.8 0 0 1 8 5Zm0 8a1 1 0 1 1 0-2 1 1 0 0 1 0 2Z',
    critical: 'M8 1a7 7 0 1 0 0 14A7 7 0 0 0 8 1Zm2.8 8.7a.8.8 0 1 1-1.1 1.1L8 9.1l-1.7 1.7a.8.8 0 1 1-1.1-1.1L6.9 8 5.2 6.3a.8.8 0 1 1 1.1-1.1L8 6.9l1.7-1.7a.8.8 0 1 1 1.1 1.1L9.1 8l1.7 1.7Z',
    info: 'M8 1a7 7 0 1 0 0 14A7 7 0 0 0 8 1Zm0 3a1 1 0 1 1 0 2 1 1 0 0 1 0-2Zm1 8H7V7.5h2V12Z',
  };
  function icon(kind) {
    const s = document.createElementNS(SVGNS, 'svg');
    s.setAttribute('viewBox', '0 0 16 16'); s.setAttribute('class', `icon ${kind}`); s.setAttribute('aria-hidden', 'true');
    const p = document.createElementNS(SVGNS, 'path'); p.setAttribute('d', ICONS[kind] || ICONS.info); s.appendChild(p);
    return s;
  }
  function el(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined && text !== null) e.textContent = text;
    return e;
  }

  // ------------------------------------------------------------ helpers
  const fmtInt = (n) => (n === null || n === undefined) ? '–' : Math.round(n).toLocaleString();
  function fmtClock(t) {
    if (S.t0 === null || t === undefined) return '';
    const s = Math.max(0, t - S.t0), m = Math.floor(s / 60), r = Math.floor(s % 60);
    return `${m}:${String(r).padStart(2, '0')}`;
  }
  function fmtAge(a) {
    if (a === undefined || a === null) return '–';
    return a < 10 ? `${a.toFixed(1)} s` : `${Math.round(a)} s`;
  }

  // ---------------------------------------------------------- websocket
  function connect() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws`);
    S.ws = ws;
    ws.onopen = () => { setConn(true, 'Live'); S.retry = 500; };
    ws.onclose = () => { setConn(false, 'Disconnected, retrying…'); setTimeout(connect, S.retry); S.retry = Math.min(S.retry * 2, 8000); };
    ws.onerror = () => ws.close();
    ws.onmessage = (ev) => {
      let msg;
      try { msg = JSON.parse(ev.data); } catch { return; }
      if (msg.type === 'world') onWorld(msg);
      else if (msg.type === 'snapshot') onSnapshot(msg);
      else if (msg.type === 'cmd_result' && msg.ok === false) localEvent(`Command failed: ${msg.error || 'unknown error'}`, 'alert');
    };
  }
  function setConn(ok, text) {
    $('conn').className = `conn ${ok ? 'ok' : 'bad'}`;
    $('conn-text').textContent = text;
  }
  function send(cmd) {
    if (S.ws && S.ws.readyState === WebSocket.OPEN) S.ws.send(JSON.stringify(cmd));
  }
  const localEvents = [];
  function localEvent(text, kind) {
    localEvents.push({ t: S.snap ? S.snap.t : 0, text, kind, local: true });
    renderLog();
  }

  // --------------------------------------------------------------- world
  function onWorld(w) {
    S.world = w;
    S.info = w.info || {};
    $('source').textContent = S.info.source === 'ros' ? 'ROS 2 fleet' : (S.info.source === 'edge' ? 'Edge fleet' : 'Simulation');
    buildControls();
    buildLegend();
    resize();
  }

  function buildControls() {
    const box = $('controls');
    box.textContent = '';
    const ctl = new Set(S.info.controls || []);
    const group = () => { const g = el('div', 'group'); box.appendChild(g); return g; };
    if (ctl.has('pause')) {
      const g = group();
      const b = el('button', null, 'Pause'); b.id = 'btn-pause'; b.type = 'button';
      b.onclick = () => send({ cmd: b.dataset.paused === '1' ? 'resume' : 'pause' });
      g.appendChild(b);
      if (ctl.has('speed')) {
        const lab = el('label', null, 'Speed'); lab.htmlFor = 'sel-speed';
        const sel = el('select'); sel.id = 'sel-speed';
        for (const v of [0.5, 1, 2, 4, 8]) { const o = el('option', null, `${v}×`); o.value = v; if (v === 1) o.selected = true; sel.appendChild(o); }
        sel.onchange = () => send({ cmd: 'speed', value: Number(sel.value) });
        g.append(lab, sel);
      }
    }
    if (ctl.has('add_tasks')) {
      const g = group();
      const b = el('button', null, 'Add 5 tasks'); b.type = 'button';
      b.onclick = () => send({ cmd: 'add_tasks', n: 5 });
      g.appendChild(b);
    }
    if (ctl.has('block')) {
      const g = group();
      const b = el('button', null, 'Place or clear obstacle'); b.type = 'button'; b.id = 'btn-place';
      b.setAttribute('aria-pressed', 'false');
      b.onclick = () => {
        S.tool = S.tool === 'place' ? 'inspect' : 'place';
        b.setAttribute('aria-pressed', String(S.tool === 'place'));
        $('map-wrap').classList.toggle('placing', S.tool === 'place');
        $('map-hint').textContent = S.tool === 'place' ? 'Click a floor cell to block or unblock it' : 'Hover a robot, aisle or task for details';
      };
      g.appendChild(b);
    }
    if (ctl.has('reset')) {
      const g = group();
      const mk = (id, label, opts, val) => {
        const lab = el('label', null, label); lab.htmlFor = id;
        const sel = el('select'); sel.id = id;
        for (const [v, t] of opts) { const o = el('option', null, t); o.value = v; if (String(v) === String(val)) o.selected = true; sel.appendChild(o); }
        g.append(lab, sel);
        return sel;
      };
      const sc = mk('sel-scenario', 'Scenario', (S.info.scenarios || ['random']).map((s) => [s, s.replace('_', ' ')]), S.info.scenario);
      const me = mk('sel-method', 'Coordination', [['swarmx', 'SwarmX'], ['stopwait', 'Stop-and-wait']], S.info.method);
      const counts = [...new Set([3, 5, 8, 12, Number(S.info.robots) || 5])].sort((a, b) => a - b);
      const nr = mk('sel-robots', 'Robots', counts.map((n) => [n, String(n)]), S.info.robots);
      const b = el('button', null, 'Restart'); b.type = 'button';
      b.onclick = () => send({ cmd: 'reset', scenario: sc.value, method: me.value, robots: Number(nr.value) });
      g.appendChild(b);
    }
  }

  function legendSwatch(kind) {
    const s = document.createElementNS(SVGNS, 'svg'); s.setAttribute('viewBox', '0 0 16 16'); s.setAttribute('aria-hidden', 'true');
    const add = (tag, attrs) => { const e = document.createElementNS(SVGNS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); s.appendChild(e); return e; };
    const c = S.colors;
    const robot = (fill) => add('circle', { cx: 8, cy: 8, r: 5, fill, stroke: c.surface, 'stroke-width': 2 });
    switch (kind) {
      case 'work': robot(c.work); break;
      case 'carry': robot(c.carry); break;
      case 'charge': robot(c.charge); break;
      case 'idle': robot(c.idle); break;
      case 'wait': robot(c.work); add('circle', { cx: 8, cy: 8, r: 7, fill: 'none', stroke: c.ink2, 'stroke-width': 1.5 }); break;
      case 'failed': robot(c.critical); add('path', { d: 'M5.5 5.5l5 5M10.5 5.5l-5 5', stroke: '#fff', 'stroke-width': 1.6 }); break;
      case 'held': add('rect', { x: 5, y: 1, width: 6, height: 14, rx: 1, fill: c.zoneHeld }); break;
      case 'claim': add('rect', { x: 5.5, y: 1.5, width: 5, height: 13, rx: 1, fill: 'none', stroke: c.zoneClaim, 'stroke-width': 1.5 }); break;
      case 'blocked': add('rect', { x: 2, y: 2, width: 12, height: 12, rx: 2, fill: c.critical, 'fill-opacity': 0.2 });
        add('path', { d: 'M5 5l6 6M11 5l-6 6', stroke: c.critical, 'stroke-width': 1.8 }); break;
      case 'pickup': add('circle', { cx: 8, cy: 8, r: 4, fill: 'none', stroke: c.ink2, 'stroke-width': 2 }); break;
      case 'assigned': add('circle', { cx: 8, cy: 8, r: 4, fill: 'none', stroke: c.ink2, 'stroke-width': 2 }); add('circle', { cx: 8, cy: 8, r: 1.6, fill: c.ink2 }); break;
      case 'dropoff': add('rect', { x: 3, y: 3, width: 10, height: 10, rx: 2, fill: 'none', stroke: c.ink2, 'stroke-width': 1.5 });
        add('path', { d: 'M8 5v5M5.8 8 8 10.2 10.2 8', fill: 'none', stroke: c.ink2, 'stroke-width': 1.4 }); break;
      case 'charger': add('rect', { x: 3, y: 3, width: 10, height: 10, rx: 2, fill: 'none', stroke: c.ink2, 'stroke-width': 1.5 });
        add('path', { d: 'M9 4 5.8 8.6h2.4L7 12l3.2-4.6H7.8Z', fill: c.ink2 }); break;
      default: break;
    }
    return s;
  }
  function buildLegend() {
    const ul = $('legend');
    ul.textContent = '';
    const items = [['work', 'To pickup / picking'], ['carry', 'Carrying a load'], ['charge', 'Charging'], ['idle', 'Idle'],
      ['wait', 'Waiting for an aisle'], ['failed', 'Failed'], ['held', 'Aisle in use'], ['claim', 'Aisle requested'],
      ['blocked', 'Blocked cell'], ['pickup', 'Pickup, unassigned'], ['assigned', 'Pickup, assigned'], ['dropoff', 'Drop-off station'], ['charger', 'Charger']];
    for (const [k, t] of items) { const li = el('li'); li.appendChild(legendSwatch(k)); li.appendChild(el('span', null, t)); ul.appendChild(li); }
  }

  // ------------------------------------------------------------ snapshot
  function onSnapshot(s) {
    S.snap = s;
    S.recvAt = performance.now();
    S.t0 = s.t_start !== undefined ? s.t_start : (S.t0 === null ? s.t : S.t0);
    renderTiles();
    renderFleet();
    renderLog();
    if (!S._lastChart || performance.now() - S._lastChart > 900) { S._lastChart = performance.now(); renderChart(); }
    if (S.hover) showTip(S.hover.px, S.hover.py);
    const pb = $('btn-pause');
    if (pb && s.sim) { pb.dataset.paused = s.sim.paused ? '1' : '0'; pb.textContent = s.sim.paused ? 'Resume' : 'Pause'; }
  }

  function renderTiles() {
    const m = S.snap.metrics || {};
    $('m-delivered').textContent = fmtInt(m.delivered);
    const sim = S.snap.sim;
    $('m-delivered-sub').textContent = sim ? `of ${fmtInt(sim.tasks_total)} created` : `${fmtInt(m.assigned)} in progress`;
    $('m-throughput').textContent = `${fmtInt(m.throughput_per_min)}/min`;
    $('m-pending').textContent = fmtInt(m.pending);
    $('m-assigned').textContent = `${fmtInt(m.assigned)} assigned to robots`;
    $('m-robots').textContent = `${fmtInt(m.robots_alive)} of ${fmtInt(m.robots_total)}`;
    const failed = (S.snap.robots || []).filter((r) => r.st === 'failed').length;
    const lost = (S.snap.robots || []).filter((r) => !r.alive && r.st !== 'failed').length;
    const sub = $('m-robots-sub'); sub.textContent = '';
    if (failed || lost) {
      sub.className = 'sub status';
      sub.appendChild(icon('critical'));
      sub.appendChild(document.createTextNode([failed ? `${failed} failed` : '', lost ? `${lost} not heard` : ''].filter(Boolean).join(', ')));
    } else { sub.className = 'sub'; sub.textContent = 'all reporting'; }
    const b = m.battery_avg;
    $('m-battery').textContent = b === null || b === undefined ? '–' : `${Math.round(b * 100)}%`;
    const fill = $('m-battery-fill');
    fill.style.width = `${Math.round((b || 0) * 100)}%`;
    fill.parentElement.className = `meter${b < 0.15 ? ' crit' : b < 0.3 ? ' warn' : ''}`;
    const tile = $('tile-collisions');
    if (sim && sim.collisions !== undefined) {
      tile.hidden = false;
      $('m-collisions').textContent = fmtInt(sim.collisions);
      const cs = $('m-collisions-sub'); cs.textContent = '';
      if (sim.collisions === 0) { cs.appendChild(icon('good')); cs.appendChild(document.createTextNode(`none · closest ${sim.min_separation ? sim.min_separation.toFixed(2) + ' m' : '–'}`)); }
      else { cs.appendChild(icon('critical')); cs.appendChild(document.createTextNode('ground-truth contact detected')); }
    } else tile.hidden = true;
    $('m-msgs').textContent = `${fmtInt(m.msgs_per_s)} msg/s`;
    $('m-kbps').textContent = `${(m.kbytes_per_s || 0).toFixed(1)} kB/s across the fleet`;
  }

  function renderFleet() {
    const tb = document.querySelector('#fleet tbody');
    const robots = S.snap.robots || [];
    const canFail = (S.info.controls || []).includes('fail');
    $('fleet-actions-h').hidden = !canFail;
    $('fleet-hint').textContent = `${robots.length} robots · click a row to follow`;
    tb.textContent = '';
    if (!robots.length) { const tr = el('tr'); const td = el('td', 'empty', 'Waiting for robots to broadcast…'); td.colSpan = 7; tr.appendChild(td); tb.appendChild(tr); return; }
    for (const r of robots) {
      const tr = el('tr'); tr.tabIndex = 0;
      if (S.selected === r.id) tr.className = 'sel';
      const sel = () => { S.selected = S.selected === r.id ? null : r.id; renderFleet(); draw(); };
      tr.onclick = sel; tr.onkeydown = (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); sel(); } };
      const c1 = el('td'); const d1 = el('div', 'cell-state'); const k = el('span', 'key'); k.style.background = colorOf(catOf(r));
      d1.append(k, el('span', null, r.id)); c1.appendChild(d1);
      const c2 = el('td'); const d2 = el('div', 'cell-state');
      if (r.st === 'failed' || !r.alive) d2.appendChild(icon('critical'));
      d2.appendChild(el('span', null, stateLabel(r)));
      if (r.wait && r.alive) { const w = el('span', 'tag', 'waits'); w.title = `Waiting for aisle ${r.wait}`; d2.appendChild(w); }
      c2.appendChild(d2);
      const c3 = el('td', r.task ? null : 'muted', r.task || '–');
      const c4 = el('td'); const bt = el('div', 'batt'); const mtr = el('div', 'meter'); const f = el('div', 'fill');
      const bat = r.bat || 0; f.style.width = `${Math.round(bat * 100)}%`;
      mtr.className = `meter${bat < 0.15 ? ' crit' : bat < 0.3 ? ' warn' : ''}`; mtr.appendChild(f); bt.appendChild(mtr);
      if (bat < 0.3) bt.appendChild(icon(bat < 0.15 ? 'critical' : 'warning'));
      bt.appendChild(el('span', null, `${Math.round(bat * 100)}%`)); c4.appendChild(bt);
      const c5 = el('td', 'num', fmtInt(r.stats ? r.stats.done : null));
      const c6 = el('td', `num${r.alive ? '' : ' muted'}`, fmtAge(r.age));
      tr.append(c1, c2, c3, c4, c5, c6);
      if (canFail) {
        const c7 = el('td');
        const failed = r.st === 'failed';
        const b = el('button', 'small', failed ? 'Recover' : 'Fail'); b.type = 'button';
        b.setAttribute('aria-label', `${failed ? 'Recover' : 'Inject failure into'} ${r.id}`);
        b.onclick = (e) => { e.stopPropagation(); send({ cmd: failed ? 'recover' : 'fail', robot: r.id }); };
        c7.appendChild(b); tr.appendChild(c7);
      }
      tb.appendChild(tr);
    }
  }

  function renderLog() {
    const ol = $('log');
    const evs = [...((S.snap && S.snap.events) || []), ...localEvents].sort((a, b) => b.t - a.t).slice(0, 60);
    ol.textContent = '';
    if (!evs.length) { ol.appendChild(el('li', 'empty', 'No events yet')); return; }
    for (const e of evs) {
      const li = el('li');
      li.appendChild(el('span', 'when', fmtClock(e.t)));
      const kind = e.kind === 'done' ? 'good' : e.kind === 'alert' ? 'warning' : e.kind === 'critical' ? 'critical' : 'info';
      const ic = icon(kind); if (kind === 'info') ic.style.fill = S.colors.ink3; li.appendChild(ic);
      li.appendChild(el('span', null, e.text));
      ol.appendChild(li);
    }
  }

  // ----------------------------------------------------------------- map
  const canvas = $('map');
  const ctx = canvas.getContext('2d');
  let cell = 20;

  function resize() {
    if (!S.world) return;
    const wrap = $('map-wrap');
    const w = wrap.clientWidth;
    cell = Math.max(8, w / S.world.width);
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(S.world.height * cell * dpr);
    canvas.style.height = `${S.world.height * cell}px`;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    draw();
  }
  window.addEventListener('resize', resize);

  const X = (x) => x * cell;
  const Y = (y) => (S.world.height - y) * cell;

  function zoneState() {
    const st = {};
    for (const r of (S.snap ? S.snap.robots : [])) {
      if (!r.alive && r.st !== 'failed') continue;
      for (const z of (r.zones || [])) {
        const e = st[z[0]] || (st[z[0]] = { held: [], claims: [] });
        (z[3] === 'hold' ? e.held : e.claims).push({ id: r.id, mode: z[1] });
      }
    }
    return st;
  }

  function robotPos(r) {
    const dt = Math.min((performance.now() - S.recvAt) / 1000, 0.15);
    const moving = r.alive && r.st !== 'failed';
    return [r.x + (moving ? (r.vx || 0) * dt : 0), r.y + (moving ? (r.vy || 0) * dt : 0)];
  }

  function draw() {
    if (!S.world || !S.colors.floor) return;
    const c = S.colors, w = S.world, W = w.width * cell, H = w.height * cell;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = c.floor; ctx.fillRect(0, 0, W, H);

    // aisles (zones)
    const zs = zoneState();
    for (const [zid, cells] of Object.entries(w.zones || {})) {
      const xs = cells.map((p) => p[0]), ys = cells.map((p) => p[1]);
      const x0 = Math.min(...xs), x1 = Math.max(...xs) + 1, y0 = Math.min(...ys), y1 = Math.max(...ys) + 1;
      const s = zs[zid];
      if (s && s.held.length) {
        ctx.fillStyle = c.zoneHeld; ctx.fillRect(X(x0), Y(y1), (x1 - x0) * cell, (y1 - y0) * cell);
        // travel direction arrows for pass-through convoys
        const dirs = new Set(s.held.map((h) => h.mode));
        ctx.fillStyle = c.ink2; ctx.font = `${Math.max(9, cell * 0.5)}px system-ui, sans-serif`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        const glyph = dirs.size === 1 ? ({ SN: '↑', NS: '↓', SS: '⇅', NN: '⇅' }[[...dirs][0]] || '') : '';
        if (glyph) ctx.fillText(glyph, X(x0 + 0.5), Y((y0 + y1) / 2));
      } else if (s && s.claims.length) {
        ctx.strokeStyle = c.zoneClaim; ctx.lineWidth = 1.5;
        ctx.strokeRect(X(x0) + 1.5, Y(y1) + 1.5, (x1 - x0) * cell - 3, (y1 - y0) * cell - 3);
      }
    }
    // shelves and walls
    for (let y = 0; y < w.height; y++) {
      for (let x = 0; x < w.width; x++) {
        const v = w.grid[y][x];
        if (!v) continue;
        ctx.fillStyle = v === 1 ? c.shelf : c.wall;
        ctx.fillRect(X(x), Y(y + 1), cell + 0.5, cell + 0.5);
      }
    }
    // stations
    const station = (p, kind) => {
      const x = X(p[0] + 0.5), y = Y(p[1] + 0.5), s = cell * 0.36;
      ctx.strokeStyle = c.ink2; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.roundRect(x - s, y - s, 2 * s, 2 * s, 3); ctx.stroke();
      ctx.fillStyle = c.ink2; ctx.font = `600 ${Math.max(9, cell * 0.42)}px system-ui, sans-serif`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(kind === 'drop' ? 'D' : '⚡', x, y + 0.5);
    };
    (w.dropoffs || []).forEach((p) => station(p, 'drop'));
    (w.chargers || []).forEach((p) => station(p, 'charge'));

    // blocked cells
    for (const b of (S.snap ? S.snap.blocked : [])) {
      const x = X(b[0]), y = Y(b[1] + 1);
      ctx.fillStyle = c.critical; ctx.globalAlpha = 0.2; ctx.fillRect(x, y, cell, cell); ctx.globalAlpha = 1;
      ctx.strokeStyle = c.critical; ctx.lineWidth = 2;
      const m = cell * 0.25;
      ctx.beginPath(); ctx.moveTo(x + m, y + m); ctx.lineTo(x + cell - m, y + cell - m); ctx.moveTo(x + cell - m, y + m); ctx.lineTo(x + m, y + cell - m); ctx.stroke();
    }
    // tasks
    for (const t of (S.snap ? S.snap.tasks : [])) {
      const x = X(t.pickup[0] + 0.5), y = Y(t.pickup[1] + 0.5), r = Math.max(4, cell * 0.2);
      ctx.strokeStyle = c.ink2; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.stroke();
      if (t.state === 'assigned') { ctx.fillStyle = c.ink2; ctx.beginPath(); ctx.arc(x, y, r * 0.4, 0, Math.PI * 2); ctx.fill(); }
    }
    const robots = S.snap ? S.snap.robots : [];
    // intent paths
    for (const r of robots) {
      if (!r.path || r.path.length < 2 || !r.alive || r.st === 'failed') continue;
      const sel = S.selected === r.id;
      ctx.strokeStyle = colorOf(catOf(r)); ctx.globalAlpha = sel ? 0.95 : 0.45; ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      const [px, py] = robotPos(r);
      ctx.beginPath(); ctx.moveTo(X(px), Y(py));
      for (const p of r.path) ctx.lineTo(X(p[0] + 0.5), Y(p[1] + 0.5));
      ctx.stroke(); ctx.globalAlpha = 1;
    }
    // robots
    const rad = Math.max(5, cell * 0.3);
    for (const r of robots) {
      const [px, py] = robotPos(r);
      const x = X(px), y = Y(py), cat = catOf(r);
      if (S.selected === r.id) {
        ctx.strokeStyle = c.ink1; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, rad + 6, 0, Math.PI * 2); ctx.stroke();
      }
      if (r.wait && r.alive) {
        ctx.strokeStyle = c.ink2; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x, y, rad + 3.5, 0, Math.PI * 2); ctx.stroke();
      }
      ctx.fillStyle = colorOf(cat);
      ctx.strokeStyle = c.surface; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, rad, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
      if (cat === 'failed' || cat === 'lost') {
        const m = rad * 0.45;
        ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.8;
        ctx.beginPath(); ctx.moveTo(x - m, y - m); ctx.lineTo(x + m, y + m); ctx.moveTo(x + m, y - m); ctx.lineTo(x - m, y + m); ctx.stroke();
      } else {
        const th = r.th || 0;
        ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(th) * rad * 0.9, y - Math.sin(th) * rad * 0.9); ctx.stroke();
      }
      // identity label (text ink, never the state color)
      const label = r.id.replace(/^robot/, 'R');
      ctx.font = `600 ${Math.max(10, cell * 0.42)}px system-ui, sans-serif`; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      ctx.lineWidth = 3; ctx.strokeStyle = c.surface; ctx.strokeText(label, x + rad + 3, y - rad);
      ctx.fillStyle = c.ink1; ctx.fillText(label, x + rad + 3, y - rad);
    }
  }
  function frame() { draw(); requestAnimationFrame(frame); }

  // ------------------------------------------------------------- tooltip
  function hitTest(wx, wy) {
    if (!S.snap) return null;
    let best = null, bd = 0.65;
    for (const r of S.snap.robots) {
      const [px, py] = robotPos(r);
      const d = Math.hypot(px - wx, py - wy);
      if (d < bd) { bd = d; best = { kind: 'robot', r }; }
    }
    if (best) return best;
    for (const t of S.snap.tasks) {
      if (Math.hypot(t.pickup[0] + 0.5 - wx, t.pickup[1] + 0.5 - wy) < 0.5) return { kind: 'task', t };
    }
    const cx = Math.floor(wx), cy = Math.floor(wy);
    if ((S.snap.blocked || []).some((b) => b[0] === cx && b[1] === cy)) return { kind: 'blocked', cell: [cx, cy] };
    for (const [zid, cells] of Object.entries(S.world.zones || {})) {
      if (cells.some((p) => p[0] === cx && p[1] === cy)) return { kind: 'zone', zid };
    }
    return null;
  }
  function tipRows(tip, rows) {
    for (const [k, v] of rows) { const d = el('div', 't-row'); d.append(el('span', null, k), el('span', null, v)); tip.appendChild(d); }
  }
  function showTip(px, py) {
    const tip = $('tip');
    const wx = px / cell, wy = S.world.height - py / cell;
    const h = hitTest(wx, wy);
    if (!h || S.tool === 'place') { tip.hidden = true; canvas.style.cursor = S.tool === 'place' ? 'crosshair' : 'default'; return; }
    canvas.style.cursor = h.kind === 'robot' ? 'pointer' : 'default';
    tip.textContent = '';
    const head = el('div', 't-head');
    if (h.kind === 'robot') {
      const r = h.r; const k = el('span', 'key'); k.style.background = colorOf(catOf(r));
      head.append(k, el('span', null, r.id)); tip.appendChild(head);
      const sp = Math.hypot(r.vx || 0, r.vy || 0);
      tipRows(tip, [['State', stateLabel(r)], ['Waiting for', r.wait ? `aisle ${r.wait}` : '–'], ['Task', r.task || '–'], ['Battery', `${Math.round((r.bat || 0) * 100)}%`],
        ['Speed', `${sp.toFixed(2)} m/s`], ['Queue', (r.bundle || []).length ? r.bundle.join(', ') : '–'],
        ['Delivered', fmtInt(r.stats && r.stats.done)], ['Waited', `${r.stats ? r.stats.wait : 0} s`],
        ['Re-routes', fmtInt(r.stats && r.stats.reroutes)], ['Last heard', fmtAge(r.age)]]);
    } else if (h.kind === 'task') {
      head.appendChild(el('span', null, `Task ${h.t.id}`)); tip.appendChild(head);
      tipRows(tip, [['Pickup', `(${h.t.pickup.join(', ')})`], ['Drop-off', `(${h.t.dropoff.join(', ')})`], ['Assigned to', h.t.robot || 'nobody yet']]);
    } else if (h.kind === 'zone') {
      const s = zoneState()[h.zid] || { held: [], claims: [] };
      head.appendChild(el('span', null, `Aisle ${h.zid}`)); tip.appendChild(head);
      const dir = { SN: 'northbound', NS: 'southbound', SS: 'in and out (south)', NN: 'in and out (north)', XX: 'exclusive' };
      tipRows(tip, [['In use by', s.held.length ? s.held.map((x) => `${x.id} ${dir[x.mode] || ''}`).join(', ') : 'free'],
        ['Requested by', s.claims.length ? s.claims.map((x) => x.id).join(', ') : '–']]);
    } else {
      head.append(icon('critical'), el('span', null, 'Blocked cell')); tip.appendChild(head);
      tipRows(tip, [['Cell', `(${h.cell.join(', ')})`], ['Status', 'robots route around it']]);
    }
    tip.hidden = false;
    const wrapW = $('map-wrap').clientWidth;
    const tw = tip.offsetWidth, th2 = tip.offsetHeight;
    let left = px + 14, top = py + 14;
    if (left + tw > wrapW) left = px - tw - 14;
    if (top + th2 > canvas.clientHeight) top = Math.max(0, py - th2 - 14);
    tip.style.left = `${Math.max(0, left)}px`; tip.style.top = `${top}px`;
  }
  canvas.addEventListener('pointermove', (e) => {
    const rect = canvas.getBoundingClientRect();
    S.hover = { px: e.clientX - rect.left, py: e.clientY - rect.top };
    if (S.world) showTip(S.hover.px, S.hover.py);
  });
  canvas.addEventListener('pointerleave', () => { S.hover = null; $('tip').hidden = true; });
  canvas.addEventListener('click', (e) => {
    if (!S.world) return;
    const rect = canvas.getBoundingClientRect();
    const wx = (e.clientX - rect.left) / cell, wy = S.world.height - (e.clientY - rect.top) / cell;
    if (S.tool === 'place') {
      const cx = Math.floor(wx), cy = Math.floor(wy);
      if (S.world.grid[cy] && S.world.grid[cy][cx] === 0) send({ cmd: 'toggle_block', cell: [cx, cy] });
      return;
    }
    const h = hitTest(wx, wy);
    if (h && h.kind === 'robot') { S.selected = S.selected === h.r.id ? null : h.r.id; renderFleet(); }
  });

  // --------------------------------------------------------------- chart
  function niceStep(max, target) {
    const raw = Math.max(max, 1) / target, mag = Math.pow(10, Math.floor(Math.log10(raw)));
    for (const m of [1, 2, 5, 10]) if (raw <= m * mag) return m * mag;
    return 10 * mag;
  }
  function renderChart() {
    const box = $('chart');
    const tl = (S.snap && S.snap.timeline) || [];
    box.textContent = '';
    if (tl.length < 2) { box.appendChild(el('div', 'empty', 'Collecting data…')); return; }
    const c = S.colors, W = Math.max(260, box.clientWidth), Hh = 190, m = { l: 40, r: 36, t: 12, b: 26 };
    const t0 = S.t0 !== null ? S.t0 : tl[0][0];
    const xs = tl.map((p) => p[0] - t0), ys = tl.map((p) => p[1]);
    const xMax = Math.max(60, xs[xs.length - 1]), yMax = Math.max(5, ...ys);
    const yStep = niceStep(yMax, 4), yTop = Math.ceil(yMax / yStep) * yStep;
    const xStep = [10, 15, 30, 60, 120, 300, 600, 900, 1800, 3600].find((v) => xMax / v <= 6) || 3600;
    const tLabel = (v) => `${Math.floor(v / 60)}:${String(Math.round(v % 60)).padStart(2, '0')}`;
    const sx = (v) => m.l + (v / xMax) * (W - m.l - m.r), sy = (v) => Hh - m.b - (v / yTop) * (Hh - m.t - m.b);
    const svg = document.createElementNS(SVGNS, 'svg');
    svg.setAttribute('viewBox', `0 0 ${W} ${Hh}`); svg.setAttribute('height', Hh); svg.setAttribute('role', 'presentation');
    const add = (tag, attrs, text) => { const e = document.createElementNS(SVGNS, tag); for (const k in attrs) e.setAttribute(k, attrs[k]); if (text !== undefined) e.textContent = text; svg.appendChild(e); return e; };
    for (let v = 0; v <= yTop + 1e-9; v += yStep) {
      add('line', { x1: m.l, x2: W - m.r, y1: sy(v), y2: sy(v), stroke: v === 0 ? c.axis : c.grid, 'stroke-width': 1 });
      add('text', { x: m.l - 6, y: sy(v) + 4, 'text-anchor': 'end', class: 'axis-text' }, fmtInt(v));
    }
    for (let v = 0; v <= xMax + 1e-9; v += xStep) add('text', { x: sx(v), y: Hh - 8, 'text-anchor': v === 0 ? 'start' : 'middle', class: 'axis-text' }, tLabel(v));
    const d = xs.map((x, i) => `${i ? 'L' : 'M'}${sx(x).toFixed(1)},${sy(ys[i]).toFixed(1)}`).join('');
    add('path', { d: `${d}L${sx(xs[xs.length - 1]).toFixed(1)},${sy(0)}L${sx(0)},${sy(0)}Z`, fill: c.work, 'fill-opacity': 0.1, stroke: 'none' });
    add('path', { d, fill: 'none', stroke: c.work, 'stroke-width': 2, 'stroke-linejoin': 'round', 'stroke-linecap': 'round' });
    const lx = sx(xs[xs.length - 1]), ly = sy(ys[ys.length - 1]);
    add('circle', { cx: lx, cy: ly, r: 4, fill: c.work, stroke: c.surface, 'stroke-width': 2 });
    add('text', { x: lx + 8, y: ly + 4, class: 'end-label' }, fmtInt(ys[ys.length - 1]));
    const cross = add('line', { y1: m.t, y2: Hh - m.b, stroke: c.axis, 'stroke-width': 1, visibility: 'hidden' });
    const dot = add('circle', { r: 4, fill: c.work, stroke: c.surface, 'stroke-width': 2, visibility: 'hidden' });
    box.appendChild(svg);
    const tip = el('div', 'tooltip'); tip.hidden = true; box.appendChild(tip);
    const show = (i) => {
      if (i === null || i < 0 || i >= xs.length) { cross.setAttribute('visibility', 'hidden'); dot.setAttribute('visibility', 'hidden'); tip.hidden = true; return; }
      const x = sx(xs[i]), y = sy(ys[i]);
      cross.setAttribute('x1', x); cross.setAttribute('x2', x); cross.setAttribute('visibility', 'visible');
      dot.setAttribute('cx', x); dot.setAttribute('cy', y); dot.setAttribute('visibility', 'visible');
      tip.textContent = '';
      const row = el('div', 't-row'); row.append(el('span', null, fmtClock(tl[i][0])), el('span', null, `${fmtInt(ys[i])} delivered`)); tip.appendChild(row);
      tip.hidden = false;
      const scale = box.clientWidth / W;
      let left = x * scale + 12; if (left + tip.offsetWidth > box.clientWidth) left = x * scale - tip.offsetWidth - 12;
      tip.style.left = `${left}px`; tip.style.top = `${Math.max(0, y * scale - 20)}px`;
    };
    const nearest = (clientX) => {
      const rect = svg.getBoundingClientRect(); const vx = (clientX - rect.left) * (W / rect.width);
      const tv = ((vx - m.l) / (W - m.l - m.r)) * xMax;
      let bi = 0, bd = Infinity; xs.forEach((x, i) => { const dd = Math.abs(x - tv); if (dd < bd) { bd = dd; bi = i; } });
      return bi;
    };
    svg.addEventListener('pointermove', (e) => { S.chartIdx = nearest(e.clientX); show(S.chartIdx); });
    svg.addEventListener('pointerleave', () => { S.chartIdx = null; show(null); });
    box.onkeydown = (e) => {
      if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return;
      e.preventDefault();
      S.chartIdx = S.chartIdx === null ? xs.length - 1 : Math.max(0, Math.min(xs.length - 1, S.chartIdx + (e.key === 'ArrowRight' ? 1 : -1)));
      show(S.chartIdx);
    };
    box.onblur = () => { S.chartIdx = null; show(null); };
    if (S.chartIdx !== null) show(Math.min(S.chartIdx, xs.length - 1));
    // table twin
    const tb = $('chart-table'); tb.textContent = '';
    const stride = Math.max(1, Math.floor(tl.length / 30));
    for (let i = 0; i < tl.length; i += stride) {
      const tr = el('tr'); tr.append(el('td', null, fmtClock(tl[i][0])), el('td', 'num', fmtInt(tl[i][1]))); tb.appendChild(tr);
    }
  }
  $('chart-toggle').onclick = () => {
    S.showTable = !S.showTable;
    $('chart-toggle').setAttribute('aria-pressed', String(S.showTable));
    $('chart-toggle').textContent = S.showTable ? 'Chart view' : 'Table view';
    $('chart').hidden = S.showTable; $('chart-table-wrap').hidden = !S.showTable;
  };

  // ---------------------------------------------------------------- boot
  readColors();
  buildLegend();
  connect();
  requestAnimationFrame(frame);
})();
