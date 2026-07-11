/**
 * Floor-plan canvas: rooms, anchors, zoom/pan, adaptive snap, room-to-room snap.
 */
(function (global) {
  function snap(v, step) {
    if (!step || step <= 0) return v;
    return Math.round(v / step) * step;
  }

  function createMap(canvas, opts) {
    const ctx = canvas.getContext('2d');
    const state = {
      // world AABB of layout (full floor)
      world: { min_x: -0.5, min_y: -0.5, max_x: 5.5, max_y: 4.5, width: 6, height: 5 },
      // visible viewport in world metres
      view: { min_x: -0.5, min_y: -0.5, width: 6, height: 5 },
      rooms: [],
      anchors: [],
      gridSnap: 0.5,        // base snap at zoom 1
      showGrid: true,
      pad: 8,               // small edge inset only — grid fills the canvas
      trail: [],
      tag: null,
      raw: null,
      distances: {},
      used: new Set(),
      interactive: !!opts.interactive,
      dragRooms: !!opts.dragRooms,
      onAnchorMove: opts.onAnchorMove || null,
      onRoomMove: opts.onRoomMove || null,
      onSelect: opts.onSelect || null,
      onSelectRoom: opts.onSelectRoom || null,
      onZoomChange: opts.onZoomChange || null,
      selectedId: null,
      selectedRoomId: null,
      drag: null, // { type:'anchor'|'room'|'pan', ... }
      zoom: 1,    // 1 = fit world; higher = closer
      minZoom: 1,
      maxZoom: 12,
      cssW: 720,
      cssH: 480,
    };

    function effectiveSnap() {
      // Finer snap when zoomed in (down to 5 cm)
      const base = state.gridSnap || 0.5;
      const z = Math.max(1, state.zoom);
      let step = base / z;
      // nice steps
      const nice = [1, 0.5, 0.25, 0.1, 0.05];
      for (let i = 0; i < nice.length; i++) {
        if (step >= nice[i] * 0.75) return nice[i];
      }
      return 0.05;
    }

    /** Expand view so world projection fills the full canvas (no letterbox bars). */
    function expandViewToCanvas() {
      const p = state.pad;
      const drawW = Math.max(1, state.cssW - 2 * p);
      const drawH = Math.max(1, state.cssH - 2 * p);
      const canvasAspect = drawW / drawH;
      const viewAspect = state.view.width / Math.max(state.view.height, 1e-9);
      if (canvasAspect > viewAspect) {
        // canvas wider → grow world width
        const newW = state.view.height * canvasAspect;
        const cx = state.view.min_x + state.view.width / 2;
        state.view.min_x = cx - newW / 2;
        state.view.width = newW;
      } else if (canvasAspect < viewAspect) {
        // canvas taller → grow world height
        const newH = state.view.width / canvasAspect;
        const cy = state.view.min_y + state.view.height / 2;
        state.view.min_y = cy - newH / 2;
        state.view.height = newH;
      }
    }

    function clampView() {
      const w = state.world;
      // keep viewport within expanded world (+ small margin)
      const margin = 2;
      const maxW = w.width + margin * 2;
      const maxH = w.height + margin * 2;
      state.view.width = Math.min(state.view.width, maxW);
      state.view.height = Math.min(state.view.height, maxH);
      const minX = w.min_x - margin;
      const minY = w.min_y - margin;
      const maxX = w.max_x + margin;
      const maxY = w.max_y + margin;
      state.view.min_x = Math.min(Math.max(state.view.min_x, minX), maxX - state.view.width);
      state.view.min_y = Math.min(Math.max(state.view.min_y, minY), maxY - state.view.height);
    }

    function applyZoom(newZoom, focusPx, focusPy) {
      const oldZ = state.zoom;
      state.zoom = Math.min(state.maxZoom, Math.max(state.minZoom, newZoom));
      if (state.zoom === oldZ) return;

      // world point under cursor before zoom
      let wx, wy;
      if (focusPx != null) {
        wx = ix(focusPx);
        wy = iy(focusPy);
      } else {
        wx = state.view.min_x + state.view.width / 2;
        wy = state.view.min_y + state.view.height / 2;
      }

      state.view.width = state.world.width / state.zoom;
      state.view.height = state.world.height / state.zoom;
      // keep focus point under same pixel
      if (focusPx != null) {
        const p = state.pad;
        const fx = (focusPx - p) / (state.cssW - 2 * p);
        const fy = 1 - (focusPy - p) / (state.cssH - 2 * p);
        state.view.min_x = wx - fx * state.view.width;
        state.view.min_y = wy - fy * state.view.height;
      } else {
        state.view.min_x = wx - state.view.width / 2;
        state.view.min_y = wy - state.view.height / 2;
      }
      clampView();
      draw();
      if (state.onZoomChange) {
        state.onZoomChange({ zoom: state.zoom, snap: effectiveSnap() });
      }
    }

    function fitWorld() {
      state.zoom = 1;
      state.view = {
        min_x: state.world.min_x,
        min_y: state.world.min_y,
        width: state.world.width,
        height: state.world.height,
      };
      clampView();
      draw();
      if (state.onZoomChange) {
        state.onZoomChange({ zoom: state.zoom, snap: effectiveSnap() });
      }
    }

    function resize() {
      const parent = canvas.parentElement;
      // Fill the map panel completely (width + height)
      const w = Math.max(320, parent ? parent.clientWidth : 720);
      let h = 480;
      if (parent) {
        const rect = parent.getBoundingClientRect();
        // Prefer parent height if set; otherwise a tall fill of the editor
        h = Math.max(360, Math.round(rect.height || parent.clientHeight || 480));
        if (h < 200) h = Math.min(Math.max(Math.round(w * 0.65), 360), 620);
      }
      state.cssW = w;
      state.cssH = h;
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(h * dpr);
      canvas.style.width = '100%';
      canvas.style.height = h + 'px';
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      expandViewToCanvas();
      draw();
    }

    function sx(x) {
      const p = state.pad;
      return p + ((x - state.view.min_x) / state.view.width) * (state.cssW - 2 * p);
    }
    function sy(y) {
      const p = state.pad;
      return state.cssH - p - ((y - state.view.min_y) / state.view.height) * (state.cssH - 2 * p);
    }
    function ix(px) {
      const p = state.pad;
      return state.view.min_x + ((px - p) / (state.cssW - 2 * p)) * state.view.width;
    }
    function iy(py) {
      const p = state.pad;
      return state.view.min_y + ((state.cssH - p - py) / (state.cssH - 2 * p)) * state.view.height;
    }

    function setConfig(viewData) {
      if (!viewData) return;
      const aabb = viewData.aabb;
      if (aabb) {
        state.world = {
          min_x: aabb.min_x, min_y: aabb.min_y,
          max_x: aabb.max_x, max_y: aabb.max_y,
          width: aabb.width, height: aabb.height,
        };
        // only reset view if not already zoomed by user, or first load
        if (!state._userZoomed) {
          state.view = {
            min_x: aabb.min_x, min_y: aabb.min_y,
            width: aabb.width, height: aabb.height,
          };
          state.zoom = 1;
        }
      }
      const cfg = viewData.config || viewData;
      if (cfg.rooms) state.rooms = cfg.rooms.map((r) => Object.assign({}, r));
      if (cfg.anchors) state.anchors = cfg.anchors.map((a) => Object.assign({}, a));
      if (cfg.grid_snap_m != null) state.gridSnap = cfg.grid_snap_m;
      if (cfg.map && cfg.map.show_grid != null) state.showGrid = cfg.map.show_grid;
      clampView();
      resize();
    }

    function setLive(d) {
      if (!d) return;
      if (d.aabb && !state._userZoomed) {
        state.world = {
          min_x: d.aabb.min_x, min_y: d.aabb.min_y,
          max_x: d.aabb.max_x, max_y: d.aabb.max_y,
          width: d.aabb.width, height: d.aabb.height,
        };
        state.view = {
          min_x: d.aabb.min_x, min_y: d.aabb.min_y,
          width: d.aabb.width, height: d.aabb.height,
        };
      }
      state.tag = { x: d.x, y: d.y };
      state.raw = { x: d.raw_x, y: d.raw_y };
      state.distances = d.distances || {};
      state.used = new Set((d.used_anchors || []).map(Number));
      if (d.x != null) {
        state.trail.push({ x: d.x, y: d.y });
        if (state.trail.length > 80) state.trail.shift();
      }
      draw();
    }

    /** Snap room origin so edges align with other rooms or sit at snap-gap. */
    function snapRoomToRooms(room, ox, oy) {
      const step = effectiveSnap();
      let nx = snap(ox, step);
      let ny = snap(oy, step);
      const thr = Math.max(step * 1.1, 0.08);
      const gaps = [0]; // 0 = adjacent (touching)
      // also allow multi-step gaps
      for (let g = step; g <= 5 + 1e-9; g += step) gaps.push(+g.toFixed(4));

      const candidatesX = [nx];
      const candidatesY = [ny];

      (state.rooms || []).forEach((other) => {
        if (other.id === room.id) return;
        const oL = other.origin_x;
        const oR = other.origin_x + other.width;
        const oB = other.origin_y;
        const oT = other.origin_y + other.height;
        // this room edges for origin (ox, oy)
        // left = ox, right = ox+w, bottom = oy, top = oy+h
        gaps.forEach((g) => {
          // this.left to other.right + g  => ox = oR + g
          candidatesX.push(oR + g);
          // this.left to other.left
          candidatesX.push(oL);
          // this.right to other.left - g => ox + w = oL - g => ox = oL - g - w
          candidatesX.push(oL - g - room.width);
          // this.right to other.right
          candidatesX.push(oR - room.width);
          // vertical
          candidatesY.push(oT + g);
          candidatesY.push(oB);
          candidatesY.push(oB - g - room.height);
          candidatesY.push(oT - room.height);
        });
      });

      let bestX = nx, bestXd = Infinity;
      candidatesX.forEach((c) => {
        const d = Math.abs(c - ox);
        if (d < bestXd && d <= thr * 2.5) { bestXd = d; bestX = c; }
      });
      let bestY = ny, bestYd = Infinity;
      candidatesY.forEach((c) => {
        const d = Math.abs(c - oy);
        if (d < bestYd && d <= thr * 2.5) { bestYd = d; bestY = c; }
      });

      // Prefer pure grid if no room snap close enough
      if (bestXd > thr * 2) bestX = nx;
      if (bestYd > thr * 2) bestY = ny;
      // Final grid quantize lightly so we stay on snap lattice when adjacent
      bestX = snap(bestX, step / 2) ; // allow half-step for touch alignments
      bestY = snap(bestY, step / 2);
      // if nearly on full step, snap to full step
      if (Math.abs(bestX - snap(bestX, step)) < 1e-6) bestX = snap(bestX, step);
      if (Math.abs(bestY - snap(bestY, step)) < 1e-6) bestY = snap(bestY, step);
      return { x: bestX, y: bestY };
    }

    function draw() {
      const W = state.cssW, H = state.cssH, p = state.pad;
      expandViewToCanvas();
      ctx.clearRect(0, 0, W, H);
      // Full-canvas background (no empty black gutters outside the grid)
      ctx.fillStyle = '#0a0e0b';
      ctx.fillRect(0, 0, W, H);

      const step = effectiveSnap();
      if (state.showGrid) {
        // Grid covers the entire canvas (edge to edge)
        const min_x = ix(0), max_x = ix(W);
        const min_y = iy(H), max_y = iy(0); // y flips in screen space
        const xLo = Math.min(min_x, max_x), xHi = Math.max(min_x, max_x);
        const yLo = Math.min(min_y, max_y), yHi = Math.max(min_y, max_y);
        let drawStep = step;
        const maxLines = 160;
        if ((xHi - xLo) / drawStep > maxLines) {
          drawStep = step * Math.ceil(((xHi - xLo) / step) / maxLines);
        }
        ctx.strokeStyle = '#1e2a22';
        ctx.lineWidth = 1;
        const x0 = Math.floor(xLo / drawStep) * drawStep;
        const y0 = Math.floor(yLo / drawStep) * drawStep;
        for (let x = x0; x <= xHi + 1e-9; x += drawStep) {
          const px = sx(x);
          ctx.beginPath();
          ctx.moveTo(px, 0);
          ctx.lineTo(px, H);
          ctx.stroke();
        }
        for (let y = y0; y <= yHi + 1e-9; y += drawStep) {
          const py = sy(y);
          ctx.beginPath();
          ctx.moveTo(0, py);
          ctx.lineTo(W, py);
          ctx.stroke();
        }
      }

      (state.rooms || []).forEach((r) => {
        const x = sx(r.origin_x), y = sy(r.origin_y + r.height);
        const w = sx(r.origin_x + r.width) - sx(r.origin_x);
        const h = sy(r.origin_y) - sy(r.origin_y + r.height);
        const sel = state.selectedRoomId === r.id;
        ctx.fillStyle = sel ? 'rgba(57,255,20,0.10)' : 'rgba(57,255,20,0.04)';
        ctx.strokeStyle = sel ? '#39ff14' : '#2a3d30';
        ctx.lineWidth = sel ? 2.5 : 2;
        ctx.fillRect(x, y, w, h);
        ctx.strokeRect(x, y, w, h);
        ctx.fillStyle = '#6f8175';
        ctx.font = '11px "JetBrainsMono Nerd Font", monospace';
        ctx.textAlign = 'left';
        ctx.fillText(`${r.name} (${Number(r.width).toFixed(2)}×${Number(r.height).toFixed(2)} m)`, x + 6, y + 14);
      });

      Object.keys(state.distances || {}).forEach((id) => {
        const a = (state.anchors || []).find((x) => String(x.id) === String(id));
        if (!a || a.enabled === false) return;
        const dist = state.distances[id];
        if (!dist) return;
        const used = state.used.has(Number(id));
        ctx.strokeStyle = used ? 'rgba(57,255,20,0.22)' : 'rgba(111,129,117,0.15)';
        ctx.lineWidth = 1.5;
        const rx = (dist / state.view.width) * (W - 2 * p);
        const ry = (dist / state.view.height) * (H - 2 * p);
        ctx.beginPath();
        ctx.ellipse(sx(a.x), sy(a.y), rx, ry, 0, 0, Math.PI * 2);
        ctx.stroke();
      });

      if (state.trail.length > 1) {
        ctx.strokeStyle = 'rgba(57,255,20,0.35)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        state.trail.forEach((pt, i) => {
          if (i === 0) ctx.moveTo(sx(pt.x), sy(pt.y));
          else ctx.lineTo(sx(pt.x), sy(pt.y));
        });
        ctx.stroke();
      }

      (state.anchors || []).forEach((a) => {
        if (a.enabled === false) ctx.globalAlpha = 0.35;
        const ax = sx(a.x), ay = sy(a.y);
        const sel = state.selectedId === a.id;
        ctx.beginPath();
        ctx.arc(ax, ay, sel ? 11 : 9, 0, Math.PI * 2);
        ctx.fillStyle = '#ff5252';
        ctx.fill();
        if (sel) {
          ctx.strokeStyle = '#39ff14';
          ctx.lineWidth = 2;
          ctx.stroke();
        }
        ctx.fillStyle = '#030504';
        ctx.font = 'bold 10px "JetBrainsMono Nerd Font", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(String(a.name || a.id).slice(0, 3), ax, ay + 0.5);
        ctx.globalAlpha = 1;
      });

      if (state.raw) {
        ctx.fillStyle = 'rgba(198,255,0,0.75)';
        ctx.beginPath();
        ctx.arc(sx(state.raw.x), sy(state.raw.y), 5, 0, Math.PI * 2);
        ctx.fill();
      }
      if (state.tag) {
        const tx = sx(state.tag.x), ty = sy(state.tag.y);
        ctx.beginPath();
        ctx.arc(tx, ty, 11, 0, Math.PI * 2);
        ctx.fillStyle = '#39ff14';
        ctx.shadowColor = 'rgba(57,255,20,0.7)';
        ctx.shadowBlur = 14;
        ctx.fill();
        ctx.shadowBlur = 0;
        ctx.fillStyle = '#030504';
        ctx.font = 'bold 9px "JetBrainsMono Nerd Font", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('TAG', tx, ty + 0.5);
      }

      // scale bar / snap readout
      ctx.fillStyle = '#4a5a50';
      ctx.font = '10px "JetBrainsMono Nerd Font", monospace';
      ctx.textAlign = 'left';
      ctx.fillText(
        `zoom ${state.zoom.toFixed(1)}× · snap ${step} m`,
        p, H - 14
      );
    }

    function hitAnchor(px, py) {
      let best = null, bestD = 14;
      (state.anchors || []).forEach((a) => {
        const d = Math.hypot(sx(a.x) - px, sy(a.y) - py);
        if (d < bestD) { bestD = d; best = a; }
      });
      return best;
    }

    function hitRoom(px, py) {
      const wx = ix(px), wy = iy(py);
      // topmost room containing point
      for (let i = state.rooms.length - 1; i >= 0; i--) {
        const r = state.rooms[i];
        if (wx >= r.origin_x && wx <= r.origin_x + r.width &&
            wy >= r.origin_y && wy <= r.origin_y + r.height) {
          return r;
        }
      }
      return null;
    }

    function pointerPos(ev) {
      const r = canvas.getBoundingClientRect();
      return { x: ev.clientX - r.left, y: ev.clientY - r.top };
    }

    if (state.interactive) {
      canvas.addEventListener('wheel', (ev) => {
        ev.preventDefault();
        state._userZoomed = true;
        const p = pointerPos(ev);
        const factor = ev.deltaY > 0 ? 0.9 : 1.12;
        applyZoom(state.zoom * factor, p.x, p.y);
      }, { passive: false });

      canvas.addEventListener('pointerdown', (ev) => {
        const p = pointerPos(ev);
        // middle button or alt = pan
        if (ev.button === 1 || ev.altKey) {
          state.drag = {
            type: 'pan',
            startX: p.x, startY: p.y,
            vMinX: state.view.min_x, vMinY: state.view.min_y,
          };
          canvas.setPointerCapture(ev.pointerId);
          return;
        }
        const a = hitAnchor(p.x, p.y);
        if (a) {
          state.selectedId = a.id;
          state.selectedRoomId = null;
          state.drag = { type: 'anchor', id: a.id };
          canvas.setPointerCapture(ev.pointerId);
          if (state.onSelect) state.onSelect(a);
          draw();
          return;
        }
        if (state.dragRooms) {
          const room = hitRoom(p.x, p.y);
          if (room) {
            state.selectedRoomId = room.id;
            state.selectedId = null;
            state.drag = {
              type: 'room',
              id: room.id,
              grabOffX: ix(p.x) - room.origin_x,
              grabOffY: iy(p.y) - room.origin_y,
              startOx: room.origin_x,
              startOy: room.origin_y,
              moveAnchors: !!opts.moveAnchorsWithRoom,
            };
            canvas.setPointerCapture(ev.pointerId);
            if (state.onSelectRoom) state.onSelectRoom(room);
            draw();
            return;
          }
        }
        // empty drag = pan
        state.drag = {
          type: 'pan',
          startX: p.x, startY: p.y,
          vMinX: state.view.min_x, vMinY: state.view.min_y,
        };
        canvas.setPointerCapture(ev.pointerId);
      });

      canvas.addEventListener('pointermove', (ev) => {
        if (!state.drag) return;
        const p = pointerPos(ev);
        if (state.drag.type === 'pan') {
          state._userZoomed = true;
          const dx = ix(state.drag.startX) - ix(p.x);
          const dy = iy(state.drag.startY) - iy(p.y);
          // use pixel delta relative to view
          const dpx = p.x - state.drag.startX;
          const dpy = p.y - state.drag.startY;
          const worldPerPxX = state.view.width / (state.cssW - 2 * state.pad);
          const worldPerPxY = state.view.height / (state.cssH - 2 * state.pad);
          state.view.min_x = state.drag.vMinX - dpx * worldPerPxX;
          state.view.min_y = state.drag.vMinY + dpy * worldPerPxY;
          clampView();
          draw();
          return;
        }
        if (state.drag.type === 'anchor') {
          const step = effectiveSnap();
          let mx = snap(ix(p.x), step);
          let my = snap(iy(p.y), step);
          if (ev.shiftKey) {
            mx = snap(ix(p.x), step / 2);
            my = snap(iy(p.y), step / 2);
          }
          const a = state.anchors.find((x) => x.id === state.drag.id);
          if (a) {
            a.x = mx; a.y = my;
            draw();
            canvas.title = `snapped (${mx.toFixed(2)}, ${my.toFixed(2)}) m · snap ${step} m`;
          }
          return;
        }
        if (state.drag.type === 'room') {
          const room = state.rooms.find((r) => r.id === state.drag.id);
          if (!room) return;
          let ox = ix(p.x) - state.drag.grabOffX;
          let oy = iy(p.y) - state.drag.grabOffY;
          const sn = snapRoomToRooms(room, ox, oy);
          const dx = sn.x - room.origin_x;
          const dy = sn.y - room.origin_y;
          room.origin_x = sn.x;
          room.origin_y = sn.y;
          if (state.drag.moveAnchors || (opts.moveAnchorsWithRoom && opts.moveAnchorsWithRoom())) {
            (state.anchors || []).forEach((a) => {
              if (a.room_id === room.id) {
                a.x += dx; a.y += dy;
              }
            });
          }
          draw();
          canvas.title = `room @ (${sn.x.toFixed(2)}, ${sn.y.toFixed(2)}) m · snap ${effectiveSnap()} m`;
        }
      });

      canvas.addEventListener('pointerup', (ev) => {
        if (!state.drag) return;
        const d = state.drag;
        state.drag = null;
        if (d.type === 'anchor') {
          const a = state.anchors.find((x) => x.id === d.id);
          if (a && state.onAnchorMove) state.onAnchorMove(a);
        } else if (d.type === 'room') {
          const room = state.rooms.find((r) => r.id === d.id);
          if (room && state.onRoomMove) {
            state.onRoomMove(room, {
              dx: room.origin_x - d.startOx,
              dy: room.origin_y - d.startOy,
            });
          }
        }
        draw();
      });
    }

    window.addEventListener('resize', resize);
    resize();

    return {
      setConfig, setLive, draw, resize, state, fitWorld,
      zoomIn() { state._userZoomed = true; applyZoom(state.zoom * 1.25); },
      zoomOut() { state._userZoomed = true; applyZoom(state.zoom / 1.25); },
      getEffectiveSnap: effectiveSnap,
      setSelected(id) { state.selectedId = id; state.selectedRoomId = null; draw(); },
      setSelectedRoom(id) { state.selectedRoomId = id; state.selectedId = null; draw(); },
      clearTrail() { state.trail = []; draw(); },
      setMoveAnchorsWithRoom(fn) { opts.moveAnchorsWithRoom = fn; },
    };
  }

  global.FloorMap = { createMap, snap };
})(window);
