/**
 * OCULUS: Perception-Guided Adaptive 2.5D LiDAR Mapping
 * World-Consistent Autonomous Vehicle Research Visualizer
 * Strictly Optimized Live Loop (< 50ms measured latency)
 * Smooth Spline Trajectory, Fixed World Space, Top-Down AV Model
 */

(function () {
  // DOM References
  const canvas = document.getElementById('lidar-canvas');
  const ctx = canvas.getContext('2d');
  const tooltip = document.getElementById('map-tooltip');
  const phaseTitle = document.getElementById('phase-title');
  const phaseDesc = document.getElementById('phase-desc');

  // Telemetry DOM
  const metricTargetFps = document.getElementById('metric-target-fps');
  const metricFps = document.getElementById('metric-fps');
  const metricLatency = document.getElementById('metric-latency');
  const metricSpeed = document.getElementById('metric-speed');
  const metricPoints = document.getElementById('metric-points');
  const metricDynamic = document.getElementById('metric-dynamic');
  const metricActiveCells = document.getElementById('metric-active-cells');
  const metricUniformCells = document.getElementById('metric-uniform-cells');
  const metricReduction = document.getElementById('metric-reduction');
  const reductionBar = document.getElementById('reduction-bar');
  const metricStorageKb = document.getElementById('metric-storage-kb');
  const inspectorContent = document.getElementById('inspector-content');
  const inspectorLevelBadge = document.getElementById('inspector-level-badge');
  const hoverCoord = document.getElementById('hover-coord');
  const zoomStatus = document.getElementById('cam-zoom-status');
  const camTrackingStatus = document.getElementById('cam-tracking-status');
  const modelModeVal = document.getElementById('model-mode-val');

  // Controls DOM
  const btnReset = document.getElementById('btn-reset');
  const btnCameraMode = document.getElementById('btn-camera-mode');
  const camModeText = document.getElementById('cam-mode-text');
  const modeToggle = document.getElementById('mode-toggle');
  const lblUniform = document.getElementById('lbl-uniform');
  const lblAdaptive = document.getElementById('lbl-adaptive');
  const fpsButtons = document.querySelectorAll('.fps-btn');

  // Layer Checkboxes
  const chkPoints = document.getElementById('chk-points');
  const chkGrid = document.getElementById('chk-grid');
  const chkObjects = document.getElementById('chk-objects');
  const chkSemantics = document.getElementById('chk-semantics');
  const chkWorld = document.getElementById('chk-world');
  const chkElevation = document.getElementById('chk-elevation');
  const chkDanger = document.getElementById('chk-danger');
  const chkRings = document.getElementById('chk-rings');

  // Simulation & Pacing State
  let targetFps = 30;
  let measuredFps = 30.0;
  let measuredLatency = 18.0;
  let simTime = 0.0; // 0.0 to 180.0 seconds
  const TOTAL_DURATION_SEC = 180.0;
  let currentKeyframeId = 1;
  let uniformMode = false;
  let cameraMode = 'follow'; // 'follow' or 'free'

  // Viewport & Coordinates
  let scale = 11.5; // pixels per meter
  let offsetX = 0;   // Pan offset in pixels
  let offsetY = 0;
  let camX = 0.0;   // World camera position
  let camY = 0.0;
  let isDragging = false;
  let dragStartX = 0;
  let dragStartY = 0;

  // Selected & Hovered Inspector State
  let selectedEntity = null;
  let hoveredEntity = null;

  // Cache & World Map
  const framesCache = {};
  let worldMapData = null;
  let currentFrameData = null;
  let isFetching = false;

  // STRICT COLOR HIERARCHY: RED = FINE (5cm), YELLOW = MEDIUM (10-20cm), BLUE = COARSE (40-80cm)
  const LEVEL_COLORS = {
    0: 'rgba(239, 68, 68, 0.72)',   // 5 cm - Ultra-Fine (RED)
    1: 'rgba(245, 158, 11, 0.60)',  // 10 cm - Fine (ORANGE)
    2: 'rgba(234, 179, 8, 0.48)',   // 20 cm - Medium (YELLOW)
    3: 'rgba(2, 132, 199, 0.36)',   // 40 cm - Coarse (LIGHT BLUE)
    4: 'rgba(30, 64, 175, 0.25)'    // 80 cm - Ultra-Coarse (DEEP BLUE)
  };

  const SEM_COLORS = {
    0: '#64748b', 1: '#10b981', 2: '#475569', 3: '#f59e0b',
    4: '#3b82f6', 5: '#ef4444', 6: '#a855f7', 7: '#0ea5e9'
  };

  // RADIAL / FOVEATED ADAPTIVE GRID FIELD ZONES (AUTONOMOUS RESEARCH PRESENTATION)
  // STRICT COLOR HIERARCHY: RED = FINE (0–10m), YELLOW = MEDIUM (10–30m), BLUE = COARSE (30–85m)
  const RADIAL_ZONES = [
    // Zone 1: VERY FINE (0–10 m) - Level 0 (5 cm, RED)
    { r1: 0.0, r2: 2.5, lvl: 0, res: 0.05, zone: 'fine', color: 'rgba(239, 68, 68, 0.44)', stroke: 'rgba(248, 113, 113, 0.48)' },
    { r1: 2.5, r2: 5.0, lvl: 0, res: 0.05, zone: 'fine', color: 'rgba(239, 68, 68, 0.40)', stroke: 'rgba(248, 113, 113, 0.44)' },
    { r1: 5.0, r2: 7.5, lvl: 0, res: 0.05, zone: 'fine', color: 'rgba(239, 68, 68, 0.36)', stroke: 'rgba(248, 113, 113, 0.40)' },
    { r1: 7.5, r2: 10.0, lvl: 0, res: 0.05, zone: 'fine', color: 'rgba(239, 68, 68, 0.32)', stroke: 'rgba(248, 113, 113, 0.36)' },

    // Zone 2: FINE / MEDIUM (10–30 m) - Level 1 & 2 (10–20 cm, YELLOW)
    { r1: 10.0, r2: 15.0, lvl: 1, res: 0.10, zone: 'medium', color: 'rgba(245, 158, 11, 0.32)', stroke: 'rgba(251, 191, 36, 0.38)' },
    { r1: 15.0, r2: 20.0, lvl: 1, res: 0.10, zone: 'medium', color: 'rgba(234, 179, 8, 0.28)', stroke: 'rgba(250, 204, 21, 0.34)' },
    { r1: 20.0, r2: 25.0, lvl: 2, res: 0.20, zone: 'medium', color: 'rgba(234, 179, 8, 0.25)', stroke: 'rgba(250, 204, 21, 0.30)' },
    { r1: 25.0, r2: 30.0, lvl: 2, res: 0.20, zone: 'medium', color: 'rgba(202, 138, 4, 0.22)', stroke: 'rgba(234, 179, 8, 0.28)' },

    // Zone 3: COARSE (30–85 m) - Level 3 & 4 (40–80 cm, BLUE)
    { r1: 30.0, r2: 42.0, lvl: 3, res: 0.40, zone: 'coarse', color: 'rgba(2, 132, 199, 0.20)', stroke: 'rgba(56, 189, 248, 0.24)' },
    { r1: 42.0, r2: 56.0, lvl: 3, res: 0.40, zone: 'coarse', color: 'rgba(3, 105, 161, 0.17)', stroke: 'rgba(56, 189, 248, 0.20)' },
    { r1: 56.0, r2: 70.0, lvl: 4, res: 0.80, zone: 'coarse', color: 'rgba(30, 64, 175, 0.15)', stroke: 'rgba(96, 165, 250, 0.18)' },
    { r1: 70.0, r2: 85.0, lvl: 4, res: 0.80, zone: 'coarse', color: 'rgba(30, 58, 138, 0.12)', stroke: 'rgba(96, 165, 250, 0.16)' }
  ];

  const NUM_SECTORS = 32;
  const SECTOR_ANGLE = (2 * Math.PI) / NUM_SECTORS;

  // Trajectory Math: Continuous ego vehicle pose across the 180s route
  // Uses exact analytical derivatives: yaw(t) = atan2(vy, vx) smoothly aligned before & during turns
  function getEgoPose(tSec) {
    const t = Math.max(0.0, Math.min(TOTAL_DURATION_SEC, tSec));
    let x, y, z, vx, vy, steering = 0.0;

    if (t <= 35.0) {
      // Segment 1: Urban Boulevard Eastbound (Straight East, yaw = 0 deg)
      const p = t / 35.0;
      x = p * 140.0;
      y = 0.0;
      z = 0.0;
      vx = 140.0 / 35.0; // 4.0 m/s
      vy = 0.0;
      steering = 0.0;
    } else if (t <= 70.0) {
      // Segment 2: S-Curved Road Navigation (Hermite smooth curve)
      const p = (t - 35.0) / 35.0;
      x = 140.0 + p * 140.0;
      y = 55.0 * (3.0 * p * p - 2.0 * p * p * p);
      z = 0.0;
      vx = 140.0 / 35.0;
      vy = 55.0 * (6.0 * p - 6.0 * p * p) / 35.0;
      // Front wheel steering angle (curvature)
      steering = Math.sin(p * Math.PI) * 0.18; // Radians
    } else if (t <= 105.0) {
      // Segment 3: Downtown Approach & Smooth 90-deg Turn into 4-Way Intersection
      const p = (t - 70.0) / 35.0;
      x = 280.0 + 40.0 * Math.sin(p * Math.PI * 0.5);
      y = 55.0 + 85.0 * (1.0 - Math.cos(p * Math.PI * 0.5));
      z = 0.0;
      vx = 40.0 * (Math.PI * 0.5 / 35.0) * Math.cos(p * Math.PI * 0.5);
      vy = 85.0 * (Math.PI * 0.5 / 35.0) * Math.sin(p * Math.PI * 0.5);
      steering = 0.28; // Turning left into intersection
    } else if (t <= 145.0) {
      // Segment 4: Northbound Avenue Drive (Straight North, yaw = 90 deg)
      const p = (t - 105.0) / 40.0;
      x = 320.0;
      y = 140.0 + 120.0 * p;
      z = 0.0;
      vx = 0.0;
      vy = 120.0 / 40.0; // 3.0 m/s
      steering = 0.0;
    } else {
      // Segment 5: Elevated Bridge Incline & Curved Overpass
      const p = (t - 145.0) / 35.0;
      const ang = p * Math.PI * 0.6;
      x = 320.0 - 140.0 * (1.0 - Math.cos(ang));
      y = 260.0 + 50.0 * Math.sin(ang);
      z = 3.4 * Math.sin(p * Math.PI);
      vx = -140.0 * (Math.PI * 0.6 / 35.0) * Math.sin(ang);
      vy = 50.0 * (Math.PI * 0.6 / 35.0) * Math.cos(ang);
      steering = -0.22; // Turning left onto bridge
    }

    const speed = Math.hypot(vx, vy);
    const yaw = Math.atan2(vy, vx) * 180.0 / Math.PI;

    return { x, y, z, yaw, speed, steering };
  }

  // Canvas Sizing
  function resizeCanvas() {
    const wrapper = document.getElementById('canvas-wrapper');
    canvas.width = wrapper.clientWidth * window.devicePixelRatio;
    canvas.height = wrapper.clientHeight * window.devicePixelRatio;
    canvas.style.width = wrapper.clientWidth + 'px';
    canvas.style.height = wrapper.clientHeight + 'px';
    ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
    render();
  }
  window.addEventListener('resize', resizeCanvas);

  // Coordinate Conversion: Fixed World (X_w, Y_w) -> Canvas (px, py)
  function worldToCanvas(x, y) {
    const w = canvas.width / (2 * window.devicePixelRatio);
    const h = canvas.height / (2 * window.devicePixelRatio);
    const cx = w + offsetX;
    const cy = h + offsetY;
    const px = cx + (x - camX) * scale;
    const py = cy - (y - camY) * scale; // Invert Y: North is Up
    return { px, py };
  }

  function canvasToWorld(px, py) {
    const w = canvas.width / (2 * window.devicePixelRatio);
    const h = canvas.height / (2 * window.devicePixelRatio);
    const cx = w + offsetX;
    const cy = h + offsetY;
    const x = camX + (px - cx) / scale;
    const y = camY - (py - cy) / scale;
    return { x, y };
  }

  // Base path resolution for local server, subpath hosting (GitHub Pages), and static CD
  function resolvePath(relativePath) {
    const path = window.location.pathname;
    const base = path.endsWith('/') ? path : path.substring(0, path.lastIndexOf('/') + 1);
    const cleanRel = relativePath.replace(/^\/+/, '');
    return (base + cleanRel).replace(/\/\//g, '/');
  }

  // Fetch or retrieve cached keyframe data
  async function fetchKeyframe(frameId) {
    const cacheKey = `${frameId}_${uniformMode}`;
    if (framesCache[cacheKey]) {
      currentFrameData = framesCache[cacheKey];
      updateTelemetryUI();
      return currentFrameData;
    }

    if (isFetching) return null;
    isFetching = true;

    try {
      // 1. Try dynamic backend endpoint first (FastAPI main.py)
      let resp = null;
      try {
        resp = await fetch(resolvePath(`api/frame/${frameId}?uniform=${uniformMode}`));
      } catch (_) {}

      // 2. If dynamic endpoint fails (e.g. 404 on static GitHub Pages), fallback to static JSON
      if (!resp || !resp.ok) {
        const staticName = uniformMode
          ? `api/frames/frame_${frameId}_uniform.json`
          : `api/frames/frame_${frameId}.json`;
        resp = await fetch(resolvePath(staticName));
        if ((!resp || !resp.ok) && uniformMode) {
          resp = await fetch(resolvePath(`api/frames/frame_${frameId}.json`));
        }
      }

      if (resp && resp.ok) {
        const data = await resp.json();
        framesCache[cacheKey] = data;
        currentFrameData = data;
        updateTelemetryUI();
        return data;
      }
    } catch (err) {
      console.warn("Frame fetch error:", err);
    } finally {
      isFetching = false;
    }
    return null;
  }

  // Background pre-fetcher for ultra-low latency playback
  function prefetchNearbyFrames(startId) {
    for (let offset = 1; offset <= 4; offset++) {
      const nextId = ((startId + offset - 1) % 180) + 1;
      const key = `${nextId}_${uniformMode}`;
      if (!framesCache[key]) {
        const staticName = uniformMode
          ? `api/frames/frame_${nextId}_uniform.json`
          : `api/frames/frame_${nextId}.json`;
        fetch(resolvePath(`api/frame/${nextId}?uniform=${uniformMode}`))
          .then(r => r.ok ? r.json() : fetch(resolvePath(staticName)).then(r2 => r2.json()))
          .then(data => { if (data) framesCache[key] = data; })
          .catch(() => {});
      }
    }
  }

  // Main Render Loop (< 50ms strictly measured)
  function render() {
    const w = canvas.width / window.devicePixelRatio;
    const h = canvas.height / window.devicePixelRatio;

    // Dark cyberpunk visualizer backdrop
    ctx.fillStyle = '#03050a';
    ctx.fillRect(0, 0, w, h);

    const ego = getEgoPose(simTime);

    // Follow-Ego mode smoothly tracks car position
    if (cameraMode === 'follow') {
      camX = ego.x;
      camY = ego.y;
    }

    // 1. Draw Fixed Static World Map (Roads, Poles, Trees, Buildings, Crosswalks, Barrier)
    if (chkWorld.checked && worldMapData) {
      drawWorldMap(worldMapData, ego);
    }

    // 2. Draw Radial / Foveated Adaptive Grid Field (Concentric Red/Yellow/Blue Resolution)
    if (chkGrid.checked) {
      drawRadialFoveatedAdaptiveGrid(ego, currentFrameData);
    }

    // 3. Draw Range Rings & Rotating LiDAR FOV Cone (Centered on Ego Vehicle)
    if (chkRings.checked) {
      drawRangeRingsAndFov(ego);
    }

    // 4. Draw Sampled LiDAR Points
    if (chkPoints.checked && currentFrameData && currentFrameData.points_sample) {
      drawSampledPoints(currentFrameData.points_sample, currentFrameData.point_labels_sample, ego);
    }

    // 5. Draw Dynamic Actors (Pedestrian Crossing, Oncoming Car, Lead Car)
    if (chkObjects.checked) {
      drawDynamicActors(ego);
    }

    // 6. Draw 2.5D Elevation Visual Cue (Hover or Selection)
    const activeInspEntity = hoveredEntity || selectedEntity;
    if (activeInspEntity) {
      drawElevationVisualCue(activeInspEntity, ego);
    }

    // 7. Draw Top-Down Autonomous Vehicle (Vector Silhouette, Windshield, Headlights, Steered Wheels, LiDAR Puck)
    drawTopDownEgoVehicle(ego);
  }

  // --------------------------------------------------------------------------
  // 1. FIXED STATIC WORLD MAP (PERMANENT WORLD COORDINATES)
  // --------------------------------------------------------------------------
  function drawWorldMap(mapData, ego) {
    ctx.save();

    // Road Segments
    if (mapData.segments) {
      mapData.segments.forEach(seg => {
        const p1 = worldToCanvas(seg.start[0], seg.start[1]);
        const p2 = worldToCanvas(seg.end[0], seg.end[1]);
        const roadWidthPx = seg.width * scale;

        ctx.beginPath();
        ctx.moveTo(p1.px, p1.py);
        if (seg.control) {
          const cp = worldToCanvas(seg.control[0], seg.control[1]);
          ctx.quadraticCurveTo(cp.px, cp.py, p2.px, p2.py);
        } else {
          ctx.lineTo(p2.px, p2.py);
        }

        // Road Asphalt
        ctx.strokeStyle = '#0e1626';
        ctx.lineWidth = roadWidthPx;
        ctx.lineCap = 'round';
        ctx.stroke();

        // Curbs / Road Outer Edges
        ctx.strokeStyle = '#223247';
        ctx.lineWidth = 2.0;
        ctx.stroke();

        // Centerline (Yellow dashed)
        ctx.strokeStyle = '#fbbf24';
        ctx.lineWidth = 1.8;
        ctx.setLineDash([8, 6]);
        ctx.stroke();
        ctx.setLineDash([]);
      });
    }

    // 4-Way Urban Intersection & Zebra Crosswalks
    if (mapData.intersection) {
      const inter = mapData.intersection;
      const center = worldToCanvas(inter.center[0], inter.center[1]);
      const wPx = inter.size[0] * scale;
      const hPx = inter.size[1] * scale;

      // Intersection Asphalt Box
      ctx.fillStyle = '#0e1626';
      ctx.fillRect(center.px - wPx / 2, center.py - hPx / 2, wPx, hPx);
      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 1.5;
      ctx.strokeRect(center.px - wPx / 2, center.py - hPx / 2, wPx, hPx);

      // Zebra Crosswalk Stripes
      if (inter.crosswalks) {
        inter.crosswalks.forEach(cw => {
          const pt1 = worldToCanvas(cw.p1[0], cw.p1[1]);
          const pt2 = worldToCanvas(cw.p2[0], cw.p2[1]);
          ctx.save();
          ctx.strokeStyle = 'rgba(255, 255, 255, 0.85)';
          ctx.lineWidth = 3.6;
          ctx.setLineDash([6, 4]);
          ctx.beginPath();
          ctx.moveTo(pt1.px, pt1.py);
          ctx.lineTo(pt2.px, pt2.py);
          ctx.stroke();
          ctx.restore();
        });
      }
    }

    // STATIC POLES (PERMANENT WORLD POSITIONS - NEVER MOVE WITH VEHICLE)
    if (mapData.poles) {
      mapData.poles.forEach(p => {
        const pt = worldToCanvas(p.x, p.y);
        ctx.save();
        // Pole base
        ctx.beginPath();
        ctx.arc(pt.px, pt.py, 3.2, 0, 2 * Math.PI);
        ctx.fillStyle = '#cbd5e1';
        ctx.fill();
        ctx.strokeStyle = '#475569';
        ctx.lineWidth = 1.0;
        ctx.stroke();

        // Soft ground illumination circle under street lamp
        ctx.beginPath();
        ctx.arc(pt.px, pt.py, 14, 0, 2 * Math.PI);
        ctx.fillStyle = 'rgba(251, 191, 36, 0.04)';
        ctx.fill();

        // Research Callout [ Pole ] matching reference image when within sensor range
        const distEgoPole = Math.hypot(p.x - ego.x, p.y - ego.y);
        if (distEgoPole <= 38.0) {
          // Yellow ground target ring on curb
          ctx.beginPath();
          ctx.arc(pt.px, pt.py, 8, 0, 2 * Math.PI);
          ctx.strokeStyle = '#facc15';
          ctx.lineWidth = 1.6;
          ctx.stroke();

          // Pointer line and callout pill
          ctx.strokeStyle = 'rgba(248, 250, 252, 0.85)';
          ctx.lineWidth = 1.0;
          ctx.beginPath();
          ctx.moveTo(pt.px + 6, pt.py - 6);
          ctx.lineTo(pt.px + 20, pt.py - 18);
          ctx.lineTo(pt.px + 36, pt.py - 18);
          ctx.stroke();

          ctx.fillStyle = 'rgba(10, 16, 28, 0.92)';
          ctx.fillRect(pt.px + 36, pt.py - 28, 40, 20);
          ctx.strokeStyle = '#facc15';
          ctx.lineWidth = 1;
          ctx.strokeRect(pt.px + 36, pt.py - 28, 40, 20);

          ctx.fillStyle = '#f8fafc';
          ctx.font = 'bold 9px JetBrains Mono, monospace';
          ctx.fillText('Pole', pt.px + 45, pt.py - 14);
        }

        ctx.restore();
      });
    }

    // STATIC TREES (PERMANENT WORLD POSITIONS)
    if (mapData.trees) {
      mapData.trees.forEach(tr => {
        const pt = worldToCanvas(tr.x, tr.y);
        const rPx = tr.r * scale;
        ctx.save();
        // Canopy outer
        ctx.beginPath();
        ctx.arc(pt.px, pt.py, rPx, 0, 2 * Math.PI);
        ctx.fillStyle = 'rgba(22, 101, 52, 0.45)';
        ctx.fill();
        ctx.strokeStyle = '#15803d';
        ctx.lineWidth = 1.2;
        ctx.stroke();
        // Inner core
        ctx.beginPath();
        ctx.arc(pt.px, pt.py, rPx * 0.4, 0, 2 * Math.PI);
        ctx.fillStyle = 'rgba(34, 197, 94, 0.6)';
        ctx.fill();
        ctx.restore();
      });
    }

    // STATIC BUILDINGS (PERMANENT WORLD POSITIONS)
    if (mapData.buildings) {
      mapData.buildings.forEach(b => {
        const bp = worldToCanvas(b.x, b.y);
        const bw = b.w * scale;
        const bh = b.h * scale;

        ctx.fillStyle = 'rgba(15, 23, 42, 0.92)';
        ctx.fillRect(bp.px - bw / 2, bp.py - bh / 2, bw, bh);

        ctx.strokeStyle = 'rgba(56, 189, 248, 0.3)';
        ctx.lineWidth = 1.2;
        ctx.strokeRect(bp.px - bw / 2, bp.py - bh / 2, bw, bh);

        // Building Label
        ctx.fillStyle = 'rgba(148, 163, 184, 0.7)';
        ctx.font = '8px JetBrains Mono, monospace';
        ctx.fillText(b.label, bp.px - bw / 2 + 5, bp.py - bh / 2 + 12);
      });
    }

    // STATIC CONSTRUCTION BARRIER (PERMANENT AT WORLD (205, 8))
    if (mapData.barrier) {
      const b = mapData.barrier;
      const bp = worldToCanvas(b.x, b.y);
      const bdx = b.dx * scale;
      const bdy = b.dy * scale;

      ctx.save();
      ctx.translate(bp.px, bp.py);
      ctx.fillStyle = '#b45309';
      ctx.fillRect(-bdx / 2, -bdy / 2, bdx, bdy);
      // Striped hazard pattern
      ctx.strokeStyle = '#fbbf24';
      ctx.lineWidth = 2.5;
      ctx.setLineDash([5, 5]);
      ctx.strokeRect(-bdx / 2, -bdy / 2, bdx, bdy);
      ctx.setLineDash([]);

      ctx.fillStyle = '#f59e0b';
      ctx.font = 'bold 8px JetBrains Mono, monospace';
      ctx.fillText("⚠ ROAD WORK BARRIER", -bdx / 2 + 4, -bdy / 2 - 4);

      // Research Callout: [ Obstacle ] matching reference image
      const distEgoBarrier = Math.hypot(b.x - ego.x, b.y - ego.y);
      if (distEgoBarrier <= 65.0) {
        ctx.strokeStyle = 'rgba(248, 250, 252, 0.85)';
        ctx.lineWidth = 1.0;
        ctx.beginPath();
        ctx.moveTo(bdx / 2, 0);
        ctx.lineTo(bdx / 2 + 18, -16);
        ctx.lineTo(bdx / 2 + 34, -16);
        ctx.stroke();

        ctx.fillStyle = 'rgba(10, 16, 28, 0.92)';
        ctx.fillRect(bdx / 2 + 34, -26, 62, 20);
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 1;
        ctx.strokeRect(bdx / 2 + 34, -26, 62, 20);

        ctx.fillStyle = '#f8fafc';
        ctx.font = 'bold 9px JetBrains Mono, monospace';
        ctx.fillText('Obstacle', bdx / 2 + 40, -12);
      }
      ctx.restore();
    }

    // STATIC PARKED CARS (PERMANENT WORLD COORDINATES)
    if (mapData.parked_cars) {
      mapData.parked_cars.forEach(pc => {
        const pcp = worldToCanvas(pc.x, pc.y);
        const pcL = pc.dx * scale;
        const pcW = pc.dy * scale;

        ctx.save();
        ctx.translate(pcp.px, pcp.py);
        ctx.fillStyle = '#1e293b';
        ctx.strokeStyle = '#64748b';
        ctx.lineWidth = 1.2;
        ctx.fillRect(-pcL / 2, -pcW / 2, pcL, pcW);
        ctx.strokeRect(-pcL / 2, -pcW / 2, pcL, pcW);
        // Windshield
        ctx.fillStyle = 'rgba(100, 116, 139, 0.4)';
        ctx.fillRect(-pcL * 0.1, -pcW * 0.35, pcL * 0.35, pcW * 0.7);
        ctx.restore();
      });
    }

    ctx.restore();
  }

  // --------------------------------------------------------------------------
  // 2. RANGE RINGS & SENSOR FOV CONE (ANCHORED TO EGO POSE)
  // --------------------------------------------------------------------------
  function drawRangeRingsAndFov(ego) {
    const center = worldToCanvas(ego.x, ego.y);
    const rings = [
      { r: 10, label: '10m (Fine 5cm)', color: 'rgba(239, 68, 68, 0.85)' },
      { r: 25, label: '25m (Medium 10-20cm)', color: 'rgba(234, 179, 8, 0.85)' },
      { r: 50, label: '50m (Coarse 40cm)', color: 'rgba(56, 189, 248, 0.80)' },
      { r: 80, label: '80m (Far 80cm)', color: 'rgba(96, 165, 250, 0.75)' }
    ];

    ctx.save();

    // Range Rings with crisp dashed stroke
    rings.forEach(ring => {
      const rPx = ring.r * scale;
      ctx.beginPath();
      ctx.arc(center.px, center.py, rPx, 0, 2 * Math.PI);
      ctx.strokeStyle = 'rgba(0, 240, 255, 0.25)';
      ctx.lineWidth = 1.2;
      ctx.setLineDash([5, 4]);
      ctx.stroke();

      // Metric distance label
      ctx.fillStyle = ring.color;
      ctx.font = 'bold 9px JetBrains Mono, monospace';
      ctx.fillText(ring.label, center.px + 6, center.py - rPx - 4);
    });

    // Rotating Forward FOV LiDAR Cone (+/- 50 deg aligned with ego yaw)
    const yawRad = (ego.yaw * Math.PI) / 180.0;
    const fovAngle = (50.0 * Math.PI) / 180.0;
    const fovDistPx = 70.0 * scale;

    const a1 = -yawRad - fovAngle;
    const a2 = -yawRad + fovAngle;

    ctx.beginPath();
    ctx.moveTo(center.px, center.py);
    ctx.arc(center.px, center.py, fovDistPx, a1, a2);
    ctx.closePath();

    ctx.fillStyle = 'rgba(0, 240, 255, 0.03)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.20)';
    ctx.lineWidth = 1.2;
    ctx.setLineDash([4, 4]);
    ctx.stroke();

    ctx.restore();
  }

  // --------------------------------------------------------------------------
  // 3. TOP-DOWN AUTONOMOUS VEHICLE MODEL (VECTOR CAR SILHOUETTE)
  // --------------------------------------------------------------------------
  function drawTopDownEgoVehicle(ego) {
    const center = worldToCanvas(ego.x, ego.y);
    const vehW = 2.0 * scale;
    const vehL = 4.6 * scale;
    const yawRad = (ego.yaw * Math.PI) / 180.0;

    ctx.save();
    ctx.translate(center.px, center.py);
    ctx.rotate(-yawRad); // Inverted screen coordinate rotation

    // 1. Dual Volumetric Headlight Beams illuminating road
    const beamDist = 38 * scale;
    const beamW = 14 * scale;

    // Left Headlight Cone
    const gradL = ctx.createLinearGradient(vehL / 2, -vehW * 0.35, vehL / 2 + beamDist, -vehW * 0.35 - beamW);
    gradL.addColorStop(0, 'rgba(0, 240, 255, 0.40)');
    gradL.addColorStop(0.3, 'rgba(0, 240, 255, 0.15)');
    gradL.addColorStop(1, 'rgba(0, 240, 255, 0.0)');

    ctx.beginPath();
    ctx.moveTo(vehL / 2, -vehW * 0.35);
    ctx.lineTo(vehL / 2 + beamDist, -vehW * 0.35 - beamW);
    ctx.lineTo(vehL / 2 + beamDist, -vehW * 0.35 + beamW * 0.3);
    ctx.closePath();
    ctx.fillStyle = gradL;
    ctx.fill();

    // Right Headlight Cone
    const gradR = ctx.createLinearGradient(vehL / 2, vehW * 0.35, vehL / 2 + beamDist, vehW * 0.35 + beamW);
    gradR.addColorStop(0, 'rgba(0, 240, 255, 0.40)');
    gradR.addColorStop(0.3, 'rgba(0, 240, 255, 0.15)');
    gradR.addColorStop(1, 'rgba(0, 240, 255, 0.0)');

    ctx.beginPath();
    ctx.moveTo(vehL / 2, vehW * 0.35);
    ctx.lineTo(vehL / 2 + beamDist, vehW * 0.35 - beamW * 0.3);
    ctx.lineTo(vehL / 2 + beamDist, vehW * 0.35 + beamW);
    ctx.closePath();
    ctx.fillStyle = gradR;
    ctx.fill();

    // 2. Four Rubber Tires with Steered Front Wheels
    const tireL = 0.85 * scale;
    const tireW = 0.32 * scale;
    const steerAngle = ego.steering || 0.0;

    // Rear Wheels (Fixed straight)
    ctx.fillStyle = '#0f172a';
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1;

    // Rear Left
    ctx.fillRect(-vehL * 0.35 - tireL / 2, -vehW / 2 - tireW * 0.7, tireL, tireW);
    ctx.strokeRect(-vehL * 0.35 - tireL / 2, -vehW / 2 - tireW * 0.7, tireL, tireW);
    // Rear Right
    ctx.fillRect(-vehL * 0.35 - tireL / 2, vehW / 2 - tireW * 0.3, tireL, tireW);
    ctx.strokeRect(-vehL * 0.35 - tireL / 2, vehW / 2 - tireW * 0.3, tireL, tireW);

    // Front Left (Rotated by steerAngle)
    ctx.save();
    ctx.translate(vehL * 0.30, -vehW / 2 - tireW * 0.2);
    ctx.rotate(-steerAngle);
    ctx.fillRect(-tireL / 2, -tireW / 2, tireL, tireW);
    ctx.strokeRect(-tireL / 2, -tireW / 2, tireL, tireW);
    ctx.restore();

    // Front Right (Rotated by steerAngle)
    ctx.save();
    ctx.translate(vehL * 0.30, vehW / 2 + tireW * 0.2);
    ctx.rotate(-steerAngle);
    ctx.fillRect(-tireL / 2, -tireW / 2, tireL, tireW);
    ctx.strokeRect(-tireL / 2, -tireW / 2, tireL, tireW);
    ctx.restore();

    // 3. Aerodynamic Car Body Silhouette (Smooth Vector Contours)
    ctx.beginPath();
    // Front bumper curve
    ctx.moveTo(vehL * 0.48, -vehW * 0.32);
    ctx.quadraticCurveTo(vehL * 0.52, 0, vehL * 0.48, vehW * 0.32);
    // Right fender & door side contour
    ctx.lineTo(vehL * 0.25, vehW * 0.48);
    ctx.lineTo(-vehL * 0.25, vehW * 0.48);
    // Rear right corner
    ctx.quadraticCurveTo(-vehL * 0.48, vehW * 0.45, -vehL * 0.50, vehW * 0.28);
    // Rear bumper
    ctx.lineTo(-vehL * 0.50, -vehW * 0.28);
    // Rear left corner
    ctx.quadraticCurveTo(-vehL * 0.48, -vehW * 0.45, -vehL * 0.25, -vehW * 0.48);
    // Left door side contour
    ctx.lineTo(vehL * 0.25, -vehW * 0.48);
    ctx.closePath();

    // Chassis Gradient Finish
    const bodyGrad = ctx.createLinearGradient(-vehL / 2, 0, vehL / 2, 0);
    bodyGrad.addColorStop(0, '#0a0f1d');
    bodyGrad.addColorStop(0.5, '#0f172a');
    bodyGrad.addColorStop(1, '#1e293b');
    ctx.fillStyle = bodyGrad;
    ctx.fill();

    // Glowing Cyan Trim Contour
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = 1.8;
    ctx.shadowColor = '#00f0ff';
    ctx.shadowBlur = 9;
    ctx.stroke();
    ctx.shadowBlur = 0;

    // 4. Cabin Glasshouse (Windshield, Panoramic Roof, Rear Window)
    // Front Windshield
    ctx.beginPath();
    ctx.moveTo(vehL * 0.18, -vehW * 0.36);
    ctx.quadraticCurveTo(vehL * 0.24, 0, vehL * 0.18, vehW * 0.36);
    ctx.lineTo(vehL * 0.02, vehW * 0.34);
    ctx.lineTo(vehL * 0.02, -vehW * 0.34);
    ctx.closePath();
    ctx.fillStyle = 'rgba(0, 240, 255, 0.30)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.6)';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Cabin Panoramic Roof Panel
    ctx.fillStyle = '#060913';
    ctx.fillRect(-vehL * 0.22, -vehW * 0.32, vehL * 0.24, vehW * 0.64);

    // Rear Window
    ctx.beginPath();
    ctx.moveTo(-vehL * 0.22, -vehW * 0.32);
    ctx.lineTo(-vehL * 0.36, -vehW * 0.28);
    ctx.quadraticCurveTo(-vehL * 0.40, 0, -vehL * 0.36, vehW * 0.28);
    ctx.lineTo(-vehL * 0.22, vehW * 0.32);
    ctx.closePath();
    ctx.fillStyle = 'rgba(0, 240, 255, 0.22)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.4)';
    ctx.lineWidth = 1;
    ctx.stroke();

    // 5. LED Lighting Details
    // Front Headlights (Cyan LEDs)
    ctx.fillStyle = '#38bdf8';
    ctx.fillRect(vehL * 0.45, -vehW * 0.42, vehL * 0.05, vehW * 0.18);
    ctx.fillRect(vehL * 0.45, vehW * 0.24, vehL * 0.05, vehW * 0.18);

    // Rear Taillights (Red LEDs)
    ctx.fillStyle = '#ef4444';
    ctx.fillRect(-vehL * 0.50, -vehW * 0.40, vehL * 0.04, vehW * 0.16);
    ctx.fillRect(-vehL * 0.50, vehW * 0.24, vehL * 0.04, vehW * 0.16);

    // 6. Roof Autonomous Sensor Pod (Velodyne HDL-64E LiDAR Puck)
    // Sensor Bar on Roof
    ctx.fillStyle = '#334155';
    ctx.fillRect(-vehL * 0.12, -vehW * 0.25, vehL * 0.20, vehW * 0.50);

    // LiDAR Puck Cylinder
    ctx.beginPath();
    ctx.arc(0, 0, 4.2, 0, 2 * Math.PI);
    ctx.fillStyle = '#0f172a';
    ctx.fill();
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    // Glowing Central Laser Emitter
    ctx.beginPath();
    ctx.arc(0, 0, 2.0, 0, 2 * Math.PI);
    ctx.fillStyle = '#ef4444';
    ctx.shadowColor = '#ef4444';
    ctx.shadowBlur = 6;
    ctx.fill();
    ctx.shadowBlur = 0;

    // Spinning Laser Sweep Line
    const sweepAngle = (performance.now() * 0.007) % (2 * Math.PI);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(Math.cos(sweepAngle) * 7.5 * scale, Math.sin(sweepAngle) * 7.5 * scale);
    ctx.strokeStyle = 'rgba(239, 68, 68, 0.40)';
    ctx.lineWidth = 1.2;
    ctx.stroke();

    // Research Callout: [ LiDAR ] matching reference image
    ctx.save();
    ctx.rotate(yawRad); // Keep text upright on screen
    ctx.strokeStyle = 'rgba(248, 250, 252, 0.85)';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(26, -20);
    ctx.lineTo(44, -20);
    ctx.stroke();

    ctx.fillStyle = 'rgba(10, 16, 28, 0.92)';
    ctx.fillRect(44, -30, 48, 20);
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.65)';
    ctx.lineWidth = 1.2;
    ctx.strokeRect(44, -30, 48, 20);

    ctx.fillStyle = '#f8fafc';
    ctx.font = 'bold 9px JetBrains Mono, monospace';
    ctx.fillText('LiDAR', 51, -16);
    ctx.restore();

    ctx.restore();
  }

  // --------------------------------------------------------------------------
  // 4. RADIAL / FOVEATED ADAPTIVE GRID FIELD (MATCHING REFERENCE RESEARCH VISUAL)
  // Concentric Rings: RED (0-10m: 5cm) -> YELLOW (10-30m: 10-20cm) -> BLUE (30-85m: 40-80cm)
  // Dynamic Refinement: High-importance actors refine locally to RED / YELLOW
  // --------------------------------------------------------------------------
  function drawRadialFoveatedAdaptiveGrid(ego, frameData) {
    const center = worldToCanvas(ego.x, ego.y);
    const yawRad = (ego.yaw * Math.PI) / 180.0;

    ctx.save();

    // Check dynamic refinement targets in world space
    let pedWorld = null;
    if (simTime >= 70.0 && simTime <= 105.0) {
      const pProg = (simTime - 70.0) / 35.0;
      const px = 310.0 + 20.0 * pProg;
      pedWorld = { x: px, y: 98.0, inLane: (px >= 318.0 && px <= 322.5) };
    }

    let oncWorld = null;
    if (simTime >= 105.0 && simTime <= 145.0) {
      const vProg = (simTime - 105.0) / 40.0;
      oncWorld = { x: 316.0, y: 270.0 - 190.0 * vProg };
    }

    // Static Barrier world coordinate
    const barrierWorld = { x: 205.0, y: 8.0 };

    // Render each annular sector cell in the foveated resolution field
    RADIAL_ZONES.forEach((ring) => {
      const r1Px = ring.r1 * scale;
      const r2Px = ring.r2 * scale;

      for (let s = 0; s < NUM_SECTORS; s++) {
        // Sector angles rotated with vehicle heading
        const a1 = -yawRad + s * SECTOR_ANGLE;
        const a2 = -yawRad + (s + 1) * SECTOR_ANGLE;

        // Sector center in world coordinates
        const midR = (ring.r1 + ring.r2) / 2.0;
        const midA = -yawRad + (s + 0.5) * SECTOR_ANGLE;
        const worldAngle = -midA;
        const cellWx = ego.x + midR * Math.cos(worldAngle);
        const cellWy = ego.y + midR * Math.sin(worldAngle);

        // Check perception refinement
        let isRefined = false;
        let refineType = null;

        if (pedWorld) {
          const dPed = Math.hypot(pedWorld.x - ego.x, pedWorld.y - ego.y);
          const aPed = Math.atan2(pedWorld.y - ego.y, pedWorld.x - ego.x);
          const normPedA = ((aPed + yawRad) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
          const pedSec = Math.floor(normPedA / SECTOR_ANGLE);
          if (Math.abs(s - pedSec) <= 1 && ring.r1 <= dPed && ring.r2 >= dPed) {
            isRefined = true;
            refineType = pedWorld.inLane ? 'DANGER_VRU' : 'CAUTION_VRU';
          }
        }

        if (!isRefined && oncWorld) {
          const dCar = Math.hypot(oncWorld.x - ego.x, oncWorld.y - ego.y);
          const aCar = Math.atan2(oncWorld.y - ego.y, oncWorld.x - ego.x);
          const normCarA = ((aCar + yawRad) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
          const carSec = Math.floor(normCarA / SECTOR_ANGLE);
          if (Math.abs(s - carSec) <= 1 && ring.r1 <= dCar && ring.r2 >= dCar) {
            isRefined = true;
            refineType = 'ONCOMING';
          }
        }

        if (!isRefined) {
          const dBar = Math.hypot(barrierWorld.x - ego.x, barrierWorld.y - ego.y);
          const aBar = Math.atan2(barrierWorld.y - ego.y, barrierWorld.x - ego.x);
          const normBarA = ((aBar + yawRad) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
          const barSec = Math.floor(normBarA / SECTOR_ANGLE);
          if (Math.abs(s - barSec) <= 1 && ring.r1 <= dBar && ring.r2 >= dBar) {
            isRefined = true;
            refineType = 'BARRIER';
          }
        }

        // Determine Cell Color based on mode / active layer
        let cellFill = ring.color;
        let cellStroke = ring.stroke;
        let lineWidth = 0.6;

        if (uniformMode) {
          // Uniform 5cm mode: All cells uniform fine red/amber
          cellFill = 'rgba(239, 68, 68, 0.36)';
          cellStroke = 'rgba(248, 113, 113, 0.45)';
          lineWidth = 0.8;
        } else if (chkDanger.checked) {
          if (refineType === 'DANGER_VRU') {
            cellFill = 'rgba(239, 68, 68, 0.75)';
            cellStroke = '#ef4444';
            lineWidth = 1.2;
          } else if (refineType === 'ONCOMING' || refineType === 'CAUTION_VRU' || refineType === 'BARRIER') {
            cellFill = 'rgba(249, 115, 22, 0.60)';
            cellStroke = '#f97316';
            lineWidth = 1.0;
          } else {
            cellFill = 'rgba(16, 185, 129, 0.16)';
            cellStroke = 'rgba(16, 185, 129, 0.25)';
          }
        } else if (chkElevation.checked) {
          const isBridge = cellWy >= 260.0;
          const elev = isBridge ? Math.min(3.4, (cellWy - 260.0) * 0.08) : (Math.abs(cellWy) >= 5.5 ? 0.15 : 0.0);
          const normE = Math.min(1.0, Math.max(0.0, (elev + 1.0) / 4.0));
          cellFill = `rgba(${Math.floor(normE * 255)}, ${Math.floor((1 - normE) * 200)}, 255, 0.38)`;
          cellStroke = 'rgba(186, 230, 253, 0.35)';
        } else if (isRefined) {
          // Dynamic Perception-Guided Refinement Bubble
          if (refineType === 'DANGER_VRU') {
            cellFill = 'rgba(239, 68, 68, 0.65)';
            cellStroke = '#f87171';
            lineWidth = 1.4;
          } else {
            cellFill = 'rgba(245, 158, 11, 0.52)';
            cellStroke = '#fbbf24';
            lineWidth = 1.2;
          }
        }

        // Draw Annular Sector Wedge
        ctx.beginPath();
        ctx.arc(center.px, center.py, r2Px, a1, a2, false);
        ctx.arc(center.px, center.py, r1Px, a2, a1, true);
        ctx.closePath();

        ctx.fillStyle = cellFill;
        ctx.fill();

        ctx.strokeStyle = cellStroke;
        ctx.lineWidth = lineWidth;
        ctx.stroke();

        // If refined, draw fine radial & circumferential sub-grid lines (Level 0 / Level 1 subdivision)
        if (isRefined) {
          const midR_Px = (r1Px + r2Px) / 2.0;
          const midA_sub = (a1 + a2) / 2.0;

          // Circumferential subdivision arc
          ctx.beginPath();
          ctx.arc(center.px, center.py, midR_Px, a1, a2, false);
          ctx.strokeStyle = 'rgba(255, 255, 255, 0.55)';
          ctx.lineWidth = 0.8;
          ctx.setLineDash([2, 2]);
          ctx.stroke();

          // Radial subdivision ray
          ctx.beginPath();
          ctx.moveTo(center.px + r1Px * Math.cos(midA_sub), center.py + r1Px * Math.sin(midA_sub));
          ctx.lineTo(center.px + r2Px * Math.cos(midA_sub), center.py + r2Px * Math.sin(midA_sub));
          ctx.stroke();
          ctx.setLineDash([]);
        }
      }
    });

    ctx.restore();
  }

  // --------------------------------------------------------------------------
  // 5. SAMPLED LIDAR POINTS
  // --------------------------------------------------------------------------
  function drawSampledPoints(points, labels, ego) {
    ctx.save();
    const yawRad = (ego.yaw * Math.PI) / 180.0;
    const cosA = Math.cos(yawRad);
    const sinA = Math.sin(yawRad);

    for (let i = 0; i < points.length; i++) {
      const pt = points[i];
      const dist = Math.hypot(pt[0], pt[1]);

      // Adaptive Point Cloud detail: High detail near ego; reduced density far
      if (dist > 70.0 && (i % 3 !== 0)) continue;
      if (dist > 45.0 && (i % 2 !== 0)) continue;

      const wx = ego.x + pt[0] * cosA - pt[1] * sinA;
      const wy = ego.y + pt[0] * sinA + pt[1] * cosA;
      const p = worldToCanvas(wx, wy);
      const semClass = (labels && chkSemantics.checked) ? labels[i] : 1;

      ctx.fillStyle = SEM_COLORS[semClass] || '#00f0ff';
      const ptSize = dist <= 15.0 ? 2.2 : (dist <= 35.0 ? 1.8 : 1.3);
      ctx.fillRect(p.px - ptSize / 2, p.py - ptSize / 2, ptSize, ptSize);
    }
    ctx.restore();
  }

  // --------------------------------------------------------------------------
  // 6. DYNAMIC ACTORS (CROSSING PEDESTRIAN, ONCOMING CAR, LEAD CAR)
  // --------------------------------------------------------------------------
  function drawDynamicActors(ego) {
    ctx.save();

    // Actor A: Crossing Pedestrian at Intersection (Active in Phase 3: t in [70, 105])
    if (simTime >= 70.0 && simTime <= 105.0) {
      const pProg = (simTime - 70.0) / 35.0;
      const pedWx = 310.0 + 20.0 * pProg; // Crossing from West to East
      const pedWy = 98.0;                 // South crosswalk
      const pCanvas = worldToCanvas(pedWx, pedWy);

      // In corridor?
      const inLane = (pedWx >= 318.0 && pedWx <= 322.5);
      const pedDanger = inLane ? 'DANGER' : 'CAUTION';
      const pedColor = inLane ? '#ef4444' : '#f59e0b';

      ctx.save();
      // Draw top-down pedestrian figure (Head + Shoulders)
      ctx.translate(pCanvas.px, pCanvas.py);

      // Shoulders
      ctx.fillStyle = pedColor;
      ctx.fillRect(-5, -2, 10, 4);
      // Head
      ctx.beginPath();
      ctx.arc(0, 0, 3, 0, 2 * Math.PI);
      ctx.fillStyle = '#f8fafc';
      ctx.fill();

      // 3D Bounding Box with corner brackets matching reference
      ctx.strokeStyle = pedColor;
      ctx.lineWidth = 1.8;
      ctx.strokeRect(-9, -9, 18, 18);

      // Research Callout: [ Pedestrian ] matching reference image
      ctx.strokeStyle = 'rgba(248, 250, 252, 0.85)';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(9, 0);
      ctx.lineTo(24, -14);
      ctx.lineTo(40, -14);
      ctx.stroke();

      ctx.fillStyle = 'rgba(10, 16, 28, 0.92)';
      ctx.fillRect(40, -26, 78, 22);
      ctx.strokeStyle = pedColor;
      ctx.lineWidth = 1.2;
      ctx.strokeRect(40, -26, 78, 22);

      ctx.fillStyle = '#f8fafc';
      ctx.font = 'bold 9px JetBrains Mono, monospace';
      ctx.fillText('Pedestrian', 46, -11);

      ctx.fillStyle = pedColor;
      ctx.font = 'bold 7.5px JetBrains Mono, monospace';
      ctx.fillText(pedDanger, 46, -2);

      // Local Adaptive Refinement Bubble (5cm RED trigger) around DANGER pedestrian
      if (inLane) {
        ctx.beginPath();
        ctx.arc(0, 0, 3.6 * scale, 0, 2 * Math.PI);
        ctx.strokeStyle = 'rgba(239, 68, 68, 0.85)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);
        ctx.stroke();
        ctx.fillStyle = 'rgba(239, 68, 68, 0.08)';
        ctx.fill();
        ctx.setLineDash([]);
      }

      ctx.restore();
    }

    // Actor B: Oncoming Vehicle in Opposite Lane (Phase 4: t in [105, 145])
    if (simTime >= 105.0 && simTime <= 145.0) {
      const vProg = (simTime - 105.0) / 40.0;
      const oncWx = 316.0;                    // Opposite lane (West side)
      const oncWy = 270.0 - 190.0 * vProg;    // Moving South
      const oCanvas = worldToCanvas(oncWx, oncWy);
      const oL = 4.6 * scale;
      const oW = 2.0 * scale;

      ctx.save();
      ctx.translate(oCanvas.px, oCanvas.py);
      ctx.rotate(Math.PI / 2); // Heading South (-90 deg in Cartesian -> +90 screen)

      // Oncoming Car Body
      ctx.fillStyle = '#1e1b4b';
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 1.6;
      ctx.fillRect(-oL / 2, -oW / 2, oL, oW);
      ctx.strokeRect(-oL / 2, -oW / 2, oL, oW);

      // Windshield
      ctx.fillStyle = 'rgba(245, 158, 11, 0.3)';
      ctx.fillRect(-oL * 0.1, -oW * 0.35, oL * 0.35, oW * 0.7);

      // Headlights illuminating South
      ctx.fillStyle = '#fef08a';
      ctx.fillRect(oL * 0.46, -oW * 0.4, oL * 0.04, oW * 0.2);
      ctx.fillRect(oL * 0.46, oW * 0.2, oL * 0.04, oW * 0.2);

      // Research Callout [ Vehicle ]
      ctx.save();
      ctx.rotate(-Math.PI / 2); // Keep upright
      ctx.strokeStyle = 'rgba(248, 250, 252, 0.85)';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(oW / 2, 0);
      ctx.lineTo(oW / 2 + 16, -14);
      ctx.lineTo(oW / 2 + 30, -14);
      ctx.stroke();

      ctx.fillStyle = 'rgba(10, 16, 28, 0.92)';
      ctx.fillRect(oW / 2 + 30, -26, 60, 20);
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 1.2;
      ctx.strokeRect(oW / 2 + 30, -26, 60, 20);

      ctx.fillStyle = '#f8fafc';
      ctx.font = 'bold 9px JetBrains Mono, monospace';
      ctx.fillText('Vehicle', oW / 2 + 38, -12);
      ctx.restore();

      // Refinement bubble
      ctx.beginPath();
      ctx.arc(0, 0, 4.0 * scale, 0, 2 * Math.PI);
      ctx.strokeStyle = 'rgba(245, 158, 11, 0.55)';
      ctx.lineWidth = 1.2;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.setLineDash([]);

      ctx.restore();
    }

    ctx.restore();
  }

  // --------------------------------------------------------------------------
  // 7. 2.5D ELEVATION VISUAL CUE & DIMENSIONING (HOVER OR SELECTION)
  // --------------------------------------------------------------------------
  function drawElevationVisualCue(entity, ego) {
    if (!entity || !entity.center_world) return;

    ctx.save();
    const cw = entity.center_world;
    const p = worldToCanvas(cw[0], cw[1]);

    // 1. Footprint Highlight on Ground Plane
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = 1.5;
    ctx.shadowColor = 'rgba(0, 240, 255, 0.7)';
    ctx.shadowBlur = 8;

    if (entity.radial_cell) {
      const rc = entity.radial_cell;
      const c = worldToCanvas(rc.cx, rc.cy);
      ctx.beginPath();
      ctx.arc(c.px, c.py, rc.r2 * scale, rc.a1, rc.a2, false);
      ctx.arc(c.px, c.py, rc.r1 * scale, rc.a2, rc.a1, true);
      ctx.closePath();
      ctx.fillStyle = 'rgba(0, 240, 255, 0.28)';
      ctx.fill();
      ctx.strokeStyle = '#00f0ff';
      ctx.lineWidth = 2.0;
      ctx.stroke();
    } else if (entity.dimensions) {
      const wPx = Math.max(12, (entity.dimensions[0] || 1.0) * scale);
      const hPx = Math.max(12, (entity.dimensions[1] || 1.0) * scale);
      ctx.fillStyle = 'rgba(0, 240, 255, 0.12)';
      ctx.fillRect(p.px - wPx / 2, p.py - hPx / 2, wPx, hPx);
      ctx.strokeRect(p.px - wPx / 2, p.py - hPx / 2, wPx, hPx);
    } else if (entity.radius) {
      const rPx = Math.max(10, entity.radius * scale);
      ctx.beginPath();
      ctx.arc(p.px, p.py, rPx, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(0, 240, 255, 0.12)';
      ctx.fill();
      ctx.stroke();
    } else {
      const sizePx = Math.max(10, (entity.res || 0.4) * scale);
      ctx.fillStyle = 'rgba(0, 240, 255, 0.16)';
      ctx.fillRect(p.px - sizePx / 2, p.py - sizePx / 2, sizePx, sizePx);
      ctx.strokeRect(p.px - sizePx / 2, p.py - sizePx / 2, sizePx, sizePx);
    }

    // 2. Vertical 2.5D Measurement Dimension Line (Z Axis Proof)
    ctx.shadowBlur = 0; // Crisp CAD lines
    const offsetX = 22; // Offset to the right of entity
    const lineX = p.px + offsetX;
    const baseElevation = entity.base_elev !== undefined ? entity.base_elev : 0.0;
    const topElevation = entity.top_elev !== undefined ? entity.top_elev : baseElevation + (entity.height || 1.5);
    const objHeight = entity.height !== undefined ? entity.height : Math.max(0.2, topElevation - baseElevation);

    // Visual line height in pixels
    const hVisualPx = Math.min(85, Math.max(34, objHeight * scale * 0.9));
    const baseY = p.py;
    const topY = baseY - hVisualPx;
    const midY = (baseY + topY) / 2;

    // Ground Baseline Tick
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.55)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(lineX - 8, baseY);
    ctx.lineTo(lineX + 16, baseY);
    ctx.stroke();

    // Base Marker Dot ●
    ctx.fillStyle = '#00f0ff';
    ctx.beginPath();
    ctx.arc(lineX, baseY, 3.2, 0, Math.PI * 2);
    ctx.fill();

    // Top Marker Dot ●
    ctx.beginPath();
    ctx.arc(lineX, topY, 3.2, 0, Math.PI * 2);
    ctx.fill();

    // Vertical Dashed Dimension Line │
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(lineX, baseY);
    ctx.lineTo(lineX, topY);
    ctx.stroke();
    ctx.setLineDash([]);

    // Height Value Callout Pill [ H: 1.52m ]
    const heightText = `H: ${objHeight.toFixed(2)}m`;
    ctx.font = 'bold 9px "JetBrains Mono", monospace';
    const textWidth = ctx.measureText(heightText).width;
    const pillW = textWidth + 10;
    const pillH = 15;
    const pillX = lineX + 6;
    const pillY = midY - pillH / 2;

    ctx.fillStyle = 'rgba(3, 5, 10, 0.92)';
    ctx.fillRect(pillX, pillY, pillW, pillH);
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = 1;
    ctx.strokeRect(pillX, pillY, pillW, pillH);

    ctx.fillStyle = '#00f0ff';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.fillText(heightText, pillX + 5, midY);

    // Base & Top Elevation Labels
    ctx.font = '8px "JetBrains Mono", monospace';
    ctx.fillStyle = '#94a3b8';
    ctx.fillText(`BASE: ${baseElevation.toFixed(2)}m`, lineX + 6, baseY + 4);
    ctx.fillText(`TOP: ${topElevation.toFixed(2)}m`, lineX + 6, topY - 4);

    ctx.restore();
  }

  // Format Floating Tooltip HTML with 2.5D Elevation Data
  function formatTooltipHtml(entity) {
    if (entity.type === 'object') {
      const isDyn = entity.is_dynamic;
      const speedStr = isDyn
        ? `Dynamic (${entity.speed ? entity.speed.toFixed(1) : '1.3'} m/s)`
        : 'STATIC';
      const colorTag = entity.danger_level === 'DANGER' ? 'text-red' : (entity.danger_level === 'WARNING' ? 'text-amber' : '');
      const resCm = (entity.resolution * 100).toFixed(0);
      const resTag = entity.resolution <= 0.05 ? 'RED' : (entity.resolution <= 0.20 ? 'YELLOW' : 'BLUE');

      return `
        <div class="tooltip-header">
          <span class="tooltip-title">${entity.class_name.toUpperCase()}</span>
          <span class="tooltip-badge-25d">2.5D ELEVATION</span>
        </div>
        <div class="tooltip-elev-box">
          <div class="tooltip-elev-title">▲ Z / ELEVATION PROFILE</div>
          <div class="tooltip-elev-row"><span class="tooltip-elev-key">Base Elevation:</span><span class="tooltip-elev-val">${entity.base_elev.toFixed(2)} m</span></div>
          <div class="tooltip-elev-row"><span class="tooltip-elev-key">Top Elevation:</span><span class="tooltip-elev-val">${entity.top_elev.toFixed(2)} m</span></div>
          <div class="tooltip-elev-row"><span class="tooltip-elev-key">Object Height (ΔZ):</span><span class="tooltip-elev-val" style="color:#00f0ff;">${entity.height.toFixed(2)} m</span></div>
          <div class="tooltip-elev-row"><span class="tooltip-elev-key">Center Z:</span><span class="tooltip-elev-val">${entity.elevation.toFixed(2)} m</span></div>
        </div>
        <div class="tooltip-row"><span class="tooltip-key">Confidence:</span><span class="tooltip-val">${(entity.confidence * 100).toFixed(0)}%</span></div>
        <div class="tooltip-row"><span class="tooltip-key">Distance:</span><span class="tooltip-val">${entity.distance.toFixed(1)} m</span></div>
        <div class="tooltip-row"><span class="tooltip-key">Complexity:</span><span class="tooltip-val">${entity.complexity || 'Medium'}</span></div>
        <div class="tooltip-row"><span class="tooltip-key">Motion State:</span><span class="tooltip-val ${isDyn ? 'text-amber' : 'text-muted'}">${speedStr}</span></div>
        <div class="tooltip-row"><span class="tooltip-key">Danger:</span><span class="tooltip-val ${colorTag}">${entity.danger_level} (${((entity.danger_prob || 0.1) * 100).toFixed(0)}%)</span></div>
        <div class="tooltip-row"><span class="tooltip-key">Resolution:</span><span class="tooltip-val text-cyan">${resCm} cm (${resTag})</span></div>
      `;
    } else {
      // Cell / Terrain
      const isCurb = entity.sem === 2;
      const resCm = (entity.res * 100).toFixed(0);
      const resTag = entity.lvl === 0 ? 'RED' : (entity.lvl <= 2 ? 'YELLOW' : 'BLUE');

      return `
        <div class="tooltip-header">
          <span class="tooltip-title">${(entity.sem_name || 'TERRAIN CELL').toUpperCase()}</span>
          <span class="tooltip-badge-25d">2.5D TERRAIN</span>
        </div>
        <div class="tooltip-elev-box">
          <div class="tooltip-elev-title">▲ Z / ELEVATION DATA</div>
          <div class="tooltip-elev-row"><span class="tooltip-elev-key">Median Elevation:</span><span class="tooltip-elev-val">${entity.elev.toFixed(2)} m</span></div>
          <div class="tooltip-elev-row"><span class="tooltip-elev-key">Base / Top:</span><span class="tooltip-elev-val">${entity.base_elev.toFixed(2)}m / ${entity.top_elev.toFixed(2)}m</span></div>
          <div class="tooltip-elev-row"><span class="tooltip-elev-key">Terrain Slope:</span><span class="tooltip-elev-val" style="color:#00f0ff;">${entity.slope_deg ? entity.slope_deg.toFixed(1) : '0.4'}°</span></div>
          <div class="tooltip-elev-row"><span class="tooltip-elev-key">Variance (σ²):</span><span class="tooltip-elev-val">${(entity.e_var || 0.035).toFixed(3)} m²</span></div>
        </div>
        <div class="tooltip-row"><span class="tooltip-key">Cell Resolution:</span><span class="tooltip-val text-cyan">${resCm} cm (${resTag})</span></div>
        <div class="tooltip-row"><span class="tooltip-key">Confidence:</span><span class="tooltip-val">${((entity.conf || 0.92) * 100).toFixed(0)}%</span></div>
        <div class="tooltip-row"><span class="tooltip-key">Occupancy:</span><span class="tooltip-val">${(entity.occupancy || 0.08).toFixed(2)}</span></div>
        <div class="tooltip-row"><span class="tooltip-key">Importance:</span><span class="tooltip-val">${(entity.imp || 0.15).toFixed(3)}</span></div>
        <div class="tooltip-row"><span class="tooltip-key">Danger:</span><span class="tooltip-val">${entity.d_lvl || 'SAFE'}</span></div>
      `;
    }
  }

  // Populate Sidebar Inspector Card with Elevation Details
  function populateInspector(data) {
    if (!data) {
      inspectorContent.innerHTML = '<div class="inspector-empty">Hover or click on any vehicle, pedestrian, pole, wall, barrier, or terrain cell to inspect live 2.5D elevation &amp; perception telemetry.</div>';
      inspectorLevelBadge.textContent = "IDLE";
      inspectorLevelBadge.className = "card-tag";
      return;
    }

    const lvl = data.danger_level || data.d_lvl || "SAFE";
    inspectorLevelBadge.textContent = lvl;
    inspectorLevelBadge.className = `card-tag tag-${lvl.toLowerCase()}`;

    if (data.type === 'object') {
      const isDyn = data.is_dynamic;
      const speedStr = isDyn
        ? `${data.speed ? data.speed.toFixed(1) : Math.hypot(data.velocity[0], data.velocity[1]).toFixed(1)} m/s ${data.direction ? `(${data.direction})` : ''}`
        : 'STATIC (Fixed Infrastructure)';
      const resTag = data.resolution <= 0.05 ? 'RED (5cm)' : (data.resolution <= 0.20 ? 'YELLOW (10-20cm)' : 'BLUE (40-80cm)');

      inspectorContent.innerHTML = `
        <div class="insp-section-title">
          <span>PERCEPTION &amp; KINEMATICS</span>
          <span class="card-tag ${isDyn ? 'tag-dynamic' : 'tag-static'}">${isDyn ? 'DYNAMIC' : 'STATIC'}</span>
        </div>
        <div class="insp-row"><span class="insp-key">Object Class:</span><span class="insp-val text-cyan">${data.class_name}</span></div>
        <div class="insp-row"><span class="insp-key">Confidence:</span><span class="insp-val">${(data.confidence * 100).toFixed(0)}%</span></div>
        <div class="insp-row"><span class="insp-key">Distance:</span><span class="insp-val">${data.distance.toFixed(1)} m</span></div>
        <div class="insp-row"><span class="insp-key">Complexity:</span><span class="insp-val">${data.complexity || 'Medium'}</span></div>
        <div class="insp-row"><span class="insp-key">Motion State:</span><span class="insp-val ${isDyn ? 'text-amber' : 'text-muted'}">${isDyn ? 'Dynamic (Moving)' : 'STATIC'}</span></div>
        <div class="insp-row"><span class="insp-key">Velocity:</span><span class="insp-val ${isDyn ? 'text-amber' : ''}">${speedStr}</span></div>
        <div class="insp-row"><span class="insp-key">Danger Level:</span><span class="insp-val ${data.danger_level === 'DANGER' ? 'text-red' : (data.danger_level === 'WARNING' ? 'text-amber' : '')}">${data.danger_level}</span></div>
        <div class="insp-row"><span class="insp-key">Danger Probability:</span><span class="insp-val text-amber">${((data.danger_prob || 0.1) * 100).toFixed(0)}%</span></div>
        <div class="insp-row highlight-row"><span class="insp-key">Current Resolution:</span><span class="insp-val text-cyan">${(data.resolution * 100).toFixed(0)} cm (${resTag})</span></div>

        <div class="insp-section-title">
          <span>2.5D ELEVATION DATA</span>
          <span class="badge-25d">Z / ELEVATION</span>
        </div>
        <div class="insp-elev-box">
          <div class="insp-row-elev"><span class="insp-key">Base Elevation (Z_base):</span><span class="insp-val">${data.base_elev.toFixed(2)} m</span></div>
          <div class="insp-row-elev"><span class="insp-key">Top Elevation (Z_top):</span><span class="insp-val">${data.top_elev.toFixed(2)} m</span></div>
          <div class="insp-row-elev"><span class="insp-key">Object Height (ΔZ):</span><span class="insp-val" style="color:#00f0ff;">${data.height.toFixed(2)} m</span></div>
          <div class="insp-row-elev"><span class="insp-key">Center Z Elevation:</span><span class="insp-val">${data.elevation.toFixed(2)} m</span></div>
        </div>
      `;
    } else {
      // Cell / Terrain
      const resTag = data.lvl === 0 ? 'RED (5cm)' : (data.lvl <= 2 ? 'YELLOW (10-20cm)' : 'BLUE (40-80cm)');

      inspectorContent.innerHTML = `
        <div class="insp-section-title">
          <span>CELL SPATIAL TELEMETRY</span>
          <span class="card-tag tag-safe">LEVEL ${data.lvl}</span>
        </div>
        <div class="insp-row"><span class="insp-key">Cell Resolution:</span><span class="insp-val text-cyan">${(data.res * 100).toFixed(0)} cm (${resTag})</span></div>
        <div class="insp-row"><span class="insp-key">Semantic Class:</span><span class="insp-val">${data.sem_name}</span></div>
        <div class="insp-row"><span class="insp-key">Semantic Confidence:</span><span class="insp-val">${((data.conf || 0.92) * 100).toFixed(0)}%</span></div>
        <div class="insp-row"><span class="insp-key">Occupancy:</span><span class="insp-val">${(data.occupancy || 0.08).toFixed(2)}</span></div>
        <div class="insp-row"><span class="insp-key">Importance Score:</span><span class="insp-val">${(data.imp || 0.15).toFixed(3)}</span></div>
        <div class="insp-row"><span class="insp-key">Danger Level:</span><span class="insp-val">${data.d_lvl || 'SAFE'}</span></div>
        <div class="insp-row"><span class="insp-key">Distance from Ego:</span><span class="insp-val">${data.distance ? data.distance.toFixed(1) : '12.4'} m</span></div>

        <div class="insp-section-title">
          <span>2.5D ELEVATION DATA</span>
          <span class="badge-25d">Z / ELEVATION</span>
        </div>
        <div class="insp-elev-box">
          <div class="insp-row-elev"><span class="insp-key">Median Elevation:</span><span class="insp-val">${data.elev.toFixed(2)} m</span></div>
          <div class="insp-row-elev"><span class="insp-key">Base Elevation (Z_base):</span><span class="insp-val">${data.base_elev.toFixed(2)} m</span></div>
          <div class="insp-row-elev"><span class="insp-key">Top Elevation (Z_top):</span><span class="insp-val">${data.top_elev.toFixed(2)} m</span></div>
          <div class="insp-row-elev"><span class="insp-key">Elevation Variance (σ²):</span><span class="insp-val">${(data.e_var || 0.035).toFixed(3)} m²</span></div>
          <div class="insp-row-elev"><span class="insp-key">Terrain Slope:</span><span class="insp-val" style="color:#00f0ff;">${data.slope_deg ? data.slope_deg.toFixed(1) : '0.4'}°</span></div>
        </div>
      `;
    }
  }

  // Comprehensive Entity Detection for Hover & Click (Objects, Actors, Infrastructure, Cells, Terrain)
  function findEntityAt(worldCoord, ego) {
    if (!ego) return null;
    const yawRad = (ego.yaw * Math.PI) / 180.0;
    const cosA = Math.cos(yawRad);
    const sinA = Math.sin(yawRad);

    // 1. Check Dynamic Detected Objects from Current Frame Pipeline
    if (currentFrameData && currentFrameData.detected_objects) {
      for (const obj of currentFrameData.detected_objects) {
        const wx = ego.x + obj.center[0] * cosA - obj.center[1] * sinA;
        const wy = ego.y + obj.center[0] * sinA + obj.center[1] * cosA;
        const distToCursor = Math.hypot(worldCoord.x - wx, worldCoord.y - wy);
        const hitRadius = Math.max(2.2, (obj.dimensions ? Math.max(obj.dimensions[0], obj.dimensions[1]) : 2.0) / 1.4);

        if (distToCursor <= hitRadius) {
          const base_elev = obj.base_elev !== undefined ? obj.base_elev : Math.max(0.0, obj.center[2] - obj.dimensions[2] / 2.0);
          const top_elev = obj.top_elev !== undefined ? obj.top_elev : (obj.center[2] + obj.dimensions[2] / 2.0);
          const height = obj.height !== undefined ? obj.height : obj.dimensions[2];
          const isDyn = obj.is_dynamic;
          const speed = isDyn && obj.velocity ? Math.hypot(obj.velocity[0], obj.velocity[1]) : 0.0;

          return {
            type: 'object',
            class_name: obj.class_name,
            semantic_class: obj.semantic_class,
            confidence: obj.confidence,
            distance: obj.distance,
            is_dynamic: isDyn,
            velocity: obj.velocity || [0.0, 0.0],
            speed: speed,
            direction: isDyn ? `${(Math.atan2(obj.velocity[1], obj.velocity[0]) * 180 / Math.PI).toFixed(0)}°` : 'Static',
            complexity: obj.complexity_cat || obj.complexity || 'Medium',
            danger_level: obj.danger_level || 'SAFE',
            danger_prob: obj.danger_prob !== undefined ? obj.danger_prob : 0.1,
            resolution: obj.resolution || 0.10,
            base_elev: Math.max(0.0, base_elev),
            top_elev: top_elev,
            height: height,
            elevation: obj.center[2],
            center_world: [wx, wy, obj.center[2]],
            dimensions: obj.dimensions,
          };
        }
      }
    }

    // 2. Check Dynamic World Actors (Crossing Pedestrian & Oncoming Car)
    // Actor A: Crossing Pedestrian at Intersection (Active in Phase 3: simTime in [70, 105])
    if (simTime >= 70.0 && simTime <= 105.0) {
      const pProg = (simTime - 70.0) / 35.0;
      const pedWx = 310.0 + 20.0 * pProg;
      const pedWy = 98.0;
      const distCursor = Math.hypot(worldCoord.x - pedWx, worldCoord.y - pedWy);
      if (distCursor <= 2.5) {
        const inLane = (pedWx >= 318.0 && pedWx <= 322.5);
        const distEgo = Math.hypot(pedWx - ego.x, pedWy - ego.y);
        return {
          type: 'object',
          class_name: 'Pedestrian',
          semantic_class: 5,
          confidence: 0.94,
          distance: distEgo,
          is_dynamic: true,
          velocity: [1.3, 0.0],
          speed: 1.3,
          direction: 'Eastbound (Crosswalk)',
          complexity: 'Low (VRU Silhouette)',
          danger_level: inLane ? 'DANGER' : 'CAUTION',
          danger_prob: inLane ? 0.92 : 0.61,
          resolution: 0.05,
          base_elev: 0.08,
          top_elev: 1.82,
          height: 1.74,
          elevation: 0.95,
          center_world: [pedWx, pedWy, 0.95],
          dimensions: [0.6, 0.5, 1.74],
        };
      }
    }

    // Actor B: Oncoming Vehicle in Adjacent Lane (Active in Phase 4: simTime in [105, 145])
    if (simTime >= 105.0 && simTime <= 145.0) {
      const cProg = (simTime - 105.0) / 40.0;
      const carWx = 316.0;
      const carWy = 250.0 - 110.0 * cProg;
      const distCursor = Math.hypot(worldCoord.x - carWx, worldCoord.y - carWy);
      if (distCursor <= 3.2) {
        const distEgo = Math.hypot(carWx - ego.x, carWy - ego.y);
        return {
          type: 'object',
          class_name: 'Vehicle',
          semantic_class: 4,
          confidence: 0.91,
          distance: distEgo,
          is_dynamic: true,
          velocity: [0.0, -14.5],
          speed: 14.5,
          direction: 'Southbound (Opposing Corridor)',
          complexity: 'Medium',
          danger_level: 'WARNING',
          danger_prob: 0.74,
          resolution: 0.10,
          base_elev: 0.14,
          top_elev: 1.66,
          height: 1.52,
          elevation: 0.90,
          center_world: [carWx, carWy, 0.90],
          dimensions: [4.6, 2.0, 1.52],
        };
      }
    }

    // 3. Check Fixed Static Infrastructure in World Map (Poles, Barrier, Parked Cars, Trees, Buildings)
    if (worldMapData) {
      // Barrier
      if (worldMapData.barrier) {
        const b = worldMapData.barrier;
        if (Math.abs(worldCoord.x - b.x) <= b.dx / 1.8 && Math.abs(worldCoord.y - b.y) <= b.dy * 1.5) {
          const distEgo = Math.hypot(b.x - ego.x, b.y - ego.y);
          return {
            type: 'object',
            class_name: b.label || 'Static Obstacle (Barrier)',
            semantic_class: 3,
            confidence: 0.96,
            distance: distEgo,
            is_dynamic: false,
            velocity: [0.0, 0.0],
            speed: 0.0,
            direction: 'Static',
            complexity: 'Medium (Interlocking)',
            danger_level: 'WARNING',
            danger_prob: 0.68,
            resolution: 0.05,
            base_elev: 0.00,
            top_elev: b.dz,
            height: b.dz,
            elevation: b.dz / 2.0,
            center_world: [b.x, b.y, b.dz / 2.0],
            dimensions: [b.dx, b.dy, b.dz],
          };
        }
      }

      // Parked Cars
      if (worldMapData.parked_cars) {
        for (const car of worldMapData.parked_cars) {
          if (Math.abs(worldCoord.x - car.x) <= car.dx / 1.7 && Math.abs(worldCoord.y - car.y) <= car.dy / 1.5) {
            const distEgo = Math.hypot(car.x - ego.x, car.y - ego.y);
            return {
              type: 'object',
              class_name: car.label || 'Parked Vehicle',
              semantic_class: 4,
              confidence: 0.94,
              distance: distEgo,
              is_dynamic: false,
              velocity: [0.0, 0.0],
              speed: 0.0,
              direction: 'Static',
              complexity: 'Medium',
              danger_level: 'CAUTION',
              danger_prob: 0.35,
              resolution: 0.10,
              base_elev: 0.12,
              top_elev: 0.12 + car.dz,
              height: car.dz,
              elevation: 0.12 + car.dz / 2.0,
              center_world: [car.x, car.y, 0.12 + car.dz / 2.0],
              dimensions: [car.dx, car.dy, car.dz],
            };
          }
        }
      }

      // Street Light Poles & Traffic Signs
      if (worldMapData.poles) {
        for (const p of worldMapData.poles) {
          const dist = Math.hypot(worldCoord.x - p.x, worldCoord.y - p.y);
          if (dist <= 1.8) {
            const distEgo = Math.hypot(p.x - ego.x, p.y - ego.y);
            const base_z = p.z || 0.0;
            return {
              type: 'object',
              class_name: 'Pole / Sign',
              semantic_class: 6,
              confidence: 0.97,
              distance: distEgo,
              is_dynamic: false,
              velocity: [0.0, 0.0],
              speed: 0.0,
              direction: 'Static',
              complexity: 'Low (Cylindrical)',
              danger_level: 'SAFE',
              danger_prob: 0.05,
              resolution: 0.10,
              base_elev: base_z,
              top_elev: base_z + p.h,
              height: p.h,
              elevation: base_z + p.h / 2.0,
              center_world: [p.x, p.y, base_z + p.h / 2.0],
              dimensions: [0.3, 0.3, p.h],
            };
          }
        }
      }

      // Trees & Foliage
      if (worldMapData.trees) {
        for (const t of worldMapData.trees) {
          const dist = Math.hypot(worldCoord.x - t.x, worldCoord.y - t.y);
          if (dist <= t.r) {
            const distEgo = Math.hypot(t.x - ego.x, t.y - ego.y);
            return {
              type: 'object',
              class_name: 'Tree / Environmental Object',
              semantic_class: 7,
              confidence: 0.93,
              distance: distEgo,
              is_dynamic: false,
              velocity: [0.0, 0.0],
              speed: 0.0,
              direction: 'Static',
              complexity: 'High (Canopy Foliage)',
              danger_level: 'SAFE',
              danger_prob: 0.04,
              resolution: 0.20,
              base_elev: 0.00,
              top_elev: 6.40,
              height: 6.40,
              elevation: 3.20,
              center_world: [t.x, t.y, 3.20],
              radius: t.r,
            };
          }
        }
      }

      // Buildings & Facades (Includes Open Parking, Tech Campus, etc.)
      if (worldMapData.buildings) {
        for (const b of worldMapData.buildings) {
          if (worldCoord.x >= b.x && worldCoord.x <= (b.x + b.w) &&
              worldCoord.y >= b.y && worldCoord.y <= (b.y + b.h)) {
            const bCenterX = b.x + b.w / 2.0;
            const bCenterY = b.y + b.h / 2.0;
            const distEgo = Math.hypot(bCenterX - ego.x, bCenterY - ego.y);
            const bHeight = 18.5;
            return {
              type: 'object',
              class_name: `Building (${b.label})`,
              semantic_class: 7,
              confidence: 0.99,
              distance: distEgo,
              is_dynamic: false,
              velocity: [0.0, 0.0],
              speed: 0.0,
              direction: 'Static',
              complexity: 'Planar Facade',
              danger_level: 'SAFE',
              danger_prob: 0.01,
              resolution: 0.40,
              base_elev: 0.00,
              top_elev: bHeight,
              height: bHeight,
              elevation: bHeight / 2.0,
              center_world: [bCenterX, bCenterY, bHeight / 2.0],
              dimensions: [b.w, b.h, bHeight],
            };
          }
        }
      }
    }

    // 4. Check Radial / Foveated Adaptive Grid Field
    const distEgo = Math.hypot(worldCoord.x - ego.x, worldCoord.y - ego.y);
    if (distEgo <= 85.0) {
      const ring = RADIAL_ZONES.find(z => distEgo >= z.r1 && distEgo < z.r2) || RADIAL_ZONES[RADIAL_ZONES.length - 1];

      // Azimuth angle relative to ego yaw
      const worldAngle = Math.atan2(worldCoord.y - ego.y, worldCoord.x - ego.x);
      const relAngle = ((worldAngle + yawRad) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
      const s = Math.floor(relAngle / SECTOR_ANGLE);
      const a1 = -yawRad + s * SECTOR_ANGLE;
      const a2 = -yawRad + (s + 1) * SECTOR_ANGLE;

      // Check dynamic refinement
      let isRefined = false;
      let refineType = null;
      if (simTime >= 70.0 && simTime <= 105.0) {
        const pProg = (simTime - 70.0) / 35.0;
        const pedWx = 310.0 + 20.0 * pProg;
        const dPed = Math.hypot(pedWx - ego.x, 98.0 - ego.y);
        const aPed = Math.atan2(98.0 - ego.y, pedWx - ego.x);
        const normPedA = ((aPed + yawRad) % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
        const pedSec = Math.floor(normPedA / SECTOR_ANGLE);
        if (Math.abs(s - pedSec) <= 1 && ring.r1 <= dPed && ring.r2 >= dPed) {
          isRefined = true;
          refineType = (pedWx >= 318.0 && pedWx <= 322.5) ? 'DANGER' : 'CAUTION';
        }
      }

      // Sample local surface elevation and variance
      const isBridge = worldCoord.y >= 260.0;
      const bridgeElev = isBridge ? Math.min(3.4, Math.max(0.0, (worldCoord.y - 260.0) * 0.08)) : 0.0;
      const isCurb = Math.abs(worldCoord.y) >= 5.5 && !isBridge;
      const cellElev = isBridge ? bridgeElev : (isCurb ? 0.15 : 0.0);
      const slope = isBridge ? 7.8 : (isCurb ? 4.7 : 0.3);
      const cellVar = isCurb ? 0.065 : (isBridge ? 0.035 : (0.0025 + ring.lvl * 0.0012));
      const spread = Math.sqrt(Math.max(0.001, cellVar)) * 1.6;

      const dangerLevel = isRefined ? refineType : 'SAFE';
      const dangerProb = isRefined ? (refineType === 'DANGER' ? 0.92 : 0.58) : 0.03;
      const impScore = isRefined ? 0.95 : (0.15 + (1.0 - distEgo / 85.0) * 0.4);

      return {
        type: 'cell',
        lvl: isRefined ? 0 : (uniformMode ? 0 : ring.lvl),
        res: isRefined ? 0.05 : (uniformMode ? 0.05 : ring.res),
        sem: isCurb ? 2 : 1,
        sem_name: ring.zone === 'fine' ? 'Near Corridor (Very Fine 5cm)' : (ring.zone === 'medium' ? 'Transition Zone (Medium 10-20cm)' : 'Distant Roadway (Coarse 40-80cm)'),
        conf: 0.96,
        elev: cellElev,
        e_var: cellVar,
        base_elev: cellElev - spread / 2.0,
        top_elev: cellElev + spread / 2.0,
        height: spread,
        slope_deg: slope,
        danger: dangerProb,
        d_lvl: dangerLevel,
        d_prob: dangerProb,
        imp: impScore,
        distance: distEgo,
        occupancy: 0.42,
        center_world: [worldCoord.x, worldCoord.y, cellElev],
        radial_cell: {
          r1: ring.r1,
          r2: ring.r2,
          a1: a1,
          a2: a2,
          cx: ego.x,
          cy: ego.y
        }
      };
    }

    // 5. Road & Terrain Surface Fallback (Curbs, Roadway, Elevated Bridge)
    if (worldMapData && worldMapData.segments) {
      for (const seg of worldMapData.segments) {
        const minX = Math.min(seg.start[0], seg.end[0]) - 16.0;
        const maxX = Math.max(seg.start[0], seg.end[0]) + 16.0;
        const minY = Math.min(seg.start[1], seg.end[1]) - 16.0;
        const maxY = Math.max(seg.start[1], seg.end[1]) + 16.0;

        if (worldCoord.x >= minX && worldCoord.x <= maxX &&
            worldCoord.y >= minY && worldCoord.y <= maxY) {
          const isBridge = (seg.type === 'bridge' || worldCoord.y >= 260.0);
          const bridgeElev = isBridge ? Math.min(3.4, Math.max(0.0, (worldCoord.y - 260.0) * 0.08)) : 0.0;
          const distEgo = Math.hypot(worldCoord.x - ego.x, worldCoord.y - ego.y);
          const isCurb = Math.abs(worldCoord.y) >= 5.5 && !isBridge;

          return {
            type: 'terrain',
            lvl: isCurb ? 1 : 2,
            res: isCurb ? 0.10 : 0.20,
            sem: isCurb ? 2 : 1,
            sem_name: isCurb ? 'Non-Drivable Curb' : 'Drivable Road Surface',
            conf: 0.95,
            elev: isCurb ? 0.15 : bridgeElev,
            e_var: isCurb ? 0.08 : 0.02,
            base_elev: isCurb ? 0.00 : bridgeElev,
            top_elev: isCurb ? 0.15 : bridgeElev + 0.03,
            height: isCurb ? 0.15 : 0.03,
            slope_deg: isBridge ? 7.8 : (isCurb ? 4.7 : 0.3),
            danger: 0.02,
            d_lvl: 'SAFE',
            d_prob: 0.02,
            imp: 0.08,
            distance: distEgo,
            occupancy: 0.05,
            center_world: [worldCoord.x, worldCoord.y, bridgeElev],
          };
        }
      }
    }

    return null;
  }

  // Update Telemetry Cards
  function updateTelemetryUI() {
    if (!currentFrameData) return;
    const m = currentFrameData.metrics;

    metricPoints.textContent = m.point_count.toLocaleString();
    metricDynamic.textContent = m.dynamic_object_count;
    metricActiveCells.textContent = m.active_adaptive_cells.toLocaleString();
    metricUniformCells.textContent = m.theoretical_uniform_cells.toLocaleString();
    metricReduction.textContent = `${m.cell_reduction_percent.toFixed(1)}%`;
    reductionBar.style.width = `${m.cell_reduction_percent}%`;
    metricStorageKb.textContent = `${m.estimated_adaptive_storage_kb.toFixed(0)} KB vs ${m.estimated_uniform_storage_kb.toFixed(0)} KB`;

    if (currentFrameData.phase_name) phaseTitle.textContent = currentFrameData.phase_name;
    if (currentFrameData.description) phaseDesc.textContent = currentFrameData.description;

    const ego = getEgoPose(simTime);
    metricSpeed.textContent = `${(ego.speed * 3.6).toFixed(1)} km/h`;
  }

  // Simulation Update Step
  function stepSimulation(dtSec) {
    simTime += dtSec;
    if (simTime >= TOTAL_DURATION_SEC) {
      simTime = 0.0;
    }

    const keyframeId = Math.min(180, Math.floor(simTime) + 1);
    if (keyframeId !== currentKeyframeId) {
      currentKeyframeId = keyframeId;
      fetchKeyframe(currentKeyframeId);
      prefetchNearbyFrames(currentKeyframeId);
    }
  }

  // Main Paced Animation Loop with requestAnimationFrame
  let lastFrameTime = performance.now();
  let frameCounter = 0;
  let lastFpsTime = performance.now();

  function animationLoop(timestamp) {
    requestAnimationFrame(animationLoop);

    const elapsed = timestamp - lastFrameTime;
    const targetInterval = 1000.0 / targetFps;

    if (elapsed >= targetInterval) {
      lastFrameTime = timestamp - (elapsed % targetInterval);

      // STRICT LATENCY MEASUREMENT (< 50ms requirement)
      const tStart = performance.now();
      stepSimulation(targetInterval / 1000.0);
      render();
      const loopDuration = performance.now() - tStart;
      measuredLatency = loopDuration;

      // Track measured FPS over sliding 400ms window
      frameCounter++;
      const now = performance.now();
      if (now - lastFpsTime >= 400) {
        measuredFps = (frameCounter * 1000.0) / (now - lastFpsTime);
        frameCounter = 0;
        lastFpsTime = now;

        // Update live measured readouts
        metricFps.textContent = measuredFps.toFixed(1);
        metricLatency.textContent = `${measuredLatency.toFixed(1)} ms`;
      }
    }
  }

  // Setup Event Listeners
  function setupEvents() {
    // FPS Target Buttons (10 / 20 / 30 / 40 / 50)
    fpsButtons.forEach(btn => {
      btn.addEventListener('click', () => {
        fpsButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        targetFps = parseInt(btn.getAttribute('data-fps'), 10);
        metricTargetFps.textContent = `${targetFps} FPS`;
      });
    });

    // Camera Follow / Free Toggle
    btnCameraMode.addEventListener('click', () => {
      if (cameraMode === 'follow') {
        cameraMode = 'free';
        btnCameraMode.classList.remove('active');
        camModeText.textContent = "Free World";
        camTrackingStatus.textContent = "Camera: Free World View";
      } else {
        cameraMode = 'follow';
        btnCameraMode.classList.add('active');
        camModeText.textContent = "Follow Ego";
        camTrackingStatus.textContent = "Camera: Following Ego Vehicle";
        offsetX = 0;
        offsetY = 0;
      }
      render();
    });

    // Recenter Camera Button
    btnReset.addEventListener('click', () => {
      cameraMode = 'follow';
      btnCameraMode.classList.add('active');
      camModeText.textContent = "Follow Ego";
      camTrackingStatus.textContent = "Camera: Following Ego Vehicle";
      offsetX = 0;
      offsetY = 0;
      scale = 11.5;
      zoomStatus.textContent = "Zoom: 100%";
      render();
    });

    // Mode Comparison Toggle (Uniform 5cm vs Adaptive Grid)
    modeToggle.addEventListener('change', (e) => {
      uniformMode = !e.target.checked;
      lblUniform.classList.toggle('active', uniformMode);
      lblAdaptive.classList.toggle('active', !uniformMode);
      fetchKeyframe(currentKeyframeId);
    });

    // Layer Toggles
    [chkPoints, chkGrid, chkObjects, chkSemantics, chkWorld, chkElevation, chkDanger, chkRings].forEach(chk => {
      if (chk) {
        chk.parentElement.addEventListener('click', () => {
          setTimeout(() => {
            chk.parentElement.classList.toggle('active', chk.checked);
            render();
          }, 10);
        });
      }
    });

    // Mouse Pan Interaction
    canvas.addEventListener('mousedown', (e) => {
      isDragging = true;
      dragStartX = e.clientX - offsetX;
      dragStartY = e.clientY - offsetY;
    });

    window.addEventListener('mouseup', () => {
      isDragging = false;
    });

    canvas.addEventListener('mousemove', (e) => {
      if (isDragging) {
        // Switching to free mode on drag
        cameraMode = 'free';
        btnCameraMode.classList.remove('active');
        camModeText.textContent = "Free World";
        camTrackingStatus.textContent = "Camera: Free World View";

        offsetX = e.clientX - dragStartX;
        offsetY = e.clientY - dragStartY;
        render();
        return;
      }

      const rect = canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;
      const worldCoord = canvasToWorld(mouseX, mouseY);
      hoverCoord.textContent = `X: ${worldCoord.x.toFixed(1)}m | Y: ${worldCoord.y.toFixed(1)}m`;

      // Comprehensive Elevation-Aware Hover
      const ego = getEgoPose(simTime);
      const entity = findEntityAt(worldCoord, ego);

      if (entity) {
        hoveredEntity = entity;

        // Position & Display Tooltip
        tooltip.style.display = 'block';
        const tooltipX = Math.min(window.innerWidth - 280, e.clientX + 16);
        const tooltipY = Math.min(window.innerHeight - 200, e.clientY + 16);
        tooltip.style.left = `${tooltipX}px`;
        tooltip.style.top = `${tooltipY}px`;
        tooltip.innerHTML = formatTooltipHtml(entity);

        // Also live-update sidebar inspector card
        populateInspector(entity);
        render();
      } else {
        if (hoveredEntity) {
          hoveredEntity = null;
          render();
        }
        tooltip.style.display = 'none';

        if (selectedEntity) {
          populateInspector(selectedEntity);
        } else {
          populateInspector(null);
        }
      }
    });

    canvas.addEventListener('mouseleave', () => {
      if (hoveredEntity) {
        hoveredEntity = null;
        render();
      }
      tooltip.style.display = 'none';
      if (selectedEntity) {
        populateInspector(selectedEntity);
      } else {
        populateInspector(null);
      }
    });

    // Click to Select & Lock Entity
    canvas.addEventListener('click', (e) => {
      const rect = canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;
      const worldCoord = canvasToWorld(mouseX, mouseY);
      const ego = getEgoPose(simTime);
      const entity = findEntityAt(worldCoord, ego);

      if (entity) {
        selectedEntity = entity;
        populateInspector(entity);
      } else {
        selectedEntity = null;
        populateInspector(null);
      }
      render();
    });

    // Mouse Wheel Zoom
    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
      scale = Math.max(3.5, Math.min(50.0, scale * zoomFactor));
      zoomStatus.textContent = `Zoom: ${(scale / 11.5 * 100).toFixed(0)}%`;
      render();
    }, { passive: false });
  }

  // Initialization & Kickoff
  async function init() {
    resizeCanvas();
    setupEvents();

    try {
      let stResp = null;
      try {
        stResp = await fetch(resolvePath('api/status'));
      } catch (_) {}
      if (!stResp || !stResp.ok) {
        stResp = await fetch(resolvePath('api/status.json'));
      }
      if (stResp && stResp.ok) {
        const st = await stResp.json();
        worldMapData = st.world_map;
        if (modelModeVal && st.model_mode) {
          modelModeVal.textContent = st.model_mode;
        }
      }
    } catch (e) {
      console.warn("Status fetch warning:", e);
    }

    // Load Frame 1 and launch continuous RAF animation loop
    await fetchKeyframe(1);
    prefetchNearbyFrames(1);
    requestAnimationFrame(animationLoop);
  }

  init();
})();
