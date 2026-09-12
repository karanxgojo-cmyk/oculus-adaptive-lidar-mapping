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

  // Selected Inspector State
  let selectedEntity = null;
  let selectedType = null; // 'object' or 'cell'

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

    // 2. Draw Range Rings & Rotating LiDAR FOV Cone (Centered on Ego Vehicle)
    if (chkRings.checked) {
      drawRangeRingsAndFov(ego);
    }

    // 3. Draw Adaptive Grid Cells (Transformed to World Coordinates)
    if (chkGrid.checked && currentFrameData && currentFrameData.active_cells) {
      drawAdaptiveGridCells(currentFrameData.active_cells, ego);
    }

    // 4. Draw Sampled LiDAR Points
    if (chkPoints.checked && currentFrameData && currentFrameData.points_sample) {
      drawSampledPoints(currentFrameData.points_sample, currentFrameData.point_labels_sample, ego);
    }

    // 5. Draw Dynamic Actors (Pedestrian Crossing, Oncoming Car, Lead Car)
    if (chkObjects.checked) {
      drawDynamicActors(ego);
    }

    // 6. Draw Selected Highlight
    if (selectedEntity) {
      drawSelectedHighlight(ego);
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
    const rings = [10, 25, 50, 80];

    ctx.save();

    // Range Rings
    rings.forEach(r => {
      const rPx = r * scale;
      ctx.beginPath();
      ctx.arc(center.px, center.py, rPx, 0, 2 * Math.PI);
      ctx.strokeStyle = 'rgba(0, 240, 255, 0.10)';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.stroke();

      ctx.fillStyle = 'rgba(0, 240, 255, 0.35)';
      ctx.font = '9px JetBrains Mono, monospace';
      ctx.fillText(`${r}m`, center.px + 5, center.py - rPx - 3);
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

    ctx.fillStyle = 'rgba(0, 240, 255, 0.025)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.15)';
    ctx.lineWidth = 1;
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

    ctx.restore();
  }

  // --------------------------------------------------------------------------
  // 4. ADAPTIVE GRID CELLS (TRANSFORMED FROM SENSOR FRAME TO WORLD FRAME)
  // --------------------------------------------------------------------------
  function drawAdaptiveGridCells(cells, ego) {
    ctx.save();
    const yawRad = (ego.yaw * Math.PI) / 180.0;
    const cosA = Math.cos(yawRad);
    const sinA = Math.sin(yawRad);

    cells.forEach(cell => {
      // Local cell center -> World frame
      const wx = ego.x + cell.cx * cosA - cell.cy * sinA;
      const wy = ego.y + cell.cx * sinA + cell.cy * cosA;

      const pCenter = worldToCanvas(wx, wy);
      const cellSizePx = cell.res * scale;

      // Color scheme based on active layer
      if (chkDanger.checked) {
        ctx.fillStyle = cell.d_lvl === 'DANGER' ? 'rgba(239, 68, 68, 0.70)' :
                        (cell.d_lvl === 'WARNING' ? 'rgba(249, 115, 22, 0.55)' :
                        (cell.d_lvl === 'CAUTION' ? 'rgba(245, 158, 11, 0.40)' : 'rgba(16, 185, 129, 0.15)'));
      } else if (chkElevation.checked) {
        const normElev = Math.min(Math.max((cell.elev + 2.0) / 4.0, 0.0), 1.0);
        ctx.fillStyle = `rgba(${Math.floor(normElev * 255)}, ${Math.floor((1 - normElev) * 200)}, 255, 0.42)`;
      } else {
        // STRICT HIERARCHICAL COLOR: RED = 5cm, YELLOW = 10-20cm, BLUE = 40-80cm
        ctx.fillStyle = LEVEL_COLORS[cell.lvl] || LEVEL_COLORS[4];
      }

      ctx.fillRect(pCenter.px - cellSizePx / 2, pCenter.py - cellSizePx / 2, cellSizePx, cellSizePx);

      ctx.strokeStyle = 'rgba(0, 0, 0, 0.25)';
      ctx.lineWidth = 0.5;
      ctx.strokeRect(pCenter.px - cellSizePx / 2, pCenter.py - cellSizePx / 2, cellSizePx, cellSizePx);
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
      const wx = ego.x + pt[0] * cosA - pt[1] * sinA;
      const wy = ego.y + pt[0] * sinA + pt[1] * cosA;
      const p = worldToCanvas(wx, wy);
      const semClass = (labels && chkSemantics.checked) ? labels[i] : 1;

      ctx.fillStyle = SEM_COLORS[semClass] || '#00f0ff';
      ctx.fillRect(p.px - 0.9, p.py - 0.9, 1.8, 1.8);
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

      // 3D Bounding Box
      ctx.strokeStyle = pedColor;
      ctx.lineWidth = 1.6;
      ctx.strokeRect(-8, -8, 16, 16);

      // Warning Badge
      ctx.fillStyle = 'rgba(6, 9, 19, 0.88)';
      ctx.fillRect(-38, -25, 76, 14);
      ctx.fillStyle = pedColor;
      ctx.font = 'bold 8px JetBrains Mono, monospace';
      ctx.fillText(`PEDESTRIAN [${pedDanger}]`, -34, -15);

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

      // Warning Badge
      ctx.fillStyle = 'rgba(6, 9, 19, 0.88)';
      ctx.fillRect(-35, -oW / 2 - 18, 70, 14);
      ctx.fillStyle = '#f59e0b';
      ctx.font = 'bold 8px JetBrains Mono, monospace';
      ctx.fillText("ONCOMING [18 km/h]", -31, -oW / 2 - 8);

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
  // 7. SELECTED ENTITY HIGHLIGHT
  // --------------------------------------------------------------------------
  function drawSelectedHighlight(ego) {
    if (selectedType === 'object' && selectedEntity) {
      const yawRad = (ego.yaw * Math.PI) / 180.0;
      const cosA = Math.cos(yawRad);
      const sinA = Math.sin(yawRad);
      const wx = ego.x + selectedEntity.center[0] * cosA - selectedEntity.center[1] * sinA;
      const wy = ego.y + selectedEntity.center[0] * sinA + selectedEntity.center[1] * cosA;
      const p = worldToCanvas(wx, wy);

      ctx.save();
      ctx.beginPath();
      ctx.arc(p.px, p.py, 4.5 * scale, 0, 2 * Math.PI);
      ctx.strokeStyle = 'rgba(239, 68, 68, 0.9)';
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 4]);
      ctx.stroke();
      ctx.fillStyle = 'rgba(239, 68, 68, 0.12)';
      ctx.fill();
      ctx.restore();
    }
  }

  // Populate Selected Entity Inspector
  function populateInspector(data, isObject = true) {
    if (!data) {
      inspectorContent.innerHTML = '<div class="inspector-empty">Click on an obstacle, vehicle, or grid cell on the map to inspect live perception and risk data.</div>';
      inspectorLevelBadge.textContent = "IDLE";
      inspectorLevelBadge.className = "card-tag";
      return;
    }

    const lvl = data.danger_level || data.d_lvl || "SAFE";
    inspectorLevelBadge.textContent = lvl;
    inspectorLevelBadge.className = `card-tag tag-${lvl.toLowerCase()}`;

    if (isObject) {
      inspectorContent.innerHTML = `
        <div class="insp-row"><span class="insp-key">Actor Class:</span><span class="insp-val text-cyan">${data.class_name}</span></div>
        <div class="insp-row"><span class="insp-key">Confidence:</span><span class="insp-val">${(data.confidence * 100).toFixed(1)}%</span></div>
        <div class="insp-row"><span class="insp-key">Distance:</span><span class="insp-val">${data.distance.toFixed(1)} m</span></div>
        <div class="insp-row"><span class="insp-key">Sensor Pos (X, Y, Z):</span><span class="insp-val">(${data.center[0].toFixed(1)}, ${data.center[1].toFixed(1)}, ${data.center[2].toFixed(1)}) m</span></div>
        <div class="insp-row"><span class="insp-key">Motion State:</span><span class="insp-val ${data.is_dynamic ? 'text-amber' : ''}">${data.is_dynamic ? 'Dynamic (Moving)' : 'Static'}</span></div>
        <div class="insp-row"><span class="insp-key">Velocity:</span><span class="insp-val">(${data.velocity[0].toFixed(1)}, ${data.velocity[1].toFixed(1)}) m/s</span></div>
        <div class="insp-row"><span class="insp-key">Danger Probability:</span><span class="insp-val text-amber">${(data.danger_prob * 100).toFixed(1)}%</span></div>
        <div class="insp-row highlight-row"><span class="insp-key">Allocated Resolution:</span><span class="insp-val text-cyan">${(data.resolution * 100).toFixed(0)} cm (${data.resolution <= 0.05 ? 'RED' : 'YELLOW'})</span></div>
      `;
    } else {
      inspectorContent.innerHTML = `
        <div class="insp-row"><span class="insp-key">Grid Level:</span><span class="insp-val text-cyan">Level ${data.lvl} (${(data.res * 100).toFixed(0)} cm)</span></div>
        <div class="insp-row"><span class="insp-key">Center (X, Y):</span><span class="insp-val">(${data.cx.toFixed(1)}, ${data.cy.toFixed(1)}) m</span></div>
        <div class="insp-row"><span class="insp-key">Dominant Class:</span><span class="insp-val">${data.sem_name}</span></div>
        <div class="insp-row"><span class="insp-key">Confidence:</span><span class="insp-val">${(data.conf * 100).toFixed(0)}%</span></div>
        <div class="insp-row"><span class="insp-key">Median Elevation:</span><span class="insp-val">${data.elev.toFixed(2)} m</span></div>
        <div class="insp-row"><span class="insp-key">Danger Level:</span><span class="insp-val">${data.d_lvl}</span></div>
        <div class="insp-row"><span class="insp-key">Importance Score:</span><span class="insp-val">${data.imp.toFixed(3)}</span></div>
      `;
    }
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

      // Tooltip Inspection
      if (!currentFrameData) return;
      const ego = getEgoPose(simTime);
      let hovered = null;

      // Check objects
      if (currentFrameData.detected_objects) {
        const yawRad = (ego.yaw * Math.PI) / 180.0;
        const cosA = Math.cos(yawRad);
        const sinA = Math.sin(yawRad);

        for (const obj of currentFrameData.detected_objects) {
          const wx = ego.x + obj.center[0] * cosA - obj.center[1] * sinA;
          const wy = ego.y + obj.center[0] * sinA + obj.center[1] * cosA;
          const dist = Math.hypot(worldCoord.x - wx, worldCoord.y - wy);
          if (dist <= 2.5) {
            hovered = { type: 'object', data: obj };
            break;
          }
        }
      }

      // Check cells
      if (!hovered && currentFrameData.active_cells) {
        const yawRad = (ego.yaw * Math.PI) / 180.0;
        const cosA = Math.cos(yawRad);
        const sinA = Math.sin(yawRad);

        for (const c of currentFrameData.active_cells) {
          const wx = ego.x + c.cx * cosA - c.cy * sinA;
          const wy = ego.y + c.cx * sinA + c.cy * cosA;
          if (Math.abs(worldCoord.x - wx) <= c.res && Math.abs(worldCoord.y - wy) <= c.res) {
            hovered = { type: 'cell', data: c };
            break;
          }
        }
      }

      if (hovered) {
        tooltip.style.display = 'block';
        tooltip.style.left = `${e.clientX + 14}px`;
        tooltip.style.top = `${e.clientY + 14}px`;
        if (hovered.type === 'object') {
          const o = hovered.data;
          tooltip.innerHTML = `<strong>${o.class_name}</strong> (${(o.confidence * 100).toFixed(0)}%)\nDist: ${o.distance.toFixed(1)}m | Danger: ${o.danger_level}\nRes: ${(o.resolution * 100).toFixed(0)}cm (${o.resolution <= 0.05 ? 'RED' : 'YELLOW'})`;
        } else {
          const c = hovered.data;
          const colorName = c.lvl === 0 ? 'RED (5cm)' : (c.lvl <= 2 ? 'YELLOW (10-20cm)' : 'BLUE (40-80cm)');
          tooltip.innerHTML = `<strong>Cell Level ${c.lvl} (${colorName})</strong>\nClass: ${c.sem_name} | Elev: ${c.elev.toFixed(2)}m\nDanger: ${c.d_lvl}`;
        }
      } else {
        tooltip.style.display = 'none';
      }
    });

    // Click to Select & Inspect Entity
    canvas.addEventListener('click', (e) => {
      const rect = canvas.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;
      const worldCoord = canvasToWorld(mouseX, mouseY);

      if (!currentFrameData) return;
      const ego = getEgoPose(simTime);
      const yawRad = (ego.yaw * Math.PI) / 180.0;
      const cosA = Math.cos(yawRad);
      const sinA = Math.sin(yawRad);

      if (currentFrameData.detected_objects) {
        for (const obj of currentFrameData.detected_objects) {
          const wx = ego.x + obj.center[0] * cosA - obj.center[1] * sinA;
          const wy = ego.y + obj.center[0] * sinA + obj.center[1] * cosA;
          const dist = Math.hypot(worldCoord.x - wx, worldCoord.y - wy);
          if (dist <= 3.0) {
            selectedEntity = obj;
            selectedType = 'object';
            populateInspector(obj, true);
            render();
            return;
          }
        }
      }

      if (currentFrameData.active_cells) {
        for (const c of currentFrameData.active_cells) {
          const wx = ego.x + c.cx * cosA - c.cy * sinA;
          const wy = ego.y + c.cx * sinA + c.cy * cosA;
          if (Math.abs(worldCoord.x - wx) <= c.res && Math.abs(worldCoord.y - wy) <= c.res) {
            selectedEntity = c;
            selectedType = 'cell';
            populateInspector(c, false);
            render();
            return;
          }
        }
      }

      // Deselect
      selectedEntity = null;
      selectedType = null;
      populateInspector(null);
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
