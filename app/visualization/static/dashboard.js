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
  const chkNavRoute = document.getElementById('chk-nav-route');

  // Live Navigation & Road Network Elements
  const routeStatusChip = document.getElementById('route-status-chip');
  const routeRoadName = document.getElementById('route-road-name');
  const routeRoadType = document.getElementById('route-road-type');
  const routeManeuverIcon = document.getElementById('route-maneuver-icon');
  const routeManeuverText = document.getElementById('route-maneuver-text');
  const routeManeuverDist = document.getElementById('route-maneuver-dist');
  const routeProgressVal = document.getElementById('route-progress-val');
  const navHudFloating = document.getElementById('nav-hud-floating');
  const navHudIcon = document.getElementById('nav-hud-icon');
  const navHudAction = document.getElementById('nav-hud-action');
  const navHudBadge = document.getElementById('nav-hud-badge');
  const navHudRoad = document.getElementById('nav-hud-road');

  // Live Device Geolocation Elements
  const geoAccuracyVal = document.getElementById('geo-accuracy-val');
  const geoCoordsVal = document.getElementById('geo-coords-val');
  const geoAltitudeVal = document.getElementById('geo-altitude-val');
  const geoSpeedVal = document.getElementById('geo-speed-val');
  const geoTimestampVal = document.getElementById('geo-timestamp-val');
  const pillOsm = document.getElementById('pill-osm');
  const pillGps = document.getElementById('pill-gps');
  const pillMatch = document.getElementById('pill-match');
  const pillRoute = document.getElementById('pill-route');
  const destStatusText = document.getElementById('dest-status-text');

  // Optical Sensor Elements
  const btnCameraSensor = document.getElementById('btn-camera-sensor');
  const cameraSensorDock = document.getElementById('camera-sensor-dock');
  const cameraVideo = document.getElementById('camera-video');
  const btnCloseCamera = document.getElementById('btn-close-camera');
  let cameraStream = null;

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

  function createBaselineInitialFrame() {
    const pts = [];
    const labels = [];
    // Generate concentric ground scan rings and curbs
    for (let r = 2; r <= 60; r += (r < 15 ? 1.5 : 3.5)) {
      const step = Math.max(0.12, 0.45 / (r * 0.1));
      for (let th = -Math.PI; th < Math.PI; th += step) {
        const px = r * Math.cos(th);
        const py = r * Math.sin(th);
        const pz = -0.05 + Math.sin(px * 0.1) * 0.04;
        pts.push([px, py, pz]);
        const sem = Math.abs(py) < 4.5 ? 1 : (Math.abs(py) < 6.5 ? 2 : (r > 35 ? 3 : 0));
        labels.push(sem);
      }
    }
    // Add point clusters for nearby vehicles & obstacles
    const objClusters = [
      { x: 18.0, y: 0.0, z: 0.7, sem: 4, count: 80, spread: 1.2 },
      { x: 12.0, y: 4.5, z: 0.8, sem: 5, count: 50, spread: 0.5 },
      { x: 12.0, y: -6.5, z: 2.5, sem: 6, count: 45, spread: 0.2 },
      { x: 12.0, y: 6.5, z: 2.5, sem: 6, count: 45, spread: 0.2 },
      { x: 35.0, y: 3.8, z: 0.7, sem: 4, count: 60, spread: 1.2 },
      { x: 24.0, y: -5.2, z: 0.5, sem: 3, count: 65, spread: 1.5 }
    ];
    for (const c of objClusters) {
      for (let k = 0; k < c.count; k++) {
        pts.push([
          c.x + (Math.random() - 0.5) * c.spread,
          c.y + (Math.random() - 0.5) * c.spread,
          c.z + (Math.random() - 0.5) * (c.spread * 1.5)
        ]);
        labels.push(c.sem);
      }
    }

    return {
      frame_id: 1,
      phase_name: 'Montgomery Arterial Corridor',
      description: 'Active perception-guided adaptive 2.5D LiDAR mapping actively optimizing radial grid resolution.',
      points_sample: pts,
      point_labels_sample: labels,
      detected_objects: [
        { id: 1, class_name: 'Pole', semantic_class: 6, confidence: 0.94, center: [12.0, -6.5, 2.67], dimensions: [0.22, 0.23, 5.05], distance: 13.66, is_dynamic: false, velocity: [0.0, 0.0], complexity: 1.0, complexity_cat: 'HIGH', danger_score: 0.21, danger_prob: 0.08, danger_level: 'SAFE', resolution: 0.1, points: 109, base_elev: 0.15, top_elev: 5.2, height: 5.05 },
        { id: 2, class_name: 'Pole', semantic_class: 6, confidence: 0.93, center: [12.0, 6.5, 2.67], dimensions: [0.22, 0.23, 5.05], distance: 13.66, is_dynamic: false, velocity: [0.0, 0.0], complexity: 1.0, complexity_cat: 'HIGH', danger_score: 0.21, danger_prob: 0.08, danger_level: 'SAFE', resolution: 0.1, points: 110, base_elev: 0.15, top_elev: 5.2, height: 5.05 },
        { id: 3, class_name: 'Pedestrian', semantic_class: 5, confidence: 0.95, center: [12.0, 4.5, 0.88], dimensions: [0.6, 0.6, 1.75], distance: 12.8, is_dynamic: true, velocity: [1.2, 0.0], complexity: 0.92, complexity_cat: 'HIGH', danger_score: 0.88, danger_prob: 0.85, danger_level: 'DANGER', resolution: 0.05, points: 128, base_elev: 0.0, top_elev: 1.75, height: 1.75 },
        { id: 4, class_name: 'Vehicle', semantic_class: 4, confidence: 0.97, center: [18.0, 0.0, 0.75], dimensions: [4.7, 2.0, 1.5], distance: 18.0, is_dynamic: true, velocity: [8.5, 0.0], complexity: 0.85, complexity_cat: 'HIGH', danger_score: 0.65, danger_prob: 0.45, danger_level: 'CAUTION', resolution: 0.05, points: 280, base_elev: 0.0, top_elev: 1.5, height: 1.5 },
        { id: 5, class_name: 'Barrier', semantic_class: 3, confidence: 0.91, center: [24.0, -5.2, 0.5], dimensions: [0.6, 6.0, 1.0], distance: 24.5, is_dynamic: false, velocity: [0.0, 0.0], complexity: 0.70, complexity_cat: 'MEDIUM', danger_score: 0.35, danger_prob: 0.15, danger_level: 'SAFE', resolution: 0.1, points: 145, base_elev: 0.0, top_elev: 1.0, height: 1.0 }
      ],
      metrics: {
        fps: 30.0,
        latency_ms: 14.2,
        point_count: pts.length,
        active_adaptive_cells: 1716,
        theoretical_uniform_cells: 3200000,
        cell_reduction_percent: 99.8,
        estimated_adaptive_storage_kb: 106.6,
        estimated_uniform_storage_kb: 200000.0,
        dynamic_object_count: 2,
        current_frame: 1
      }
    };
  }

  let currentFrameData = createBaselineInitialFrame();
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

  // Trajectory Math: Continuous ego vehicle pose
  // Evaluates live dynamic OSM routes (Downtown Grid, Roundabout, Overpass, Live GPS)
  // or the calibrated multi-topology benchmark loop.
  function getEgoPose(tSec) {
    if (window.osmRouter) {
      return window.osmRouter.getEgoPose(tSec);
    }

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
    if (chkWorld.checked) {
      drawWorldMap(worldMapData, ego);
    }

    // 1b. Draw Active Dynamic OSM Route Trajectory & Turn Markers
    if (chkNavRoute && chkNavRoute.checked && window.osmRouter) {
      window.osmRouter.drawRouteOverlay(ctx, worldToCanvas, ego);
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
  // --------------------------------------------------------------------------
  // 1. FIXED STATIC WORLD MAP & ACTIVE ROAD NETWORK (PERMANENT WORLD COORDINATES)
  // --------------------------------------------------------------------------

  // Helper: Draw continuous connecting return roadways for 4-Way Intersection presets
  function draw4WayReturnRoadways(mode) {
    ctx.save();
    ctx.strokeStyle = '#0b111d';
    ctx.lineWidth = 12.0 * scale;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    let returnPts = [];
    if (mode === '4way_left') {
      returnPts = [
        [-180, 0], [-190, -80], [-150, -160], [-60, -185], [0, -180]
      ];
    } else if (mode === '4way_right') {
      returnPts = [
        [180, 0], [190, -80], [150, -160], [60, -185], [0, -180]
      ];
    } else {
      returnPts = [
        [0, 180], [60, 190], [120, 150], [120, -150], [60, -190], [0, -180]
      ];
    }

    if (returnPts.length > 1) {
      // Road Asphalt Base
      ctx.beginPath();
      const p0 = worldToCanvas(returnPts[0][0], returnPts[0][1]);
      ctx.moveTo(p0.px, p0.py);
      for (let i = 1; i < returnPts.length; i++) {
        const cp = worldToCanvas(returnPts[i][0], returnPts[i][1]);
        ctx.lineTo(cp.px, cp.py);
      }
      ctx.stroke();

      // Road Curbs
      ctx.strokeStyle = '#223247';
      ctx.lineWidth = 2.0;
      ctx.stroke();

      // Yellow Dashed Centerline
      ctx.strokeStyle = '#fbbf24';
      ctx.lineWidth = 1.8;
      ctx.setLineDash([8, 6]);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    ctx.restore();
  }

  // Helper: Draw fixed static landmarks around 4-Way Intersection
  function draw4WayStaticLandmarks() {
    ctx.save();

    // 4 Corner Urban Buildings
    const cornerBuildings = [
      { x: 28, y: 28, w: 48, h: 48, label: 'Tech Tower (NE)' },
      { x: -76, y: 28, w: 48, h: 48, label: 'Financial Center (NW)' },
      { x: -76, y: -76, w: 48, h: 48, label: 'Transit Terminal (SW)' },
      { x: 28, y: -76, w: 48, h: 48, label: 'Commerce Plaza (SE)' }
    ];

    cornerBuildings.forEach(b => {
      const bp = worldToCanvas(b.x + b.w / 2, b.y + b.h / 2);
      const bw = b.w * scale;
      const bh = b.h * scale;

      ctx.fillStyle = 'rgba(15, 23, 42, 0.94)';
      ctx.fillRect(bp.px - bw / 2, bp.py - bh / 2, bw, bh);
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
      ctx.lineWidth = 1.2;
      ctx.strokeRect(bp.px - bw / 2, bp.py - bh / 2, bw, bh);

      ctx.fillStyle = 'rgba(148, 163, 184, 0.75)';
      ctx.font = '8px JetBrains Mono, monospace';
      ctx.fillText(b.label, bp.px - bw / 2 + 5, bp.py - bh / 2 + 14);
    });

    // Street light poles along sidewalks
    const poles = [
      { x: 8.5, y: -40 }, { x: -8.5, y: -40 },
      { x: 8.5, y: -90 }, { x: -8.5, y: -90 },
      { x: 8.5, y: -140 }, { x: -8.5, y: -140 },
      { x: 8.5, y: 40 }, { x: -8.5, y: 40 },
      { x: 8.5, y: 90 }, { x: -8.5, y: 90 },
      { x: 8.5, y: 140 }, { x: -8.5, y: 140 },
      { x: -40, y: 8.5 }, { x: -40, y: -8.5 },
      { x: -90, y: 8.5 }, { x: -90, y: -8.5 },
      { x: -140, y: 8.5 }, { x: -140, y: -8.5 },
      { x: 40, y: 8.5 }, { x: 40, y: -8.5 },
      { x: 90, y: 8.5 }, { x: 90, y: -8.5 },
      { x: 140, y: 8.5 }, { x: 140, y: -8.5 }
    ];

    poles.forEach(p => {
      const pt = worldToCanvas(p.x, p.y);
      ctx.beginPath();
      ctx.arc(pt.px, pt.py, 3.2, 0, 2 * Math.PI);
      ctx.fillStyle = '#475569';
      ctx.fill();
      ctx.strokeStyle = '#94a3b8';
      ctx.lineWidth = 1;
      ctx.stroke();
    });

    // Sidewalk shade trees along sidewalks
    const trees = [
      { x: 11.5, y: -60, r: 3.5 }, { x: -11.5, y: -60, r: 3.5 },
      { x: 11.5, y: -115, r: 3.5 }, { x: -11.5, y: -115, r: 3.5 },
      { x: 11.5, y: 60, r: 3.5 }, { x: -11.5, y: 60, r: 3.5 },
      { x: 11.5, y: 115, r: 3.5 }, { x: -11.5, y: 115, r: 3.5 },
      { x: -60, y: 11.5, r: 3.5 }, { x: -60, y: -11.5, r: 3.5 },
      { x: -115, y: 11.5, r: 3.5 }, { x: -115, y: -11.5, r: 3.5 },
      { x: 60, y: 11.5, r: 3.5 }, { x: 60, y: -11.5, r: 3.5 },
      { x: 115, y: 11.5, r: 3.5 }, { x: 115, y: -11.5, r: 3.5 }
    ];

    trees.forEach(tr => {
      const pt = worldToCanvas(tr.x, tr.y);
      const rPx = tr.r * scale;
      ctx.beginPath();
      ctx.arc(pt.px, pt.py, rPx, 0, 2 * Math.PI);
      ctx.fillStyle = 'rgba(22, 101, 52, 0.45)';
      ctx.fill();
      ctx.strokeStyle = '#15803d';
      ctx.lineWidth = 1.2;
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(pt.px, pt.py, rPx * 0.4, 0, 2 * Math.PI);
      ctx.fillStyle = 'rgba(34, 197, 94, 0.6)';
      ctx.fill();
    });

    ctx.restore();
  }

  // Helper: Draw geometric roundabout road network (Arc de Triomphe)
  function drawRoundaboutRoadways(net) {
    ctx.save();
    const rCenter = worldToCanvas(net.roundaboutCenter[0], net.roundaboutCenter[1]);
    const rOuter = (net.roundaboutRadius + 9.0) * scale;
    const rCenterline = net.roundaboutRadius * scale;
    const rInner = net.islandRadius * scale;

    // 1. Outer asphalt rotary circle
    ctx.beginPath();
    ctx.arc(rCenter.px, rCenter.py, rOuter, 0, 2 * Math.PI);
    ctx.fillStyle = '#0b111d';
    ctx.fill();

    // 2. Inner Central Island
    ctx.beginPath();
    ctx.arc(rCenter.px, rCenter.py, rInner, 0, 2 * Math.PI);
    ctx.fillStyle = '#061018';
    ctx.fill();
    ctx.strokeStyle = '#223247';
    ctx.lineWidth = 3.0;
    ctx.stroke();

    // 3. Central Monument Plinth (Arc de Triomphe)
    const plinthW = 26.0 * scale;
    const plinthH = 18.0 * scale;
    ctx.fillStyle = '#1e293b';
    ctx.fillRect(rCenter.px - plinthW / 2, rCenter.py - plinthH / 2, plinthW, plinthH);
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = 1.5;
    ctx.shadowColor = '#00f0ff';
    ctx.shadowBlur = 8;
    ctx.strokeRect(rCenter.px - plinthW / 2, rCenter.py - plinthH / 2, plinthW, plinthH);
    ctx.shadowBlur = 0;

    // Archway cutout
    ctx.fillStyle = '#060a12';
    ctx.fillRect(rCenter.px - plinthW * 0.22, rCenter.py - plinthH / 2, plinthW * 0.44, plinthH);

    ctx.fillStyle = '#f8fafc';
    ctx.font = 'bold 8px JetBrains Mono, monospace';
    ctx.fillText('Arc de Triomphe', rCenter.px - 38, rCenter.py + 3);

    // 4. Circular Lane Centerline & Dashes
    // Outer curb
    ctx.beginPath();
    ctx.arc(rCenter.px, rCenter.py, rOuter, 0, 2 * Math.PI);
    ctx.strokeStyle = '#223247';
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Centerline Yellow Dash
    ctx.beginPath();
    ctx.arc(rCenter.px, rCenter.py, rCenterline, 0, 2 * Math.PI);
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth = 2.0;
    ctx.setLineDash([8, 6]);
    ctx.stroke();
    ctx.setLineDash([]);

    // 5. Radial Spoke Avenues
    if (net.segments) {
      net.segments.forEach(seg => {
        const p1 = worldToCanvas(seg.start[0], seg.start[1]);
        const p2 = worldToCanvas(seg.end[0], seg.end[1]);
        const wPx = seg.width * scale;

        ctx.beginPath();
        ctx.moveTo(p1.px, p1.py);
        ctx.lineTo(p2.px, p2.py);
        ctx.strokeStyle = '#0b111d';
        ctx.lineWidth = wPx;
        ctx.stroke();

        ctx.strokeStyle = '#223247';
        ctx.lineWidth = 2.0;
        ctx.stroke();

        ctx.strokeStyle = '#fbbf24';
        ctx.lineWidth = 1.8;
        ctx.setLineDash([8, 6]);
        ctx.stroke();
        ctx.setLineDash([]);
      });
    }

    // 6. Return Connector Road from Exit to Entry
    const returnPts = [
      [80, 70], [140, 120], [80, 160], [-60, 160], [-170, 0], [-140, -120]
    ];
    ctx.beginPath();
    const rp0 = worldToCanvas(returnPts[0][0], returnPts[0][1]);
    ctx.moveTo(rp0.px, rp0.py);
    for (let i = 1; i < returnPts.length; i++) {
      const cp = worldToCanvas(returnPts[i][0], returnPts[i][1]);
      ctx.lineTo(cp.px, cp.py);
    }
    ctx.strokeStyle = '#0b111d';
    ctx.lineWidth = 12.0 * scale;
    ctx.lineCap = 'round';
    ctx.stroke();
    ctx.strokeStyle = '#223247';
    ctx.lineWidth = 2.0;
    ctx.stroke();
    ctx.strokeStyle = '#fbbf24';
    ctx.lineWidth = 1.8;
    ctx.setLineDash([8, 6]);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.restore();
  }

  // Helper: Draw Downtown Street Grid
  function drawGridRoadways(net) {
    ctx.save();
    if (net.segments) {
      net.segments.forEach(seg => {
        const p1 = worldToCanvas(seg.start[0], seg.start[1]);
        const p2 = worldToCanvas(seg.end[0], seg.end[1]);
        const wPx = seg.width * scale;

        ctx.beginPath();
        ctx.moveTo(p1.px, p1.py);
        ctx.lineTo(p2.px, p2.py);
        ctx.strokeStyle = '#0b111d';
        ctx.lineWidth = wPx;
        ctx.stroke();

        ctx.strokeStyle = '#223247';
        ctx.lineWidth = 2.0;
        ctx.stroke();

        ctx.strokeStyle = '#fbbf24';
        ctx.lineWidth = 1.8;
        ctx.setLineDash([8, 6]);
        ctx.stroke();
        ctx.setLineDash([]);
      });
    }

    if (net.intersections) {
      net.intersections.forEach(inter => {
        const cp = worldToCanvas(inter.center[0], inter.center[1]);
        const wPx = inter.size[0] * scale;
        const hPx = inter.size[1] * scale;

        ctx.fillStyle = '#0b111d';
        ctx.fillRect(cp.px - wPx / 2, cp.py - hPx / 2, wPx, hPx);
        ctx.strokeStyle = '#223247';
        ctx.lineWidth = 2.0;
        ctx.strokeRect(cp.px - wPx / 2, cp.py - hPx / 2, wPx, hPx);
      });
    }
    ctx.restore();
  }

  // Helper: Draw Coastal Overpass & Viaduct
  function drawCoastalRoadways(net) {
    ctx.save();
    if (net.segments) {
      net.segments.forEach(seg => {
        const p1 = worldToCanvas(seg.start[0], seg.start[1]);
        const p2 = worldToCanvas(seg.end[0], seg.end[1]);
        const wPx = seg.width * scale;

        ctx.beginPath();
        ctx.moveTo(p1.px, p1.py);
        if (seg.control) {
          const cp = worldToCanvas(seg.control[0], seg.control[1]);
          ctx.quadraticCurveTo(cp.px, cp.py, p2.px, p2.py);
        } else {
          ctx.lineTo(p2.px, p2.py);
        }

        ctx.strokeStyle = '#0b111d';
        ctx.lineWidth = wPx;
        ctx.lineCap = 'round';
        ctx.stroke();

        ctx.strokeStyle = '#223247';
        ctx.lineWidth = 2.0;
        ctx.stroke();

        ctx.strokeStyle = '#fbbf24';
        ctx.lineWidth = 1.8;
        ctx.setLineDash([8, 6]);
        ctx.stroke();
        ctx.setLineDash([]);
      });
    }
    ctx.restore();
  }

  function drawWorldMap(mapData, ego) {
    ctx.save();

    const activeNet = (window.osmRouter && typeof window.osmRouter.getRoadNetworkElements === 'function')
      ? window.osmRouter.getRoadNetworkElements()
      : null;

    if (activeNet && activeNet.segments && activeNet.segments.length > 0) {
      // 1. Draw all road segments in active OSM network
      activeNet.segments.forEach(seg => {
        const p1 = worldToCanvas(seg.start[0], seg.start[1]);
        const p2 = worldToCanvas(seg.end[0], seg.end[1]);
        const roadWidthPx = (seg.width || 14.0) * scale;

        // Shoulder / sidewalk edge
        ctx.beginPath();
        ctx.moveTo(p1.px, p1.py);
        ctx.lineTo(p2.px, p2.py);
        ctx.strokeStyle = '#151f30';
        ctx.lineWidth = roadWidthPx + 4.5 * scale;
        ctx.lineCap = 'butt';
        ctx.stroke();

        // Asphalt surface
        ctx.beginPath();
        ctx.moveTo(p1.px, p1.py);
        ctx.lineTo(p2.px, p2.py);
        ctx.strokeStyle = '#0b111d';
        ctx.lineWidth = roadWidthPx;
        ctx.lineCap = 'butt';
        ctx.stroke();

        // Outer Curbs
        ctx.strokeStyle = '#2d3f57';
        ctx.lineWidth = 2.0;
        ctx.stroke();

        // Yellow dashed centerline
        ctx.save();
        ctx.strokeStyle = '#fbbf24';
        ctx.lineWidth = 1.8;
        ctx.setLineDash([8, 6]);
        ctx.beginPath();
        ctx.moveTo(p1.px, p1.py);
        ctx.lineTo(p2.px, p2.py);
        ctx.stroke();
        ctx.restore();

        // Floating Street Name Label (if road segment is long enough and camera is near)
        const segLen = Math.hypot(seg.end[0] - seg.start[0], seg.end[1] - seg.start[1]);
        const midX = (seg.start[0] + seg.end[0]) * 0.5;
        const midY = (seg.start[1] + seg.end[1]) * 0.5;
        const distEgo = Math.hypot(midX - ego.x, midY - ego.y);
        if (segLen >= 40 && distEgo <= 120 && seg.name && seg.name !== 'Unnamed Roadway') {
          const midCp = worldToCanvas(midX, midY);
          ctx.fillStyle = 'rgba(148, 163, 184, 0.45)';
          ctx.font = '7.5px JetBrains Mono, monospace';
          ctx.fillText(seg.name, midCp.px - 30, midCp.py - roadWidthPx / 2 - 4);
        }
      });

      // 2. Intersections & Crosswalks
      if (activeNet.intersections) {
        activeNet.intersections.forEach(inter => {
          const center = worldToCanvas(inter.center[0], inter.center[1]);
          const wPx = inter.size[0] * scale;
          const hPx = inter.size[1] * scale;

          ctx.fillStyle = '#0b111d';
          ctx.fillRect(center.px - wPx / 2, center.py - hPx / 2, wPx, hPx);
          ctx.strokeStyle = '#223247';
          ctx.lineWidth = 2.0;
          ctx.strokeRect(center.px - wPx / 2, center.py - hPx / 2, wPx, hPx);

          if (inter.crosswalks) {
            inter.crosswalks.forEach(cw => {
              const pt1 = worldToCanvas(cw.p1[0], cw.p1[1]);
              const pt2 = worldToCanvas(cw.p2[0], cw.p2[1]);
              ctx.save();
              ctx.strokeStyle = 'rgba(255, 255, 255, 0.85)';
              ctx.lineWidth = (cw.width || 3.6) * scale * 0.8;
              ctx.setLineDash([7, 5]);
              ctx.beginPath();
              ctx.moveTo(pt1.px, pt1.py);
              ctx.lineTo(pt2.px, pt2.py);
              ctx.stroke();
              ctx.restore();
            });
          }
        });
      }

      // 3. Traffic Signals
      if (activeNet.trafficSignals) {
        activeNet.trafficSignals.forEach(sig => {
          const sp = worldToCanvas(sig.x, sig.y);
          ctx.save();
          ctx.beginPath();
          ctx.arc(sp.px, sp.py, 4.0, 0, 2 * Math.PI);
          ctx.fillStyle = sig.state === 'green' ? '#10b981' : '#ef4444';
          ctx.shadowColor = sig.state === 'green' ? '#10b981' : '#ef4444';
          ctx.shadowBlur = 8;
          ctx.fill();
          ctx.strokeStyle = '#ffffff';
          ctx.lineWidth = 1.0;
          ctx.stroke();
          ctx.restore();
        });
      }

      // 4. Fixed Static Landmarks around the road network
      draw4WayStaticLandmarks();
    } else {
      // Benchmark Mode: Default calibrated multi-topology road segments
      if (mapData && mapData.segments) {
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

          ctx.strokeStyle = '#0e1626';
          ctx.lineWidth = roadWidthPx;
          ctx.lineCap = 'round';
          ctx.stroke();

          ctx.strokeStyle = '#223247';
          ctx.lineWidth = 2.0;
          ctx.stroke();

          ctx.strokeStyle = '#fbbf24';
          ctx.lineWidth = 1.8;
          ctx.setLineDash([8, 6]);
          ctx.stroke();
          ctx.setLineDash([]);
        });
      }

      if (mapData && mapData.intersection) {
        const inter = mapData.intersection;
        const center = worldToCanvas(inter.center[0], inter.center[1]);
        const wPx = inter.size[0] * scale;
        const hPx = inter.size[1] * scale;

        ctx.fillStyle = '#0e1626';
        ctx.fillRect(center.px - wPx / 2, center.py - hPx / 2, wPx, hPx);
        ctx.strokeStyle = '#334155';
        ctx.lineWidth = 1.5;
        ctx.strokeRect(center.px - wPx / 2, center.py - hPx / 2, wPx, hPx);

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
    }

    // STATIC POLES (PERMANENT WORLD POSITIONS - NEVER MOVE WITH VEHICLE)
    if (mapData && mapData.poles) {
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
    if (mapData && mapData.trees) {
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
    if (mapData && mapData.buildings) {
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
    if (mapData && mapData.barrier) {
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
    if (mapData && mapData.parked_cars) {
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
  // 3. TOP-DOWN AUTONOMOUS VEHICLE MODEL (HIGH-FIDELITY RESEARCH AV)
  // --------------------------------------------------------------------------
  function drawTopDownEgoVehicle(ego) {
    const center = worldToCanvas(ego.x, ego.y);
    const vehW = 2.0 * scale;
    const vehL = 4.7 * scale;
    const yawRad = (ego.yaw * Math.PI) / 180.0;
    const steerAngle = ego.steering || 0.0;

    ctx.save();
    ctx.translate(center.px, center.py);
    ctx.rotate(-yawRad); // Correct screen coordinate rotation (aligns local +X with vehicle forward heading)

    // 1. Dual Projector Headlights & Volumetric Road Beams
    const beamDist = 38.0 * scale;
    const beamSpread = 12.0 * scale;
    const hlX = vehL * 0.48;
    const hlYLeft = -vehW * 0.35;
    const hlYRight = vehW * 0.35;

    // Left Headlight Volumetric Cone
    const gradL = ctx.createLinearGradient(hlX, hlYLeft, hlX + beamDist, hlYLeft - beamSpread);
    gradL.addColorStop(0.0, 'rgba(0, 240, 255, 0.45)');
    gradL.addColorStop(0.25, 'rgba(0, 240, 255, 0.20)');
    gradL.addColorStop(0.7, 'rgba(0, 240, 255, 0.05)');
    gradL.addColorStop(1.0, 'rgba(0, 240, 255, 0.0)');

    ctx.beginPath();
    ctx.moveTo(hlX, hlYLeft);
    ctx.lineTo(hlX + beamDist, hlYLeft - beamSpread);
    ctx.lineTo(hlX + beamDist, hlYLeft + beamSpread * 0.35);
    ctx.closePath();
    ctx.fillStyle = gradL;
    ctx.fill();

    // Right Headlight Volumetric Cone
    const gradR = ctx.createLinearGradient(hlX, hlYRight, hlX + beamDist, hlYRight + beamSpread);
    gradR.addColorStop(0.0, 'rgba(0, 240, 255, 0.45)');
    gradR.addColorStop(0.25, 'rgba(0, 240, 255, 0.20)');
    gradR.addColorStop(0.7, 'rgba(0, 240, 255, 0.05)');
    gradR.addColorStop(1.0, 'rgba(0, 240, 255, 0.0)');

    ctx.beginPath();
    ctx.moveTo(hlX, hlYRight);
    ctx.lineTo(hlX + beamDist, hlYRight - beamSpread * 0.35);
    ctx.lineTo(hlX + beamDist, hlYRight + beamSpread);
    ctx.closePath();
    ctx.fillStyle = gradR;
    ctx.fill();

    // 2. Four Treaded Tires with Steered Front Wheels
    const tireL = 0.88 * scale;
    const tireW = 0.34 * scale;

    function drawTire(centerX, centerY, angleRad) {
      ctx.save();
      ctx.translate(centerX, centerY);
      if (angleRad) ctx.rotate(angleRad);

      ctx.fillStyle = '#0a0e17';
      ctx.fillRect(-tireL / 2, -tireW / 2, tireL, tireW);
      ctx.strokeStyle = '#334155';
      ctx.lineWidth = 1.2;
      ctx.strokeRect(-tireL / 2, -tireW / 2, tireL, tireW);

      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1.0;
      [-0.25, 0, 0.25].forEach(offsetRatio => {
        ctx.beginPath();
        ctx.moveTo(offsetRatio * tireL, -tireW / 2 + 1);
        ctx.lineTo(offsetRatio * tireL, tireW / 2 - 1);
        ctx.stroke();
      });

      ctx.fillStyle = '#475569';
      ctx.fillRect(-tireL * 0.18, -tireW * 0.18, tireL * 0.36, tireW * 0.36);

      ctx.restore();
    }

    // Rear Wheels (Fixed straight)
    drawTire(-vehL * 0.32, -vehW * 0.48, 0);
    drawTire(-vehL * 0.32, vehW * 0.48, 0);

    // Front Wheels (Steered by steerAngle)
    drawTire(vehL * 0.30, -vehW * 0.48, -steerAngle);
    drawTire(vehL * 0.30, vehW * 0.48, -steerAngle);

    // 3. Aerodynamic Sculpted Vehicle Body
    // Front Air Dam / Splitter Lip
    ctx.beginPath();
    ctx.moveTo(vehL * 0.50, -vehW * 0.30);
    ctx.quadraticCurveTo(vehL * 0.53, 0, vehL * 0.50, vehW * 0.30);
    ctx.lineTo(vehL * 0.48, vehW * 0.34);
    ctx.quadraticCurveTo(vehL * 0.51, 0, vehL * 0.48, -vehW * 0.34);
    ctx.closePath();
    ctx.fillStyle = '#060a12';
    ctx.fill();

    // Sculpted Main Body Silhouette
    ctx.beginPath();
    ctx.moveTo(vehL * 0.48, -vehW * 0.32);
    ctx.quadraticCurveTo(vehL * 0.52, 0, vehL * 0.48, vehW * 0.32);
    ctx.lineTo(vehL * 0.32, vehW * 0.48);
    ctx.lineTo(vehL * 0.05, vehW * 0.45);
    ctx.lineTo(-vehL * 0.08, vehW * 0.45);
    ctx.lineTo(-vehL * 0.28, vehW * 0.49);
    ctx.quadraticCurveTo(-vehL * 0.48, vehW * 0.46, -vehL * 0.50, vehW * 0.30);
    ctx.lineTo(-vehL * 0.50, -vehW * 0.30);
    ctx.quadraticCurveTo(-vehL * 0.48, -vehW * 0.46, -vehL * 0.28, -vehW * 0.49);
    ctx.lineTo(-vehL * 0.08, -vehW * 0.45);
    ctx.lineTo(vehL * 0.05, -vehW * 0.45);
    ctx.lineTo(vehL * 0.32, -vehW * 0.48);
    ctx.closePath();

    const bodyGrad = ctx.createLinearGradient(-vehL / 2, 0, vehL / 2, 0);
    bodyGrad.addColorStop(0.0, '#090e1a');
    bodyGrad.addColorStop(0.4, '#111d30');
    bodyGrad.addColorStop(0.8, '#1a2c47');
    bodyGrad.addColorStop(1.0, '#223859');
    ctx.fillStyle = bodyGrad;
    ctx.fill();

    // Glowing Cyan Trim Contour
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = 1.8;
    ctx.shadowColor = '#00f0ff';
    ctx.shadowBlur = 8;
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Sculpted Hood Power-Dome Crease Lines
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.40)';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(vehL * 0.45, -vehW * 0.16);
    ctx.lineTo(vehL * 0.20, -vehW * 0.24);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(vehL * 0.45, vehW * 0.16);
    ctx.lineTo(vehL * 0.20, vehW * 0.24);
    ctx.stroke();

    // Rear Aero Diffuser Fins
    ctx.fillStyle = '#060a12';
    ctx.fillRect(-vehL * 0.51, -vehW * 0.20, vehL * 0.03, vehW * 0.06);
    ctx.fillRect(-vehL * 0.51, -vehW * 0.03, vehL * 0.03, vehW * 0.06);
    ctx.fillRect(-vehL * 0.51, vehW * 0.14, vehL * 0.03, vehW * 0.06);

    // 4. Aerodynamic Side Mirrors with Amber LED Turn Indicators
    function drawMirror(mirrorY, isLeft) {
      const mX = vehL * 0.14;
      const mLen = vehL * 0.07;
      const mWidth = vehW * 0.12;

      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 2.0;
      ctx.beginPath();
      ctx.moveTo(mX, mirrorY * 0.85);
      ctx.lineTo(mX - vehL * 0.02, mirrorY);
      ctx.stroke();

      ctx.fillStyle = '#152238';
      ctx.strokeStyle = '#00f0ff';
      ctx.lineWidth = 1.0;
      ctx.fillRect(mX - mLen / 2, isLeft ? mirrorY - mWidth : mirrorY, mLen, mWidth);
      ctx.strokeRect(mX - mLen / 2, isLeft ? mirrorY - mWidth : mirrorY, mLen, mWidth);

      const isSignaling = (isLeft && steerAngle > 0.04) || (!isLeft && steerAngle < -0.04);
      const amberPulse = isSignaling ? (Math.sin(performance.now() * 0.012) > 0 ? '#f59e0b' : '#78350f') : '#d97706';
      ctx.fillStyle = amberPulse;
      if (isSignaling) {
        ctx.shadowColor = '#f59e0b';
        ctx.shadowBlur = 6;
      }
      ctx.fillRect(mX + mLen * 0.2, isLeft ? mirrorY - mWidth + 1 : mirrorY + 1, mLen * 0.25, mWidth - 2);
      ctx.shadowBlur = 0;
    }

    drawMirror(-vehW * 0.52, true);  // Left Mirror
    drawMirror(vehW * 0.52, false);  // Right Mirror

    // 5. Cabin Glasshouse (Windshield, Panoramic Roof, Rear Window)
    ctx.beginPath();
    ctx.moveTo(vehL * 0.20, -vehW * 0.36);
    ctx.quadraticCurveTo(vehL * 0.25, 0, vehL * 0.20, vehW * 0.36);
    ctx.lineTo(vehL * 0.04, vehW * 0.34);
    ctx.lineTo(vehL * 0.04, -vehW * 0.34);
    ctx.closePath();

    const glassGrad = ctx.createLinearGradient(vehL * 0.04, 0, vehL * 0.25, 0);
    glassGrad.addColorStop(0.0, 'rgba(6, 182, 212, 0.25)');
    glassGrad.addColorStop(1.0, 'rgba(0, 240, 255, 0.45)');
    ctx.fillStyle = glassGrad;
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.70)';
    ctx.lineWidth = 1.2;
    ctx.stroke();

    // Panoramic Glass Roof Panel
    ctx.fillStyle = '#050912';
    ctx.fillRect(-vehL * 0.22, -vehW * 0.33, vehL * 0.26, vehW * 0.66);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
    ctx.lineWidth = 1.0;
    ctx.strokeRect(-vehL * 0.22, -vehW * 0.33, vehL * 0.26, vehW * 0.66);

    // Rear Window
    ctx.beginPath();
    ctx.moveTo(-vehL * 0.22, -vehW * 0.33);
    ctx.lineTo(-vehL * 0.38, -vehW * 0.28);
    ctx.quadraticCurveTo(-vehL * 0.42, 0, -vehL * 0.38, vehW * 0.28);
    ctx.lineTo(-vehL * 0.22, vehW * 0.33);
    ctx.closePath();
    ctx.fillStyle = 'rgba(0, 240, 255, 0.22)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.45)';
    ctx.lineWidth = 1.0;
    ctx.stroke();

    // 6. Full-Width Modern Rear LED Taillight Bar
    const barX = -vehL * 0.50;
    const barYStart = -vehW * 0.38;
    const barYEnd = vehW * 0.38;

    ctx.save();
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 3.2;
    ctx.shadowColor = '#ef4444';
    ctx.shadowBlur = 10;
    ctx.beginPath();
    ctx.moveTo(barX, barYStart);
    ctx.lineTo(barX, barYEnd);
    ctx.stroke();

    ctx.strokeStyle = '#fecaca';
    ctx.lineWidth = 1.2;
    ctx.beginPath();
    ctx.moveTo(barX, barYStart + 2);
    ctx.lineTo(barX, barYEnd - 2);
    ctx.stroke();
    ctx.restore();

    // 7. Roof Autonomous Sensor Pod (Velodyne HDL-64E LiDAR Puck)
    ctx.fillStyle = '#1e293b';
    ctx.fillRect(-vehL * 0.08, -vehW * 0.22, vehL * 0.16, vehW * 0.44);
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.3)';
    ctx.lineWidth = 1;
    ctx.strokeRect(-vehL * 0.08, -vehW * 0.22, vehL * 0.16, vehW * 0.44);

    ctx.beginPath();
    ctx.arc(0, 0, 4.8, 0, 2 * Math.PI);
    ctx.fillStyle = '#090d16';
    ctx.fill();
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 1.8;
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(0, 0, 2.2, 0, 2 * Math.PI);
    ctx.fillStyle = '#ef4444';
    ctx.shadowColor = '#ef4444';
    ctx.shadowBlur = 8;
    ctx.fill();
    ctx.shadowBlur = 0;

    const sweepAngle = (performance.now() * 0.007) % (2 * Math.PI);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(Math.cos(sweepAngle) * 8.5 * scale, Math.sin(sweepAngle) * 8.5 * scale);
    ctx.strokeStyle = 'rgba(239, 68, 68, 0.45)';
    ctx.lineWidth = 1.4;
    ctx.stroke();

    // Upright LiDAR Callout Bracket
    ctx.save();
    ctx.rotate(yawRad); // Counter-rotate so callout text stays upright on screen
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

        // Dynamic Forward Path Relevance Booster (along active OSM route)
        if (!isRefined && window.osmRouter) {
          const pathInfo = window.osmRouter.getDistanceToPath(cellWx, cellWy, ego, 40.0);
          if (pathInfo.inForwardCorridor && ring.r1 <= 30.0) {
            isRefined = true;
            refineType = 'PATH_CORRIDOR';
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
          } else if (refineType === 'PATH_CORRIDOR') {
            // Forward driving corridor boosted to fine resolution / active guidance
            cellFill = 'rgba(6, 182, 212, 0.38)';
            cellStroke = 'rgba(34, 211, 238, 0.65)';
            lineWidth = 1.2;
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
    if (!currentFrameData || !currentFrameData.metrics) return;
    const m = currentFrameData.metrics;

    const ptCount = m.point_count !== undefined ? m.point_count : (m.sampled_points || 13855);
    const dynCount = m.dynamic_object_count !== undefined ? m.dynamic_object_count : (currentFrameData.detected_objects ? currentFrameData.detected_objects.filter(o => o.is_dynamic).length : 0);
    const activeCells = m.active_adaptive_cells !== undefined ? m.active_adaptive_cells : 1716;
    const uniformCells = m.theoretical_uniform_cells !== undefined ? m.theoretical_uniform_cells : (m.baseline_uniform_cells || 3200000);
    const redPct = m.cell_reduction_percent !== undefined ? m.cell_reduction_percent : (m.reduction_percent || 99.8);
    const adaptKb = m.estimated_adaptive_storage_kb !== undefined ? m.estimated_adaptive_storage_kb : (m.storage_kb || 106.6);
    const unifKb = m.estimated_uniform_storage_kb !== undefined ? m.estimated_uniform_storage_kb : 200000.0;

    if (metricPoints) metricPoints.textContent = Number(ptCount).toLocaleString();
    if (metricDynamic) metricDynamic.textContent = dynCount;
    if (metricActiveCells) metricActiveCells.textContent = Number(activeCells).toLocaleString();
    if (metricUniformCells) metricUniformCells.textContent = Number(uniformCells).toLocaleString();
    if (metricReduction) metricReduction.textContent = `${Number(redPct).toFixed(1)}%`;
    if (reductionBar) reductionBar.style.width = `${Math.min(100, Number(redPct))}%`;
    if (metricStorageKb) metricStorageKb.textContent = `${Number(adaptKb).toFixed(0)} KB vs ${Number(unifKb).toFixed(0)} KB`;

    if (window.osmRouter) {
      const tel = window.osmRouter.telemetry;
      if (phaseTitle) phaseTitle.textContent = tel.roadName || 'OSM Roadway';
      if (phaseDesc) phaseDesc.textContent = `${tel.maneuverText} (${tel.maneuverDistM}m)`;
    } else {
      if (currentFrameData.phase_name && phaseTitle) phaseTitle.textContent = currentFrameData.phase_name;
      if (currentFrameData.description && phaseDesc) phaseDesc.textContent = currentFrameData.description;
    }

    const ego = getEgoPose(simTime);
    if (metricSpeed) metricSpeed.textContent = `${(ego.speed * 3.6).toFixed(1)} km/h`;
  }

  // Simulation Update Step
  function stepSimulation(dtSec) {
    simTime += dtSec;
    if (simTime >= TOTAL_DURATION_SEC) {
      simTime = 0.0;
    }

    const ego = getEgoPose(simTime);
    if (metricSpeed) {
      metricSpeed.textContent = `${(ego.speed * 3.6).toFixed(1)} km/h`;
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
        updateTelemetryUI();
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
    [chkPoints, chkGrid, chkObjects, chkSemantics, chkWorld, chkElevation, chkDanger, chkRings, chkNavRoute].forEach(chk => {
      if (chk) {
        chk.parentElement.addEventListener('click', () => {
          setTimeout(() => {
            chk.parentElement.classList.toggle('active', chk.checked);
            render();
          }, 10);
        });
      }
    });

    // Live Navigation & Road Network Telemetry Listener
    if (window.osmRouter) {
      window.osmRouter.onUpdate((tel) => {
        if (routeRoadName) routeRoadName.textContent = tel.roadName;
        if (routeRoadType) routeRoadType.textContent = tel.roadType;
        if (routeManeuverIcon) routeManeuverIcon.textContent = tel.maneuverIcon;
        if (routeManeuverText) routeManeuverText.textContent = tel.maneuverText;
        if (routeManeuverDist) routeManeuverDist.textContent = `in ${tel.maneuverDistM} m`;
        if (routeProgressVal) routeProgressVal.textContent = `${tel.progressPercent.toFixed(0)}%`;
        if (routeStatusChip) {
          routeStatusChip.textContent = tel.isLiveNetwork ? '● OSM LIVE' : '● OSM CONNECTED';
        }

        // Live Geolocation readout updates
        if (geoAccuracyVal) geoAccuracyVal.textContent = `ACCURACY: \u00b1${tel.locationAccuracy || 8} m`;
        if (geoCoordsVal && tel.coordsText) geoCoordsVal.textContent = tel.coordsText;
        if (geoAltitudeVal) {
          const altPart = tel.altSpeedText ? tel.altSpeedText.split('|')[0].trim() : 'ALT: 12 m';
          geoAltitudeVal.textContent = altPart;
        }
        if (geoSpeedVal) {
          const spdPart = tel.altSpeedText ? tel.altSpeedText.split('|')[1]?.trim() : `SPEED: ${tel.speedKmh ? tel.speedKmh.toFixed(1) : '35.0'} km/h`;
          geoSpeedVal.textContent = spdPart || 'SPEED: 35.0 km/h';
        }
        if (geoTimestampVal && tel.timestampText) {
          geoTimestampVal.textContent = `TIME: ${tel.timestampText}`;
        }

        // Update pills
        if (pillOsm) {
          pillOsm.textContent = tel.isLiveNetwork ? '● OSM LIVE' : '● OSM CONNECTED';
          pillOsm.className = tel.isLiveNetwork ? 'status-pill status-pill-green' : 'status-pill status-pill-cyan';
        }
        if (pillGps) {
          pillGps.textContent = tel.locationBadge || '● DEVICE GEOLOCATION';
        }
        if (pillMatch) {
          pillMatch.textContent = '● SIMULATED ON LIVE ROAD';
        }
        if (pillRoute) {
          pillRoute.textContent = '● DYNAMIC ROAD GRAPH';
        }

        // Update destination status tip
        if (destStatusText) {
          if (window.osmRouter.destination) {
            destStatusText.innerHTML = `<strong>Target:</strong> (${Math.round(window.osmRouter.destination.x)}m, ${Math.round(window.osmRouter.destination.y)}m) <button id="btn-clear-target" style="background:#ef4444;color:#fff;border:none;border-radius:4px;padding:2px 8px;margin-left:8px;cursor:pointer;font-size:10px;font-weight:bold;">Clear</button>`;
            const btnClr = document.getElementById('btn-clear-target');
            if (btnClr) {
              btnClr.onclick = (ev) => {
                ev.stopPropagation();
                window.osmRouter.clearDestination();
                destStatusText.textContent = 'Click anywhere on map to set a dynamic route destination';
                render();
              };
            }
          } else {
            destStatusText.textContent = 'Click anywhere on map to set a dynamic route destination';
          }
        }

        // Update floating turn banner on canvas
        if (navHudFloating && chkNavRoute && chkNavRoute.checked) {
          navHudFloating.style.display = 'flex';
          if (navHudIcon) navHudIcon.textContent = tel.maneuverIcon;
          if (navHudAction) navHudAction.textContent = `In ${tel.maneuverDistM} m`;
          if (navHudRoad) navHudRoad.textContent = tel.roadName;
          if (navHudBadge) {
            navHudBadge.textContent = tel.isLiveNetwork ? '● LIVE OSM' : '● OSM VERIFIED';
          }
        } else if (navHudFloating) {
          navHudFloating.style.display = 'none';
        }
      });

      // Kick off live location tracking after listener is established
      window.osmRouter.startLiveTracking();
    }

    // Optional Live Optical Camera Sensor Hookup
    if (btnCameraSensor && cameraSensorDock && cameraVideo) {
      btnCameraSensor.addEventListener('click', async () => {
        if (cameraStream) {
          cameraStream.getTracks().forEach(track => track.stop());
          cameraStream = null;
          cameraSensorDock.style.display = 'none';
          btnCameraSensor.innerHTML = '<span class="cam-icon">📷</span> Connect Live Camera Sensor (Optional)';
          return;
        }

        try {
          if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
            alert('MediaDevices API not supported on this browser or insecure origin.');
            return;
          }
          btnCameraSensor.innerHTML = '<span class="cam-icon">⏳</span> Requesting Camera Access...';
          cameraStream = await navigator.mediaDevices.getUserMedia({
            video: { facingMode: 'environment', width: { ideal: 640 }, height: { ideal: 480 } },
            audio: false
          });
          cameraVideo.srcObject = cameraStream;
          cameraSensorDock.style.display = 'block';
          btnCameraSensor.innerHTML = '<span class="cam-icon">📷</span> Disconnect Optical Sensor';
        } catch (err) {
          console.warn('Live camera permission note:', err);
          btnCameraSensor.innerHTML = '<span class="cam-icon">📷</span> Camera Access Denied / Unavailable';
          setTimeout(() => {
            btnCameraSensor.innerHTML = '<span class="cam-icon">📷</span> Connect Live Camera Sensor (Optional)';
          }, 3000);
        }
      });

      if (btnCloseCamera) {
        btnCloseCamera.addEventListener('click', () => {
          if (cameraStream) {
            cameraStream.getTracks().forEach(track => track.stop());
            cameraStream = null;
          }
          cameraSensorDock.style.display = 'none';
          btnCameraSensor.innerHTML = '<span class="cam-icon">📷</span> Connect Live Camera Sensor (Optional)';
        });
      }
    }

    // Mouse Pan Interaction
    let dragMoved = false;
    let mouseDownX = 0;
    let mouseDownY = 0;

    canvas.addEventListener('mousedown', (e) => {
      isDragging = true;
      dragMoved = false;
      mouseDownX = e.clientX;
      mouseDownY = e.clientY;
      dragStartX = e.clientX - offsetX;
      dragStartY = e.clientY - offsetY;
    });

    window.addEventListener('mouseup', () => {
      isDragging = false;
    });

    canvas.addEventListener('mousemove', (e) => {
      if (isDragging) {
        if (Math.hypot(e.clientX - mouseDownX, e.clientY - mouseDownY) > 5) {
          dragMoved = true;
        }
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

    // Click to Select & Lock Entity or Set Dynamic Route Destination
    canvas.addEventListener('click', (e) => {
      if (dragMoved) return; // Prevent triggering destination click on pan drag release

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

        // Set dynamic destination along active road network
        if (window.osmRouter) {
          window.osmRouter.setDestination(worldCoord.x, worldCoord.y);
        }
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

  // Initialization & Kickoff: Non-blocking, instant render loop startup
  function init() {
    resizeCanvas();
    setupEvents();

    // Launch RAF animation loop and UI immediately on frame 0 (Zero blank-state startup!)
    updateTelemetryUI();
    requestAnimationFrame(animationLoop);

    // Asynchronously fetch status and high-fidelity dataset keyframes in background
    (async () => {
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

      try {
        await fetchKeyframe(1);
        prefetchNearbyFrames(1);
      } catch (e) {
        console.warn("Initial keyframe fetch warning:", e);
      }
    })();
  }

  init();
})();
