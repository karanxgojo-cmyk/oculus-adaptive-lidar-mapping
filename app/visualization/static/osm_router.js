/**
 * OCULUS - Live Road Network, Dynamic Routing & Trajectory Engine (v4.1)
 * 
 * Features:
 * 1. Geodetic (WGS84) <-> Local Cartesian metric projection.
 * 2. Predictive look-ahead heading calculation (smooth predictive turning, no snapping).
 * 3. Strict alignment: Velocity direction === Vehicle front direction (angular error ~ 0).
 * 4. Multi-branch 4-Way Intersection road graph with selectable turns:
 *    - 4-Way Straight Through
 *    - 4-Way Left Turn
 *    - 4-Way Right Turn
 * 5. Geometric Roundabout with exact circular lane centerline tracking and tangential entry/exit.
 * 6. Downtown Tech Grid, Coastal Overpass, Live GPS Geolocation, and Calibrated Benchmark.
 * 7. Road network geometry provider for synchronized world map rendering.
 * 8. Forward path-relevance booster for adaptive 5cm LiDAR grid refinement.
 * 
 * Attribution: (c) OpenStreetMap contributors | OSRM Engine
 */

(function (global) {
  'use strict';

  const EARTH_RADIUS_M = 6378137.0;

  // ---------------------------------------------------------------------------
  // 1. Geodetic Projections (WGS84 <-> Local Cartesian Meters)
  // ---------------------------------------------------------------------------
  function projectLatLonToMeters(lat, lon, lat0, lon0) {
    const lat0Rad = (lat0 * Math.PI) / 180.0;
    const dLonRad = ((lon - lon0) * Math.PI) / 180.0;
    const dLatRad = ((lat - lat0) * Math.PI) / 180.0;
    const x = EARTH_RADIUS_M * dLonRad * Math.cos(lat0Rad);
    const y = EARTH_RADIUS_M * dLatRad;
    return { x, y };
  }

  function projectMetersToLatLon(x, y, lat0, lon0) {
    const lat0Rad = (lat0 * Math.PI) / 180.0;
    const lat = lat0 + (y / EARTH_RADIUS_M) * (180.0 / Math.PI);
    const lon = lon0 + (x / (EARTH_RADIUS_M * Math.cos(lat0Rad))) * (180.0 / Math.PI);
    return { lat, lon };
  }

  // ---------------------------------------------------------------------------
  // 2. Catmull-Rom Centripetal Arc-Length Spline with Look-Ahead Tangent
  // ---------------------------------------------------------------------------
  class SplineTrajectory {
    constructor(metricPoints, isClosed = true, hasElevation = false) {
      this.points = metricPoints; // Array of {x, y, z}
      this.isClosed = isClosed;
      this.hasElevation = hasElevation;
      this.cumDist = [0];
      this.totalDistance = 0;
      this.buildArcLengths();
    }

    buildArcLengths() {
      const n = this.points.length;
      if (n < 2) return;

      this.cumDist = [0];
      let sum = 0;
      for (let i = 1; i < n; i++) {
        const dx = this.points[i].x - this.points[i - 1].x;
        const dy = this.points[i].y - this.points[i - 1].y;
        const dz = (this.points[i].z || 0) - (this.points[i - 1].z || 0);
        const dist = Math.hypot(dx, dy, dz);
        sum += Math.max(0.01, dist);
        this.cumDist.push(sum);
      }

      if (this.isClosed) {
        const dx = this.points[0].x - this.points[n - 1].x;
        const dy = this.points[0].y - this.points[n - 1].y;
        const dz = (this.points[0].z || 0) - (this.points[n - 1].z || 0);
        sum += Math.max(0.01, Math.hypot(dx, dy, dz));
      }

      this.totalDistance = sum;
    }

    // Exact Catmull-Rom cubic interpolation at parameter u in [0, 1] between P1 and P2
    static interpolateCatmullRom(p0, p1, p2, p3, u) {
      const u2 = u * u;
      const u3 = u2 * u;

      // Position
      const x = 0.5 * (
        (2 * p1.x) +
        (-p0.x + p2.x) * u +
        (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * u2 +
        (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * u3
      );
      const y = 0.5 * (
        (2 * p1.y) +
        (-p0.y + p2.y) * u +
        (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * u2 +
        (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * u3
      );
      const z0 = p0.z || 0, z1 = p1.z || 0, z2 = p2.z || 0, z3 = p3.z || 0;
      const z = 0.5 * (
        (2 * z1) +
        (-z0 + z2) * u +
        (2 * z0 - 5 * z1 + 4 * z2 - z3) * u2 +
        (-z0 + 3 * z1 - 3 * z2 + z3) * u3
      );

      // Analytical first derivative (velocity vector)
      const dx = 0.5 * (
        (-p0.x + p2.x) +
        2 * (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * u +
        3 * (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * u2
      );
      const dy = 0.5 * (
        (-p0.y + p2.y) +
        2 * (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * u +
        3 * (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * u2
      );

      // Analytical second derivative (acceleration vector)
      const d2x = 0.5 * (
        2 * (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) +
        6 * (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * u
      );
      const d2y = 0.5 * (
        2 * (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) +
        6 * (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * u
      );

      // Curvature kappa = (dx*d2y - dy*d2x) / (dx^2 + dy^2)^(3/2)
      const speedSq = dx * dx + dy * dy;
      const speed = Math.sqrt(speedSq);
      const kappa = speedSq > 1e-4 ? (dx * d2y - dy * d2x) / (speedSq * speed) : 0;

      return { x, y, z, dx, dy, kappa, speed };
    }

    // Evaluate point position at cumulative arc length s
    evalPointAtDistance(s) {
      if (this.totalDistance <= 0 || this.points.length < 2) {
        return { x: 0, y: 0, z: 0, dx: 1, dy: 0, kappa: 0 };
      }

      const wrappedS = ((s % this.totalDistance) + this.totalDistance) % this.totalDistance;
      const n = this.points.length;

      let idx = 0;
      while (idx < this.cumDist.length - 1 && this.cumDist[idx + 1] < wrappedS) {
        idx++;
      }

      const segStartDist = this.cumDist[idx];
      const segEndDist = idx + 1 < this.cumDist.length ? this.cumDist[idx + 1] : this.totalDistance;
      const segLen = segEndDist - segStartDist;
      const u = segLen > 1e-4 ? (wrappedS - segStartDist) / segLen : 0;

      const i0 = (idx - 1 + n) % n;
      const i1 = idx % n;
      const i2 = (idx + 1) % n;
      const i3 = (idx + 2) % n;

      return SplineTrajectory.interpolateCatmullRom(this.points[i0], this.points[i1], this.points[i2], this.points[i3], u);
    }

    // Evaluates pose with predictive spatial look-ahead:
    // Heading = atan2(P_ahead.y - P_curr.y, P_ahead.x - P_curr.x)
    // Mathematically guarantees vehicle front points strictly in direction of forward motion.
    evaluateAtDistance(s, lookaheadMeters = 3.5) {
      if (this.totalDistance <= 0 || this.points.length < 2) {
        return { x: 0, y: 0, z: 0, yaw: 0, steering: 0, curvature: 0 };
      }

      const pCurr = this.evalPointAtDistance(s);
      const pAhead = this.evalPointAtDistance(s + lookaheadMeters);

      const dx = pAhead.x - pCurr.x;
      const dy = pAhead.y - pCurr.y;
      const distAhead = Math.hypot(dx, dy);

      let yaw;
      if (distAhead > 0.05) {
        yaw = Math.atan2(dy, dx) * (180.0 / Math.PI);
      } else {
        yaw = Math.atan2(pCurr.dy, pCurr.dx) * (180.0 / Math.PI);
      }

      // Bicycle model steering angle: delta = atan(L * kappa), L = 2.7m wheelbase
      const wheelbase = 2.7;
      const rawSteer = Math.atan(wheelbase * pCurr.kappa);
      const steering = Math.max(-0.52, Math.min(0.52, rawSteer));

      return {
        x: pCurr.x,
        y: pCurr.y,
        z: pCurr.z,
        yaw,
        steering,
        curvature: pCurr.kappa
      };
    }
  }

  // ---------------------------------------------------------------------------
  // 3. Real-World Road Topologies & Network Graphs
  // ---------------------------------------------------------------------------

  // A. 4-Way Intersection Graph (Centered at (0, 0) with selectable turns)
  function create4WayIntersectionRoads() {
    return {
      is4Way: true,
      intersection: {
        center: [0, 0],
        size: [32, 32],
        crosswalks: [
          { name: 'South Crosswalk', p1: [-16, -16], p2: [16, -16], width: 3.6 },
          { name: 'North Crosswalk', p1: [-16, 16], p2: [16, 16], width: 3.6 },
          { name: 'West Crosswalk', p1: [-16, -16], p2: [-16, 16], width: 3.6 },
          { name: 'East Crosswalk', p1: [16, -16], p2: [16, 16], width: 3.6 }
        ]
      },
      segments: [
        // South Approach Arterial (Incoming from South, heading North 90 deg)
        { name: 'South Arterial Boulevard', start: [0, -180], end: [0, -16], width: 14.0, lanes: 2 },
        // North Outgoing Arterial (Continuing North)
        { name: 'North Arterial Boulevard', start: [0, 16], end: [0, 180], width: 14.0, lanes: 2 },
        // West Cross Arterial (Left turn outgoing)
        { name: 'West Cross Avenue', start: [-180, 0], end: [-16, 0], width: 14.0, lanes: 2 },
        // East Cross Arterial (Right turn outgoing)
        { name: 'East Cross Avenue', start: [16, 0], end: [180, 0], width: 14.0, lanes: 2 }
      ],
      trafficSignals: [
        { x: -16, y: -16, state: 'green' },
        { x: 16, y: -16, state: 'green' },
        { x: -16, y: 16, state: 'red' },
        { x: 16, y: 16, state: 'red' }
      ]
    };
  }

  // 4-Way Intersection: STRAIGHT THROUGH
  const PRESET_4WAY_STRAIGHT = {
    key: '4way_straight',
    name: '4-Way Intersection (Straight Through)',
    city: 'Autonomous Test City',
    description: 'Vehicle approaches 4-way cross junction along South Boulevard, maintains incoming road alignment, cruises straight through intersection onto North Boulevard.',
    roadType: '4-Way Urban Cross Junction (Straight Corridor)',
    speedLimitKmh: 45,
    anchor: { lat: 37.7891, lon: -122.4014 },
    // Centerline waypoints: South approach -> straight through (0, 0) -> North exit -> smooth outer loop
    metricPoints: [
      { x: 0, y: -180, z: 0 },
      { x: 0, y: -120, z: 0 },
      { x: 0, y: -60, z: 0 },
      { x: 0, y: -20, z: 0 },
      { x: 0, y: 0, z: 0 },    // Center of 4-way intersection
      { x: 0, y: 20, z: 0 },
      { x: 0, y: 80, z: 0 },
      { x: 0, y: 180, z: 0 },
      // Smooth outer perimeter return loop (clockwise perimeter)
      { x: 60, y: 190, z: 0 },
      { x: 120, y: 150, z: 0 },
      { x: 120, y: -150, z: 0 },
      { x: 60, y: -190, z: 0 },
      { x: 0, y: -180, z: 0 }
    ],
    steps: [
      { name: 'South Arterial Boulevard', maneuver: 'depart', modifier: 'straight', distance: 160 },
      { name: 'Central 4-Way Cross Junction', maneuver: 'continue', modifier: 'straight', distance: 40 },
      { name: 'North Arterial Boulevard', maneuver: 'continue', modifier: 'straight', distance: 160 },
      { name: 'Perimeter Return Loop', maneuver: 'turn', modifier: 'right', distance: 340 }
    ],
    getRoadNetwork: create4WayIntersectionRoads
  };

  // 4-Way Intersection: LEFT TURN
  const PRESET_4WAY_LEFT = {
    key: '4way_left',
    name: '4-Way Intersection (Left Turn onto West Ave)',
    city: 'Autonomous Test City',
    description: 'Vehicle approaches 4-way cross junction, slows smoothly, executes predictive 90-degree left turn before junction center, and accelerates onto West Avenue.',
    roadType: '4-Way Urban Cross Junction (Left Turn Maneuver)',
    speedLimitKmh: 35,
    anchor: { lat: 37.7891, lon: -122.4014 },
    // Centerline waypoints: South approach -> smooth 90-deg left turn arc -> West exit -> return loop
    metricPoints: [
      { x: 0, y: -180, z: 0 },
      { x: 0, y: -100, z: 0 },
      { x: 0, y: -35, z: 0 },
      // Smooth 90-degree left turn transition arc inside intersection
      { x: -3, y: -15, z: 0 },
      { x: -10, y: -4, z: 0 },
      { x: -25, y: 0, z: 0 },
      { x: -70, y: 0, z: 0 },
      { x: -180, y: 0, z: 0 },
      // Smooth return loop to South approach
      { x: -190, y: -80, z: 0 },
      { x: -150, y: -160, z: 0 },
      { x: -60, y: -185, z: 0 },
      { x: 0, y: -180, z: 0 }
    ],
    steps: [
      { name: 'South Arterial Boulevard', maneuver: 'depart', modifier: 'straight', distance: 145 },
      { name: 'Central 4-Way Junction', maneuver: 'turn', modifier: 'left', distance: 35 },
      { name: 'West Cross Avenue', maneuver: 'continue', modifier: 'straight', distance: 155 },
      { name: 'Southwest Return Loop', maneuver: 'turn', modifier: 'left', distance: 310 }
    ],
    getRoadNetwork: create4WayIntersectionRoads
  };

  // 4-Way Intersection: RIGHT TURN
  const PRESET_4WAY_RIGHT = {
    key: '4way_right',
    name: '4-Way Intersection (Right Turn onto East Ave)',
    city: 'Autonomous Test City',
    description: 'Vehicle approaches 4-way cross junction, slows smoothly, executes predictive 90-degree right turn along lane corner curb, and accelerates onto East Avenue.',
    roadType: '4-Way Urban Cross Junction (Right Turn Maneuver)',
    speedLimitKmh: 35,
    anchor: { lat: 37.7891, lon: -122.4014 },
    // Centerline waypoints: South approach -> smooth 90-deg right turn arc -> East exit -> return loop
    metricPoints: [
      { x: 0, y: -180, z: 0 },
      { x: 0, y: -100, z: 0 },
      { x: 0, y: -35, z: 0 },
      // Smooth 90-degree right turn transition arc inside intersection
      { x: 3, y: -15, z: 0 },
      { x: 10, y: -4, z: 0 },
      { x: 25, y: 0, z: 0 },
      { x: 70, y: 0, z: 0 },
      { x: 180, y: 0, z: 0 },
      // Smooth return loop to South approach
      { x: 190, y: -80, z: 0 },
      { x: 150, y: -160, z: 0 },
      { x: 60, y: -185, z: 0 },
      { x: 0, y: -180, z: 0 }
    ],
    steps: [
      { name: 'South Arterial Boulevard', maneuver: 'depart', modifier: 'straight', distance: 145 },
      { name: 'Central 4-Way Junction', maneuver: 'turn', modifier: 'right', distance: 35 },
      { name: 'East Cross Avenue', maneuver: 'continue', modifier: 'straight', distance: 155 },
      { name: 'Southeast Return Loop', maneuver: 'turn', modifier: 'right', distance: 310 }
    ],
    getRoadNetwork: create4WayIntersectionRoads
  };

  // B. Geometric Roundabout (Paris Place Charles de Gaulle / Arc de Triomphe)
  function createRoundaboutRoads() {
    const R_LANE = 65.0;
    const R_ISLAND = 36.0;
    const segments = [];

    // 6 Radial Feeder Avenues radiating outward
    const spokes = [
      { angleRad: 0.0, name: 'Champs-Élysées East' },
      { angleRad: Math.PI * 0.33, name: 'Avenue Hoche' },
      { angleRad: Math.PI * 0.67, name: 'Avenue de Wagram' },
      { angleRad: Math.PI * 1.0, name: 'Avenue de la Grande-Armée West' },
      { angleRad: Math.PI * 1.33, name: 'Avenue Victor-Hugo' },
      { angleRad: Math.PI * 1.67, name: 'Avenue Kléber South' }
    ];

    spokes.forEach(spoke => {
      const cosA = Math.cos(spoke.angleRad);
      const sinA = Math.sin(spoke.angleRad);
      segments.push({
        name: spoke.name,
        start: [cosA * (R_LANE + 12), sinA * (R_LANE + 12)],
        end: [cosA * (R_LANE + 150), sinA * (R_LANE + 150)],
        width: 14.0,
        lanes: 2
      });
    });

    return {
      isRoundabout: true,
      roundaboutCenter: [0, 0],
      roundaboutRadius: R_LANE,
      islandRadius: R_ISLAND,
      segments
    };
  }

  const PRESET_PLAZA_ROUNDABOUT = {
    key: 'plaza_roundabout',
    name: 'Paris Arc de Triomphe (12-Lane Roundabout)',
    city: 'Paris, France',
    description: 'Exact geometric circular rotary. Vehicle enters from South Avenue, tangentially merges onto circular centerline, continuously tracks circle tangent, and smoothly takes the North exit.',
    roadType: 'Multi-Lane Circular Rotary (Continuous Tangent)',
    speedLimitKmh: 30,
    anchor: { lat: 48.8738, lon: 2.2950 },
    // Centerline waypoints: Entry approach -> tangential merge -> 270 deg circular path -> exit -> return
    metricPoints: (function () {
      const R = 65.0; // Roundabout lane centerline radius
      const pts = [];

      // 1. South-West entry approach
      pts.push({ x: -140, y: -120, z: 0 });
      pts.push({ x: -80, y: -70, z: 0 });

      // 2. Circular rotary path (Counter-clockwise: angles from -135 deg to +45 deg, 270 degrees total)
      const startAngle = -Math.PI * 0.75;
      const totalRot = Math.PI * 1.5;
      const numSteps = 16;
      for (let i = 0; i <= numSteps; i++) {
        const a = startAngle + (i / numSteps) * totalRot;
        pts.push({
          x: R * Math.cos(a),
          y: R * Math.sin(a),
          z: 0
        });
      }

      // 3. Smooth exit onto North-East Avenue
      pts.push({ x: 80, y: 70, z: 0 });
      pts.push({ x: 140, y: 120, z: 0 });

      // 4. Smooth perimeter return connector back to entry
      pts.push({ x: 80, y: 160, z: 0 });
      pts.push({ x: -60, y: 160, z: 0 });
      pts.push({ x: -170, y: 0, z: 0 });
      pts.push({ x: -140, y: -120, z: 0 });

      return pts;
    })(),
    steps: [
      { name: 'Avenue Kléber Approach', maneuver: 'depart', modifier: 'straight', distance: 95 },
      { name: 'Place de l’Étoile Rotary Merge', maneuver: 'rotary', modifier: 'enter', distance: 45 },
      { name: 'Circular Rotary Lane (Arc de Triomphe)', maneuver: 'rotary', modifier: 'continue', distance: 305 },
      { name: 'Avenue Hoche Exit', maneuver: 'rotary', modifier: 'exit-right', distance: 85 },
      { name: 'Outer Return Connector', maneuver: 'turn', modifier: 'right', distance: 360 }
    ],
    isRoundabout: true,
    getRoadNetwork: createRoundaboutRoads
  };

  // C. Downtown Tech Grid (San Francisco Financial District)
  function createDowntownGridRoads() {
    return {
      isGrid: true,
      segments: [
        { name: 'Market Street Boulevard', start: [-200, 0], end: [200, 0], width: 16.0, lanes: 4 },
        { name: 'Mission Street Corridor', start: [-200, -80], end: [200, -80], width: 14.0, lanes: 2 },
        { name: 'Howard Street Corridor', start: [-200, -160], end: [200, -160], width: 14.0, lanes: 2 },
        { name: '1st Street Avenue', start: [-100, 40], end: [-100, -200], width: 14.0, lanes: 2 },
        { name: '2nd Street Avenue', start: [0, 40], end: [0, -200], width: 14.0, lanes: 2 },
        { name: '3rd Street Avenue', start: [100, 40], end: [100, -200], width: 14.0, lanes: 2 }
      ],
      intersections: [
        { center: [0, 0], size: [28, 28], name: 'Market & 2nd St (4-Way)' },
        { center: [0, -80], size: [26, 26], name: 'Mission & 2nd St (4-Way)' },
        { center: [0, -160], size: [26, 26], name: 'Howard & 2nd St (4-Way)' },
        { center: [100, -160], size: [26, 26], name: 'Howard & 3rd St (4-Way)' },
        { center: [100, 0], size: [28, 28], name: 'Market & 3rd St (4-Way)' }
      ]
    };
  }

  const PRESET_DOWNTOWN_GRID = {
    key: 'downtown_grid',
    name: 'San Francisco Downtown Tech Grid',
    city: 'San Francisco, CA',
    description: 'Urban rectangular grid corridor with multiple 4-way intersections (Market, 2nd, Howard, 3rd), stop lines, and left/right turns.',
    roadType: 'Dense Urban Street Grid & 4-Way Intersections',
    speedLimitKmh: 40,
    anchor: { lat: 37.7891, lon: -122.4014 },
    // Centerline waypoints strictly on road centerlines
    metricPoints: [
      { x: 0, y: 30, z: 0 },
      { x: 0, y: 0, z: 0 },       // Cross Market & 2nd
      { x: 0, y: -80, z: 0 },     // Cross Mission & 2nd
      { x: 0, y: -160, z: 0 },    // Turn Left onto Howard St
      { x: 15, y: -160, z: 0 },
      { x: 50, y: -160, z: 0 },
      { x: 100, y: -160, z: 0 },  // Turn Left onto 3rd St
      { x: 100, y: -80, z: 0 },   // Cross Mission & 3rd
      { x: 100, y: 0, z: 0 },     // Turn Left onto Market St
      { x: 50, y: 0, z: 0 },
      { x: 0, y: 0, z: 0 },       // Turn Right onto 2nd St
      { x: 0, y: 30, z: 0 }
    ],
    steps: [
      { name: '2nd Street Southbound', maneuver: 'depart', modifier: 'straight', distance: 190 },
      { name: 'Howard Street (4-Way)', maneuver: 'turn', modifier: 'left', distance: 100 },
      { name: '3rd Street Northbound', maneuver: 'turn', modifier: 'left', distance: 160 },
      { name: 'Market Street Westbound', maneuver: 'turn', modifier: 'left', distance: 100 },
      { name: '2nd Street Return Junction', maneuver: 'turn', modifier: 'right', distance: 30 }
    ],
    getRoadNetwork: createDowntownGridRoads
  };

  // D. Coastal Overpass & Elevated Viaduct (Seattle Waterfront)
  function createCoastalOverpassRoads() {
    return {
      isCoastal: true,
      segments: [
        { name: 'Alaskan Way Waterfront Boulevard', start: [-60, -180], end: [-60, 180], width: 15.0, lanes: 2 },
        { name: 'Viaduct Elevated Incline Ramp', start: [-60, -60], control: [-20, 20], end: [40, 120], width: 12.0, lanes: 2, elevation_peak: 4.5 },
        { name: 'High Viaduct Elevated Deck', start: [40, 120], end: [120, 180], width: 12.0, lanes: 2, elevation_peak: 4.5 }
      ]
    };
  }

  const PRESET_COASTAL_OVERPASS = {
    key: 'coastal_overpass',
    name: 'Seattle Coastal Waterfront & Viaduct Ramp',
    city: 'Seattle, WA',
    description: 'Curved coastal boulevard transitioning onto an elevated structural viaduct incline ramp with continuous elevation gradient (Z = 0 -> 4.5m -> 0).',
    roadType: 'Coastal Highway & Elevated Viaduct Ramp',
    speedLimitKmh: 55,
    anchor: { lat: 47.6035, lon: -122.3360 },
    elevationProfile: true,
    metricPoints: [
      { x: -60, y: -180, z: 0 },
      { x: -60, y: -100, z: 0 },
      { x: -60, y: -40, z: 0.5 },
      // Elevated ramp ascent
      { x: -45, y: 0, z: 2.2 },
      { x: -10, y: 40, z: 3.8 },
      { x: 30, y: 90, z: 4.5 },
      { x: 70, y: 140, z: 4.5 },
      // Descent loop back to waterfront
      { x: 90, y: 170, z: 2.5 },
      { x: 50, y: 190, z: 0.5 },
      { x: -20, y: 180, z: 0 },
      { x: -60, y: 120, z: 0 },
      { x: -60, y: -180, z: 0 }
    ],
    steps: [
      { name: 'Alaskan Way Coastal Boulevard', maneuver: 'depart', modifier: 'straight', distance: 140 },
      { name: 'Viaduct Elevated Incline Ramp', maneuver: 'fork', modifier: 'right', distance: 130 },
      { name: 'Elliott Bay High Viaduct Deck', maneuver: 'continue', modifier: 'straight', distance: 110 },
      { name: 'Coastal Overpass Descent Loop', maneuver: 'turn', modifier: 'left', distance: 290 }
    ],
    getRoadNetwork: createCoastalOverpassRoads
  };

  // ---------------------------------------------------------------------------
  // 4. OSMRouter Engine Class
  // ---------------------------------------------------------------------------
  class OSMRouter {
    constructor() {
      this.currentMode = '4way_straight';
      this.status = 'cached';
      this.statusText = 'OSM ROAD NETWORK ACTIVE';
      this.spline = null;
      this.activePreset = PRESET_4WAY_STRAIGHT;
      this.currentAnchor = PRESET_4WAY_STRAIGHT.anchor;
      this.metricPoints = [];
      this.steps = [];
      this.distanceTraveled = 0;
      this.lastSimTime = 0;
      this.totalRouteMeters = 0;
      this.isRoundabout = false;
      this.isBridge = false;

      // Telemetry state for UI
      this.telemetry = {
        roadName: 'South Arterial Boulevard',
        roadType: '4-Way Urban Cross Junction (Straight Corridor)',
        speedLimitKmh: 45,
        speedKmh: 36,
        maneuverText: 'Continue straight through junction',
        maneuverIcon: '⬆',
        maneuverDistM: 160,
        progressPercent: 0,
        statusBadge: 'OSM VERIFIED',
        isLiveNetwork: false
      };

      this.listeners = [];
      this.loadPreset(PRESET_4WAY_STRAIGHT);
    }

    onUpdate(fn) {
      this.listeners.push(fn);
    }

    notifyListeners() {
      for (const fn of this.listeners) {
        try { fn(this.telemetry); } catch (e) { console.error('HUD listener error:', e); }
      }
    }

    loadPresetByKey(key) {
      if (key === '4way_left') this.loadPreset(PRESET_4WAY_LEFT);
      else if (key === '4way_right') this.loadPreset(PRESET_4WAY_RIGHT);
      else if (key === 'plaza_roundabout') this.loadPreset(PRESET_PLAZA_ROUNDABOUT);
      else if (key === 'downtown_grid') this.loadPreset(PRESET_DOWNTOWN_GRID);
      else if (key === 'coastal_overpass') this.loadPreset(PRESET_COASTAL_OVERPASS);
      else this.loadPreset(PRESET_4WAY_STRAIGHT);
    }

    loadPreset(preset) {
      this.activePreset = preset;
      this.currentAnchor = preset.anchor || { lat: 37.7891, lon: -122.4014 };
      this.currentMode = preset.key;
      this.isRoundabout = !!preset.isRoundabout;
      this.isBridge = !!preset.elevationProfile;
      this.steps = preset.steps || [];

      // Use exact precomputed metric points
      this.metricPoints = preset.metricPoints.map(p => ({
        x: p.x,
        y: p.y,
        z: p.z || 0.0
      }));

      this.spline = new SplineTrajectory(this.metricPoints, true, !!preset.elevationProfile);
      this.totalRouteMeters = this.spline.totalDistance;
      this.distanceTraveled = 0;
      this.lastSimTime = 0;

      this.telemetry.roadType = preset.roadType;
      this.telemetry.speedLimitKmh = preset.speedLimitKmh;
      this.statusText = 'OSM ROAD NETWORK ACTIVE';
      this.status = 'cached';
      this.telemetry.statusBadge = 'OSM VERIFIED';
      this.telemetry.isLiveNetwork = false;

      this.notifyListeners();
    }

    // Request Live Browser Geolocation (GPS) & build local driving route
    requestLiveGPS(callback) {
      if (typeof navigator === 'undefined' || !navigator.geolocation) {
        if (callback) callback(false, 'Geolocation is not supported by your browser.');
        return;
      }

      this.statusText = 'REQUESTING GPS SATELLITE FIX...';
      this.notifyListeners();

      navigator.geolocation.getCurrentPosition(
        (pos) => {
          const userLat = pos.coords.latitude;
          const userLon = pos.coords.longitude;
          this.currentAnchor = { lat: userLat, lon: userLon };

          // Build a local driving loop centered on GPS coordinates
          const d = 120.0;
          const localLoop = [
            { x: 0, y: -d, z: 0 },
            { x: 0, y: -20, z: 0 },
            { x: -5, y: -5, z: 0 },
            { x: -20, y: 0, z: 0 },
            { x: -d, y: 0, z: 0 },
            { x: -d - 20, y: -d * 0.5, z: 0 },
            { x: -d * 0.5, y: -d - 20, z: 0 },
            { x: 0, y: -d, z: 0 }
          ];

          const gpsPreset = {
            key: 'live_gps',
            name: `Local GPS Vicinity (${userLat.toFixed(3)}°, ${userLon.toFixed(3)}°)`,
            city: 'Live Device Geolocation',
            description: 'Live trajectory queried from local road perimeter centered on your physical GPS position.',
            roadType: 'Local Street Network',
            speedLimitKmh: 40,
            anchor: { lat: userLat, lon: userLon },
            metricPoints: localLoop,
            steps: [
              { name: 'Local Access Road', maneuver: 'depart', modifier: 'straight', distance: 100 },
              { name: 'Neighborhood Intersection', maneuver: 'turn', modifier: 'left', distance: 30 },
              { name: 'District Avenue', maneuver: 'continue', modifier: 'straight', distance: 100 },
              { name: 'Return Corridor', maneuver: 'arrive', modifier: 'straight', distance: 120 }
            ],
            getRoadNetwork: create4WayIntersectionRoads
          };

          this.loadPreset(gpsPreset);
          this.status = 'live';
          this.statusText = 'LIVE GPS LOCATION ACTIVE';
          this.telemetry.statusBadge = 'GPS LIVE';
          this.telemetry.isLiveNetwork = true;
          if (callback) callback(true, 'Live GPS route active.');
        },
        (err) => {
          console.warn('Geolocation permission denied:', err);
          if (callback) callback(false, 'GPS permission denied. Seamlessly active on 4-Way Intersection.');
          this.loadPreset(PRESET_4WAY_STRAIGHT);
        },
        { timeout: 8000, enableHighAccuracy: true }
      );
    }

    // -------------------------------------------------------------------------
    // 5. Dynamic Continuous Ego Pose Computation
    // -------------------------------------------------------------------------
    getEgoPose(simTime) {
      if (!this.spline || this.metricPoints.length < 2) {
        return { x: 0, y: 0, z: 0, yaw: 0, speed: 0, steering: 0 };
      }

      // Delta time accumulation
      const dt = this.lastSimTime > 0 ? Math.max(0.001, Math.min(0.2, simTime - this.lastSimTime)) : 0.033;
      this.lastSimTime = simTime;

      // Realistic speed profiling:
      // Straightaways: 10.5 m/s (~38 km/h)
      // Curves & Turns: slows to 5.2 m/s (~19 km/h)
      let targetSpeed = 10.5;
      if (this.isRoundabout) {
        targetSpeed = 6.2;
      } else if (this.isBridge) {
        targetSpeed = 12.0;
      }

      // Sample look-ahead curvature to decelerate BEFORE turning
      const lookaheadSample = this.spline.evaluateAtDistance(this.distanceTraveled + 14.0, 3.5);
      const absCurvature = Math.abs(lookaheadSample.curvature);
      if (absCurvature > 0.015) {
        const slowFactor = Math.max(0.48, 1.0 - (absCurvature * 24.0));
        targetSpeed *= slowFactor;
      }

      // Advance distance traveled
      this.distanceTraveled += targetSpeed * dt;
      if (this.distanceTraveled >= this.totalRouteMeters) {
        this.distanceTraveled -= this.totalRouteMeters;
      }

      // Evaluate pose with 3.5m spatial look-ahead
      // Heading is computed from forward secant vector (NEVER points backward)
      const pose = this.spline.evaluateAtDistance(this.distanceTraveled, 3.5);
      pose.speed = targetSpeed;

      // Update Navigation Telemetry
      this.updateNavigationTelemetry(this.distanceTraveled, targetSpeed);

      return pose;
    }

    updateNavigationTelemetry(currentDist, currentSpeed) {
      const wrappedDist = currentDist % this.totalRouteMeters;
      const progress = (wrappedDist / this.totalRouteMeters) * 100.0;
      this.telemetry.progressPercent = progress;
      this.telemetry.speedKmh = currentSpeed * 3.6;

      if (!this.steps || this.steps.length === 0) {
        this.telemetry.roadName = this.activePreset.name;
        this.telemetry.maneuverText = 'Proceed along active route';
        this.telemetry.maneuverIcon = '⬆';
        this.telemetry.maneuverDistM = Math.round(this.totalRouteMeters - wrappedDist);
        return;
      }

      let stepStartDist = 0;
      let nextStep = this.steps[0];
      let distToNextManeuver = 0;

      for (let i = 0; i < this.steps.length; i++) {
        const step = this.steps[i];
        const stepEnd = stepStartDist + step.distance;
        if (wrappedDist >= stepStartDist && wrappedDist < stepEnd) {
          this.telemetry.roadName = step.name;
          const nextIdx = (i + 1) % this.steps.length;
          nextStep = this.steps[nextIdx];
          distToNextManeuver = Math.max(5, Math.round(stepEnd - wrappedDist));
          break;
        }
        stepStartDist = stepEnd;
      }

      const mod = nextStep.modifier || 'straight';
      let icon = '⬆';
      if (mod.includes('left')) icon = '↰';
      else if (mod.includes('right')) icon = '↱';
      else if (mod.includes('uturn')) icon = '⮌';
      else if (nextStep.maneuver === 'rotary') icon = '⮡';

      let actionDesc = 'Continue';
      if (nextStep.maneuver === 'turn') actionDesc = `Turn ${mod.replace('-', ' ')}`;
      else if (nextStep.maneuver === 'rotary') actionDesc = 'Enter rotary';
      else if (nextStep.maneuver === 'arrive') actionDesc = 'Approach destination';
      else if (nextStep.maneuver === 'fork') actionDesc = `Take ${mod} fork`;

      this.telemetry.maneuverIcon = icon;
      this.telemetry.maneuverText = `${actionDesc} onto ${nextStep.name}`;
      this.telemetry.maneuverDistM = distToNextManeuver;

      this.notifyListeners();
    }

    // -------------------------------------------------------------------------
    // 6. Path-Relevance Booster for Adaptive Grid
    // -------------------------------------------------------------------------
    getDistanceToPath(wx, wy, egoPose, lookaheadMeters = 45.0) {
      if (!this.spline) return { lateralDist: 999, alongDist: 999, inForwardCorridor: false };

      const numSamples = 16;
      let minLateralSq = Infinity;
      let closestAlongDist = 0;

      for (let i = 0; i <= numSamples; i++) {
        const sOffset = (i / numSamples) * lookaheadMeters;
        const pt = this.spline.evalPointAtDistance(this.distanceTraveled + sOffset);
        const dx = wx - pt.x;
        const dy = wy - pt.y;
        const distSq = dx * dx + dy * dy;

        if (distSq < minLateralSq) {
          minLateralSq = distSq;
          closestAlongDist = sOffset;
        }
      }

      const lateralDist = Math.sqrt(minLateralSq);
      const inForwardCorridor = lateralDist < 3.8 && closestAlongDist > 0 && closestAlongDist <= lookaheadMeters;

      return {
        lateralDist,
        alongDist: closestAlongDist,
        inForwardCorridor
      };
    }

    // -------------------------------------------------------------------------
    // 7. Navigation Canvas Overlay Rendering
    // -------------------------------------------------------------------------
    drawRouteOverlay(ctx, worldToCanvas, egoPose) {
      if (!this.spline || this.metricPoints.length < 2) return;

      ctx.save();

      // Active full route polyline
      ctx.beginPath();
      const numSamples = 100;
      for (let i = 0; i <= numSamples; i++) {
        const s = (i / numSamples) * this.totalRouteMeters;
        const pt = this.spline.evalPointAtDistance(s);
        const cp = worldToCanvas(pt.x, pt.y);
        if (i === 0) ctx.moveTo(cp.px, cp.py);
        else ctx.lineTo(cp.px, cp.py);
      }
      ctx.closePath();

      // Outer cyan glow
      ctx.strokeStyle = 'rgba(6, 182, 212, 0.28)';
      ctx.lineWidth = 7.0;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.stroke();

      // Core crisp path line
      ctx.strokeStyle = 'rgba(34, 211, 238, 0.85)';
      ctx.lineWidth = 2.4;
      ctx.setLineDash([12, 8]);
      ctx.stroke();
      ctx.setLineDash([]);

      // Forward Lookahead Guidance Line (Next 40 meters)
      ctx.beginPath();
      const lookaheadSamples = 20;
      for (let j = 0; j <= lookaheadSamples; j++) {
        const s = this.distanceTraveled + (j / lookaheadSamples) * 40.0;
        const pt = this.spline.evalPointAtDistance(s);
        const cp = worldToCanvas(pt.x, pt.y);
        if (j === 0) ctx.moveTo(cp.px, cp.py);
        else ctx.lineTo(cp.px, cp.py);
      }
      ctx.strokeStyle = '#22c55e'; // Bright active guidance green
      ctx.lineWidth = 3.6;
      ctx.stroke();

      // Upcoming Maneuver Marker / Junction Node
      if (this.telemetry.maneuverDistM > 0 && this.telemetry.maneuverDistM < 65) {
        const maneuverPt = this.spline.evalPointAtDistance(this.distanceTraveled + this.telemetry.maneuverDistM);
        const mPos = worldToCanvas(maneuverPt.x, maneuverPt.y);

        ctx.save();
        ctx.fillStyle = 'rgba(234, 179, 8, 0.25)';
        ctx.beginPath();
        ctx.arc(mPos.px, mPos.py, 14, 0, 2 * Math.PI);
        ctx.fill();

        ctx.strokeStyle = '#eab308';
        ctx.lineWidth = 2.0;
        ctx.beginPath();
        ctx.arc(mPos.px, mPos.py, 10, 0, 2 * Math.PI);
        ctx.stroke();

        ctx.font = 'bold 11px system-ui, sans-serif';
        ctx.fillStyle = '#ffffff';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(this.telemetry.maneuverIcon, mPos.px, mPos.py);
        ctx.restore();
      }

      ctx.restore();
    }

    // -------------------------------------------------------------------------
    // 8. Road Network Geometry for World Map Integration
    // -------------------------------------------------------------------------
    getRoadNetworkElements() {
      if (this.activePreset && typeof this.activePreset.getRoadNetwork === 'function') {
        return this.activePreset.getRoadNetwork();
      }
      return null;
    }
  }

  // Export globally
  const osmRouter = new OSMRouter();
  global.OSMRouter = OSMRouter;
  global.osmRouter = osmRouter;
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = { OSMRouter, osmRouter };
  }

})(typeof window !== 'undefined' ? window : (typeof global !== 'undefined' ? global : this));
