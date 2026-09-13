// test_live_osm.js
// Verification of Live Location, Topological RoadGraph, Dynamic Continuous Routing, and Vehicle Motion

const assert = require('assert');
const {
  OSMRouter,
  projectLatLonToMeters,
  projectMetersToLatLon,
  SplineTrajectory,
  RoadGraph
} = require('./app/visualization/static/osm_router.js');

console.log('=== RUNNING LIVE OSM & DYNAMIC CONTINUOUS MOTION TEST SUITE ===\n');

// Test 1: Coordinate Projections
console.log('Test 1: Coordinate Projections (WGS84 Lat/Lon <-> Metric Plane)');
const originLat = 37.7891;
const originLon = -122.4014;
const targetLat = 37.7900;
const targetLon = -122.4000;

const xy = projectLatLonToMeters(targetLat, targetLon, originLat, originLon);
console.log(`  Projected (${targetLat}, ${targetLon}) -> X: ${xy.x.toFixed(2)}m, Y: ${xy.y.toFixed(2)}m`);
assert(xy.x > 0, 'Target is East of origin, X must be positive');
assert(xy.y > 0, 'Target is North of origin, Y must be positive');

const roundTrip = projectMetersToLatLon(xy.x, xy.y, originLat, originLon);
assert(Math.abs(roundTrip.lat - targetLat) < 1e-5, 'Round-trip latitude mismatch');
assert(Math.abs(roundTrip.lon - targetLon) < 1e-5, 'Round-trip longitude mismatch');
console.log('  [PASS] Projections are accurate and reversible within <0.00001 deg.\n');

// Test 2: Catmull-Rom Spline Trajectory & Heading Tangency
console.log('Test 2: Catmull-Rom Spline Trajectory & Heading Tangency');
const loopPoints = [
  { x: -50, y: 0, z: 0 },
  { x: 0, y: 0, z: 0 },
  { x: 50, y: 0, z: 0 },
  { x: 100, y: 50, z: 0 },
  { x: 100, y: 100, z: 0 },
  { x: 50, y: 150, z: 0 },
  { x: -50, y: 150, z: 0 },
  { x: -50, y: 0, z: 0 }
];
const spline = new SplineTrajectory(loopPoints, false);
console.log(`  Total Spline Length: ${spline.totalDistance.toFixed(2)} m`);
assert(spline.totalDistance > 200, 'Spline total distance should exceed 200m');

// Sample heading at straight segment
const straightPose = spline.evaluateAtDistance(75, 3.5);
console.log(`  Straightaway Heading: ${straightPose.yaw.toFixed(2)} deg (expected close to 0 deg)`);
assert(Math.abs(straightPose.yaw) < 12.0, 'Straightaway heading should align with +X');

// Sample curvature and steering
const cornerPose = spline.evaluateAtDistance(130, 3.5);
console.log(`  Corner Pose: Yaw=${cornerPose.yaw.toFixed(1)} deg, Curvature=${cornerPose.curvature.toFixed(4)}, Steering=${cornerPose.steering.toFixed(3)} rad`);
assert(cornerPose.steering !== 0, 'Steering angle should be non-zero during curve');
console.log('  [PASS] Spline heading and steering dynamics verified.\n');

// Test 3: RoadGraph Construction
console.log('Test 3: Topological RoadGraph Construction');
const router = new OSMRouter();
assert(router.roadGraph !== null, 'RoadGraph instance must exist');
assert(router.roadGraph.nodes.size > 0, 'RoadGraph must contain connected nodes');
console.log(`  Graph Nodes: ${router.roadGraph.nodes.size}`);

// Verify intersection nodes (degree >= 3)
let intersectionCount = 0;
for (const node of router.roadGraph.nodes.values()) {
  if (node.edges.length >= 3) {
    intersectionCount++;
  }
}
console.log(`  Intersection / Junction Nodes (degree >= 3): ${intersectionCount}`);
assert(intersectionCount >= 4, 'Must have at least 4 intersection nodes in regional graph');
console.log('  [PASS] RoadGraph constructed with verified multi-branch junctions.\n');

// Test 4: Dynamic Route Generation (No Predefined Waypoints)
console.log('Test 4: Dynamic Route Generation from RoadGraph');
assert(router.spline !== null, 'Continuous spline trajectory must exist');
assert(router.totalRouteMeters > 500, `Route must be at least 500m long, got ${router.totalRouteMeters}m`);
console.log(`  Initial Route Horizon: ${router.totalRouteMeters.toFixed(1)} m`);
console.log(`  Turn-by-Turn Steps: ${router.steps.length}`);
assert(router.steps.length >= 3, 'Must have generated multiple navigation steps');
console.log('  [PASS] Route dynamically generated directly from road graph.\n');

// Test 5: Ego Vehicle Motion Without Physical Device Movement (~3 Minutes Continuous Simulation)
console.log('Test 5: Continuous Ego Vehicle Motion (~3 Minutes / 180s simulation)');
let prevPose = router.getEgoPose(0.0);
let totalDistanceCovered = 0;
const simDurationSec = 180.0;
const dt = 0.1; // 100ms simulation steps (1800 steps)

let maxStepJump = 0;
let routeExtensionObserved = false;
let initialTotalMeters = router.totalRouteMeters;

for (let t = dt; t <= simDurationSec; t += dt) {
  const pose = router.getEgoPose(t);
  const stepDist = Math.hypot(pose.x - prevPose.x, pose.y - prevPose.y);
  if (stepDist > maxStepJump) maxStepJump = stepDist;
  assert(stepDist < 4.0, `Step jump too large (${stepDist.toFixed(2)}m at t=${t.toFixed(1)}s): vehicle teleported!`);
  totalDistanceCovered += stepDist;

  if (router.totalRouteMeters > initialTotalMeters) {
    routeExtensionObserved = true;
  }

  prevPose = pose;
}

console.log(`  Simulated Vehicle Advance: ${totalDistanceCovered.toFixed(1)} m over ${simDurationSec}s`);
console.log(`  Max Single-Step Displacement: ${maxStepJump.toFixed(3)} m (smooth, no jumps)`);
console.log(`  Dynamic Horizon Extension Occurred: ${routeExtensionObserved ? 'YES' : 'NO'}`);
console.log(`  Final Route Horizon: ${router.totalRouteMeters.toFixed(1)} m`);

assert(totalDistanceCovered > 1200, `Vehicle should have traveled > 1200m in 3 minutes, covered ${totalDistanceCovered}m`);
assert(maxStepJump < 3.5, 'Displacement per step must be smooth (< 3.5m)');
console.log('  [PASS] Vehicle drives continuously along live road network for 3+ minutes without device movement.\n');

// Test 6: Critical Heading Rule (Front Points in Direction of Movement)
console.log('Test 6: Critical Heading Rule (Vehicle Front Points Along Movement Tangent)');
const headingSamples = [10, 50, 100, 200, 400, 600, 900, 1200];
for (const s of headingSamples) {
  const p1 = router.spline.evalPointAtDistance(s);
  const p2 = router.spline.evalPointAtDistance(s + 3.5);
  const expectedYaw = Math.atan2(p2.y - p1.y, p2.x - p1.x) * (180.0 / Math.PI);
  const pose = router.spline.evaluateAtDistance(s, 3.5);
  const yawDiff = Math.abs(pose.yaw - expectedYaw);
  assert(yawDiff < 0.1 || Math.abs(yawDiff - 360) < 0.1, `Heading mismatch at s=${s}m: ${pose.yaw} vs ${expectedYaw}`);
}
console.log('  [PASS] Vehicle front heading strictly matches movement velocity tangent.\n');

// Test 7: Interactive Destination Setting
console.log('Test 7: Interactive Destination Setting');
router.setDestination(110, 100);
assert(router.destination !== null, 'Destination must be registered');
assert(router.destination.x === 110 && router.destination.y === 100, 'Destination coordinates match');
console.log(`  New Route Length to Destination: ${router.totalRouteMeters.toFixed(1)} m`);
assert(router.totalRouteMeters > 0, 'Must have valid route length');

// Clear Destination
router.clearDestination();
assert(router.destination === null, 'Destination must be cleared');
console.log('  [PASS] Destination set and clear functioning cleanly.\n');

console.log('=== ALL TESTS PASSED SUCCESSFULLY ===');
