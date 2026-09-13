const assert = require('assert');
const {
  OSMRouter,
  projectLatLonToMeters,
  projectMetersToLatLon,
  SplineTrajectory
} = require('./app/visualization/static/osm_router.js');

console.log('=== RUNNING LIVE OSM ROUTER & MAP MATCHING TESTS ===\n');

console.log('Test 1: Coordinate Projections (WGS84 Lat/Lon <-> Metric Plane)');
const originLat = 37.7891;
const originLon = -122.4014;
const targetLat = 37.7900;
const targetLon = -122.4000;

const xy = projectLatLonToMeters(targetLat, targetLon, originLat, originLon);
console.log('  Projected (' + targetLat + ', ' + targetLon + ') -> X: ' + xy.x.toFixed(2) + 'm, Y: ' + xy.y.toFixed(2) + 'm');
assert(xy.x > 0, 'Target is East of origin, X must be positive');
assert(xy.y > 0, 'Target is North of origin, Y must be positive');

const roundTrip = projectMetersToLatLon(xy.x, xy.y, originLat, originLon);
assert(Math.abs(roundTrip.lat - targetLat) < 1e-5, 'Round-trip latitude mismatch');
assert(Math.abs(roundTrip.lon - targetLon) < 1e-5, 'Round-trip longitude mismatch');
console.log('  [PASS] Projections are accurate and reversible within <0.00001 deg.\n');

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
const spline = new SplineTrajectory(loopPoints, true);
console.log('  Total Spline Length: ' + spline.totalDistance.toFixed(2) + ' m');
assert(spline.totalDistance > 200, 'Spline total distance should exceed 200m');

// Sample heading at straight segment (between 0 and 50m)
const straightPose = spline.evaluateAtDistance(75, 3.5);
console.log('  Straightaway Heading: ' + straightPose.yaw.toFixed(2) + ' deg (expected close to 0 deg)');
assert(Math.abs(straightPose.yaw) < 8.0, 'Straightaway heading should align with +X');

// Sample curvature and steering
const cornerPose = spline.evaluateAtDistance(130, 3.5);
console.log('  Corner Pose: Yaw=' + cornerPose.yaw.toFixed(1) + ' deg, Curvature=' + cornerPose.curvature.toFixed(4) + ', Steering=' + cornerPose.steering.toFixed(3) + ' rad');
assert(cornerPose.steering !== 0, 'Steering angle should be non-zero during curve');
console.log('  [PASS] Spline heading and steering dynamics verified.\n');

console.log('Test 3: OSMRouter Initialization & Fallback Network');
const router = new OSMRouter();
const netElements = router.getRoadNetworkElements();
assert(netElements !== null, 'Network elements must exist');
assert(netElements.segments.length > 0, 'Segments must be populated');
console.log('  Road Segments: ' + netElements.segments.length);
console.log('  Intersections: ' + netElements.intersections.length);
console.log('  [PASS] Road graph elements successfully generated.\n');

console.log('Test 4: Map Matching to Centerline');
const testPt = { lat: originLat + 0.00045, lon: originLon + 0.00003 };
router.matchLocationToRoad(testPt.lat, testPt.lon);
console.log('  Matched Road: "' + router.currentRoad.name + '" (' + router.currentRoad.type + ')');
assert(router.matchedSegment !== null, 'Should match a road segment');
console.log('  [PASS] Map matching successfully snapped coordinate to centerline.\n');

console.log('Test 5: Dynamic Destination Routing');
const destX = 85.0;
const destY = 120.0;
router.setDestination(destX, destY);
assert(router.destination !== null, 'Destination must be set');
assert(router.destination.x === destX && router.destination.y === destY, 'Destination coordinates match');
console.log('  New Total Route Distance: ' + router.totalRouteMeters.toFixed(1) + ' m');
assert(router.totalRouteMeters > 0, 'Route must have positive length');

let prevPose = router.getEgoPose(0.0);
let totalAdvanced = 0;
for (let t = 0.1; t <= 5.0; t += 0.1) {
  const currentPose = router.getEgoPose(t);
  const stepDist = Math.hypot(currentPose.x - prevPose.x, currentPose.y - prevPose.y);
  assert(stepDist < 5.0, 'Pose step too large at t=' + t.toFixed(1) + 's (teleportation detected)');
  totalAdvanced += stepDist;
  prevPose = currentPose;
}
console.log('  Traveled smoothly over 5 seconds: ' + totalAdvanced.toFixed(2) + ' m');
assert(totalAdvanced > 30.0, 'Vehicle must advance over time');
console.log('  [PASS] Destination route computed and vehicle drives smoothly without teleportation.\n');

console.log('Test 6: Clear Destination');
router.clearDestination();
assert(router.destination === null, 'Destination must be cleared');
const defaultPose = router.getEgoPose(5.1);
assert(defaultPose !== null && typeof defaultPose.x === 'number', 'Pose continues valid execution');
console.log('  [PASS] Exploration route restored smoothly.\n');

console.log('=== ALL TESTS PASSED SUCCESSFULLY ===');
