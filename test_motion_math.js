const { OSMRouter } = require('./app/visualization/static/osm_router.js');

function assert(condition, message) {
  if (!condition) {
    console.error(`❌ FAIL: ${message}`);
    process.exit(1);
  }
  console.log(`  ✓ PASS: ${message}`);
}

console.log('====================================================');
console.log('OCULUS VEHICLE TRAJECTORY & HEADING VERIFICATION');
console.log('====================================================\n');

const router = new OSMRouter();

// TEST 1: Presets Load & Road Network Elements
console.log('Test 1: Presets and Road Network Elements');
const presetsToTest = ['4way_straight', '4way_left', '4way_right', 'plaza_roundabout', 'downtown_grid', 'coastal_overpass'];
presetsToTest.forEach(key => {
  router.loadPresetByKey(key);
  const roadNet = router.getRoadNetworkElements();
  assert(roadNet !== null, `Preset [${key}] returns road network elements`);
  assert(router.totalRouteMeters > 50, `Preset [${key}] has valid total route distance (${router.totalRouteMeters.toFixed(1)}m)`);
});

// TEST 2: 4-Way Straight Heading Alignment & Continuity
console.log('\nTest 2: 4-Way Straight Motion & Heading Alignment');
router.loadPresetByKey('4way_straight');
router.distanceTraveled = 0;
router.lastSimTime = 0;

let prevPose = null;
let maxHeadingStep = 0;
const numSteps = 200;
const dt = 0.5;

for (let step = 0; step < numSteps; step++) {
  const t = step * dt;
  const pose = router.getEgoPose(t);

  assert(!isNaN(pose.x) && !isNaN(pose.y) && !isNaN(pose.yaw), `Step ${step}: pose values are finite`);

  if (prevPose) {
    const dx = pose.x - prevPose.x;
    const dy = pose.y - prevPose.y;
    const distStep = Math.hypot(dx, dy);

    if (distStep > 0.05) {
      const vDirX = dx / distStep;
      const vDirY = dy / distStep;

      const yawRad = (pose.yaw * Math.PI) / 180.0;
      const uFrontX = Math.cos(yawRad);
      const uFrontY = Math.sin(yawRad);

      const dot = vDirX * uFrontX + vDirY * uFrontY;
      assert(dot >= 0.85, `Step ${step} (s=${router.distanceTraveled.toFixed(1)}m): u_front dot v_dir = ${dot.toFixed(4)} >= 0.85`);

      let diffDeg = Math.abs(pose.yaw - prevPose.yaw);
      if (diffDeg > 180) diffDeg = 360 - diffDeg;
      if (diffDeg > maxHeadingStep) maxHeadingStep = diffDeg;
    }
  }
  prevPose = pose;
}
assert(maxHeadingStep < 25.0, `Heading is smooth: max single-step change = ${maxHeadingStep.toFixed(2)} deg (< 25 deg)`);

// TEST 3: 4-Way Left Turn Behavior
console.log('\nTest 3: 4-Way Left Turn Maneuver Verification');
router.loadPresetByKey('4way_left');
router.distanceTraveled = 0;
router.lastSimTime = 0;

const poseApproach = router.spline.evaluateAtDistance(40.0, 3.5);
assert(Math.abs(poseApproach.yaw - 90.0) < 15.0, `Approach heading is North: yaw = ${poseApproach.yaw.toFixed(1)} deg`);

const poseTurn = router.spline.evaluateAtDistance(170.0, 3.5);
assert(poseTurn.steering > 0.05, `Front wheels steer left during turn: steering = ${(poseTurn.steering * 180 / Math.PI).toFixed(1)} deg (> 0.05 rad)`);

const poseWestExit = router.spline.evaluateAtDistance(210.0, 3.5);
let westYaw = poseWestExit.yaw;
if (westYaw < 0) westYaw += 360;
assert(Math.abs(westYaw - 180.0) < 20.0, `West Ave exit heading is West: yaw = ${poseWestExit.yaw.toFixed(1)} deg`);

// TEST 4: 4-Way Right Turn Behavior
console.log('\nTest 4: 4-Way Right Turn Maneuver Verification');
router.loadPresetByKey('4way_right');
router.distanceTraveled = 0;
router.lastSimTime = 0;

const poseRightTurn = router.spline.evaluateAtDistance(170.0, 3.5);
assert(poseRightTurn.steering < -0.05, `Front wheels steer right during turn: steering = ${(poseRightTurn.steering * 180 / Math.PI).toFixed(1)} deg (< -0.05 rad)`);

const poseEastExit = router.spline.evaluateAtDistance(210.0, 3.5);
assert(Math.abs(poseEastExit.yaw) < 20.0, `East Ave exit heading is East: yaw = ${poseEastExit.yaw.toFixed(1)} deg`);

// TEST 5: Geometric Roundabout Continuous Tangency
console.log('\nTest 5: Roundabout Circular Tracking & Tangency Verification');
router.loadPresetByKey('plaza_roundabout');
const R_EXPECTED = 65.0;

for (let s = 140; s <= 400; s += 25) {
  const pose = router.spline.evaluateAtDistance(s, 3.5);
  const rActual = Math.hypot(pose.x, pose.y);
  assert(Math.abs(rActual - R_EXPECTED) < 6.0, `At s=${s}m in rotary: radius = ${rActual.toFixed(1)}m (expected ~${R_EXPECTED}m)`);

  const tx = -pose.y / rActual;
  const ty = pose.x / rActual;
  const yawRad = (pose.yaw * Math.PI) / 180.0;
  const uFrontX = Math.cos(yawRad);
  const uFrontY = Math.sin(yawRad);

  const tangentDot = uFrontX * tx + uFrontY * ty;
  assert(tangentDot >= 0.88, `At s=${s}m: vehicle front is tangent to circular road: dot = ${tangentDot.toFixed(3)} >= 0.88`);
}

console.log('\n====================================================');
console.log('✅ ALL TRAJECTORY & HEADING TESTS PASSED SUCCESSFULLY');
console.log('====================================================');
