// test_encounter_engine.js
// Verification of EncounterManager: Scene Diversity, Dynamic Lifecycles, Active Density, and CAD Profiles

const assert = require('assert');

// Mock browser globals for testing EncounterManager
global.window = {
  addEventListener: () => {},
  devicePixelRatio: 1
};
global.performance = { now: () => Date.now() };
global.document = {
  getElementById: () => ({
    getContext: () => ({
      fillRect: () => {},
      strokeRect: () => {},
      beginPath: () => {},
      arc: () => {},
      ellipse: () => {},
      moveTo: () => {},
      lineTo: () => {},
      stroke: () => {},
      fill: () => {},
      save: () => {},
      restore: () => {},
      translate: () => {},
      rotate: () => {},
      createLinearGradient: () => ({ addColorStop: () => {} }),
      setLineDash: () => {},
      roundRect: () => {}
    }),
    style: {},
    classList: { add: () => {}, remove: () => {}, toggle: () => {} },
    addEventListener: () => {}
  }),
  querySelectorAll: () => []
};

// Import OSM router for real spline evaluations
const { OSMRouter, SplineTrajectory } = require('./app/visualization/static/osm_router.js');
const { EncounterManager } = require('./app/visualization/static/dashboard.js');

console.log('=== RUNNING OCULUS ENCOUNTER MANAGER & SCENE DIVERSITY TEST SUITE ===\n');

// Initialize router with road graph
const router = new OSMRouter();
router.buildInitialContinuousRoute();
const initialEgo = router.getEgoPose(0);

const em = new EncounterManager();

// Test 1: Initial Seeding
console.log('Test 1: Initial Population Seeding');
em.seedInitialEncounters(initialEgo, router);

const initialEntities = em.getActiveEntities();
console.log(`  Initial Active Entity Count: ${initialEntities.length}`);
assert(initialEntities.length >= 7, 'Must initialize with at least 7 entities');

const vehicles = initialEntities.filter(e => e.type === 'vehicle');
const peds = initialEntities.filter(e => e.type === 'pedestrian');
const animals = initialEntities.filter(e => e.type === 'animal');
const obstacles = initialEntities.filter(e => e.type === 'obstacle');

console.log(`  - Vehicles: ${vehicles.length} (oncoming, lead, parked)`);
console.log(`  - Pedestrians: ${peds.length} (crossing, sidewalk)`);
console.log(`  - Animals: ${animals.length} (dogs/quadrupeds)`);
console.log(`  - Obstacles: ${obstacles.length} (cones, barriers)`);

assert(vehicles.length >= 3, 'Must have at least 3 vehicles initially');
assert(peds.length >= 2, 'Must have at least 2 pedestrians initially');
assert(animals.length >= 1, 'Must have at least 1 animal initially');
assert(obstacles.length >= 1, 'Must have at least 1 obstacle initially');
console.log('  [PASS] Initial scene diversity and density verified.\n');

// Test 2: Animal Entity Specification & CAD Elevation Profile
console.log('Test 2: Animal (Canine / Dog) Entity Verification');
const dog = animals[0];
console.log(`  Name: ${dog.name}`);
console.log(`  Dimensions: L=${dog.length}m, W=${dog.width}m, H=${dog.height}m`);
console.log(`  Semantic Class: ${dog.semantic_class} (Expected 8 for Animal/VRU)`);
console.log(`  Elevation: Base=${dog.base_elev}m, Top=${dog.top_elev}m`);
assert.strictEqual(dog.semantic_class, 8, 'Animal semantic class must be 8');
assert(dog.height > 0.4 && dog.height < 0.8, 'Dog height must be realistic (~0.55m)');
assert(dog.localPoints && dog.localPoints.length >= 8, 'Dog must have synthetic LiDAR surface points');
console.log('  [PASS] Animal entity matches CAD elevation and LiDAR specifications.\n');

// Test 3: 2.5D CAD Elevation Inspector Hit Testing
console.log('Test 3: CAD Elevation Inspector Hit Testing');
const hitPed = em.findEntityAt({ x: peds[0].x, y: peds[0].y }, initialEgo);
assert(hitPed, 'Must detect pedestrian under cursor');
console.log(`  Detected Entity: ${hitPed.class_name}`);
console.log(`  - Confidence: ${(hitPed.confidence * 100).toFixed(0)}%`);
console.log(`  - Danger Level: ${hitPed.danger_level}`);
console.log(`  - Base Elevation: ${hitPed.base_elev.toFixed(2)} m`);
console.log(`  - Top Elevation: ${hitPed.top_elev.toFixed(2)} m`);
console.log(`  - Elevation Variance: ${hitPed.e_var} m²`);
assert(hitPed.base_elev !== undefined && hitPed.top_elev !== undefined, 'CAD profile must include elevation range');
assert(hitPed.dimensions && hitPed.dimensions.length === 3, 'CAD profile must include 3D dimensions [L, W, H]');
console.log('  [PASS] CAD Elevation Inspector hit query verified.\n');

// Test 4: Dynamic Lifecycle & Continuous 5-Minute Simulation (300 seconds)
console.log('Test 4: Continuous 300-Second Simulation (Lifecycle & Density Stability)');
const dt = 0.033; // 30 FPS step
let minCount = 999;
let maxCount = 0;
let totalDynamicCounts = 0;
let stepCount = 0;

for (let t = 0; t <= 300.0; t += dt) {
  const ego = router.getEgoPose(t);
  em.update(dt, ego, router);

  const activeCount = em.getActiveEntities().length;
  const dynCount = em.getActiveDynamicCount();

  if (activeCount < minCount) minCount = activeCount;
  if (activeCount > maxCount) maxCount = activeCount;

  totalDynamicCounts += dynCount;
  stepCount++;
}

const avgDynCount = (totalDynamicCounts / stepCount).toFixed(1);
console.log(`  Simulated Steps: ${stepCount} (300.0 seconds)`);
console.log(`  Active Entity Pool Range: [${minCount}, ${maxCount}]`);
console.log(`  Average Active Dynamic Objects: ${avgDynCount}`);

assert(minCount >= 5, 'Active entity count must never drop below 5 (no empty roads)');
assert(maxCount <= 18, 'Active entity count must not explode indefinitely (no memory leak)');
assert(avgDynCount >= 3.0, 'Average dynamic object count must sustain target density');
console.log('  [PASS] 300-second continuous simulation maintains consistent density without leaks.\n');

console.log('=== ALL ENCOUNTER ENGINE TESTS PASSED SUCCESSFULLY ===\n');
