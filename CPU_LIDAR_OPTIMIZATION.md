# CPU Lidar Performance Optimization

**Result: 7% → ~95% real-time factor** (Gazebo Classic parity)

---

## The Problem

The 3D CPU lidar demo ran at ~7% RTF in Gazebo Jetty while Gazebo Classic ran the same
scene at ~100% RTF. With a 128-beam × 80-ring lidar (10,240 rays per scan), the physics
step dominated the simulation time budget.

---

## Profiling the Baseline

The first step was a `perf` flamegraph on the running demo. The top call stacks were:

```
GetRayIntersectionFromLastStep
  → BulletCollisionDetector::createBulletCollisionShape
    → [libBulletCollision BVH traversal]
```

Two root causes were visible:

1. **`createBulletCollisionShape` on every ray.** DART's `BulletCollisionDetector::raycast()`
   wraps each ray query in a temporary `btCollisionObject + btShape`, fires the test, then
   discards it. For 10,240 rays per scan this meant 10,240 heap allocations + BVH setups
   per scan — work the Bullet `btCollisionWorld::rayTest()` API avoids entirely.

2. **Raycasting at the physics rate, not the sensor rate.** `Physics::UpdateRayIntersections`
   was called every simulation step (default 1 ms, i.e. 1 kHz). The CPU lidar fires at
   10 Hz. This meant ~100× more raycasting work was being done than the sensor ever needed,
   and all the excess results were silently discarded.

---

## Fix 1 — Bypass DART's raycast wrapper (~2× improvement)

### What DART's `raycast()` does

`BulletCollisionDetector::raycast(group, from, to)` constructs a `btBoxShape`-based
collision object to represent the ray, inserts it into the Bullet world, calls
`contactTest`, then tears it all down. This is the general shape–shape collision path and
is extremely wasteful for rays.

### The direct path: `btCollisionWorld::rayTest()`

Bullet has a native ray API that requires no shape objects:

```cpp
btCollisionWorld::ClosestRayResultCallback cb(btFrom, btTo);
btWorld->rayTest(btFrom, btTo, cb);
```

It uses Bullet's DBVT broadphase + per-shape BVH traversal internally, with zero heap
allocation for the ray itself. This is what the underlying `btCollisionWorld` is designed
for.

### Exposing `btCollisionWorld` without patching DART

`BulletCollisionGroup::getBulletCollisionWorld()` is `protected` — not callable from
outside the class hierarchy. `BulletCollisionDetector` reaches it via a `friend class`
declaration, but external code cannot.

The fix: add a thin subclass that bridges the protected method via normal C++ inheritance:

```cpp
// dartsim/src/GzCollisionDetector.hh
class GzBulletCollisionGroup : public dart::collision::BulletCollisionGroup {
public:
  explicit GzBulletCollisionGroup(const CollisionDetectorPtr &detector);
  btCollisionWorld *getCollisionWorld();   // calls protected getBulletCollisionWorld()
};
```

`GzBulletCollisionDetector::createCollisionGroup()` is overridden to return
`GzBulletCollisionGroup` instances. DART's `ConstraintSolver::setCollisionDetector()`
calls `createCollisionGroup()` via virtual dispatch, so every world automatically gets a
`GzBulletCollisionGroup` as its constraint solver group.

In `GetBatchRayIntersectionFromLastStep()` a `dynamic_cast` checks for the subclass and
reaches `btCollisionWorld*` directly — no upstream patches required:

```cpp
auto *gzGroup = dynamic_cast<GzBulletCollisionGroup *>(rawGroup);
if (gzGroup) {
    btCollisionWorld *btWorld = gzGroup->getCollisionWorld();
    // use btWorld->rayTest() ...
}
// fallback: DART raycast() loop for non-Bullet detectors
```

### New gz-physics feature: `GetBatchRayIntersectionFromLastStepFeature`

Rather than calling the existing per-ray `GetRayIntersectionFromLastStep` N times (one
virtual dispatch per ray through the gz-physics plugin ABI), a new feature was added that
accepts all rays in a single call:

```cpp
std::vector<RayIntersection> GetBatchRayIntersectionFromLastStep(
    const Identity &worldID,
    const std::vector<RayQuery> &rays) const;
```

Inside the dartsim implementation, each ray calls `btWorld->rayTest()` — no DART
`raycast()`, no shape allocation.

`Physics.cc` tries `EntityCast<BatchRayIntersectionFeatureList>` first and falls back to
the old per-ray loop for non-Bullet backends (bullet-featherstone, tpe).

**Result: ~7% → ~13% RTF**

---

## Dead End — Bundle AABB broadphase pre-culling

Inspired by Gazebo Classic's `dSpaceCollide2`, an earlier attempt tried:

1. Compute the AABB of the entire ray bundle.
2. One `btBroadphase::aabbTest()` → small candidate list.
3. Per-ray `rayTestSingle()` against candidates only.

The reasoning: ODE Classic gets a single broadphase call that sweeps all rays against the
world's hash space, eliminating most objects. The idea was to replicate this with Bullet.

This was wrong for a 360° lidar. The bundle AABB — the bounding box of all 10,240 ray
origins and endpoints — spans a full sphere around the sensor. The broadphase returns
*every object in the scene* as a candidate. No culling happens, and the cost is actually
higher than `btWorld->rayTest()` which does its own per-ray broadphase correctly.

The deeper lesson: ODE's `dSpaceCollide2` is fast not because it's a "batch API" but
because it uses a *hash space* where each ray's narrow AABB only touches the few cells it
actually occupies. Replicating the API shape without replicating the spatial data
structure gives no speedup.

---

## Fix 2 — Gate raycasting to sensor frequency (~7× improvement)

### The root cause of the remaining gap

Even with fast `btCollisionWorld::rayTest()` calls, Physics raycasted every 1 ms
simulation step. The lidar only needs fresh results at 10 Hz (every 100 steps). This
means 99% of the raycast work was being silently discarded — a problem that existed since
the original implementation and was completely independent of the raycasting algorithm.

### The fix: `needsRaycast` flag in `RaycastDataInfo`

A single boolean was added to the shared component:

```cpp
struct RaycastDataInfo {
  std::vector<RayInfo> rays;
  std::vector<RaycastResultInfo> results;
  bool needsRaycast = false;   // ← new
};
```

The gz-sim system execution order makes this clean:

```
CpuLidar::PreUpdate   →  Physics::Update (raycasting)  →  CpuLidar::PostUpdate
   (set flag)                (check flag)                    (read results)
```

`CpuLidar::PreUpdate` sets the flag per-sensor when a scan is actually due:

```cpp
comp->Data().needsRaycast =
    (sensor->NextDataUpdateTime() <= simTime && sensor->HasConnections());
```

`Physics::UpdateRayIntersections` skips entities where the flag is false and clears it
after computing:

```cpp
if (!_raycastData->Data().needsRaycast)
    return true;   // skip — sensor doesn't need data this step
_raycastData->Data().needsRaycast = false;
// ... raycasting happens here
```

The guard was applied to **both** the batch path (Bullet) and the fallback single-ray
loop (other backends), so non-Bullet physics engines benefit equally.

This reduces raycast operations from
`physics_rate × rays_per_scan` (1,000 Hz × 10,240 = 10.2 M/s) to
`sensor_rate × rays_per_scan` (10 Hz × 10,240 = 102,400/s) — a **100× reduction**.

**Result: ~13% → ~95% RTF**

---

## Why All of This Is Needed (Nothing Is Dead Weight)

After the work was done, the code was reviewed to see if anything could be removed.

| Component | Why it's load-bearing |
|---|---|
| `GzBulletCollisionGroup` + `createCollisionGroup()` | The only path to `btCollisionWorld*` without patching upstream DART. Without it, all raycast calls go through DART's `raycast()` → `createBulletCollisionShape` — the original bottleneck. |
| `GetBatchRayIntersectionFromLastStepFeature` + headers + tests | One ABI crossing per scan vs N per-ray crossings. Clean, independently tested API. Correct architectural boundary for future true batching (e.g. GPU). Avoids a silent behavior change to the existing `GetRayIntersectionFromLastStep` contract. |
| `needsRaycast` guard on fallback path | Non-Bullet backends that implement `GetRayIntersectionFromLastStep` (but not the batch feature) would otherwise still raycast at the physics rate. Both paths benefit from the frequency gate. |

### The simpler counterfactual

Could we have skipped the batch API and just overridden `GetRayIntersectionFromLastStep()`
in the dartsim plugin to call `btWorld->rayTest()` directly? Yes — and it would have been
a smaller diff. The trade-offs:

- **Pro:** Fewer new files, no new feature version.
- **Con:** Silent behavior change to an existing public API with no independent test
  coverage. No clean boundary for future batching. Physics.cc would still call it N times
  per scan through the ABI (minor, but real overhead at 10,240 rays).

The batch API was kept because it is the right abstraction: physics engines should be
able to accept a bundle of rays and return a bundle of results in a single call.

---

## Summary of Changes

### gz-physics (`src/gz-physics`, branch `gz-physics9`)

| File | Change |
|---|---|
| `include/gz/physics/GetBatchRayIntersection.hh` | New feature API: `GetBatchRayIntersectionFromLastStepFeature` |
| `include/gz/physics/detail/GetBatchRayIntersection.hh` | Template dispatch boilerplate (standard gz-physics pattern) |
| `dartsim/src/GzCollisionDetector.hh` | `GzBulletCollisionGroup` class; `createCollisionGroup()` override on `GzBulletCollisionDetector` |
| `dartsim/src/GzCollisionDetector.cc` | Implementations of the above |
| `dartsim/src/SimulationFeatures.hh` | Feature added to `SimulationFeatureList`; type aliases; method declaration |
| `dartsim/src/SimulationFeatures.cc` | `GetBatchRayIntersectionFromLastStep()` using `btWorld->rayTest()` per ray, with DART fallback |
| `test/common_test/simulation_features.cc` | 3 typed tests: hit+miss, empty input, unsupported backends |

### gz-sim (`src/gz-sim`, branch `feature/cpu-lidar`)

| File | Change |
|---|---|
| `include/gz/sim/components/RaycastData.hh` | `needsRaycast` bool field added to `RaycastDataInfo` |
| `src/systems/cpu_lidar/CpuLidar.cc` | `PreUpdate`: per-sensor `needsRaycast` flag set when scan is due |
| `src/systems/physics/Physics.cc` | `BatchRayIntersectionFeatureList`; `UpdateRayIntersections` gated on `needsRaycast`; both batch and fallback paths guarded |

---

## Physics Backend Compatibility

CPU lidar raycasting only works with **dartsim + Bullet collision detector** (the default).
Here is what happens with every other officially supported backend:

| Backend | What happens |
|---|---|
| **dartsim + bullet** (default) | Fully optimized: `btCollisionWorld::rayTest()` path, frequency-gated. |
| **dartsim + ode / fcl / dart** | `EntityCast<BatchRayIntersection>` succeeds (dartsim implements it), but the `dynamic_cast<GzBulletCollisionGroup*>` fails. A **one-time `gzwarn`** is emitted naming the unsupported detector and the SDF fix. All results are NaN. No log flood. |
| **bullet-featherstone** | `EntityCast<BatchRayIntersection>` fails. Falls through to `EntityCast<GetRayIntersection>`, which also fails. The existing Physics.cc warning fires: *"physics engine doesn't support ray intersection features"*. |
| **tpe** | Same as bullet-featherstone. |

### Fixing a misconfigured world

If you see the warning:

```
GetBatchRayIntersectionFromLastStep: collision detector [ode] does not support
raycasting. CPU lidar requires the [bullet] collision detector in dartsim.
```

Add this to your world SDF `<physics>` element:

```xml
<physics type="dart">
  <dart>
    <collision_detector>bullet</collision_detector>
  </dart>
</physics>
```

### Why only Bullet?

ODE, FCL, and the native DART collision detector do not implement `raycast()` in DART
6.13. The optimization that makes CPU lidar fast (`btCollisionWorld::rayTest()`) is
inherently Bullet-specific. There is no equivalent API in the other detectors.

---

## Lessons

**Profile before optimizing.** The first assumption was that "batching the API call" would
help. The flamegraph showed the real bottleneck was `createBulletCollisionShape` — DART
constructing a Bullet shape object for every single ray — not virtual dispatch overhead.
Without the profile, the batch API would have been implemented on the naive DART
`raycast()` path and shown no improvement.

**Understand what "batching" actually means.** ODE Classic's speed comes from its hash
space data structure, not from an API that accepts multiple rays. The bundle AABB
broadphase attempt replicated the API shape without the spatial structure and was
therefore useless for a 360° sensor.

**Check update rates before algorithm tuning.** The biggest single win (7×, from 13% to
95% RTF) came from two lines of code that had nothing to do with the ray intersection
algorithm. The raycasting was simply happening 100× more often than it needed to. Always
verify that work is only done when it is actually needed before optimizing how that work
is performed.
