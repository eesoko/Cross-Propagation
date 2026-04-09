# Cross Propagation All-Reduce Implementation Notes

## Overview

This repository extends the baseline BookSim 2 simulator with a custom
`Cross Propagation (CP) All-Reduce` communication model on a `5x5 mesh`.

The implementation is not a generic collective library. It is a
topology-aware simulation model that:

- assigns fixed roles to nodes in a `5x5` mesh,
- uses a custom `Mesh` network class with non-uniform injection/ejection
  bandwidth (fat-bus topology),
- injects phase-specific packets from `TrafficManager`,
- uses packet completion as a proxy for aggregation progress, and
- models a hierarchical reduce-scatter and all-gather flow over a
  cross-shaped spine.

BookSim still does not perform numerical reduction. In this model,
"aggregation" means "a node waits until the required number of packets has
been received, then injects the next-phase packet(s)."

## Topology Roles

The current CP implementation assumes `k = 5`, so the center index is
`k / 2 = 2`.

Node roles are defined as follows:

- `Center`: node `N12` (row 2, col 2)
- `Col-cross`: nodes with `col == 2` and `row != 2`
  - `{N2, N7, N17, N22}`
- `Row-cross`: nodes with `row == 2` and `col != 2`
  - `{N10, N11, N13, N14}`
- `Outer`: all remaining 16 nodes

## Fat-Bus Topology

Links are assigned parallel channels based on the node types at each end:

| Link type         | Parallel channels |
|-------------------|-------------------|
| Outer — Outer     | 1                 |
| Cross — any       | 2                 |
| Center — any      | 4                 |

Each node also has non-uniform injection and ejection slot counts:

| Node type | Injection slots | Ejection slots |
|-----------|----------------|----------------|
| Outer     | 2 (Px / Py)    | 2              |
| Cross     | 2 (directional)| 2              |
| Center    | 4 (N/S/E/W)    | 4 (N/S/E/W)    |

Ejection credits are routed per-slot via `Mesh::WriteEjectCredit()` to
ensure each router output buffer receives credits on the correct channel.

## Communication Phases

The CP All-Reduce is implemented as a 4-phase state machine in
`TrafficManager`. Phases 1 and 2 form the **Reduce-scatter**; Phases 3
and 4 form the **All-gather**.

### Phase 1: Outer → Cross  (Reduce-scatter, hop 1)

Each outer node injects two half-sized packets toward the cross structure.

- `Px` packets (`is_px = true`)  use XY routing
- `Py` packets (`is_px = false`) use YX routing

Each cross node collects contributions from its 4 outer neighbors. When
all 4 have arrived, Phase 2 injection is triggered for that cross node.

Measured: **17 cycles**

### Phase 2: Cross → Center  (Reduce-scatter, hop 2)

Each cross node injects one `D/2`-flit packet toward center `N12`.

The center waits until all 8 cross packets have arrived before
triggering Phase 3.

Measured: **18 cycles**

### Phase 3: Center → Cross  (All-gather, hop 1)

Center injects `D/2`-flit packets to all 8 cross nodes simultaneously,
using its 4 directional injection slots (N/S/E/W).

**Far-first injection order**: 2-hop cross nodes `{N2, N10, N14, N22}`
are enqueued before 1-hop cross nodes `{N7, N11, N13, N17}` within each
slot queue. This avoids link contention at the intermediate near-cross
router and reduces overall latency.

As soon as each cross node receives its Phase 3 packet, it immediately
begins Phase 4 injection. Phases 3 and 4 therefore overlap in a
pipelined fashion.

Measured: **18 cycles** (all 8 cross nodes receive by cycle 53)

### Phase 4: Cross → Outer  (All-gather, hop 2)

Each cross node injects `D/2`-flit packets to its 4 outer neighbors,
using its 2 directional injection slots.

- **Col-cross** nodes (N/S spine): broadcast horizontally (`is_px = true`)
- **Row-cross** nodes (E/W spine): broadcast vertically (`is_px = false`)

Each outer node therefore receives exactly 2 packets (Px + Py) from two
different cross nodes.

**Far-first injection order within each slot**:
- Col-cross slot 0 (west): col 0 → col 1  (ascending, far first)
- Col-cross slot 1 (east): col 4 → col 3  (descending, far first)
- Row-cross slot 0 (north): row 0 → row 1 (ascending, far first)
- Row-cross slot 1 (south): row 4 → row 3 (descending, far first)

This ensures the 2-hop outer nodes are not blocked behind 1-hop nodes
within the same slot queue.

Measured: **36 cycles** from Phase 2 completion; all 32 outer packets
arrive at the same cycle due to symmetric path lengths.

## Simulation Results  (packet_size = 8 flits)

```
Phase 1 (Outer→Cross, Reduce-scatter hop 1) :  17 cycles  [cycle  0 ~  17]
Phase 2 (Cross→Center, Reduce-scatter hop 2):  18 cycles  [cycle 17 ~  35]
Phase 3 (Center→Cross, All-gather hop 1)    :  18 cycles  [cycle 35 ~  53]  ─┐ pipelined
Phase 4 (Cross→Outer,  All-gather hop 2)    :  36 cycles  [cycle 35 ~  71]  ─┘
─────────────────────────────────────────────────────────────────────────────
Reduce-scatter (Phase 1+2)                  :  35 cycles
All-gather     (Phase 3+4)                  :  36 cycles
Total All-Reduce                            :  71 cycles
```

All 32 outer nodes receive their final result at the same cycle (71),
confirming symmetric path lengths across the fat-bus topology.

## Routing Model

The custom routing function `cross_propagation_dor` is implemented in
`src/routefunc.cpp`.

Key properties:

- When `cur == dest`, the ejection port is selected based on the source
  direction of the arriving packet, dispatching to the correct ejection
  slot (`base_L + slot`).
- `is_px = true`  → XY routing
- `is_px = false` → YX routing
- Fat-bus parallel ports are handled through per-direction base offsets
  computed from `par_E / par_W / par_S / par_N`.

## Configuration

### cp_trace

Set `cp_trace = 1` in the config file to enable per-packet event logging:

```
[CP TRACE][Cycle X] ARRIVE PhaseN  NX -> NY  Px/Py
```

Default is `cp_trace = 0` (summary only).

### Example config

`src/examples/cross_propagation_55.cfg`

## Network-Level Changes

### Custom `Mesh` Class

Files: `src/networks/mesh.hpp`, `src/networks/mesh.cpp`

Main responsibilities:

- classify nodes as `Outer`, `Cross`, or `Center`
- assign per-role injection and ejection slot counts
- instantiate parallel channels for fat-bus links
- expose `WriteEjectCredit(c, dest, slot)` for correct per-slot credit
  routing
- expose `ReadFlitFromEjectSlot(dest, slot)` for multi-slot ejection
  reading

### Deferred Build in `KNCube`

`KNCube` supports a `defer_build` path so that the CP-specific `Mesh`
class can take over `_BuildNet()` instead of inheriting the original
construction behavior unmodified.

### Topology Dispatch

When `topology = mesh`, the simulator instantiates the custom `Mesh`
class and registers routing functions through `Mesh::RegisterRoutingFunctions()`.

## TrafficManager Changes

Files: `src/trafficmanager.hpp`, `src/trafficmanager.cpp`

Major additions:

- CP-specific phase state variables and per-cross receive tracking
- node classification helpers (`_CpIsColCross`, `_CpIsRowCross`, etc.)
- `_CpInjectPacket()`: common packet injector with per-node slot selection
- `_InjectAllPhase1()`: bulk Phase 1 injection at simulation start
- `_InjectCrossToCenter()`: Phase 2 trigger per cross node
- `_InjectCenterToCross()`: Phase 3 with far-first ordering
- `_InjectColCrossToOuter()`: Phase 4 col-cross with far-first slot ordering
- `_InjectRowCrossToOuter()`: Phase 4 row-cross with far-first slot ordering
- barrier-based phase transitions in `_RetireFlit()`
- `flits` type changed to `vector<map<int, vector<pair<Flit*,int>>>>` to
  track ejection slot per flit for correct credit routing

## Current Limitations

### Fixed `5x5` Assumptions

The following logic is currently parameterized only for `k = 5`:

- role classification around center index `2`
- barrier thresholds `4` (per-cross), `8` (cross count), `32` (outer total)
- cross tracking arrays of size `8`

Generalizing to arbitrary `k` requires parameterizing role generation
and barrier thresholds.

### No PE Computation Delay Model

The simulator models communication timing only. PE-side local reduction
compute delay is not modeled.

## File Summary

Main CP-related files:

- `src/flit.hpp` / `src/flit.cpp` — `is_px` flag
- `src/networks/kncube.hpp` / `src/networks/kncube.cpp` — `defer_build` support
- `src/networks/network.cpp` — topology dispatch to `Mesh`
- `src/networks/mesh.hpp` / `src/networks/mesh.cpp` — fat-bus `Mesh` class
- `src/routefunc.cpp` — `cross_propagation_dor` routing function
- `src/traffic.hpp` / `src/traffic.cpp` — `CrossPropagationTrafficPattern` stub
- `src/trafficmanager.hpp` / `src/trafficmanager.cpp` — CP phase state machine
- `src/buffer.hpp` / `src/buffer_state.hpp` / `src/routers/iq_router.hpp` — buffer accessors
- `src/booksim_config.cpp` — `cp_trace` config parameter
- `src/examples/cross_propagation_55.cfg` — example configuration
