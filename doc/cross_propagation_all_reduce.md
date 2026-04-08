# Cross Propagation All-Reduce Implementation Notes

## Overview

This repository extends the baseline BookSim 2 simulator with a custom
`Cross Propagation (CP) All-Reduce` communication model on a `5x5 mesh`.

The implementation is not a generic collective library. It is a
topology-aware simulation model that:

- assigns fixed roles to nodes in a `5x5` mesh,
- uses a custom `Mesh` network class with non-uniform injection bandwidth,
- injects phase-specific packets from `TrafficManager`,
- uses packet completion as a proxy for aggregation progress, and
- models a hierarchical reduce and broadcast flow over a cross-shaped spine.

BookSim still does not perform numerical reduction. In this model,
"aggregation" means "a node waits until the required number of packets has been
received, then injects the next-phase packet(s)."

## Topology Roles

The current CP implementation assumes `k = 5`, so the center index is `k / 2 = 2`.

Node roles are defined as follows:

- `Center`: node `N12`
- `Col-cross`: nodes with `col == 2` and `row != 2`
  - `{N2, N7, N17, N22}`
- `Row-cross`: nodes with `row == 2` and `col != 2`
  - `{N10, N11, N13, N14}`
- `Outer`: all remaining nodes

This role split is intentionally asymmetric.

`Col-cross` nodes share the same columns as the outer nodes that must receive the
final broadcast, so they naturally serve as the final redistribution points in
Phase 1.5. `Row-cross` nodes do not provide an equivalent non-redundant path for
that final step, so they terminate after receiving from the center in Phase 1.

## Communication Phases

The CP flow is implemented as a multi-phase state machine in `TrafficManager`.

### Phase 0: Outer to Cross

Outer nodes inject packets toward the cross structure.

- `Px` packets use `is_px = true`
- `Py` packets use `is_px = false`
- The routing function interprets these as `XY` vs `YX` direction choices

This phase moves traffic from the outer nodes into the row/column cross nodes.

### Phase 0.5: Cross to Center

Cross nodes inject toward the single center node `N12`.

- The center receives `8` packets of size `D/2 flits`
- In the current implementation, `is_px = true` is sufficient for this phase
  because the communicating nodes already share the needed row or column

No explicit arithmetic reduction is modeled. The center simply waits until all
required packets arrive.

### Phase 1: Center to Cross

After Phase 0.5 completion, the center injects packets back to the cross nodes.

- The center sends `8` packets of size `D flits`
- `Row-cross` and `Col-cross` nodes both receive in this phase
- `Row-cross` nodes are terminal after this reception

This step represents the broadcast from the aggregation point back to the cross
spine.

### Phase 1.5: Col-cross to Outer

Only `Col-cross` nodes participate in the final redistribution.

- Each `Col-cross` node injects to its associated outer nodes
- This step is currently serialized through a single injection slot
- The implementation is therefore not fully parallel during the final broadcast

This serialization is one reason the measured broadcast latency can exceed the
reduce latency.

## Routing Model

The custom routing path is implemented by
`cross_propagation_dor_mesh` in `src/routefunc.cpp`.

Key properties:

- Packet routing policy is decided at injection time
- `is_px = true` means `XY` routing
- `is_px = false` means `YX` routing
- Phase 0.5, Phase 1, and Phase 1.5 use `is_px = true`
- Fat-bus parallel ports are handled through per-direction base offsets

The `is_px` flag is stored in the flit itself, so the routing choice remains
fixed for the lifetime of the packet.

## Network-Level Changes

The baseline `mesh` behavior in BookSim has been replaced with a dedicated
`Mesh` class for CP experiments.

### Custom `Mesh` Class

Files:

- `src/networks/mesh.hpp`
- `src/networks/mesh.cpp`

Main responsibilities:

- classify nodes as `Outer`, `Cross`, or `Center`
- assign different injection slot counts by role
- instantiate parallel channels for fat-bus links
- expose custom injection and credit paths

Injection slot policy:

- `Outer = 2`
- `Cross = 1`
- `Center = 4`

Parallel channel policy:

- cross links use `2` parallel channels
- center links use `4` parallel channels

### Deferred Build in `KNCube`

Files:

- `src/networks/kncube.hpp`
- `src/networks/kncube.cpp`

`KNCube` now supports a `defer_build` path so that the CP-specific `Mesh` class
can take over `_BuildNet()` instead of inheriting the original construction
behavior unmodified.

### Topology Dispatch Change

File:

- `src/networks/network.cpp`

When `topology = mesh`, the simulator now instantiates the custom `Mesh` class
instead of the baseline `KNCube` implementation, and registers the routing
functions through `Mesh`.

## TrafficManager Changes

Files:

- `src/trafficmanager.hpp`
- `src/trafficmanager.cpp`

This is the largest functional change in the repository.

Major additions:

- CP-specific phase state variables
- per-cross receive tracking arrays
- node classification helpers
- packet injection helpers for each CP phase
- barrier-based phase transitions in `_RetireFlit()`
- CP-specific suppression of BookSim summary statistics

The current implementation uses fixed thresholds tied to the `5x5` design:

- `4`
- `8`
- `16`

These thresholds correspond to the expected arrivals required to advance the
phase logic and are not yet generalized for arbitrary mesh sizes.

## Supporting Infrastructure Changes

### Flit Metadata

Files:

- `src/flit.hpp`
- `src/flit.cpp`

Added:

- `is_px` flag to store the packet's routing orientation

`Reset()` now clears `is_px` to `false`.

### Traffic Pattern Stub

Files:

- `src/traffic.hpp`
- `src/traffic.cpp`

Added:

- `CrossPropagationTrafficPattern`

This class is currently a placeholder. In practice, CP packet injection is
driven directly from `TrafficManager`.

### Buffer and Router Accessors

Files:

- `src/buffer.hpp`
- `src/buffer_state.hpp`
- `src/routers/iq_router.hpp`

Added helper APIs to support non-uniform channel and buffer configuration:

- `Buffer::GetSize()`
- `Buffer::SetSize()`
- `BufferState::SetVCBufSize()`
- `IQRouter::GetInputBuffer()`
- `IQRouter::GetOutputBufferState()`

These are required to configure the fat-bus path widths and related buffering.

## Configuration Files

Added example configurations:

- `src/examples/cross_propagation_55.cfg`
- `src/examples/mesh55_test.cfg`

These files provide experiment setups for the custom `5x5` mesh and CP flow.

## Current Limitations

The current CP implementation is intentionally specialized and has several known
limitations.

### Fixed `5x5` Assumptions

The following logic is currently hard-coded for `k = 5`:

- role classification around center index `2`
- barrier thresholds `4`, `8`, `16`
- cross tracking arrays of size `8`

Generalizing to arbitrary `k` will require both role generation and barrier
logic to be parameterized.

### Serialized Final Broadcast

Phase 1.5 is serialized through a single slot on the `Col-cross` side.

Observed consequence:

- broadcast latency can exceed reduce latency
- one reported result is approximately `76 cycles` for broadcast vs
  `67 cycles` for reduce

This is a model choice in the current implementation, not a generic property of
CP itself.

### No PE Computation Delay Model

The current simulator models communication timing only.

- PE-side local reduction compute delay is not modeled
- real-world all-reduce comparisons may therefore hide part of the end-to-end
  computation cost difference

This should be kept in mind when interpreting CP latency results against
implementations that include explicit compute stages.

## Suggested Next Steps

Natural follow-up work for this codebase includes:

- parameterizing the CP logic for general `k x k` meshes
- revisiting Phase 1.5 serialization and possible parallel broadcast variants
- adding a PE computation delay model
- documenting expected packet counts and latency per phase in a reproducible form

## File Summary

The main CP-related changes currently span:

- `src/flit.hpp`
- `src/flit.cpp`
- `src/networks/kncube.hpp`
- `src/networks/kncube.cpp`
- `src/networks/network.cpp`
- `src/networks/mesh.hpp`
- `src/networks/mesh.cpp`
- `src/routefunc.cpp`
- `src/traffic.hpp`
- `src/traffic.cpp`
- `src/trafficmanager.hpp`
- `src/trafficmanager.cpp`
- `src/buffer.hpp`
- `src/buffer_state.hpp`
- `src/routers/iq_router.hpp`
- `src/examples/cross_propagation_55.cfg`
- `src/examples/mesh55_test.cfg`
