# Geometry and transform correctness fix plan

## Goal

Repair the audited logic, algorithm, degeneracy, and scale-sensitivity defects while preserving the
crate's supported surface:

- `no_std` operation through the existing `std`, `libm`, and `system-libm` backends;
- generic storage and arithmetic for the current scalar types;
- current public query traits and return types;
- column-major matrices and column-vector transform conventions;
- existing `Line` direction storage and parameter-dependent `t` values;
- successful behavior for well-formed vectors, matrices, rotations, transforms, and geometry.

P0 is reserved for core logic and algorithm defects that can return contradictory or mathematically
wrong results independently of scene scale or degenerate geometry. Degenerate-shape behavior is P1.
Scale/tolerance invariance is intentionally deferred to P2 as requested. Audited incorrect outcomes
are defects, not compatibility requirements.

This plan is for maintainers implementing and reviewing the fixes. It does not redesign the query
traits, add arbitrary-precision or robust-predicate dependencies, add general shear decomposition,
change matrix inverse behavior, or broaden analytic support beyond `f32` and `f64`.

## Compatibility boundary

- Keep the signatures of `transforms::decompose`, `Distance::distance`,
  `Plane::intersect_line`, `shortest_segment3d_between_lines3d`, and every existing
  `Intersect`/`Intersection` implementation.
- Keep `Line::new` and `Line::from_start_end` directions unnormalized. Normalizing storage would
  change returned `t` values and remove the useful `t = 0`/`t = 1` endpoint mapping from
  `from_start_end`.
- Keep boundary-inclusive triangle barycentric, box, and sphere semantics.
- Keep `decompose`'s current reflection convention: a negative linear determinant produces three
  negative scale components and a proper rotation quaternion.
- Intentionally change `Quat::default()` from zero to identity and make total normalization of an
  explicit zero quaternion fall back to identity. Add
  `Quat::try_normalize(&Self, epsilon) -> Option<Self>` for callers that need checked behavior.
- Intentionally change `try_basis_from_unit`/`basis_from_unit` ordering to a right-handed
  `[u, v, w]` frame satisfying `u × v = w`; document this behavioral correction.
- Correct generic plane-distance mathematics to divide by normal length, even though safe plane
  constructors currently normalize the normal and mask the defect.
- Enforce strict ray half-line semantics: every returned ray intersection has `t >= 0`.
- Do not add dependencies, allocate, require `std`, or introduce feature-specific algorithms.

## Current architecture

- `src/scalar.rs` owns `FloatScalar::epsilon()` and `EPS_F32`/`EPS_F64`.
- `src/vector.rs` owns vector dot/cross products, lengths, and checked/unchecked normalization.
- `src/primitives.rs` owns `Line`, `Segment`, `Ray`, `Plane`, and the shortest-line-segment routine.
  `Plane::try_new` normalizes valid plane normals; `Line::new` preserves direction magnitude.
- `src/queries.rs` implements distance and intersection traits. It currently:
  - divides point/plane distance by `n · n` rather than `sqrt(n · n)`;
  - returns `[u, w, v]` after computing `w = u × v`, producing a left-handed ordered frame;
  - accepts ray/triangle hits with `t` in `[-epsilon, 0)`;
  - requires all three optional edge distances for sphere/triangle fallback;
  - compares raw direction/triangle determinants to a fixed epsilon.
- `src/transforms.rs::decompose` extracts column lengths, uses the 4x4 determinant only for sign,
  and feeds potentially nonorthogonal columns to `Quat::of_matrix3`.
- `src/quaternion.rs` derives zero-valued `Default`, preserves zero in `normalize`, and gives that
  invalid value conflicting matrix and axis-angle meanings.
- Tests are inline in each module. `transforms::tests::test_decompose` uses one-sided approximate
  assertions. There are no integration tests, runnable examples, benchmarks, or CI files.

The scale-sensitive flow has no authoritative owner:

```text
FloatScalar::epsilon()
   ├─ primitives.rs compares raw line/plane dot and line/line cross magnitudes
   ├─ queries.rs compares a raw ray-or-line/triangle determinant
   └─ vector.rs compares actual vector length to epsilon
```

## Target architecture

Core invariants have direct owners and are repaired first:

```text
P0 core contracts
   ├─ transforms.rs  → affine, shear-free TRS decomposition only
   ├─ quaternion.rs  → identity default and coherent zero fallback
   ├─ queries.rs     → correct plane distance, right-handed basis, strict ray t
   └─ module tests   → two-sided/reconstruction-based assertions

P1 degenerate geometry
   └─ queries.rs     → triangle closure as point/segment/face for sphere tests

P2 scale policy
   └─ vector.rs crate-private relative predicates
          ├─ primitives.rs line/plane and line/line callers
          └─ queries.rs line/ray/triangle callers
```

Authoritative ownership after the work:

- `transforms::decompose` validates that its input is representable as affine TRS.
- `Quat` owns identity/default/normalization semantics.
- `queries.rs::Distance<_, Plane<_>>` implements the general plane-distance equation.
- `try_basis_from_unit` owns the ordered right-handed frame invariant.
- Each ray query owns the strict `t >= 0` boundary.
- `queries.rs` owns total sphere/triangle behavior for face-, segment-, and point-like triangles.
- `src/vector.rs` owns crate-private dimensionless orientation predicates used by P2 callers.

## What makes the work complicated

- **Inherent:** TRS decomposition must separate rotation, reflection, scale, shear, and singularity;
  ordered orthonormal frames must choose handedness; degenerate triangles reduce to segments or
  points; tolerances must distinguish world-space length from dimensionless angle.
- **Induced:** The crate must remain `no_std`, dependency-free beyond existing backends, generic over
  `f32`/`f64`, and compatible with current `Option`-returning APIs.
- **Accidental:** Some core contracts are implicit, quantities with different units share epsilon
  comparisons, and tests validate components one-sidedly instead of validating reconstructed
  results.

## Current inconsistencies and known defects

### P0 — Core logic and algorithms

1. `decompose` returns `Some` for nonsingular sheared affine matrices that scale plus quaternion
   cannot reconstruct, and for singular matrices with nonzero dependent columns.
2. Ray/triangle intersection returns `Some` with negative `t`, contradicting the README and
   ray/plane semantics.
3. `Quat::default()` is zero; matrix conversion treats it as identity while axis-angle conversion
   reports pi and normalization does not produce a unit quaternion.
4. `try_basis_from_unit` computes `w = u × v` but returns `[u, w, v]`, whose ordered determinant is
   `-1`. This is inconsistent with the crate's right-handed `Basis` convention and common ordered
   frame use.
5. Point/plane distance divides by squared normal length instead of normal length. Constructor
   normalization masks this today, but the implementation is not the general equation it claims.
6. `test_decompose` omits `abs()` from approximate comparisons, allowing arbitrarily low incorrect
   values to pass.

### P1 — Degenerate geometry

7. Sphere/triangle intersection returns `false` when any zero-length edge produces `None`, even if
   another represented edge or vertex intersects the sphere.

### P2 — Scale and tolerance

8. Line/plane and line/line decisions change when equivalent line directions are rescaled.
9. Line/ray-to-triangle decisions compare a raw determinant to linear epsilon, making hit/miss
   depend on direction magnitude and triangle size.

## Priority and completion rules

- **P0 — Core logic and algorithms:** release-blocking mathematical or contract errors. Each item
  lands directly with its regression tests and documentation; none waits for scale work.
- **P1 — Degenerate geometry:** define total behavior for constructible but lower-dimensional
  shapes after core query semantics are sound.
- **P2 — Scale and tolerance:** centralize dimensionless orientation policy and migrate all affected
  consumers. This is deliberately lower priority, but P2.1 must precede P2.2/P2.3.
- **P3 — Cleanup and full validation:** remove stale comments, reconcile documentation, and run the
  complete feature/platform matrix.

No temporary adapter, feature flag, or permanent old/new algorithm split is needed. Every old branch
is removed in the same checklist item that replaces it. P0's strict `t` fix touches the shared
triangle routine first; P2 later replaces only its independent raw determinant threshold.

## Ordered checklist

### P0 — Core logic and algorithm correctness

- [ ] **P0.1 — Make `decompose` reject non-TRS matrices and strengthen its oracle**

  **Problem**

  `transforms::decompose` treats nonzero column lengths as sufficient for invertibility and rotation.
  A shear is a valid, nonsingular affine transform but cannot be represented by its return tuple.
  The current test can also pass grossly low results because it uses one-sided differences.

  **Decision needed: Yes (resolved by this plan)**

  1. **Keep `Option` and accept affine TRS without shear (recommended).**
     - Benefits: honors the existing signature and documented failure path; every `Some` can be
       required to reconstruct the input.
     - Consequences: sheared matrices that currently return misleading values begin returning
       `None`; failure reasons remain undifferentiated.
     - Shape:

       ```rust
       if !m.is_affine(T::epsilon()) || !normalized_columns_are_orthogonal {
           return None;
       }
       ```

  2. **Add a new affine result containing shear and retain `decompose` as a wrapper.**
     - Benefits: represents general affine matrices.
     - Consequences: adds a new type, composition contract, and migration beyond this bug-fix scope.
     - Shape:

       ```rust
       struct AffineParts<T> {
           scale: Vector3<T>,
           shear: Vector3<T>,
           rotation: Quat<T>,
           translation: Vector3<T>,
       }
       ```

  3. **Reject determinant zero only.**
     - Benefits: minimal singular-matrix patch.
     - Consequences: continues silently producing false decompositions for shear.
     - Shape: `if linear.determinant().tabs() <= T::epsilon() { return None; }`

  **Recommendation and why**

  Use option 1. The existing tuple is explicitly TRS-shaped, and `Option` already provides a
  compatibility-safe rejection channel.

  **Target contract or migration**

  In `src/transforms.rs::decompose`:

  1. Reject a non-affine last row using `m.is_affine(T::epsilon())`.
  2. Extract translation and the three linear columns.
  3. Reject any column length at or below `T::epsilon()`.
  4. Determine reflection from the upper-left `Matrix3` determinant and preserve the current
     all-negative scale convention for a negative determinant.
  5. Normalize the columns by the selected signed scales.
  6. Compare pairwise dot products of those unit columns against the dimensionless epsilon and
     reject nonorthogonality; also require normalized determinant approximately `+1`.
  7. Convert only the validated rotation through `Quat::of_matrix3`.

  P0 uses normalized-column dot checks locally and does not depend on the later P2 scale predicates.
  Keep the public signature and do not add a shear result or error enum.

  **Acceptance tests**

  - Replace all one-sided component assertions in `test_decompose` with an absolute
    `assert_matrix4_close` reconstruction helper.
  - Recompose every successful result as
    `translate(t) * rotation_from_quat(&q) * scale(s)` and compare the complete matrix, avoiding
    assumptions about quaternion sign.
  - Cover ordinary TRS, nonuniform scale, translation, rotation, and one/two/three negative scales.
  - Reject nonsingular shear, dependent nonzero columns, zero/near-zero scale, and non-affine rows.
  - Update `decompose` rustdoc and README to say "affine TRS without shear."
  - Remove the unchecked column-to-quaternion path and every one-sided approximate assertion.

- [ ] **P0.2 — Establish identity as the quaternion default and zero fallback**

  **Problem**

  Derived `Default` creates an invalid zero rotation. `normalize` promises a unit quaternion but
  returns zero, after which matrix and axis-angle conversions disagree.

  **Decision needed: Yes (resolved by this plan)**

  1. **Default to identity, add checked normalization, and make total normalization fall back to
     identity (recommended).**
     - Benefits: defaults are valid rotations; conversions agree; callers can detect zero through
       an additive checked API.
     - Consequences: intentionally changes `Quat::default()` and `Quat::normalize(&zero)` behavior.
     - Shape:

       ```rust
       impl<T: Scalar> Default for Quat<T> {
           fn default() -> Self {
               Self { x: T::zero(), y: T::zero(), z: T::zero(), w: T::one() }
           }
       }

       pub fn try_normalize(q: &Self, epsilon: T) -> Option<Self>;
       ```

  2. **Keep zero default and declare it invalid.**
     - Benefits: preserves current `Default` bytes.
     - Consequences: keeps a public default that violates the rotation-oriented type contract and
       still requires every conversion to choose a fallback.
     - Shape: `/// Quat::default() is not a valid rotation.`

  3. **Make normalization and conversions return `Option`.**
     - Benefits: invalid values cannot be overlooked.
     - Consequences: breaks established signatures and downstream callers.
     - Shape: `pub fn mat3(&self) -> Option<Matrix3<T>>`

  **Recommendation and why**

  Use option 1. It matches the rotation-focused API and mirrors `FloatVector::try_normalize` without
  breaking existing methods.

  **Target contract or migration**

  Remove `Default` from the derive list and implement identity directly for `Quat<T: Scalar>`.
  `try_normalize` returns `None` when squared length is at or below `epsilon * epsilon`.
  `normalize` remains total, preserves current normalization for every nonzero input, and maps exact
  zero to identity. `mat3`, `mat4`, and `to_axis_angle` continue through total normalization.
  `inverse` retains its separately documented too-small fallback and is not described as a valid
  inverse for zero.

  **Acceptance tests**

  - Assert default fields equal identity fields for `f32` and `f64`.
  - Assert default and explicit zero convert to identity matrices and `(unit_x, 0)` axis-angle.
  - Assert `try_normalize(zero, EPS_F32)` is `None` and valid inputs return unit quaternions.
  - Preserve non-unit matrix conversion, multiplication, inverse, composition, and round trips.
  - Add a README migration note; remove the zero-derived default and zero-preserving normalization
    branch.

- [ ] **P0.3 — Enforce the strict ray half-line in triangle intersection**

  **Problem**

  `triangle_intersection_from_point_dir` accepts ray parameters down to `-T::epsilon()`. A valid,
  nondegenerate triangle behind a normalized ray can therefore return `Some` with negative `t`, in
  direct conflict with README and ray/plane behavior.

  **Decision needed: Yes (resolved by this plan)**

  1. **Reject every `t < 0` (recommended).**
     - Benefits: implements a mathematical ray and agrees with `Ray::intersect_plane`.
     - Consequences: removes the old behind-origin tolerance band.
     - Shape: `if ray && t < T::zero() { return None; }`

  2. **Accept the epsilon band but clamp returned `t` to zero.**
     - Benefits: tolerates numerical drift while hiding negative results.
     - Consequences: can return the ray origin even when it is not on the triangle.
     - Shape: `let t = T::max(t, T::zero());`

  3. **Preserve and document `t >= -epsilon`.**
     - Benefits: no implementation change.
     - Consequences: contradicts true-ray semantics and the parallel ray/plane API.
     - Shape: `if t < -epsilon { return None; }`

  **Recommendation and why**

  Use option 1. Boundary tolerance belongs to barycentric inclusion, not to the half-line's domain.

  **Target contract or migration**

  Change only the `TriangleIntersectionKind::Ray` parameter-domain check in P0. Keep the existing
  determinant tolerance until P2.3 addresses scale sensitivity independently. Both symmetric
  `Ray`/`Tri3` trait directions must return identical results.

  **Acceptance tests**

  - Test valid triangles at `t = -EPS_F32 / 2`, `t = -0.0`, `t = 0`, and `t > 0`.
  - Require `None` for every strictly negative result and `Some` at the origin boundary.
  - Preserve infinite-line hits on both sides and forward ray hits.
  - Keep the README true-ray statement; remove only the `t < -epsilon` branch in this item.

- [ ] **P0.4 — Return a documented right-handed orthonormal basis**

  **Problem**

  `try_basis_from_unit` computes `w = normalize(u × v)` but returns `[u, w, v]`. The ordered frame
  has determinant `-1`. Existing tests check lengths and orthogonality but never orientation, and
  their variable order mirrors the implementation instead of establishing a contract.

  **Decision needed: Yes (resolved by this plan)**

  1. **Return `[u, v, w]` and require `u × v = w` (recommended).**
     - Benefits: produces a conventional right-handed frame and aligns with `Basis::default`'s
       documented handedness.
     - Consequences: intentionally swaps the last two output elements for downstream callers.
     - Shape: `Some([u, v, w])`

  2. **Preserve `[u, w, v]` and document it as left-handed.**
     - Benefits: no output-order change.
     - Consequences: preserves surprising orientation and makes the cited `w = u × v` construction
       awkward for consumers.
     - Shape: `/// Returns a left-handed [u, w, v] frame.`

  3. **Replace the array with a named public frame type.**
     - Benefits: eliminates positional ambiguity.
     - Consequences: breaks the public return type and duplicates the existing `Basis` concept.
     - Shape: `pub struct OrthonormalFrame<T> { u: Vector3<T>, v: Vector3<T>, w: Vector3<T> }`

  **Recommendation and why**

  Use option 1. Handedness is a core ordered-basis invariant, and the behavioral correction is
  smaller than introducing another public frame type.

  **Target contract or migration**

  Keep the signatures of `try_basis_from_unit` and `basis_from_unit`; change the result ordering to
  `[u, v, w]`. State explicitly that element zero is the normalized input, the frame is orthonormal,
  and `cross(result[0], result[1])` equals `result[2]` within epsilon.

  **Acceptance tests**

  - Replace test destructuring `[u, w, v]` with `[u, v, w]`.
  - Assert unit lengths, pairwise orthogonality, `u × v ≈ w`, and determinant `+1`.
  - Cover axis-aligned, tied-component, arbitrary, and negative input directions.
  - Add an explicit README/rustdoc migration note about the last-two-element swap.
  - Remove tests or comments that encode only the old left-handed ordering.

- [ ] **P0.5 — Correct the generic point-to-plane distance equation**

  **Problem**

  `impl Distance<T, Vector3<T>> for Plane<T>` computes
  `abs(n · p + d) / (n · n)`. The general Euclidean distance is
  `abs(n · p + d) / sqrt(n · n)`. Safe constructors normalize `n`, masking the discrepancy, but
  the algorithm and its claimed equation remain wrong and brittle to representation changes or FFI.

  **Decision needed: Yes (resolved by this plan)**

  1. **Implement the general equation (recommended).**
     - Benefits: mathematically correct for every nonzero normal and independent of constructor
       normalization; keeps the public return type.
     - Consequences: performs one square root per query.
     - Shape: `Some(nom.tabs() / denom.tsqrt())`

  2. **Make unit-normal representation an explicit invariant and divide by one.**
     - Benefits: avoids a square root for safely constructed planes.
     - Consequences: couples the trait implementation to private construction and cannot validate
       the general formula; default/raw/FFI representations remain fragile.
     - Shape: `Some(nom.tabs())`

  3. **Keep division by squared norm.**
     - Benefits: no code change.
     - Consequences: returns a non-distance for unnormalized coefficients.
     - Shape: `Some(nom.tabs() / denom)`

  **Recommendation and why**

  Use option 1. Correctness takes precedence over an unmeasured square-root optimization, and no
  benchmark identifies plane distance as a hot path.

  **Target contract or migration**

  Keep `Distance::distance` unchanged. In `src/queries.rs`, reject squared normal length at or below
  `T::epsilon() * T::epsilon()` and otherwise divide the absolute plane equation by its square root.
  Do not add a public raw-plane constructor solely for testing.

  **Acceptance tests**

  - In `src/primitives.rs`'s descendant test module, import `crate::queries::Distance`, construct
    private `Plane` coefficients directly, and verify `(n, d)` and `(k*n, k*d)` return the same
    distance.
  - Cover normalized planes, unnormalized coefficients, points on each side, points on the plane,
    and the zero/default plane returning `None`.
  - Preserve current results for all safely constructed normalized planes.
  - Update trait/rustdoc formula; remove the squared-denominator implementation.

### P1 — Degenerate geometry behavior

- [ ] **P1.1 — Make sphere/triangle intersection total for lower-dimensional triangles**

  **Problem**

  `Tri3::new` and `Tri3::default` allow degenerate triangles, but sphere intersection computes
  barycentrics before confirming a plane and returns `false` if any one fallback edge distance is
  `None`.

  **Decision needed: Yes (resolved by this plan)**

  1. **Treat a degenerate triangle as the closure of its vertices and edges (recommended).**
     - Benefits: gives every constructible `Tri3` deterministic, boundary-inclusive boolean
       behavior without changing signatures.
     - Consequences: point-/segment-like triangles that previously always missed may now intersect.
     - Shape:

       ```rust
       segment.distance(point, epsilon)
           .unwrap_or_else(|| Vector3::distance(&segment.s, point))
       ```

  2. **Declare non-degenerate triangles a precondition.**
     - Benefits: smallest implementation.
     - Consequences: `Intersect` cannot report the violated precondition and `Tri3::default` remains
       a silently unusable input.
     - Shape: `/// Returns false for degenerate triangles.`

  3. **Add checked triangle construction and reject existing unchecked values.**
     - Benefits: can encode validity at creation sites.
     - Consequences: does not help existing public fields/default values and expands scope.
     - Shape: `pub fn try_new(vertices, epsilon) -> Option<Self>`

  **Recommendation and why**

  Use option 1. Boolean intersection should describe the represented closed set: face when valid,
  segment when collinear/duplicated, and point when all vertices coincide.

  **Target contract or migration**

  In `impl Intersect<Tri3<T>> for Sphere3<T>`:

  - Attempt `Plane::try_from_tri` before the face-interior path.
  - If a valid plane exists and projected barycentrics are inside, use corrected plane distance.
  - Otherwise compute all edge distances, replacing a zero-length edge's `None` with endpoint
    distance, and compare their minimum with the canonical radius.
  - Keep `Segment::distance` returning `None` for zero length; its parameter remains undefined.

  **Acceptance tests**

  - Cover coincident-point, duplicate-vertex segment, and three-distinct-collinear triangles.
  - Include hit, miss, exact boundary, and zero-radius sphere cases.
  - Include the audit fixture `(0,0,0)`, `(0,0,0)`, `(2,0,0)` with a sphere at `(1,0,0)`.
  - Preserve nondegenerate face-interior, edge, vertex, and plane-distance behavior.
  - Document closure semantics; remove the all-three-`Some` match.

### P2 — Scale and tolerance invariance

- [ ] **P2.1 — Centralize dimensionless 3D orientation predicates**

  **Problem**

  Raw dot, cross, and determinant magnitudes are compared to a linear epsilon in multiple modules.
  Fixing consumers independently would preserve duplicated policy and future drift.

  **Decision needed: Yes (resolved by this plan)**

  1. **Use norm-scaled crate-private predicates in `vector.rs` (recommended).**
     - Benefits: invariant under positive direction scaling; preserves stored directions and `t`;
       requires no dependencies or allocation.
     - Consequences: callers must reject zero length separately; current extreme-magnitude overflow
       limitations remain.
     - Shape:

       ```rust
       dot * dot <= epsilon * epsilon * left_len_sq * right_len_sq
       cross_len_sq <= epsilon * epsilon * left_len_sq * right_len_sq
       ```

  2. **Normalize every line direction at construction.**
     - Benefits: simplifies later angular checks.
     - Consequences: changes public `t` values and endpoint parameterization; does not independently
       solve triangle-size sensitivity.
     - Shape: `d: *d / d.length()`

  3. **Use exact-zero orientation checks.**
     - Benefits: accepts all nonzero small geometry.
     - Consequences: unstable for nearly parallel inputs and inconsistent with existing epsilon APIs.
     - Shape: `if det == T::zero() { return None; }`

  **Recommendation and why**

  Use option 1. Length epsilon remains a world-unit cutoff; angular epsilon becomes dimensionless.

  **Target contract or migration**

  Add crate-private helpers such as:

  ```rust
  pub(crate) fn nearly_perpendicular3<T: FloatScalar>(
      left: &Vector3<T>, right: &Vector3<T>, angular_epsilon: T,
  ) -> bool;

  pub(crate) fn nearly_parallel3<T: FloatScalar>(
      left: &Vector3<T>, right: &Vector3<T>, angular_epsilon: T,
  ) -> bool;
  ```

  Treat a zero operand as indeterminate/near orientation; production callers validate length first.
  Keep helpers `pub(crate)`. Do not migrate consumers until helper tests pass.

  **Acceptance tests**

  - Test parallel, anti-parallel, perpendicular, oblique, near-threshold, and zero operands.
  - Independently scale nonzero operands by `0.25`, `1`, and `1024`; classification must not change.
  - Cover `f32` and `f64` through shared fixtures or a test macro.
  - Do not alter public vector traits or stored geometry.

- [ ] **P2.2 — Make line/plane and shortest-line queries invariant to direction magnitude**

  **Problem**

  `Plane::intersect_line` compares a raw normal/direction dot to epsilon, while
  `shortest_segment3d_between_lines3d` compares raw cross length squared to epsilon squared.

  **Decision needed: No**

  P2.1 owns the selected orientation policy and helpers.

  **Target contract or migration**

  Replace the plane denominator threshold with `nearly_perpendicular3` and the shortest-line cross
  threshold with `nearly_parallel3`, after existing length validation. Keep original directions for
  calculating `t` and endpoints. Use an inclusive threshold convention at the exact boundary.

  **Acceptance tests**

  - Scale the same line direction across positive magnitudes; hit/miss and point stay fixed while
    `t` scales inversely.
  - Orthogonal directions just above constructor epsilon must produce a shortest segment regardless
    of rescaling.
  - Parallel and near-parallel lines still return `None` at the selected angular threshold.
  - Preserve zero-direction constructor tests and normalized-direction behavior.
  - Update epsilon docs; remove raw denominator/cross comparison branches.

- [ ] **P2.3 — Make line/ray-triangle orientation tests independent of raw determinant scale**

  **Problem**

  Möller–Trumbore rejection compares a determinant combining direction magnitude and triangle area
  directly to epsilon. Valid well-shaped small triangles and equivalent line parameterizations can
  therefore change hit/miss.

  **Decision needed: No**

  P2.1 supplies relative orientation policy; P0.3 already fixed the independent ray-domain check.

  **Target contract or migration**

  In `triangle_intersection_from_point_dir`:

  1. Validate edge and direction lengths against the length epsilon.
  2. Compute the triangle normal and reject dimensionlessly near-parallel edges.
  3. Reject a direction dimensionlessly perpendicular to the triangle normal.
  4. Use the determinant only for barycentric/`t` division after validation; do not compare its raw
     magnitude to epsilon.
  5. Keep the existing dimensionless barycentric boundary tolerance and P0's strict ray `t >= 0`.

  Preserve both symmetric trait implementations and line `t` parameterization.

  **Acceptance tests**

  - Hit a well-shaped triangle whose edges are approximately `0.0008f32` and exceed length epsilon.
  - Scale line direction across positive magnitudes; point and hit/miss stay fixed and `t` scales
    inversely.
  - Scale/translate triangle fixtures while keeping edges above length cutoff.
  - Cover parallel, near-parallel, truly degenerate, outside-barycentric, edge, and vertex cases.
  - Re-run P0's negative/zero/positive ray tests after removing the raw determinant branch.
  - Remove the raw determinant epsilon comparison and document length-versus-angle semantics.

### P3 — Documentation, cleanup, and full validation

- [ ] **P3.1 — Reconcile documentation and validate every supported backend**

  **Problem**

  Core behavior, handedness, degeneracy, and tolerance contracts span several modules. Without a
  final sweep, stale prose or one-sided test helpers could preserve contradictory expectations.

  **Decision needed: No**

  P0–P2 resolve all material behavior and require focused tests.

  **Target contract or migration**

  Update:

  - `README.md` behavior notes for TRS-only decomposition, quaternion identity default,
    right-handed basis ordering, strict ray origin, corrected plane distance, degenerate triangle
    closure, and dimensionless orientation tolerance;
  - rustdoc on every affected public function and trait implementation;
  - test helpers so approximate equality is absolute or reconstruction-based.

  Do not add a changelog solely for these fixes. Use existing README and crate/module rustdoc for
  migration-visible changes. Remove stale comments describing old ordering/formulas/tolerances. No
  deprecated wrapper is needed because no existing signature is replaced.

  **Acceptance tests**

  - Search tests for one-sided approximate assertions and correct every unintended ordered check.
  - Run every command in Cross-cutting validation.
  - Confirm `git diff --check` and intended `cargo package --list` contents.
  - Confirm no dependencies, features, unsafe blocks, or public query signatures were added except
    additive `Quat::try_normalize`.

## Known defect matrix and ownership

| Priority | Defect | Current cause | Fix/verification item |
|---|---|---|---|
| P0 | Sheared/singular matrices return `Some` from `decompose` | No affine/orthogonality validation | P0.1 |
| P0 | Decomposition regressions can pass tests | Approximate assertions omit absolute value | P0.1 |
| P0 | Default/zero quaternion conversions disagree | Zero-derived default and zero-preserving normalization | P0.2 |
| P0 | Ray/triangle returns negative `t` | Epsilon band applied to ray domain | P0.3 |
| P0 | Basis builder returns an ordered left-handed frame | Computes `u × v = w` but returns `[u, w, v]` | P0.4 |
| P0 | Plane distance uses squared normal length | Denominator omits square root | P0.5 |
| P1 | Degenerate triangle can miss an intersecting sphere | Fallback requires all edge distances to be `Some` | P1.1 |
| P2 | Equivalent line directions change plane/shortest-line results | Raw dot/cross compared to epsilon | P2.1, P2.2 |
| P2 | Small/rescaled triangles change line/ray hit results | Raw Möller–Trumbore determinant compared to epsilon | P2.1, P2.3 |

## Cross-cutting validation

Run from the repository root:

```text
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-targets
cargo test --all-targets --features std
cargo test --all-targets --features libm
cargo test --all-targets --features system-libm
cargo test --all-targets --all-features
cargo test --doc --all-features
cargo check --no-default-features
cargo check --target x86_64-pc-windows-msvc --no-default-features
cargo check --target x86_64-pc-windows-msvc --no-default-features --features libm
cargo doc --no-deps --all-features
cargo package --allow-dirty --no-verify --list
git diff --check
```

`system-libm` execution is host-platform dependent because it links C math symbols. The Windows
commands are compile checks for the installed target. `--all-features` verifies documented feature
precedence. There are no benchmarks or runnable examples in this repository.

Regression assertions must be deterministic and mathematical:

- compare points, matrices, distances, determinants, handedness, and reconstructed transforms;
- use metamorphic coefficient/direction scaling for plane and P2 query tests;
- assert exact `Option`/boolean boundaries for invalid and degenerate inputs;
- exercise shared generic policy with `f32` and `f64` where practical;
- never retain an audited wrong output solely to keep an old test passing.

## Suggested implementation sequence

1. Implement P0.1 and replace the weak decomposition oracle in the same change.
2. Land the independent P0 quaternion, strict-ray, right-handed-basis, and plane-distance fixes with
   their focused tests; none depends on tolerance refactoring.
3. Implement P1's total lower-dimensional sphere/triangle behavior on top of corrected plane
   distance.
4. Add P2's crate-private orientation predicates, then migrate line/plane and line/ray/triangle
   consumers in separate compile-safe changes.
5. Complete the P3 documentation sweep and run the full feature/platform validation matrix.

No temporary adapter, feature flag, or dual runtime path is required.

## Completion definition

The work is complete when:

- `decompose` succeeds only for affine, nonsingular, shear-free TRS matrices and every success
  reconstructs its input within tolerance;
- decomposition tests use two-sided/reconstruction-based assertions;
- `Quat::default()` is identity, checked zero normalization returns `None`, and total zero
  conversions consistently yield identity;
- every ray/triangle result has `t >= 0` while infinite lines continue to hit on either side;
- `basis_from_unit` and `try_basis_from_unit` return orthonormal `[u, v, w]` frames with determinant
  `+1` and `u × v = w`;
- point/plane distance follows `abs(n · p + d) / |n|` and is invariant under nonzero coefficient
  scaling;
- sphere/triangle queries handle face-, segment-, and point-like triangles with boundary-inclusive
  behavior;
- positive direction scaling cannot change geometric hit/miss, parallelism, intersection points,
  or shortest-segment endpoints;
- valid triangles above the length cutoff are not rejected solely because a raw area determinant is
  below linear epsilon;
- old raw tolerance branches, the all-edge-`Some` fallback, zero-derived quaternion default,
  left-handed basis ordering, squared plane-distance denominator, negative-ray epsilon band, and
  one-sided decomposition assertions are gone;
- public signatures remain stable except for additive `Quat::try_normalize`;
- README, rustdoc, tests, doctests, Clippy, formatting, backend tests, platform checks,
  documentation, and package contents agree with the final contracts.
