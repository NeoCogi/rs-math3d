# Collision experiment

[`examples/collision/mod.rs`](../examples/collision/mod.rs) is an example-only
discrete narrow phase, generic over `RealScalar`. It reuses `Sphere3` and `Tri3`
and adds local `Obb` and `Capsule` types. It is a starting point for measuring a
small physics engine; it does not add a supported collision API to the crate.

All ten unordered shape pairings return a boolean. Boundary contact counts as
overlap in exact arithmetic. There are no contact points, normals, penetration
depths, manifolds, broad phase, integration, constraint solver or continuous
collision detection. Swapping arguments to a mixed pair means calling the same
function with its documented shape order.

| Pair | Algorithm |
| --- | --- |
| Sphere / sphere | Squared center distance |
| Sphere / capsule | Squared point-segment distance |
| Sphere / box | Clamp the box-local center to the box |
| Sphere / triangle | Closest point in triangle Voronoi regions |
| Capsule / capsule | Closest points on two closed segments |
| Capsule / box | Minimize piecewise quadratic segment-box distance |
| Capsule / triangle | Face piercing, endpoint-face and edge-edge distances |
| Box / box | 15 separating axes: face normals and edge crosses |
| Box / triangle | 13 separating axes: face normals and edge crosses |
| Triangle / triangle | Face normals, edge crosses and in-plane edge normals |

Queries perform no allocation, square root, trigonometry, normalization or
matrix inversion. Separating axes do not need unit length. Box orientation is
supplied by the caller. Capsule-box distance uses the rounded edges and corners
of the swept shape; simply expanding box slabs by a radius would introduce
false positives. Triangle-triangle includes the in-plane axes needed to reject
disjoint coplanar triangles.

All arithmetic uses the library's ordinary, unchecked operators and vector
kernels. This is safe Rust and introduces no `unsafe` or unchecked memory
access. The caller supplies:

- Finite coordinates, nonnegative radii and half extents.
- Orthonormal box axes, represented to the scalar's available precision.
- Nondegenerate triangles with resolvable region-test denominators.
- Representable intermediates, including squared distances and products of dot
  products, and enough fractional precision for projections and divisions.

Coincident capsule endpoints are supported as spheres. Exact parallel segment
directions are handled without dividing by a zero Gram determinant. Small
directions, almost parallel axes, or tiny triangles can quantize to degeneracy
in fixed point; these routines do not promise robust predicates for arbitrary
scales. Rounding can move a contact decision either way. No tolerance or hidden
collision margin is added. A bounded coordinate domain and explicit contact
margin would be necessary before turning this into a production physics API.
Choosing a format only by the range of the input coordinates is insufficient:
some region tests use fourth-order intermediates. Larger worlds need local
coordinates, rescaling or wider storage.

Run the regressions with `cargo test --test collision_examples`, and repeat with
`--no-default-features` for the fixed-only path. They cover all ten pairings,
touches and misses, all point-triangle Voronoi regions, collapsed and parallel
capsule centerlines, triangle face piercing, box corner contacts, coplanar
triangles, reversed winding and rigid transforms. Independent f64 references
check segment-box distance by convex minimization and oblique box overlap by
edge clipping, including misses requiring edge-cross axes.
