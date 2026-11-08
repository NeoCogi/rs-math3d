// Copyright 2020-Present (c) Raja Lehtihet & Wael El Oraiby
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//! Example-only static AABB tree for owned, nondegenerate triangle soups.
//!
//! Construction validates represented geometry and allocates. Balanced median
//! splits preserve input triangle IDs; geometry and topology remain immutable.
//! Queries recurse without heap allocation and use ordinary unchecked arithmetic.
//! Supply finite valid rays, representable translations and narrow-phase
//! intermediates. Intersection means surface contact, without solid containment.

use super::triangle_triangle;
use rs_math3d::{Box3, CrossProduct, Intersect, MathError, Ray, RealScalar, Tri3, Vector, Vector3};

/// Failures at the tree's checked construction boundary.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BuildError {
    /// Leaves must hold at least one triangle.
    InvalidLeafSize,
    /// A triangle has nonfinite, degenerate or unrepresentable edge geometry.
    InvalidTriangle {
        /// Original input triangle index.
        index: usize,
        /// Numeric reason for rejection.
        error: MathError,
    },
    /// The soup's full coordinate span does not fit the scalar.
    InvalidSpan(MathError),
}

/// Work counters included in both accelerated and brute-force query timings.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct QueryStats {
    /// Bounding-box tests, including leaf primitive bounds.
    pub bounds_tests: usize,
    /// Narrow-phase triangle tests after bounding rejection.
    pub triangle_tests: usize,
}

/// Owned query outcome; H identifies the concrete geometric result.
#[derive(Clone, Copy, Debug)]
pub struct QueryResult<H> {
    /// Nearest ray hit or any intersecting input triangle pair.
    pub hit: Option<H>,
    /// Traversal/narrow-phase work performed for this result.
    pub stats: QueryStats,
}

impl<H> Default for QueryResult<H> {
    fn default() -> Self {
        Self {
            hit: None,
            stats: QueryStats::default(),
        }
    }
}

/// Nearest two-sided forward-ray intersection in the ray's stored parameterization.
#[derive(Clone, Copy, Debug)]
pub struct RayHit<T: RealScalar> {
    /// Stable original input triangle ID; smallest ID wins exact parameter ties.
    pub triangle: usize,
    /// Nonnegative t in start + direction*t.
    pub parameter: T,
    /// Represented hit point computed from that parameterization.
    pub point: Vector3<T>,
}

/// Build size information for reproducible benchmark reporting.
#[derive(Clone, Copy, Debug)]
pub struct TreeInfo {
    /// Number of input triangles, including duplicates.
    pub triangles: usize,
    /// Number of immutable hierarchy nodes.
    pub nodes: usize,
    /// Maximum node depth; empty trees have zero depth and a root has depth one.
    pub depth: usize,
    /// Reserved heap bytes for entries, IDs and nodes, excluding allocator overhead.
    pub reserved_bytes: usize,
}

/// Authoritative geometry plus a cached bound used by both query implementations.
struct Entry<T: RealScalar> {
    /// Input-order triangle; its position remains its stable ID.
    triangle: Tri3<T>,
    /// Componentwise vertex bounds, without arithmetic expansion or padding.
    bounds: Box3<T>,
}

/// A node owns a leaf range or a pair of child node indices, never both.
enum Kind {
    /// First slot in the tree's permutation; node.count gives the range length.
    Leaf { start: usize },
    /// Two nonempty child subtrees partitioning the parent's ID range.
    Branch { left: usize, right: usize },
}

/// Immutable hierarchy state. Only the builder can create or connect nodes.
struct Node<T: RealScalar> {
    /// Union of every descendant triangle bound.
    bounds: Box3<T>,
    /// Descendant triangle count, also the authoritative leaf range length.
    count: usize,
    /// Leaf storage location or branch topology.
    kind: Kind,
}

/// Static, balanced triangle-soup hierarchy with stable input IDs.
pub struct AabbTree<T: RealScalar> {
    /// Owned input-order entries; queries cannot mutate geometry behind bounds.
    entries: Vec<Entry<T>>,
    /// Input IDs partitioned into disjoint leaf ranges during construction.
    order: Vec<usize>,
    /// Root-first nodes; empty input has no nodes.
    nodes: Vec<Node<T>>,
    /// Maximum depth computed by the builder, for reporting.
    depth: usize,
}

/// Componentwise closed bound of exactly three represented vertices.
fn triangle_bounds<T: RealScalar>(triangle: &Tri3<T>) -> Box3<T> {
    let [a, b, c] = *triangle.vertices();
    Box3::new(
        &Vector3::min(&a, &Vector3::min(&b, &c)),
        &Vector3::max(&a, &Vector3::max(&b, &c)),
    )
}

/// Closed interval overlap, preserving face/edge/point contacts.
fn overlap<T: RealScalar>(a: &Box3<T>, b: &Box3<T>) -> bool {
    a.min.x <= b.max.x
        && a.max.x >= b.min.x
        && a.min.y <= b.max.y
        && a.max.y >= b.min.y
        && a.min.z <= b.max.z
        && a.max.z >= b.min.z
}

/// Translates a bound in the query's represented arithmetic. Callers establish
/// range; componentwise addition preserves containment of translated vertices.
fn translated<T: RealScalar>(bounds: &Box3<T>, offset: Vector3<T>) -> Box3<T> {
    Box3::new(&(bounds.min + offset), &(bounds.max + offset))
}

/// Translation of the right-hand triangle into the left soup's coordinate frame.
fn translated_triangle<T: RealScalar>(triangle: &Tri3<T>, offset: Vector3<T>) -> Tri3<T> {
    let v = triangle.vertices();
    Tri3::new([v[0] + offset, v[1] + offset, v[2] + offset])
}

/// Box center along one axis. Construction validates the full soup span, so
/// subtraction fits and adding half of a nonnegative span stays within bounds.
fn center_axis<T: RealScalar>(bounds: &Box3<T>, axis: usize) -> T {
    let lo = [bounds.min.x, bounds.min.y, bounds.min.z][axis];
    let hi = [bounds.max.x, bounds.max.y, bounds.max.z][axis];
    lo + (hi - lo) * T::half()
}

/// Union of the nonempty set of cached primitive bounds in an ID slice.
fn union<T: RealScalar>(entries: &[Entry<T>], ids: &[usize]) -> Box3<T> {
    let mut result = entries[ids[0]].bounds;
    for &id in &ids[1..] {
        result.min = Vector3::min(&result.min, &entries[id].bounds.min);
        result.max = Vector3::max(&result.max, &entries[id].bounds.max);
    }
    result
}

/// Builds one nonempty subtree using a longest-extent axis and median box center.
/// ID tie-breaking makes coincident centers deterministic. Median splitting
/// always reduces both child lengths, including fully overlapping geometry.
fn build<T: RealScalar>(
    entries: &[Entry<T>],
    ids: &mut [usize],
    nodes: &mut Vec<Node<T>>,
    start: usize,
    leaf_size: usize,
) -> (usize, usize) {
    let bounds = union(entries, ids);
    let index = nodes.len();
    // Commit the branch only after both children exist. The temporary leaf owns
    // a valid full ID range, and no partially constructed tree escapes new().
    nodes.push(Node {
        bounds,
        count: ids.len(),
        kind: Kind::Leaf { start },
    });
    if ids.len() <= leaf_size {
        return (index, 1);
    }
    let extent = bounds.max - bounds.min;
    let mut axis = if extent.y > extent.x { 1 } else { 0 };
    if extent.z > [extent.x, extent.y, extent.z][axis] {
        axis = 2;
    }
    let middle = ids.len() / 2;
    ids.select_nth_unstable_by(middle, |&a, &b| {
        center_axis(&entries[a].bounds, axis)
            .partial_cmp(&center_axis(&entries[b].bounds, axis))
            .expect("constructor rejects nonfinite coordinates")
            .then_with(|| a.cmp(&b))
    });
    let (left, right) = ids.split_at_mut(middle);
    let (left, left_depth) = build(entries, left, nodes, start, leaf_size);
    let (right, right_depth) = build(entries, right, nodes, start + middle, leaf_size);
    nodes[index].kind = Kind::Branch { left, right };
    (index, 1 + left_depth.max(right_depth))
}

/// Two-sided Moller-Trumbore solve with unscaled barycentric numerators. Delay
/// division until after all sign/range rejection. A represented zero determinant
/// is parallel/unresolved; fixed-point rounding can change edge classifications.
fn ray_triangle<T: RealScalar>(
    ray: &Ray<T, Vector3<T>>,
    triangle: &Tri3<T>,
) -> Option<(T, Vector3<T>)> {
    let [a, b, c] = *triangle.vertices();
    let e1 = b - a;
    let e2 = c - a;
    let p = Vector3::cross(&ray.direction, &e2);
    let mut determinant = Vector3::dot(&e1, &p);
    if determinant == T::zero() {
        return None;
    }
    let delta = ray.start - a;
    let mut u = Vector3::dot(&delta, &p);
    let q = Vector3::cross(&delta, &e1);
    let mut v = Vector3::dot(&ray.direction, &q);
    let mut numerator = Vector3::dot(&e2, &q);
    if determinant < T::zero() {
        determinant = -determinant;
        u = -u;
        v = -v;
        numerator = -numerator;
    }
    if u < T::zero()
        || v < T::zero()
        || u > determinant
        || v > determinant - u
        || numerator < T::zero()
    {
        return None;
    }
    let t = numerator / determinant;
    Some((t, ray.start + ray.direction * t))
}

impl<T: RealScalar> AabbTree<T> {
    /// Validates finite nondegenerate represented triangles and a representable
    /// full coordinate span, then builds deterministic median partitions.
    /// Empty input is valid. Allocation failures follow Vec's standard behavior.
    pub fn new(triangles: Vec<Tri3<T>>, leaf_size: usize) -> Result<Self, BuildError> {
        if leaf_size == 0 {
            return Err(BuildError::InvalidLeafSize);
        }
        let mut entries = Vec::with_capacity(triangles.len());
        for (index, triangle) in triangles.into_iter().enumerate() {
            let validate = || -> Result<(), MathError> {
                let [a, b, c] = *triangle.vertices();
                if [a, b, c]
                    .iter()
                    .any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite())
                {
                    return Err(MathError::NonFinite);
                }
                let e1 = Vector3::checked_sub_vv(&b, &a)?;
                let e2 = Vector3::checked_sub_vv(&c, &a)?;
                let normal = Vector3::checked_cross(&e1, &e2)?;
                if normal.x == T::zero() && normal.y == T::zero() && normal.z == T::zero() {
                    return Err(MathError::Degenerate);
                }
                Ok(())
            };
            validate().map_err(|error| BuildError::InvalidTriangle { index, error })?;
            let bounds = triangle_bounds(&triangle);
            entries.push(Entry { triangle, bounds });
        }
        let mut order: Vec<_> = (0..entries.len()).collect();
        let mut nodes = Vec::new();
        let depth = if order.is_empty() {
            0
        } else {
            let bounds = union(&entries, &order);
            Vector3::checked_sub_vv(&bounds.max, &bounds.min).map_err(BuildError::InvalidSpan)?;
            build(&entries, &mut order, &mut nodes, 0, leaf_size).1
        };
        Ok(Self {
            entries,
            order,
            nodes,
            depth,
        })
    }

    /// Reports owned storage and hierarchy size without exposing mutable topology.
    pub fn info(&self) -> TreeInfo {
        TreeInfo {
            triangles: self.entries.len(),
            nodes: self.nodes.len(),
            depth: self.depth,
            reserved_bytes: self.entries.capacity() * core::mem::size_of::<Entry<T>>()
                + self.nodes.capacity() * core::mem::size_of::<Node<T>>()
                + self.order.capacity() * core::mem::size_of::<usize>(),
        }
    }

    /// Retrieves geometry by stable input ID, including duplicate triangles.
    pub fn triangle(&self, id: usize) -> Option<&Tri3<T>> {
        self.entries.get(id).map(|e| &e.triangle)
    }

    /// Finds the nearest two-sided forward-ray hit; exact t ties choose the
    /// smallest input ID. No nearest-distance pruning is used, so rounded slab
    /// parameters cannot incorrectly prune a closer rounded triangle result.
    pub fn raycast(&self, ray: &Ray<T, Vector3<T>>) -> QueryResult<RayHit<T>> {
        let mut result = QueryResult::default();
        if !self.nodes.is_empty() {
            self.visit_ray(0, ray, &mut result);
        }
        result
    }

    /// Independent flat traversal of the same bounding/narrow-phase predicate.
    /// Useful as an acceleration oracle; it is not an exact-arithmetic oracle.
    pub fn brute_raycast(&self, ray: &Ray<T, Vector3<T>>) -> QueryResult<RayHit<T>> {
        let mut result = QueryResult::default();
        if self.nodes.is_empty() {
            return result;
        }
        // Give flat traversal the same whole-soup broad rejection. Internal
        // node traversal and ID permutation remain exclusive to the tree query.
        result.stats.bounds_tests += 1;
        if !self.nodes[0].bounds.intersect(ray) {
            return result;
        }
        for id in 0..self.entries.len() {
            self.test_ray(id, ray, &mut result);
        }
        result
    }

    /// Tests one primitive's mandatory bounding filter and narrow-phase solve.
    /// Both traversals use this predicate so quantization-induced SAT/edge
    /// false positives outside a triangle's own bounds cannot defeat the tree.
    fn test_ray(&self, id: usize, ray: &Ray<T, Vector3<T>>, result: &mut QueryResult<RayHit<T>>) {
        let entry = &self.entries[id];
        result.stats.bounds_tests += 1;
        if !entry.bounds.intersect(ray) {
            return;
        }
        result.stats.triangle_tests += 1;
        if let Some((parameter, point)) = ray_triangle(ray, &entry.triangle) {
            let replace = result.hit.as_ref().map_or(true, |old| {
                parameter < old.parameter || (parameter == old.parameter && id < old.triangle)
            });
            if replace {
                result.hit = Some(RayHit {
                    triangle: id,
                    parameter,
                    point,
                });
            }
        }
    }

    /// Traverses node bounds before visiting leaf IDs, with bounded recursion
    /// depth established by median construction and no heap scratch storage.
    fn visit_ray(
        &self,
        index: usize,
        ray: &Ray<T, Vector3<T>>,
        result: &mut QueryResult<RayHit<T>>,
    ) {
        let node = &self.nodes[index];
        result.stats.bounds_tests += 1;
        if !node.bounds.intersect(ray) {
            return;
        }
        match node.kind {
            Kind::Leaf { start } => {
                for &id in &self.order[start..start + node.count] {
                    self.test_ray(id, ray, result);
                }
            }
            Kind::Branch { left, right } => {
                self.visit_ray(left, ray, result);
                self.visit_ray(right, ray, result);
            }
        }
    }

    /// Returns any surface-intersecting input triangle pair. The right soup is
    /// translated into the left soup's coordinates; rotations require rebuilding
    /// transformed geometry. Fully contained disjoint surfaces return no hit.
    pub fn intersects(&self, other: &Self, offset: Vector3<T>) -> QueryResult<(usize, usize)> {
        let mut result = QueryResult::default();
        if !self.nodes.is_empty() && !other.nodes.is_empty() {
            result.hit = self.visit_pair(0, other, 0, offset, &mut result.stats);
        }
        result
    }

    /// Flat nested traversal using the exact same per-pair predicate as the tree.
    /// The first witness can differ; hit/miss classification must agree.
    pub fn brute_intersects(
        &self,
        other: &Self,
        offset: Vector3<T>,
    ) -> QueryResult<(usize, usize)> {
        let mut result = QueryResult::default();
        if self.nodes.is_empty() || other.nodes.is_empty() {
            return result;
        }
        result.stats.bounds_tests += 1;
        if !overlap(
            &self.nodes[0].bounds,
            &translated(&other.nodes[0].bounds, offset),
        ) {
            return result;
        }
        for a in 0..self.entries.len() {
            for b in 0..other.entries.len() {
                if self.test_pair(a, other, b, offset, &mut result.stats) {
                    result.hit = Some((a, b));
                    return result;
                }
            }
        }
        result
    }

    /// AABB rejection is part of the leaf predicate in both implementations.
    /// This also rejects disjoint bounds when fixed SAT axes quantize to zero.
    fn test_pair(
        &self,
        a: usize,
        other: &Self,
        b: usize,
        offset: Vector3<T>,
        stats: &mut QueryStats,
    ) -> bool {
        stats.bounds_tests += 1;
        if !overlap(
            &self.entries[a].bounds,
            &translated(&other.entries[b].bounds, offset),
        ) {
            return false;
        }
        stats.triangle_tests += 1;
        triangle_triangle(
            &self.entries[a].triangle,
            &translated_triangle(&other.entries[b].triangle, offset),
        )
    }

    /// Traverses overlapping node pairs, splitting the larger non-leaf subtree.
    /// Every recursion descends at least one tree, bounding stack depth by the
    /// sum of tree depths. A witness terminates traversal immediately.
    fn visit_pair(
        &self,
        a: usize,
        other: &Self,
        b: usize,
        offset: Vector3<T>,
        stats: &mut QueryStats,
    ) -> Option<(usize, usize)> {
        let lhs = &self.nodes[a];
        let rhs = &other.nodes[b];
        stats.bounds_tests += 1;
        if !overlap(&lhs.bounds, &translated(&rhs.bounds, offset)) {
            return None;
        }
        match (&lhs.kind, &rhs.kind) {
            (Kind::Leaf { start: la }, Kind::Leaf { start: lb }) => {
                for &i in &self.order[*la..*la + lhs.count] {
                    for &j in &other.order[*lb..*lb + rhs.count] {
                        if self.test_pair(i, other, j, offset, stats) {
                            return Some((i, j));
                        }
                    }
                }
                None
            }
            (Kind::Branch { left, right }, _)
                if lhs.count >= rhs.count || matches!(rhs.kind, Kind::Leaf { .. }) =>
            {
                self.visit_pair(*left, other, b, offset, stats)
                    .or_else(|| self.visit_pair(*right, other, b, offset, stats))
            }
            (_, Kind::Branch { left, right }) => self
                .visit_pair(a, other, *left, offset, stats)
                .or_else(|| self.visit_pair(a, other, *right, offset, stats)),
            (Kind::Branch { .. }, Kind::Leaf { .. }) => {
                unreachable!("a branch always splits against a leaf")
            }
        }
    }
}

#[cfg(test)]
mod tests;
