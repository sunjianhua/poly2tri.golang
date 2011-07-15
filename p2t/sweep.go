/*
 * Poly2Tri Copyright (c) 2009-2011, Poly2Tri Contributors
 * http://code.google.com/p/poly2tri/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package p2t
 
import (
	"math"
	"fmt"
)

// Triangulate simple polygon with holes
func triangulate(tcx *SweepContext) {
	//s.nodes = new(vector.Vector)
	tcx.initTriangulation()
	tcx.createAdvancingFront()
	// Sweep points; build mesh
	sweepPoints(tcx)
	// Clean up
	finalizationPolygon(tcx)
}

func sweepPoints(tcx *SweepContext) {
	for i := 1; i < len(tcx.points); i++ {
		var point = tcx.points[i]
		var node = pointEvent(tcx, point)
		for i := 0; i < point.edge_list.Len(); i++ {
			initSweepEdgeEvent(tcx, point.edge_list[i].(*Edge), node)
		}
	}
}

// TODO: Move this function to sweep_context?
func finalizationPolygon(tcx *SweepContext) {
	// Get an Internal triangle to start with
	var t *Triangle = tcx.front.head.next.triangle
	var p *Point = tcx.front.head.next.point
	for t != nil && !t.getConstrainedEdgeCW(p) {
		t = t.neighborCCW(p)
	}

	// Collect interior triangles constrained by edges
	tcx.meshClean(t)
}

func pointEvent(tcx *SweepContext, point *Point) *Node {

	var node = tcx.locateNode(point)
	var new_node = newFrontTriangle(tcx, point, node)

	// Only need to check +epsilon since point never have smaller
	// x value than node due to how we fetch nodes from the front
	if point.X <= node.point.X+EPSILON {
		fill(tcx, node)
	}

	fillAdvancingFront(tcx, new_node)
	return new_node
}

func newFrontTriangle(tcx *SweepContext, point *Point, node *Node) *Node {

	triangle := new(Triangle)
	triangle.init(point, node.point, node.next.point)

	triangle.markNeighbor2(node.triangle)
	triangle.eref = tcx.tmap.PushBack(triangle)

	new_node := &Node{point: point, value: point.X}
	//s.nodes.Push(new_node)

	new_node.next = node.next
	new_node.prev = node
	node.next.prev = new_node
	node.next = new_node

	if !legalize(tcx, triangle) {
		tcx.mapTriangleToNodes(triangle)
	}

	return new_node
}

func legalize(tcx *SweepContext, t *Triangle) bool {

	// To legalize a triangle we start by finding if any of the three edges
	// violate the Delaunay condition
	for i := 0; i < 3; i++ {
		if t.delaunay_edge[i] {
			continue
		}

		var ot = t.neighbor[i]

		if ot != nil {

			var p = t.Point[i]
			var op = ot.oppositePoint(t, p)
			var oi = ot.index(op)

			// If this is a Constrained Edge or a Delaunay Edge(only during recursive legalization)
			// then we should not try to legalize
			if ot.constrained_edge[oi] || ot.delaunay_edge[oi] {
				t.constrained_edge[i] = ot.constrained_edge[oi]
				continue
			}

			var inside = incircle(p, t.pointCCW(p), t.pointCW(p), op)

			if inside {
				// Lets mark this shared edge as Delaunay
				t.delaunay_edge[i] = true
				ot.delaunay_edge[oi] = true

				// Lets rotate shared edge one vertex CW to legalize it
				rotateTrianglePair(t, p, ot, op)

				// We now got one valid Delaunay Edge shared by two triangles
				// This gives us 4 new edges to check for Delaunay

				// Make sure that triangle to node mapping is done only one time for a specific triangle
				var not_legalized = !legalize(tcx, t)
				if not_legalized {
					tcx.mapTriangleToNodes(t)
				}

				not_legalized = !legalize(tcx, ot)
				if not_legalized {
					tcx.mapTriangleToNodes(ot)
				}

				// Reset the Delaunay edges, since they only are valid Delaunay edges
				// until we add a new triangle or point.
				// XXX: need to think about this. Can these edges be tried after we
				//      return to previous recursive level?
				t.delaunay_edge[i] = false
				ot.delaunay_edge[oi] = false

				// If triangle have been legalized no need to check the other edges since
				// the recursive legalization will handles those so we can end here.
				return true
			}
		}
	}
	return false
}

func incircle(pa, pb, pc, pd *Point) bool {

	var adx = pa.X - pd.X
	var ady = pa.Y - pd.Y
	var bdx = pb.X - pd.X
	var bdy = pb.Y - pd.Y

	var adxbdy = adx * bdy
	var bdxady = bdx * ady
	var oabd = adxbdy - bdxady

	if oabd <= 0 {
		return false
	}

	var cdx = pc.X - pd.X
	var cdy = pc.Y - pd.Y

	var cdxady = cdx * ady
	var adxcdy = adx * cdy
	var ocad = cdxady - adxcdy

	if ocad <= 0 {
		return false
	}

	var bdxcdy = bdx * cdy
	var cdxbdy = cdx * bdy

	var alift = adx*adx + ady*ady
	var blift = bdx*bdx + bdy*bdy
	var clift = cdx*cdx + cdy*cdy

	var det = alift*(bdxcdy-cdxbdy) + blift*ocad + clift*oabd

	return det > 0
}

func rotateTrianglePair(t *Triangle, p *Point, ot *Triangle, op *Point) {

	var n1, n2, n3, n4 *Triangle
	n1 = t.neighborCCW(p)
	n2 = t.neighborCW(p)
	n3 = ot.neighborCCW(op)
	n4 = ot.neighborCW(op)

	var ce1, ce2, ce3, ce4 bool
	ce1 = t.getConstrainedEdgeCCW(p)
	ce2 = t.getConstrainedEdgeCW(p)
	ce3 = ot.getConstrainedEdgeCCW(op)
	ce4 = ot.getConstrainedEdgeCW(op)

	var de1, de2, de3, de4 bool
	de1 = t.getDelunayEdgeCCW(p)
	de2 = t.getDelunayEdgeCW(p)
	de3 = ot.getDelunayEdgeCCW(op)
	de4 = ot.getDelunayEdgeCW(op)

	t.legalize2(p, op)
	ot.legalize2(op, p)

	// Remap delaunay_edge
	ot.setDelunayEdgeCCW(p, de1)
	t.setDelunayEdgeCW(p, de2)
	t.setDelunayEdgeCCW(op, de3)
	ot.setDelunayEdgeCW(op, de4)

	// Remap constrained_edge
	ot.setConstrainedEdgeCCW(p, ce1)
	t.setConstrainedEdgeCW(p, ce2)
	t.setConstrainedEdgeCCW(op, ce3)
	ot.setConstrainedEdgeCW(op, ce4)

	// Remap neighbors
	// XXX: might optimize the markNeighbor by keeping track of
	//      what side should be assigned to what neighbor after the
	//      rotation. Now mark neighbor does lots of testing to find
	//      the right side.
	t.clearNeighbors()
	ot.clearNeighbors()
	if n1 != nil {
		ot.markNeighbor2(n1)
	}
	if n2 != nil {
		t.markNeighbor2(n2)
	}
	if n3 != nil {
		t.markNeighbor2(n3)
	}
	if n4 != nil {
		ot.markNeighbor2(n4)
	}
	t.markNeighbor2(ot)
}

func initSweepEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {

	tcx.edge_event.constrained_edge = edge
	tcx.edge_event.right = (edge.p.X > edge.q.X)

	if isEdgeSideOfTriangle(node.triangle, edge.p, edge.q) {
		return
	}

	// For now we will do all needed filling
	// TODO: integrate with flip process might give some better performance
	//       but for now this avoid the issue with cases that needs both flips and fills
	fillEdgeEvent(tcx, edge, node)
	sweepEdgeEvent(tcx, edge.p, edge.q, node.triangle, edge.q)
}

func fillAdvancingFront(tcx *SweepContext, n *Node) {

	// fill right holes
	var node = n.next

	for node.next != nil {
		var angle = holeAngle(node)
		if angle > M_PI_2 || angle < -M_PI_2 {
			break
		}
		fill(tcx, node)
		node = node.next
	}

	// fill left holes
	node = n.prev

	for node.prev != nil {
		var angle = holeAngle(node)
		if angle > M_PI_2 || angle < -M_PI_2 {
			break
		}
		fill(tcx, node)
		node = node.prev
	}

	// fill right basins
	if n.next != nil && n.next.next != nil {
		var angle = basinAngle(n)
		if angle < PI_3div4 {
			fillBasin(tcx, n)
		}
	}
}

func sweepEdgeEvent(tcx *SweepContext, ep, eq *Point, triangle *Triangle, point *Point) {

	if isEdgeSideOfTriangle(triangle, ep, eq) {
		return
	}

	var p1 *Point = triangle.pointCCW(point)
	var o1 = orient2d(eq, p1, ep)
	if o1 == COLLINEAR {
		if triangle.containsPoints(eq, p1) {
			triangle.markConstrainedEdge3(eq, p1)
			// We are modifying the constraint maybe it would be better to
			// not change the given constraint and just keep a variable for the new constraint
			tcx.edge_event.constrained_edge.q = p1
			triangle = triangle.neighborAcross(point)
			sweepEdgeEvent(tcx, ep, p1, triangle, p1)
		} else {
			panic(fmt.Sprintf("EdgeEvent - collinear points not supported"))
		}
		return
	}

	var p2 = triangle.pointCW(point)
	var o2 = orient2d(eq, p2, ep)
	if o2 == COLLINEAR {
		if triangle.containsPoints(eq, p2) {
			triangle.markConstrainedEdge3(eq, p2)
			// We are modifying the constraint maybe it would be better to
			// not change the given constraint and just keep a variable for the new constraint
			tcx.edge_event.constrained_edge.q = p2
			triangle = triangle.neighborAcross(point)
			sweepEdgeEvent(tcx, ep, p2, triangle, p2)
		} else {
			panic(fmt.Sprintf("EdgeEvent - collinear points not supported"))
		}
		return
	}

	if o1 == o2 {
		// Need to decide if we are rotating CW or CCW to get to a triangle
		// that will cross edge
		if o1 == CW {
			triangle = triangle.neighborCCW(point)
		} else {
			triangle = triangle.neighborCW(point)
		}
		sweepEdgeEvent(tcx, ep, eq, triangle, point)
	} else {
		// This triangle crosses constraint so lets flippin start!
		flipEdgeEvent(tcx, ep, eq, triangle, point)
	}
}

func isEdgeSideOfTriangle(t *Triangle, ep, eq *Point) bool {

	var index = t.edgeIndex(ep, eq)

	if index != -1 {
		t.markConstrainedEdge(index)
		var tn = t.neighbor[index]
		if tn != nil {
			tn.markConstrainedEdge3(ep, eq)
		}
		return true
	}
	return false
}

func fill(tcx *SweepContext, node *Node) {

	t := new(Triangle)
	t.init(node.prev.point, node.point, node.next.point)

	// TODO: should copy the constrained_edge value from neighbor triangles
	//       for now constrained_edge values are copied during the legalize
	t.markNeighbor2(node.prev.triangle)
	t.markNeighbor2(node.triangle)

	t.eref = tcx.tmap.PushBack(t)

	// Update the advancing front
	node.prev.next = node.next
	node.next.prev = node.prev

	// If it was legalized the triangle has already been mapped
	if !legalize(tcx, t) {
		tcx.mapTriangleToNodes(t)
	}

}

func basinAngle(node *Node) float64 {
	var ax = node.point.X - node.next.next.point.X
	var ay = node.point.Y - node.next.next.point.Y
	return math.Atan2(ay, ax)
}

func holeAngle(node *Node) float64 {
	/* Complex plane
	 * ab = cosA +i*sinA
	 * ab = (ax + ay*i)(bx + by*i) = (ax*bx + ay*by) + i(ax*by-ay*bx)
	 * atan2(y,x) computes the principal value of the argument function
	 * applied to the complex number x+iy
	 * Where x = ax*bx + ay*by
	 *       y = ax*by - ay*bx
	 */
	var ax float64 = node.next.point.X - node.point.X
	var ay float64 = node.next.point.Y - node.point.Y
	var bx float64 = node.prev.point.X - node.point.X
	var by float64 = node.prev.point.Y - node.point.Y
	return math.Atan2(ax*by-ay*bx, ax*bx+ay*by)
}

func fillBasin(tcx *SweepContext, node *Node) {

	if orient2d(node.point, node.next.point, node.next.next.point) == CCW {
		tcx.basin.left_node = node.next.next
	} else {
		tcx.basin.left_node = node.next
	}

	// Find the bottom and right node
	tcx.basin.bottom_node = tcx.basin.left_node
	for tcx.basin.bottom_node.next != nil &&
		tcx.basin.bottom_node.point.Y >= tcx.basin.bottom_node.next.point.Y {
		tcx.basin.bottom_node = tcx.basin.bottom_node.next
	}
	if tcx.basin.bottom_node == tcx.basin.left_node {
		// No valid basin
		return
	}

	tcx.basin.right_node = tcx.basin.bottom_node
	for tcx.basin.right_node.next != nil &&
		tcx.basin.right_node.point.Y < tcx.basin.right_node.next.point.Y {
		tcx.basin.right_node = tcx.basin.right_node.next
	}
	if tcx.basin.right_node == tcx.basin.bottom_node {
		// No valid basins
		return
	}

	tcx.basin.width = tcx.basin.right_node.point.X - tcx.basin.left_node.point.X
	tcx.basin.left_highest = tcx.basin.left_node.point.Y > tcx.basin.right_node.point.Y

	fillBasinReq(tcx, tcx.basin.bottom_node)
}

func fillBasinReq(tcx *SweepContext, node *Node) {
	// if shallow stop filling
	if isShallow(tcx, node) {
		return
	}

	fill(tcx, node)

	if node.prev == tcx.basin.left_node && node.next == tcx.basin.right_node {
		return
	} else if node.prev == tcx.basin.left_node {
		var o = orient2d(node.point, node.next.point, node.next.next.point)
		if o == CW {
			return
		}
		node = node.next
	} else if node.next == tcx.basin.right_node {
		var o = orient2d(node.point, node.prev.point, node.prev.prev.point)
		if o == CCW {
			return
		}
		node = node.prev
	} else {
		// Continue with the neighbor node with lowest Y value
		if node.prev.point.Y < node.next.point.Y {
			node = node.prev
		} else {
			node = node.next
		}
	}

	fillBasinReq(tcx, node)
}

func isShallow(tcx *SweepContext, node *Node) bool {

	var height float64

	if tcx.basin.left_highest {
		height = tcx.basin.left_node.point.Y - node.point.Y
	} else {
		height = tcx.basin.right_node.point.Y - node.point.Y
	}

	// if shallow, stop filling
	if tcx.basin.width > height {
		return true
	}
	return false
}

func fillEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	if tcx.edge_event.right {
		fillRightAboveEdgeEvent(tcx, edge, node)
	} else {
		fillLeftAboveEdgeEvent(tcx, edge, node)
	}
}

func fillRightAboveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	for node.next.point.X < edge.p.X {
		// Check if next node is below the edge
		if orient2d(edge.q, node.next.point, edge.p) == CCW {
			fillRightBelowEdgeEvent(tcx, edge, node)
		} else {
			node = node.next
		}
	}
}

func fillRightBelowEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	if node.point.X < edge.p.X {
		if orient2d(node.point, node.next.point, node.next.next.point) == CCW {
			// Concave
			fillRightConcaveEdgeEvent(tcx, edge, node)
		} else {
			// Convex
			fillRightConvexEdgeEvent(tcx, edge, node)
			// Retry this one
			fillRightBelowEdgeEvent(tcx, edge, node)
		}
	}
}

func fillRightConcaveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	fill(tcx, node.next)
	if node.next.point != edge.p {
		// Next above or below edge?
		if orient2d(edge.q, node.next.point, edge.p) == CCW {
			// Below
			if orient2d(node.point, node.next.point, node.next.next.point) == CCW {
				// Next is concave
				fillRightConcaveEdgeEvent(tcx, edge, node)
			} else {
				// Next is convex
			}
		}
	}
}

func fillRightConvexEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	// Next concave or convex?
	if orient2d(node.next.point, node.next.next.point, node.next.next.next.point) == CCW {
		// Concave
		fillRightConcaveEdgeEvent(tcx, edge, node.next)
	} else {
		// Convex
		// Next above or below edge?
		if orient2d(edge.q, node.next.next.point, edge.p) == CCW {
			// Below
			fillRightConvexEdgeEvent(tcx, edge, node.next)
		} else {
			// Above
		}
	}
}

func fillLeftAboveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	for node.prev.point.X > edge.p.X {
		// Check if next node is below the edge
		if orient2d(edge.q, node.prev.point, edge.p) == CW {
			fillLeftBelowEdgeEvent(tcx, edge, node)
		} else {
			node = node.prev
		}
	}
}

func fillLeftBelowEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	if node.point.X > edge.p.X {
		if orient2d(node.point, node.prev.point, node.prev.prev.point) == CW {
			// Concave
			fillLeftConcaveEdgeEvent(tcx, edge, node)
		} else {
			// Convex
			fillLeftConvexEdgeEvent(tcx, edge, node)
			// Retry this one
			fillLeftBelowEdgeEvent(tcx, edge, node)
		}
	}
}

func fillLeftConvexEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	// Next concave or convex?
	if orient2d(node.prev.point, node.prev.prev.point, node.prev.prev.prev.point) == CW {
		// Concave
		fillLeftConcaveEdgeEvent(tcx, edge, node.prev)
	} else {
		// Convex
		// Next above or below edge?
		if orient2d(edge.q, node.prev.prev.point, edge.p) == CW {
			// Below
			fillLeftConvexEdgeEvent(tcx, edge, node.prev)
		} else {
			// Above
		}
	}
}

func fillLeftConcaveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	fill(tcx, node.prev)
	if node.prev.point != edge.p {
		// Next above or below edge?
		if orient2d(edge.q, node.prev.point, edge.p) == CW {
			// Below
			if orient2d(node.point, node.prev.point, node.prev.prev.point) == CW {
				// Next is concave
				fillLeftConcaveEdgeEvent(tcx, edge, node)
			} else {
				// Next is convex
			}
		}
	}
}

func flipEdgeEvent(tcx *SweepContext, ep, eq *Point, t *Triangle, p *Point) {

	var ot = t.neighborAcross(p)
	var op = ot.oppositePoint(t, p)

	if ot == nil {
		// If we want to integrate the fillEdgeEvent do it here
		// With current implementation we should never get here
		panic(fmt.Sprintf("[BUG:FIXME] FLIP failed due to missing triangle"))
	}

	if inScanArea(p, t.pointCCW(p), t.pointCW(p), op) {
		// Lets rotate shared edge one vertex CW
		rotateTrianglePair(t, p, ot, op)
		tcx.mapTriangleToNodes(t)
		tcx.mapTriangleToNodes(ot)

		if p == eq && op == ep {
			if eq == tcx.edge_event.constrained_edge.q && ep == tcx.edge_event.constrained_edge.p {
				t.markConstrainedEdge3(ep, eq)
				ot.markConstrainedEdge3(ep, eq)
				legalize(tcx, t)
				legalize(tcx, ot)
			} else {
				// XXX: I think one of the triangles should be legalized here?
			}
		} else {
			var o = orient2d(eq, op, ep)
			t = nextFlipTriangle(tcx, o, t, ot, p, op)
			flipEdgeEvent(tcx, ep, eq, t, p)
		}
	} else {
		var newP = nextFlipPoint(ep, eq, ot, op)
		flipScanEdgeEvent(tcx, ep, eq, t, ot, newP)
		sweepEdgeEvent(tcx, ep, eq, t, p)
	}
}

func nextFlipTriangle(tcx *SweepContext, o int, t, ot *Triangle, p, op *Point) *Triangle {
	if o == CCW {
		// ot is not crossing edge after flip
		var edge_index = ot.edgeIndex(p, op)
		ot.delaunay_edge[edge_index] = true
		legalize(tcx, ot)
		ot.clearDelunayEdges()
		return t
	}

	// t is not crossing edge after flip
	var edge_index = t.edgeIndex(p, op)

	t.delaunay_edge[edge_index] = true
	legalize(tcx, t)
	t.clearDelunayEdges()
	return ot
}

func nextFlipPoint(ep, eq *Point, ot *Triangle, op *Point) *Point {
	var o2d = orient2d(eq, op, ep)
	if o2d == CW {
		// Right
		return ot.pointCCW(op)
	} else if o2d == CCW {
		// Left
		return ot.pointCW(op)
	} else {
		panic(fmt.Sprintf("[Unsupported] Opposing point on constrained edge"))
	}
	return nil
}

func flipScanEdgeEvent(tcx *SweepContext, ep, eq *Point, flip_triangle, t *Triangle, p *Point) {

	var ot = t.neighborAcross(p)
	var op = ot.oppositePoint(t, p)

	if t.neighborAcross(p) == nil {
		// If we want to integrate the fillEdgeEvent do it here
		// With current implementation we should never get here
		//throw new RuntimeException( "[BUG:FIXME] FLIP failed due to missing triangle");
		panic(fmt.Sprintf("[BUG:FIXME] FLIP failed due to missing triangle"))
	}

	if inScanArea(eq, flip_triangle.pointCCW(eq), flip_triangle.pointCW(eq), op) {
		// flip with new edge op.eq
		flipEdgeEvent(tcx, eq, op, ot, op)
		// TODO: Actually I just figured out that it should be possible to
		//       improve this by getting the next ot and op before the the above
		//       flip and continue the flipScanEdgeEvent here
		// set new ot and op here and loop back to inScanArea test
		// also need to set a new flip_triangle first
		// Turns out at first glance that this is somewhat complicated
		// so it will have to wait.
	} else {
		var newP = nextFlipPoint(ep, eq, ot, op)
		flipScanEdgeEvent(tcx, ep, eq, flip_triangle, ot, newP)
	}
}
