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
	"container/vector"
	"container/list"
)

// TODO: make everything in here private

// Inital t factor, seed t will extend 30% of
// PointSet width to both left and right.
const kAlpha = 0.3

type SweepContext struct {
	basin      *Basin
	edge_event *EdgeEvent
	edge_list  *vector.Vector

	triangles TriArray
	points    PointArray
	tmap      *list.List

	// Advancing front
	front *AdvancingFront
	// head and tail point used with advancing front
	head, tail *Point

	af_head, af_middle, af_tail *Node
}

type Basin struct {
	left_node, bottom_node, right_node *Node
	width                              float64
	left_highest                       bool
}

type EdgeEvent struct {
	constrained_edge *Edge
	right            bool
}

func (b *Basin) Clear() {
	b.left_node = nil
	b.bottom_node = nil
	b.right_node = nil
	b.width = 0.0
	b.left_highest = false
}


func (s *SweepContext) init(polyline []*Point) {
	s.triangles = make(TriArray, 0, 100)
	s.edge_list = new(vector.Vector)
	s.tmap = list.New()
	s.basin = new(Basin)
	s.edge_event = new(EdgeEvent)
	s.points = polyline
	s.initEdges(s.points)
}

func (s *SweepContext) initTriangulation() {

	var xmax = s.points[0].X
	var xmin = s.points[0].X
	var ymax = s.points[0].Y
	var ymin = s.points[0].Y

	// Calculate bounds.
	for i := 0; i < len(s.points); i++ {
		var p = s.points[i]
		if p.X > xmax {
			xmax = p.X
		}
		if p.X < xmin {
			xmin = p.X
		}
		if p.Y > ymax {
			ymax = p.X
		}
		if p.Y < ymin {
			ymin = p.Y
		}
	}

	var dx = kAlpha * (xmax - xmin)
	var dy = kAlpha * (ymax - ymin)
	s.head = &Point{X: xmax + dx, Y: ymin - dy}
	s.tail = &Point{X: xmin - dx, Y: ymin - dy}

	// Sort points along y-axis
	var pa PointArray = s.points
	pa.Sort()
	//sort.Sort(s.points)
}

func (s *SweepContext) initEdges(polyline []*Point) {
	var num_points = len(polyline)
	for i := 0; i < num_points; i++ {
		var j int
		if i < num_points-1 {
			j = i + 1
		} else {
			j = 0
		}
		var p1 = polyline[i]
		var p2 = polyline[j]
		var e = new(Edge)
		e.init(p1, p2)
		s.edge_list.Push(e)
	}
}


func (s *SweepContext) addHole(polyline []*Point) {
	s.initEdges(polyline)
	n := len(polyline)
	n2 := len(s.points)
	s.points = s.points[0 : n2+n]
	for i := 0; i < n; i++ {
		s.points[n2+i+1] = polyline[i]
	}
}

func (s *SweepContext) addPoint(point *Point) {
	n := len(s.points)
	s.points = s.points[0 : n+1]
	s.points[n+1] = point
}

func (s *SweepContext) locateNode(point *Point) *Node {
	// TODO implement search tree
	return s.front.locateNode(point.X)
}

// TODO: why pass nodes into this function?
func (s *SweepContext) createAdvancingFront() {

	// Initial t
	t := new(Triangle)
	t.init(s.points[0], s.tail, s.head)

	t.eref = s.tmap.PushBack(t)

	s.af_head = &Node{point: t.Point[1], triangle: t, value: t.Point[1].X}
	s.af_middle = &Node{point: t.Point[0], triangle: t, value: t.Point[0].X}
	s.af_tail = &Node{point: t.Point[2], value: t.Point[2].X}
	s.front = new(AdvancingFront)
	s.front.init(s.af_head, s.af_tail)

	// TODO: More intuitive if head is middles next and not previous?
	//       so swap head and tail
	s.af_head.next = s.af_middle
	s.af_middle.next = s.af_tail
	s.af_middle.prev = s.af_head
	s.af_tail.prev = s.af_middle
}

func (s *SweepContext) RemoveNode(node *Node) {
	node = nil
}

func (s *SweepContext) mapTriangleToNodes(t *Triangle) {
	for i := 0; i < 3; i++ {
		if t.neighbor[i] == nil {
			n := s.front.locatePoint(t.pointCW(t.Point[i]))
			if n != nil {
				n.triangle = t
			}
		}
	}
}

func (s *SweepContext) RemoveFromMap(t *Triangle) {
	s.tmap.Remove(t.eref)
}

func (s *SweepContext) meshClean(t *Triangle) {
	if t != nil && !t.interior {
		t.interior = true
		var n = len(s.triangles)
		if n < cap(s.triangles) {
			s.triangles = s.triangles[0 : n+1]
		} else {
			// Resize the array and double it
			tmp := make(TriArray, n*2)
			copy(tmp, s.triangles)
			s.triangles = tmp
		}
		s.triangles[n] = t
		for i := 0; i < 3; i++ {
			if !t.constrained_edge[i] {
				s.meshClean(t.neighbor[i])
			}
		}
	}
}
