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

package poly2tri

import (
	"fmt"
	"math"
	"container/vector"
)
	
type Point struct {
	x, y float64
	edge_list vector.Vector
}

type Edge struct { p, q *Point }

func (p *Point) Add(q *Point) {
	p.x += q.x
	p.y += q.y
}

func (p *Point) Sub(q *Point) {
	p.x -= q.x
	p.y -= q.y
}

func (p *Point) Mul(a float64) {
	p.x *= a
	p.y *= a
}

func (p *Point) Neg() {
	p.x = -p.x
	p.y = -p.y
}

func (p *Point) Length() float64 {
    return math.Sqrt(p.x * p.x + p.y * p.y)
}

// Convert this point into a unit point. Returns the Length.
func (p *Point) Normalize() float64 {
    var len float64 = p.Length()
    p.x /= len;
    p.y /= len;
    return len;
}

func (e *Edge) Init(p1, p2 *Point) {
	if p1.y > p2.y {
		e.q = p1
		e.p = p2
    } else if p1.y == p2.y {
		if p1.x > p2.x {
			e.q = p1;
			e.p = p2;
		} else if p1.x == p2.x {
			panic(fmt.Sprintf("repeat points"))
		}
	}
	e.q.edge_list.Push(e)
}

/*


// Triangle-based data structures are know to have better performance than quad-edge structures
// See: J. Shewchuk, "Triangle: Engineering a 2D Quality Mesh Generator and Delaunay Triangulator"
//      "Triangulations in CGAL"
class Triangle {
public:

/// Constructor
Triangle(Point& a, Point& b, Point& c);

/// Flags to determine if an edge is a Constrained edge
bool constrained_edge[3];
/// Flags to determine if an edge is a Delauney edge
bool delaunay_edge[3];

Point* GetPoint(const int& index);
Point* PointCW(Point& point);
Point* PointCCW(Point& point);
Point* OppositePoint(Triangle& t, Point& p);

Triangle* GetNeighbor(const int& index);
void MarkNeighbor(Point* p1, Point* p2, Triangle* t);
void MarkNeighbor(Triangle& t);

void MarkConstrainedEdge(const int index);
void MarkConstrainedEdge(Edge& edge);
void MarkConstrainedEdge(Point* p, Point* q);

int Index(const Point* p);
int EdgeIndex(const Point* p1, const Point* p2);

Triangle* NeighborCW(Point& point);
Triangle* NeighborCCW(Point& point);
bool GetConstrainedEdgeCCW(Point& p);
bool GetConstrainedEdgeCW(Point& p);
void SetConstrainedEdgeCCW(Point& p, bool ce);
void SetConstrainedEdgeCW(Point& p, bool ce);
bool GetDelunayEdgeCCW(Point& p);
bool GetDelunayEdgeCW(Point& p);
void SetDelunayEdgeCCW(Point& p, bool e);
void SetDelunayEdgeCW(Point& p, bool e);

bool Contains(Point* p);
bool Contains(const Edge& e);
bool Contains(Point* p, Point* q);
void Legalize(Point& point);
void Legalize(Point& opoint, Point& npoint);

// Clears all references to all other triangles and points

void Clear();
void ClearNeighbor(Triangle *triangle );
void ClearNeighbors();
void ClearDelunayEdges();

inline bool IsInterior();
inline void IsInterior(bool b);

Triangle& NeighborAcross(Point& opoint);

void DebugPrint();

private:

/// Triangle points
Point* points_[3];
/// Neighbor list
Triangle* neighbors_[3];

/// Has this triangle been marked as an interior triangle?
bool interior_;
};

inline bool cmp(const Point* a, const Point* b)
{
  if (a->y < b->y) {
    return true;
  } else if (a->y == b->y) {
    // Make sure q is point with greater x value
    if (a->x < b->x) {
      return true;
    }
  }
  return false;
}

/// Add two points_ component-wise.
inline Point operator +(const Point& a, const Point& b)
{
  return Point(a.x + b.x, a.y + b.y);
}

/// Subtract two points_ component-wise.
inline Point operator -(const Point& a, const Point& b)
{
  return Point(a.x - b.x, a.y - b.y);
}

/// Multiply point by scalar
inline Point operator *(double s, const Point& a)
{
  return Point(s * a.x, s * a.y);
}

inline bool operator ==(const Point& a, const Point& b)
{
  return a.x == b.x && a.y == b.y;
}

inline bool operator !=(const Point& a, const Point& b)
{
  return a.x != b.x && a.y != b.y;
}

/// Peform the dot product on two vectors.
inline double Dot(const Point& a, const Point& b)
{
  return a.x * b.x + a.y * b.y;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline double Cross(const Point& a, const Point& b)
{
  return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a point and a scalar. In 2D this produces
/// a point.
inline Point Cross(const Point& a, double s)
{
  return Point(s * a.y, -s * a.x);
}

/// Perform the cross product on a scalar and a point. In 2D this produces
/// a point.
inline Point Cross(const double s, const Point& a)
{
  return Point(-s * a.y, s * a.x);
}

inline Point* Triangle::GetPoint(const int& index)
{
  return points_[index];
}

inline Triangle* Triangle::GetNeighbor(const int& index)
{
  return neighbors_[index];
}

inline bool Triangle::Contains(Point* p)
{
  return p == points_[0] || p == points_[1] || p == points_[2];
}

inline bool Triangle::Contains(const Edge& e)
{
  return Contains(e.p) && Contains(e.q);
}

inline bool Triangle::Contains(Point* p, Point* q)
{
  return Contains(p) && Contains(q);
}

inline bool Triangle::IsInterior()
{
  return interior_;
}

inline void Triangle::IsInterior(bool b)
{
  interior_ = b;
}

}

#endif


*/
