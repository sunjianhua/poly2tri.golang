package p2t

import (
	"fmt"
)

var tcx *SweepContext

func Init(polyline PointArray) {
	tcx = new(SweepContext)
	tcx.init(polyline)
}

// Returns the contstrained triangles
func Triangulate() TriArray {
	if tcx != nil {
		triangulate(tcx)
	} else {
		panic(fmt.Sprintf("ERROR: p2t uninitialized"))
	}
	return tcx.triangles
}

func AddHole(polyline PointArray) {
	if tcx != nil {
		tcx.addHole(polyline)
	} else {
		panic(fmt.Sprintf("ERROR: p2t uninitialized"))
	}
}

func AddPoint(p *Point) {
	if tcx != nil {
		tcx.addPoint(p)
	} else {
		panic(fmt.Sprintf("ERROR: p2t uninitialized"))
	}
}

// Returns the entire triangle mesh for debugging purposes
func Mesh() TriArray {
	if tcx != nil {
		// Convert from Vector to slice for convenience
		n := tcx.tmap.Len()
		var triangles = make(TriArray, n)
		for e, i := tcx.tmap.Front(), 0; e != nil; e, i = e.Next(), i+1 {
			triangles[i] = e.Value.(*Triangle)
		}
		return triangles
	}
	panic(fmt.Sprintf("ERROR: p2t uninitialized"))
}
