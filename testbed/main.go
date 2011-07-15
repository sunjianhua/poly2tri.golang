package main

import (
	"fmt"
	"p2t"
)

func main() {

	var polyline = make(p2t.PointArray, 4)

	polyline[0] = &p2t.Point{X: 5.0, Y: 5.0}
	polyline[1] = &p2t.Point{X: 5.0, Y: -5.0}
	polyline[2] = &p2t.Point{X: -5.0, Y: -5.0}
	polyline[3] = &p2t.Point{X: -5.0, Y: 5.0}

	p2t.Init(polyline)

	var triangles p2t.TriArray = p2t.Triangulate()

	fmt.Println("*** triangles ***")
	for i := 0; i < len(triangles); i++ {
		var a = triangles[i].Point[0]
		var b = triangles[i].Point[1]
		var c = triangles[i].Point[2]
		fmt.Println(a.X, a.Y, ",", b.X, b.Y, ",", c.X, c.Y)
	}

	var mesh p2t.TriArray = p2t.Mesh()

	fmt.Println("*** mesh ***")
	for i := 0; i < len(mesh); i++ {
		var a = mesh[i].Point[0]
		var b = mesh[i].Point[1]
		var c = mesh[i].Point[2]
		fmt.Println(a.X, a.Y, ",", b.X, b.Y, ",", c.X, c.Y)
	}

	fmt.Println("success")

}
