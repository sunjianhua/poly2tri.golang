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
