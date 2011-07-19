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
	"os"
	"flag"
	"p2t"
	"gl"
	"gl/glu"
	"github.com/jteeuwen/glfw"
)

const (
	Title  = "Poly2Tri Demo"
	Width  = 640
	Height = 480
)

var running bool
var filename *string
var cx, cy, zoom *int

var left, right, bottom, top float64
  
func main() {

	filename = flag.String("file", "testbed/data/dude.dat", "enter filename path")
	cx = flag.Int("cx", 0, "enter x-coordinate center")
	cy = flag.Int("cy", 0, "enter y-coordinate center")
	zoom = flag.Int("zoom", 25, "enter zoom")
		
	flag.Parse()
	
	fmt.Println("opening...", *filename)
	f, err := os.Open(*filename)
	if f == nil {
		fmt.Fprintf(os.Stderr, "cat: can't open %s: error %s\n", *filename, err)
		os.Exit(1)
	}

	f.Close()

	left = -Width / float64(*zoom)
	right = Width / float64(*zoom)
	bottom = -Height / float64(*zoom)
	top = Height / float64(*zoom)
  
	
	fmt.Println("cx has value ", *cx)
	fmt.Println("cy has value ", *cy)
	
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
	
	//var err os.Error
	if err = glfw.Init(); err != nil {
		fmt.Fprintf(os.Stderr, "[e] %v\n", err)
		return
	}

	defer glfw.Terminate()

	if err = glfw.OpenWindow(Width, Height, 8, 8, 8, 8, 0, 8, glfw.Windowed); err != nil {
		fmt.Fprintf(os.Stderr, "[e] %v\n", err)
		return
	}

	defer glfw.CloseWindow()

	glfw.SetSwapInterval(1)
	glfw.SetWindowTitle(Title)
	glfw.SetWindowSizeCallback(onResize)
	glfw.SetKeyCallback(onKey)

	initGL()

	running = true
	for running && glfw.WindowParam(glfw.Opened) == 1 {
		drawScene(triangles)
	}

}

func onResize(w, h int) {
	if h == 0 {
		h = 1
	}

	gl.Viewport(0, 0, w, h)
	gl.MatrixMode(gl.PROJECTION)
	gl.LoadIdentity()
	glu.Perspective(45.0, float64(w)/float64(h), 0.1, 100.0)
	gl.MatrixMode(gl.MODELVIEW)
	gl.LoadIdentity()
}

func resetZoom() {
	
  // Reset viewport
  gl.LoadIdentity();
  gl.MatrixMode(gl.PROJECTION);
  gl.LoadIdentity();

  // Reset ortho view
  gl.Ortho(left, right, bottom, top, 1, -1)
  gl.Translatef(float32(-*cx), float32(-*cy), 0)
  gl.MatrixMode(gl.MODELVIEW)
  gl.Disable(gl.DEPTH_TEST)
  gl.LoadIdentity()

  // Clear the screen
  gl.Clear(gl.COLOR_BUFFER_BIT)
}

func onKey(key, state int) {
	switch key {
	case glfw.KeyEsc:
		running = false
	}
}

func initGL() {
	gl.ShadeModel(gl.SMOOTH)
	gl.ClearColor(0, 0, 0, 0)
	gl.ClearDepth(1)
	gl.Enable(gl.DEPTH_TEST)
	gl.DepthFunc(gl.LEQUAL)
	gl.Hint(gl.PERSPECTIVE_CORRECTION_HINT, gl.NICEST)
}

func drawScene(triangles p2t.TriArray) {
	
	resetZoom()

	for i := 0; i < len(triangles); i++ {
		var t = triangles[i]
		var a = t.Point[0]
		var b = t.Point[1]
		var c = t.Point[2]

		// Red
		gl.Color3f(1, 0, 0);

		gl.Begin(gl.LINE_LOOP)
		gl.Vertex2f(float32(a.X), float32(a.Y))
		gl.Vertex2f(float32(b.X), float32(b.Y))
		gl.Vertex2f(float32(c.X), float32(c.Y))
		gl.End()
	}

	glfw.SwapBuffers()
}

func drawMap(cx, cy, zoom float64, triangles p2t.TriArray) {
	
  resetZoom()

  for i :=  0; i < len(triangles); i++ {
    t := triangles[i]
    a := t.Point[0]
    b := t.Point[1]
    c := t.Point[2]

    //constrainedColor(t.constrained_edge[2])
    gl.Begin(gl.LINES)
    gl.Vertex2f(float32(a.X), float32(a.Y))
    gl.Vertex2f(float32(b.X), float32(b.Y))
    gl.End( )

    //constrainedColor(t.constrained_edge[0])
    gl.Begin(gl.LINES)
    gl.Vertex2f(float32(b.X), float32(b.Y))
    gl.Vertex2f(float32(c.X), float32(c.Y))
    gl.End( )

    //constrainedColor(t.constrained_edge[1])
    gl.Begin(gl.LINES)
    gl.Vertex2f(float32(c.X), float32(c.Y))
    gl.Vertex2f(float32(a.X), float32(a.Y))
    gl.End( )
  }
}

func constrainedColor(constrain bool) {
  if (constrain) {
    // Green
    gl.Color3f(0, 1, 0)
  } else {
    // Red
    gl.Color3f(1, 0, 0)
  }
}
