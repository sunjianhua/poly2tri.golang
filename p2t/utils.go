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
)

const M_PI_2 = math.Pi / 2
const PI_3div4 = 3 * math.Pi / 4
const EPSILON = 1e-12

const (
	CW = iota
	CCW
	COLLINEAR
)

/**
 * Forumla to calculate signed area<br>
 * Positive if CCW<br>
 * Negative if CW<br>
 * 0 if collinear<br>
 * <pre>
 * A[P1,P2,P3]  =  (x1*y2 - y1*x2) + (x2*y3 - y2*x3) + (x3*y1 - y3*x1)
 *              =  (x1-x3)*(y2-y3) - (y1-y3)*(x2-x3)
 * </pre>
 */
func orient2d(pa, pb, pc *Point) int {
	var detleft = (pa.X - pc.X) * (pb.Y - pc.Y)
	var detright = (pa.Y - pc.Y) * (pb.X - pc.X)
	var val = detleft - detright
	if val > -EPSILON && val < EPSILON {
		return COLLINEAR
	} else if val > 0 {
		return CCW
	}
	return CW
}

func inScanArea(pa, pb, pc, pd *Point) bool {
	var pdx = pd.X
	var pdy = pd.Y
	var adx = pa.X - pdx
	var ady = pa.Y - pdy
	var bdx = pb.X - pdx
	var bdy = pb.Y - pdy

	var adxbdy = adx * bdy
	var bdxady = bdx * ady
	var oabd = adxbdy - bdxady

	if oabd <= EPSILON {
		return false
	}

	var cdx = pc.X - pdx
	var cdy = pc.Y - pdy

	var cdxady = cdx * ady
	var adxcdy = adx * cdy
	var ocad = cdxady - adxcdy

	if ocad <= EPSILON {
		return false
	}

	return true
}
