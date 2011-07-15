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
 *   af list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   af list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from af software without specific
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

import "fmt"

// Advancing front node
type Node struct {
	point    *Point
	triangle *Triangle

	next *Node
	prev *Node

	value float64
}

// Advancing front
type AdvancingFront struct {
	head, tail, search_node *Node
}

func (af *AdvancingFront) init(head, tail *Node) {
	af.head = head
	af.tail = tail
	af.search_node = head
}

func (af *AdvancingFront) locateNode(x float64) *Node {

	var node = af.search_node

	if x < node.value {
		for node = node.prev; node != nil; node = node.prev {
			if x >= node.value {
				af.search_node = node
				return node
			}
		}
	} else {
		for node = node.next; node != nil; node = node.next {
			if x < node.value {
				af.search_node = node.prev
				return node.prev
			}
		}
	}
	return nil
}

func (af *AdvancingFront) locatePoint(point *Point) *Node {

	var px = point.X
	var node = af.search_node
	var nx = node.point.X

	if px == nx {
		if point != node.point {
			// We might have two nodes with same x value for a short time
			if point == node.prev.point {
				node = node.prev
			} else if point == node.next.point {
				node = node.next
			} else {
				panic(fmt.Sprintf("invalid point"))
			}
		}
	} else if px < nx {
		for node = node.prev; node != nil; node = node.prev {
			if point == node.point {
				break
			}
		}
	} else {
		for node = node.next; node != nil; node = node.next {
			if point == node.point {
				break
			}
		}
	}
	if node != nil {
		af.search_node = node
	}
	return node
}
