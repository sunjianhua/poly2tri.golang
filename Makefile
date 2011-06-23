# Copyright 2009 The Go Authors. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

include $(GOROOT)/src/Make.inc

.PHONY: all p2t testbed clean

all: install testbed

p2t:
	gomake -C poly2tri

install: p2t
	gomake -C poly2tri install

testbed:
	gomake -C testbed

clean:
	gomake -C poly2tri clean
	gomake -C testbed clean
