include $(GOROOT)/src/Make.inc

TARGETS=main

CGO_LDFLAGS:=-lglu32 -lopengl32 -lglew32

.PHONY: all clean

all: $(TARGETS)

$(TARGETS):
	$(GC) $@.go 
	$(LD) -o $@ $@.$(O) 

clean:
	rm -f -r *.8 *.6 *.o */*.8 */*.6 */*.o */_obj main $(TARGETS)
