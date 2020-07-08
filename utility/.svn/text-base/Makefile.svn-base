LIBOBJS = errprint.o delay.o dataset.o system_state_var.o record.o choose_filename.o kbhit.o

ALL = libutility.a
CFLAGS = -g3 -O2 -I..

default: $(ALL)

libutility.a: $(LIBOBJS)
	ar cru $@ $^

%.o : %.cpp; g++ $(CFLAGS)  -c -o $@ $<

clean:
	-rm $(ALL) *.o

dist-clean: clean
