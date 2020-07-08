# Makefile : for the xsens support library
#
# Copyright (c) 2006 Garth Zeglin. Provided under the terms of the
# GNU General Public License as included in the top level directory.

default: libxsens.a MTCommExample test_xsens 


# some sample programs
MTCommExample: MTCommExample.o MTComm.o
test_xsens: test_xsens.o xsens_glue.o MTComm.o

################################################################
## C compilation rule for the C test program
#test_xsens.o: test_xsens.c
#	$(CC) -g -c -o $@ $^

################################################################
# entry to build the small library
libxsens.a: MTComm.o xsens_glue.o
	ar cru $@ $^

################################################################

%.o: %.cpp ; g++ -g3 -O2 -Wall -c $<

%: %.o ; g++ -o $@ $^

################################################################

clean:
	-rm *.o MTCommExample test_xsens libxsens.a

dist-clean: clean

################################################################
