# $Id: Makefile,v 1.1 2005/12/09 11:31:58 garthz Exp $ 
#
# Makefile for the utility programs for the dynamic simulations and robot controllers.
#
# Copyright (C) 2001-2005 Garth Zeglin.  Provided under the terms of the
# GNU General Public License as included in the top level directory.

BINARIES     = dsinfo dsplot
INCLUDES     = -I..
FLAME_LIBS   = -L../utility -lutility -lm
CFLAGS       = -g -O2
LIBDEPENDS   = ../utility/libutility.a

# The default entry builds all the programs
default: $(BINARIES)

################################################################
# default rules

%.o : %.cpp
	g++ -c $< $(CFLAGS) ${INCLUDES}

% : %.o $(LIBDEPENDS)
	g++ -o $@ $< ${FLAME_LIBS}

% : %.cpp $(LIBDEPENDS)
	g++ -o $@ $< $(CFLAGS) ${INCLUDES} ${FLAME_LIBS}

clean:
	-rm *.o $(BINARIES)

################################################################
# automatic dependencies
depend:
	makedepend -I.. -fMakefile -Y -- *.cpp 

# DO NOT DELETE
