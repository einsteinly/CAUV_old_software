CC=g++
CFLAGS=-I. -Wall
LFLAGS=-lrt
DEPS = cmt1.h 
OBJSTATIC = cmt1.o cmt2.o cmt3.o cmtmessage.o cmtpacket.o cmtscan.o xsens_std.o xsens_time.o

all: static

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $<

static: $(OBJSTATIC)
	ar rcs libcmt.a $^

clean:
	rm -f *.o
	rm -f libcmt.a
	rm -f libcmt.so
