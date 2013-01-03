CC= g++
INCLUDES= -I/usr/local/Aria/include -I/usr/local/Aria/ArNetworking/include
CPPFLAGS= -Wall -g $(INCLUDES)
LDLIBS= -lpthread -L/usr/local/Aria/lib -lAria -lArNetworking -lrt -ldl

SRC= server.cc
PROG= $(SRC:.cc=)

all: $(PROG)

$(PROG): $(SRC)
	$(CC) $(CPPFLAGS) $^ $(LDLIBS) -o $@

clean:
	rm -f *.o a.out $(PROG)
