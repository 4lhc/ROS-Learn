CFLAGS = -Wall -Wextra -pedantic -g -std=c++14 -c
CXX = g++
SOURCES = Graph.cpp $(FNAME)
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = test

all: $(SOURCES) $(EXECUTABLE) testcase

$(EXECUTABLE): $(OBJECTS)
	$(CXX) -o $@ $^

$(OBJECTS): $(SOURCES)
	$(CXX) $(CFLAGS) $^


clean:
	rm -rf *.o $(EXECUTABLE)

testcase: ${INPUTFILE}
	./${EXECUTABLE} < $<

