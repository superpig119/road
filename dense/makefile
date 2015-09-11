CXX=g++

R: main.o road.o conf.o 
	$(CXX) -g -o R road.o conf.o main.o
	mv *.o obj

conf.o:conf.cpp
	$(CXX) -g -c conf.cpp
road.o:road.cpp
	$(CXX) -g -c road.cpp
main.o:main.cpp
	$(CXX) -g -c main.cpp
	
clean:
	rm R
