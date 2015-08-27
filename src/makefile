CXX=g++

R: main.o road.o conf.o trajectory.o quadtree.o
	$(CXX) -g -o R road.o conf.o trajectory.o quadtree.o main.o
	mv *.o obj

conf.o:conf.cpp
	$(CXX) -g -c conf.cpp
road.o:road.cpp
	$(CXX) -g -c road.cpp
trajectory.o:trajectory.cpp
	$(CXX) -g -c trajectory.cpp
quadtree.o:quadtree.cpp
	$(CXX) -g -c quadtree.cpp
main.o:main.cpp
	$(CXX) -g -c main.cpp
	
clean:
	rm R
