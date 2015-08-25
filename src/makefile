CXX=g++

R: main.o road.o conf.o trajectory.o
	$(CXX) -g -o R road.o conf.o trajectory.o main.o
	mv *.o obj

conf.o:conf.cpp
	$(CXX) -g -c conf.cpp
road.o:road.cpp
	$(CXX) -g -c road.cpp
trajectory.o:trajectory.cpp
	$(CXX) -g -c trajectory.cpp
main.o:main.cpp
	$(CXX) -g -c main.cpp
	
clean:
	rm R
