build:
	clear
	g++ ./listener.cpp ./speaker.cpp -c -I../ 
	g++ ../*.cpp listener.o -o ./listener.exe
	g++ ../*.cpp speaker.o -o ./speaker.exe
 clean:
	rm ./*.o
	rm ./*.exe
