#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>
#include <vector>

int main()
{
	FILE* thefile = fopen("log.txt", "w");
	char buff[4];
	setvbuf(stdin, buff, _IOFBF, 0);
	for (int i = 0; i < 5; i++)
	{
		sleep(1);
		std::cout << std::string(buff) << std::endl;
	}
	fclose(thefile);
	
}
