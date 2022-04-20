#include "include/camthread.h"
#include <thread>
#include <iostream>
#include <chrono>
#include <ctime>

using namespace std;
int main()
{
    std::thread thrd_0(cam0);
    std::thread thrd_1(cam1);
    std::thread thrd_2(cam2);

    std::thread savethrd_0(save0);
    std::thread savethrd_1(save1);
    std::thread savethrd_2(save2);

    //std::thread finishgrabth(finishgrab);

    thrd_0.join();
    thrd_1.join();
    thrd_2.join();
	
    savethrd_0.join();
    savethrd_1.join();
    savethrd_2.join();

    //finishgrabth.join();

	std::cout << "finish ..." << std::endl;
	return 0;
}
