#include <iostream>
#include <string>

#include "CameraGrabberBasler.h"

using namespace std;


///
/// ImageGrabController: a main program for grabbing image using Basler camera
///
///

int main()
{
    std::cout << "********** Basler Camera Grabbing Controller******** " << std::endl;


    try{

        // params
        // currently I don't have any cam config file
        // but if you do, you could just load your camera configfile
        //参数,目前我没有任何cam配置文件,但如果你这样做了，你可以只加载你的摄像头配置文件

        std::string camConfig = "";
        // whether display image grabbed
        bool showView = true;

        // image saving directory , you could change to any directory you want
        //图像保存目录，您可以更改到任何您想要的目录
        std::string saveImgDir = "/home/jinln/jinln/c++test/BaslerCameraHardwareTrigger-master/flow/";

        // prefix of image file 图像文件的前缀
        std::string imgPrefix = "image";
        // post fix of image file 图像文件的后缀
        std::string imgPostfix = ".png";

        CameraGrabberBasler camGrab(camConfig , showView);
        camGrab.SetSaveImgParams(saveImgDir , imgPrefix , imgPostfix);
        camGrab.Init();
        std::string status = "";
        camGrab.PrintStatus(status);

        std::cout << "cam status \n" << status << std::endl;

        // start grabbing
        camGrab.StartGrab();

        std::string command = "";
        std::cout << "Press [Enter] to quit grabbing" << std::endl;

        std::cin.get();

        camGrab.StopGrab();





    }catch(exception & e){
        std::cerr << "\n\n---\n\n EXCEPTION OCCURED! :\n " << e.what() << std::endl;
        std::cerr << "\n\n Press [Enter] to exit program " << std::endl;
        std::cin.get();
        return -1;

    }


    std::cout << "Press [Enter] to exit" << std::endl;
    std::cin.get();
    return 0;
}
