#include <socketcan_interface/bcm.h>
#include <socketcan_interface/string.h>

using namespace can;

#include <iostream>

int main(int argc, char *argv[]){
    BCMsocket bcm;

    int extra_frames = argc - 4; 

    if(extra_frames < 0){
        std::cout << "usage: "<< argv[0] << " DEVICE PERIOD HEADER#DATA [DATA*]" << std::endl;
        return 1;
    }

    if(!bcm.init(argv[1])){
        return 2;
    }

    int num_frames = extra_frames+1;
    Frame *frames = new Frame[num_frames];
    Header header = frames[0] = toframe(argv[3]);

    if(extra_frames > 0){
        for(int i=0; i< extra_frames; ++i){
            frames[1+i] = toframe(tostring(header,true)+"#"+argv[4+i]);
        }
    }
    for(int i = 0; i < num_frames; ++i){
        std::cout << frames[i] << std::endl;
    }
    if(bcm.startTX(boost::chrono::duration<double>(atof(argv[2])), header, num_frames, frames)){
        pause();
        return 0;
    }
    return 4;
}