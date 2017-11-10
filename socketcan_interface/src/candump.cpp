#include <iostream>
#include <boost/unordered_set.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/make_shared.hpp>
#include <class_loader/class_loader.h>
#include <socketcan_interface/socketcan.h>

using namespace can;

#include <iostream>

void print_error(const State & s);

void print_frame(const Frame &f){
    
    if(f.is_error){
        std::cout << "E " << std::hex << f.id << std::dec;
    }else if(f.is_extended){
        std::cout << "e " << std::hex << f.id << std::dec;
    }else{
        std::cout << "s " << std::hex << f.id << std::dec;
    }
    
    std::cout << "\t";
    
    if(f.is_rtr){
        std::cout << "r";
    }else{
        std::cout << (int) f.dlc << std::hex;
        
        for(int i=0; i < f.dlc; ++i){
            std::cout << std::hex << " " << (int) f.data[i];
        }
    }
    
    std::cout << std::dec << std::endl;
}


boost::shared_ptr<class_loader::ClassLoader> g_loader;
boost::shared_ptr<DriverInterface> g_driver;

void print_error(const State & s){
    std::string err;
    g_driver->translateError(s.internal_error,err);
    std::cout << "ERROR: state=" << s.driver_state << " internal_error=" << s.internal_error << "('" << err << "') asio: " << s.error_code << std::endl;
}


int main(int argc, char *argv[]){
    
    if(argc != 2 && argc != 4){
        std::cout << "usage: "<< argv[0] << " DEVICE [PLUGIN_PATH PLUGIN_NAME]" << std::endl;
        return 1;
    }

    if(argc == 4 ){
        try
        {
            g_loader = boost::make_shared<class_loader::ClassLoader>(argv[2]);
            g_driver = g_loader->createInstance<DriverInterface>(argv[3]);
        }
        
        catch(std::exception& ex)
        {
            std::cerr << boost::diagnostic_information(ex) << std::endl;;
            return 1;
        }
    }else{
        g_driver = boost::make_shared<SocketCANInterface>();
    }
    


    CommInterface::FrameListener::Ptr frame_printer = g_driver->createMsgListener(print_frame);
    StateInterface::StateListener::Ptr error_printer = g_driver->createStateListener(print_error);
    
    if(!g_driver->init(argv[1], false)){
        print_error(g_driver->getState());
        return 1;
    }

    g_driver->run();
    
    g_driver->shutdown();
    g_driver.reset();
    g_loader.reset();
    
    return 0;
    
}