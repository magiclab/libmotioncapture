#include "CDPParser/CDPClient.h"

class Globals{
public:
    static bool run;
};

bool Globals::run = false;

void terminate(int){
    Globals::run = false;
}

void update(CDPClient & c){
    Globals::run = true;
    while(Globals::run){
        c.update();
        if(c.isNewFrameReady()){
            std::cout<<c.getLastFrame()<<std::endl;
        }
    }
}

int main(int argc, char * argv[]){
    signal(SIGINT, terminate);
    
    CDPClient client;
    std::vector<uint32_t> ser;
    ser.push_back(16778578);
    ser.push_back(16778717);
    client.setSerials(ser);
    client.connect();
    update(client);
    
    return 0;
}