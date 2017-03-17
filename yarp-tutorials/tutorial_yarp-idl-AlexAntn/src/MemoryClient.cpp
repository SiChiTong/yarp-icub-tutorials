#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <Memory.h>

using namespace yarp::os;

int main(int argc, char ** argv)
{
    Network yarp;

    // create and open a rpc port
    RpcClient port;
    port.open("/MemoryModule/client");
    if(!Network::connect(port.getName(), "/MemoryModule/rpc")) {
        yError()<<"Cannot connect to /MemoryModule/rpc";
        return 0;
    }

    // create an instance of Memory class and attach to the port
    Memory client;
    client.yarp().attachAsClient(port);

    //use functions defined in Memory.h
    client.push("hello", "ciao");
    std::string answer = client.get_answer("hello");
    yInfo()<<"get_answer 'hello' returns"<<answer;
    return 0;
}
