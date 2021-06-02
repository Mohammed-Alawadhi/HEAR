#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "HEAR_core/Graph.hpp"
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/ExternalTrigger.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <atomic>

/**
 * We can use graph concept for the blocks connectivity 
 * 
 **/

namespace HEAR{

class System{
public:
    System(const int frequency) : _dt(1.f/frequency), _exit_flag(false) {}
    ~System();
    void printSystem();
    int init();
    void execute();
    void runPubLoop();
    void mainLoop();
    void pubLoop();
    void terminate() { _exit_flag = true;}
    int addBlock(Block* blk, std::string name);
    void addPub(Block* ros_pub);
    int addExternalTrigger(ExternalTrigger* ext_tirg, std::string name);
    template <class T> int createExternalOutputPort(TYPE dtype, std::string port_name);
    template <class T> int createExternalInputPort(TYPE dtype, std::string port_name);
    template <class T> ExternalOutputPort<T>* getExternalOutputPort(int ext_op_idx);
    template <class T> ExternalInputPort<T>* getExternalInputPort(int ext_ip_idx);
    template <class T> void connectToExternalInput(int ext_ip_idx, int dest_block_uid, int ip_idx);
    template <class T> void connectToExternalInput(int ext_ip_idx, ExternalOutputPort<T>* ext_port);
    template <class T> void connectToExternalOutput(ExternalInputPort<T>* ext_port, int ext_op_idx);
    template <class T> void connectToExternalOutput(int src_block_uid, int op_idx, int ext_op_idx);
    template <class T> void connect(int out_block_uid, int op, int in_block_uid, int ip);
    void connectExtTrig(int block_uid, int ext_trig_idx);
private:
    int num_blocks = 0;
    int num_ext_trigs = 0;
    float _dt;
    std::atomic<bool> _exit_flag;
    std::vector<Edge> _edges;
    Graph _graph;
    std::vector<ExternalTrigger*> _external_triggers;
    std::vector<std::string> _trig_names;
    std::vector<Block*> _blocks;
    std::vector<Block*> _ros_pubs;
    std::vector<std::string> _block_names;
    std::vector<Block*> seq;
    std::unique_ptr<std::thread> system_thread, pub_thread;
    static bool sortbyconnectivity(const Block* a, const Block* b);
    void findsequence();

};

System::~System(){
    terminate();
    if(system_thread){
        system_thread->join();
    }
    if(pub_thread){
        pub_thread->join();
    }
    for (auto &block : _blocks){
        delete block;
    }
}
int System::init(){
    // do some checks for errors in connectivity etc
    this->findsequence();
    _graph = Graph(_edges, _blocks.size());
    return true;
}

template <class T>
int System::createExternalOutputPort(TYPE dtype, std::string port_name){
    ExternalOutputPort<T>* ext_port = new ExternalOutputPort<T>(dtype);
    return this->addBlock(ext_port, port_name);
}
template <class T>
int System::createExternalInputPort(TYPE dtype, std::string port_name){
    ExternalInputPort<T>* ext_port = new ExternalInputPort<T>(dtype);
    return this->addBlock(ext_port, port_name);
}

template <class T> 
ExternalOutputPort<T>* System::getExternalOutputPort(int ext_op_idx){
    return (ExternalOutputPort<T>*)_blocks[ext_op_idx];
}

template <class T> 
ExternalInputPort<T>* System::getExternalInputPort(int ext_ip_idx){
    return (ExternalInputPort<T>*)_blocks[ext_ip_idx];
}

int System::addBlock(Block* blk, std::string name){
    blk->_block_uid = num_blocks;
    this->_blocks.push_back(blk);
    this->_block_names.push_back(name);
    num_blocks++;
    return blk->_block_uid;
}

void System::addPub(Block* ros_pub){
    _ros_pubs.push_back(ros_pub);
}

int System::addExternalTrigger(ExternalTrigger* ext_trig, std::string name){
    _trig_names.push_back(name);
    _external_triggers.push_back(ext_trig);
    num_ext_trigs++;
    return num_ext_trigs-1;
}

template <class T>
void System::connect(int out_block_uid, int op, int in_block_uid, int ip){
   ((InputPort<T>*) _blocks[in_block_uid]->getInputPort<T>(ip))->connect(_blocks[out_block_uid]->getOutputPort<T>(op));
   Edge ed;
   ed.src_block_idx = out_block_uid;
   ed.src_port = op;
   ed.dest_block_idx = in_block_uid;
   ed.dest_port = ip;
   _edges.push_back(ed);
}

void System::connectExtTrig(int block_uid, int ext_trig_idx){
    _external_triggers[ext_trig_idx]->connect(_blocks[block_uid]);
}

template <class T>
void System::connectToExternalInput(int ext_ip_idx, int dest_block_uid, int ip_idx){
    this->connect<T>(ext_ip_idx, 0, dest_block_uid, ip_idx);
}

template <class T>
void System::connectToExternalInput(int ext_ip_idx, ExternalOutputPort<T>* ext_port){
    ((ExternalInputPort<T>*)_blocks[ext_ip_idx])->connect(ext_port);  //TODO: connect only if the type of port matches using Block ID and port ID
}

template <class T>
void System::connectToExternalOutput(ExternalInputPort<T>* ext_port, int ext_op_idx){
    ext_port->connect(((ExternalOutputPort<T>*)_blocks[ext_op_idx]));  //TODO: connect only if the type of port matches using Block ID and port ID
}

template <class T>
void System::connectToExternalOutput(int src_block_uid, int op_idx, int ext_op_idx){
    this->connect<T>(src_block_uid, op_idx, ext_op_idx, 0);
}


bool System::sortbyconnectivity(const Block* a, const Block* b)
{
    for(auto const &iport : b->getInputPorts()){
        if( iport.second->getConnectedBlockUID() == a->_block_uid){
            return true;
        }
    }
    return false;
}
void System::findsequence(){
    seq = _blocks;
    std::sort(seq.begin(), seq.end(), System::sortbyconnectivity);
}

void System::mainLoop(){
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto step_time = std::chrono::microseconds((int)(_dt*1e6));
    std::chrono::duration<double> avglooptime;
    int i = 0;
    std::cout <<"mainloop started\n";
    while(!_exit_flag){
//        std::cout <<"mainloop running\n";

        start = std::chrono::steady_clock::now();
        // call external triggers   
        for (const auto& ext_trig : _external_triggers){
            ext_trig->process();
        }

        for(const auto &it : seq){
            it->process();
        }

        end = std::chrono::steady_clock::now();
        auto loop_time = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        if(loop_time > step_time){
            // print warning
        }
        else{
            std::this_thread::sleep_for(step_time -loop_time);
        }
        if(i == 100){
            i = 0;
            std::cout << "Loop Frequency : " << 1.0/avglooptime.count() << std::endl;    
            avglooptime = std::chrono::duration<double>(0);
        }   
        avglooptime += (loop_time/100);
        ++i;

    }
    std::cout << "mainloop ended\n";
}

void System::pubLoop(){
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto step_time = std::chrono::microseconds((int)(_dt*1e6));

    while(!_exit_flag){
        start = std::chrono::steady_clock::now();
        for(const auto& ros_pub : _ros_pubs){
            ros_pub->process();
        }
        end = std::chrono::steady_clock::now();
        auto loop_time = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
        if(loop_time > step_time){
            // print warning
        }
        else{
            std::this_thread::sleep_for(step_time -loop_time);
        }
    }
}

void System::execute(){
    system_thread = std::unique_ptr<std::thread>(
                    new std::thread(&mainLoop, this));
    sleep(2);
}

void System::runPubLoop(){
    pub_thread = std::unique_ptr<std::thread>(
                    new std::thread(&pubLoop, this));
    sleep(2);
}

void System::printSystem(){
    std::cout <<seq.size()<<std::endl;
    for (const auto &blk_in_seq : seq){
        int src_blk_idx = blk_in_seq->_block_uid;
        std::cout<< _block_names[src_blk_idx] <<std::endl;
        auto connections = _graph.adjList[src_blk_idx];
        for(auto const &connection : connections){
            std::cout << _block_names[src_blk_idx] << " | " << blk_in_seq->getOutputPortName(connection.src_port) << " -----> " 
                        << _blocks[connection.dest_block_idx]->getInputPortName(connection.dest_port)  << " | " << _block_names[connection.dest_block_idx] << std::endl;
        }
    }
}

}
#endif