#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "HEAR_core/Graph.hpp"
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/ExternalTrigger.hpp"

#include "HEAR_library/Library.hpp"

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
    System(const int frequency, const std::string& sys_name) : _dt(1.0/frequency), _sys_name(sys_name), _exit_flag(false) {}
    virtual ~System();
    int init(bool print_diagram);
    Block* createBlock(BLOCK_ID b_type, const std::string& name, TYPE d_type=TYPE::NA);
    void addExternalTrigger(ExternalTrigger* ext_tirg, const std::string& name);
    void connectExternalTrigger(ExternalTrigger* ext_tirg, Block* dest_block);
    template <class T> ExternalOutputPort<T>* createExternalOutputPort(const std::string& port_name, OutputPort<T>* op);
    template <class T> ExternalInputPort<T>* createExternalInputPort(const std::string& port_name);
    template <class T> ExternalInputPort<T>* createExternalInputPort(const std::string& port_name, InputPort<T>* ip);
    template <class T> void connectExternalInput(ExternalInputPort<T>* sys_ip, InputPort<T>* ip);
    template <class T> void connectExternalInput(ExternalInputPort<T>* sys_ip, ExternalOutputPort<T>* ext_port);
    template <class T> void connect(OutputPort<T>* op, InputPort<T>* ip);
    void loop();    
    double _dt;
    std::string _sys_name;

private:
    int num_blocks = 0;
    int num_ext_trigs = 0;
    std::atomic<bool> _exit_flag;
    std::vector<Edge> _edges;
    Graph _graph;
    std::vector<Block*> _blocks;
    std::vector<std::string> _block_names;
    std::vector<Block*> seq;
    std::vector<ExternalTrigger*> _external_triggers;
    std::vector<std::string> _trig_names;
    static bool sortbyconnectivity(const Block* a, const Block* b);
    bool checkConnectivity(const int &idx , const std::vector<Block*> &vec);
    void findsequence();
    void printSystem();
};

System::~System(){
    std::cout << "deleting blocks \n";
    for (auto &block : _blocks){
        delete block;
    }
}
int System::init(bool print_diagram){
    // do some checks for errors in connectivity etc
    this->findsequence();
    // check unconnected ports
    for(const auto &blk : seq){
        for(const auto &iport : blk->getInputPorts()){
            if(iport.second->getConnectedBlockUID() < 0){
                std::cout << "[WARN]" << _block_names[blk->getBlockUID()] << " | " << blk->getInputPortName(iport.first) << " not connected." << std::endl; 
            }
        }
    }
    _graph = Graph(_edges, _blocks.size());
    if(print_diagram){
        printSystem();
    }
    return true;
}

Block* System::createBlock(BLOCK_ID b_type, const std::string& name, TYPE d_type){
    Block* blk = Library::createBlock(b_type, num_blocks++, _dt, d_type);
    _blocks.push_back(blk);
    _block_names.push_back(name);
    return blk;
}

template <class T>
ExternalOutputPort<T>* System::createExternalOutputPort(const std::string& port_name, OutputPort<T>* op){
    ExternalOutputPort<T>* ext_port = new ExternalOutputPort<T>(num_blocks++);
    _block_names.push_back(port_name);
    _blocks.push_back(ext_port);
    this->connect(op, (InputPort<T>*)ext_port->getInputPort(0));
    return ext_port;
}

template <class T>
ExternalInputPort<T>* System::createExternalInputPort(const std::string& port_name){
    ExternalInputPort<T>* ext_port = new ExternalInputPort<T>(num_blocks++);
    _block_names.push_back(port_name);
    _blocks.push_back(ext_port);
    return ext_port;
}

template <class T>
ExternalInputPort<T>* System::createExternalInputPort(const std::string& port_name, InputPort<T>* ip){
    auto e_port = this->createExternalInputPort<T>(port_name);
    this->connectExternalInput(e_port, ip);
}

void System::addExternalTrigger(ExternalTrigger* ext_tirg, const std::string& name){
    _trig_names.push_back(name);
    _external_triggers.push_back(ext_tirg);
    num_ext_trigs++;
}

template <class T>
void System::connect(OutputPort<T>* op, InputPort<T>* ip){
   ip->connect(op);
   Edge ed;
   ed.src_block_idx = op->getHostBlockUID();
   ed.src_port = op->getPortID();
   ed.dest_block_idx = ip->getHostBlockUID();
   ed.dest_port = ip->getPortID();
   _edges.push_back(ed);
}

template <class T>
void System::connectExternalInput(ExternalInputPort<T>* sys_ip, InputPort<T>* ip){
    this->connect(((Block*)sys_ip)->getOutputPort<T>(0), ip);
}

template <class T>
void System::connectExternalInput(ExternalInputPort<T>* sys_ip, ExternalOutputPort<T>* ext_port){
    sys_ip->connect(ext_port);
}

void System::connectExternalTrigger(ExternalTrigger* ext_tirg, Block* dest_block){
    ext_tirg->connect(dest_block);

}

bool System::checkConnectivity(const int &idx , const std::vector<Block*> &vec){
    std::vector<int> connected_blocks_idx;
    for(const auto &iport : vec[idx]->getInputPorts()){
        connected_blocks_idx.push_back(iport.second->getConnectedBlockUID());
    }
    if(!connected_blocks_idx.empty()){
        for(int i=0; i < vec.size(); i++){
            if(idx != i){
                auto con_blk = vec[i]->getBlockUID();
                for(const auto& connected_block_idx : connected_blocks_idx){
                    if(connected_block_idx == con_blk){
                        return false;
                    }
                }
            }
        }
    }
    return true;
}
void System::findsequence(){
    seq = std::vector<Block*>();
    auto tmp = _blocks;
    while(tmp.size()){
        for(int i=0; i<tmp.size(); i++){
            if(checkConnectivity(i, tmp)){
                seq.push_back(tmp[i]);
                tmp[i] = tmp.back();
                tmp.pop_back();
            }
            
        }
    }
}

void System::loop(){
    for (const auto& ext_trig : _external_triggers){
        ext_trig->process();
    }

    for(const auto &it : seq){
        it->process();
    }
}

void System::printSystem(){
    std::cout << std::endl << "*****" <<_sys_name<< "*****" << std::endl;
    for (const auto &blk_in_seq : seq){
        int src_blk_idx = blk_in_seq->getBlockUID();
        std::cout<< std::endl << _block_names[src_blk_idx] <<std::endl;
        auto connections = _graph.adjList[src_blk_idx];
        for(auto const &connection : connections){
            std::cout << _block_names[src_blk_idx] << " | " << blk_in_seq->getOutputPortName(connection.src_port) << " -----> " 
                        << _blocks[connection.dest_block_idx]->getInputPortName(connection.dest_port)  << " | " << _block_names[connection.dest_block_idx] << std::endl;
        }
    }
    std::cout << std::endl << "***********************************" << std::endl; 
}

}
#endif