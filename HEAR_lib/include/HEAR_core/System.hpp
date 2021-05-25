#pragma once

#include "Block.hpp"
#include "Port.hpp"
#include "ExternalPort.hpp"
#include "Graph.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <iostream>

/**
 * We can use graph concept for the blocks connectivity 
 * 
 **/

namespace HEAR{

class System{
public:
    int init();
    void execute();
    void mainLoop();
    template <class T> int createExternalOutputPort(int dtype, std::string port_name);
    template <class T> int createExternalInputPort(int dtype, std::string port_name);
    int addBlock(Block* blk, std::string name);
    template <class T> void connectToExternalInput(int ext_ip_idx, int dest_block_uid, int ip_idx);
    template <class T> void connectToExternalOutput(int ext_op_idx, int src_block_uid, int op_idx);
    template <class T> void connect(int out_block_uid, int op_idx, int in_block_uid, int ip_idx);
    int num_blocks = 0;
private:
    float _dt;
    std::vector<Edge> _edges;
    System(int frequency);
    Graph _graph;
    virtual ~System();
    void printSystem();
    // std::vector<Block*> _external_output_ports;
    // std::vector<Block*> _external_input_ports;
    std::vector<Block*> _external_triggers;
    std::vector<Block*> _blocks;
    std::vector<std::string> _block_names;
    std::vector<Block*> seq;
    // void readExternalInput();
    // void writeExternalOutput();
    static bool sortbyconnectivity(const Block* a, const Block* b);
    void findsequence();

};

}