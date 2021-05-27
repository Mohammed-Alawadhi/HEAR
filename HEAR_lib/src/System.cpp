
#include "HEAR_system/System.hpp"

namespace HEAR{

System::System(const int frequency){
    this->_dt = 1.f/frequency;
}

int System::init(){
    // do some checks for errors in connectivity etc
    this->findsequence();
    _graph = Graph(_edges, num_blocks);
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
template <class T>
void System::connect(int src_block_uid, int op_idx, int dest_block_uid, int ip_idx){
   ((InputPort<T>*) _blocks[dest_block_uid]->getInputPort<T>(ip_idx))->connect(_blocks[src_block_uid]->getOutputPort<T>(op_idx));
   _edges.push_back(Edge({std::make_pair(src_block_uid, op_idx), std::make_pair(dest_block_uid, ip_idx)}));
}

template <class T>
void System::connectToExternalInput(int ext_ip_idx, int dest_block_uid, int ip_idx){
    this->connect<T>(ext_ip_idx, ExternalPort::OP::OUTPUT, dest_block_uid, ip_idx);
}

template <class T>
void System::connectToExternalOutput(int ext_op_idx, int src_block_uid, int op_idx){
    this->connect<T>(src_block_uid, op_idx, ext_op_idx, ExternalPort::IP::INPUT);
}

// void System::readExternalInput(){
//     for (auto &e_ip : _external_input_ports){
//         e_ip->process();
//     }
// }

// void System::writeExternalOutput(){
//     for (auto &e_ip : _external_output_ports){
//         e_ip->process();
//     }
// }

bool System::sortbyconnectivity(const Block* a, const Block* b)
{
    for(auto const &iport : b->getInputPorts()){
        if( iport.second->getConnectedBlockUID() == a->_block_uid){
            return false;
        }
    }
    return true;
}
void System::findsequence(){
    seq = _blocks;
    std::sort(seq.begin(), seq.end(), System::sortbyconnectivity);
}

void System::mainLoop(){
    // call external triggers   
    for(auto &it : seq){
       it->process();
    }
    
    // add delay for the loop to run at specified frequency
}

void System::execute(){
    while(true){ // add condition to stop
        this->mainLoop();
    }
}

void System::printSystem(){
    for (int i = 0; i < seq.size(); i++){
        int src_blk_idx = seq[i]->_block_uid;
        auto connections = _graph.adjList[src_blk_idx];
        for(auto const &connection : connections){
            std::cout << _block_names[src_blk_idx] << " | " << seq[i]->getOutputPortName((Block::OP)connection[0]) << " -----> " 
                        << _block_names[connection[1]] << " | " << seq[i]->getInputPortName((Block::IP)connection[2]) << std::endl;
        }
    }
}
}