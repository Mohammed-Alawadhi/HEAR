
#include "HEAR_core/System.hpp"

namespace HEAR{

System::System(const int frequency){
    this->_dt = 1.f/frequency;
}

System::~System(){
    if(system_thread){
        system_thread->join();
    }
}
bool System::init(){
    // do some checks for errors in connectivity etc
    this->findsequence();
    _graph = new Graph(_edges, num_blocks);
    return true;
}

template <class T>
int System::createExternalOutputPort(int dtype, std::string port_name){
    ExternalOutputPort<T>* ext_port = new ExternalOutputPort<T>(dtype);
    return this->addBlock(ext_port, port_name);
}

template <class T>
int System::createExternalInputPort(int dtype, std::string port_name){
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
        if( iport->getConnectedBlockUID() == a->_block_uid){
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
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    auto step_time = std::chrono::microseconds((int)(_dt*1e6));
    std::chrono::duration<double> avglooptime;
    int i = 0;
    while(!exit){
        start = std::chrono::steady_clock::now();
        // call external triggers   
        for(auto &it : seq){
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

    }
}

void System::execute(){
    system_thread = std::unique_ptr<std::thread>(
                    new std::thread(&System::mainLoop, this));
    sleep(3);
}

void System::terminate(){
    exit = true;
}
void System::printSystem(){
    for (int i = 0; i < seq.size(); i++){
        int src_blk_idx = seq[i]->_block_uid;
        auto connections = _graph->adjList[src_blk_idx];
        for(auto const &connection : connections){
            std::cout << _block_names[src_blk_idx] << " | " << seq[i]->getOutputPortName(connection[0]) << " -----> " 
                        << _block_names[connection[1]] << " | " << seq[i]->getInputPortName(connection[2]) << std::endl;
        }
    }
}


}