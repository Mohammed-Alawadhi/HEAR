#ifndef MEDIANFILTER_HPP
#define MEDIANFILTER_HPP

#include "HEAR_core/Block.hpp"
#include <queue>
#include <list>
#include <algorithm>

namespace HEAR{

class MedianFilter : public Block {
private :
    InputPort<float>* _input_port;
    OutputPort<float>* _output_port;
    unsigned _n_samples = 10;
    std::deque<float> vals;
public :
    enum IP{INPUT};
    enum OP{OUTPUT};
    MedianFilter(int b_uid);
    void setWinSize(unsigned win_size){
        _n_samples = win_size;
        vals.resize(_n_samples, 0);
    } 
    void process();   

};

}

#endif