#include "HEAR_control/MedianFilter.hpp"

namespace HEAR{

MedianFilter::MedianFilter(int b_uid) : Block(BLOCK_ID::MEDIAN_FILTER, b_uid) {
    _input_port = createInputPort<float>(IP::INPUT, "INPUT");
    _output_port = createOutputPort<float>(OP::OUTPUT, "OUTPUT");
    vals = std::deque<float>(_n_samples, 0);
}

void MedianFilter::process(){
    float inp;
    _input_port->read(inp);
    vals.push_back(inp);
    vals.pop_front();
    auto vec = std::vector<float>({vals.begin(), vals.end()});
    std::sort(vec.begin(), vec.end());
    _output_port->write(vec[_n_samples/2]);
}

}