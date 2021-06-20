#ifndef INVERTEDSWITCH_HPP
#define INVERTEDSWITCH_HPP

#include "HEAR_core/Block.hpp"

namespace HEAR {

class InvertedSwitch : public Block {
private :
    bool _triggered = false;
    InputPort<float> *_default_port, *_other_port;
    OutputPort<float> *_output_port;
public:
    enum IP{NC, NO};
    enum OP{COM};
    InvertedSwitch(int b_uid);
    void process();
    void update(UpdateMsg* u_msg) override;

};

}

#endif