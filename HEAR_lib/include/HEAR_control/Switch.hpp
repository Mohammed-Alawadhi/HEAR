#ifndef SWITCH_HPP
#define SWITCH_HPP

#include "HEAR_core/Block.hpp"

namespace HEAR {

class Switch : public Block {
private :
    bool _triggered = false;
    InputPort<float> *_input_port;
    OutputPort<float> *_default_port, *_other_port;
public:
    enum IP{COM};
    enum OP{NC, NO};
    Switch(int b_uid);
    void process();
    void update(UpdateMsg* u_msg) override;

};

}

#endif