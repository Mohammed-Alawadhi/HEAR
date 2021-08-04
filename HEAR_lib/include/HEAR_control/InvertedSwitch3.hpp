#ifndef INVERTEDSWITCH3_HPP
#define INVERTEDSWITCH3_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Vector3D.hpp"

namespace HEAR {

class InvertedSwitch3 : public Block {
private :
    bool _triggered = false;
    InputPort<Vector3D<float>> *_default_port, *_other_port;
    OutputPort<Vector3D<float>> *_output_port;
public:
    enum IP{NC, NO};
    enum OP{COM};
    InvertedSwitch3(int b_uid);
    void process();
    void update(UpdateMsg* u_msg) override;

};

}

#endif