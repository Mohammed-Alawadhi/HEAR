#ifndef HOLDVAL_HPP
#define HOLDVAL_HPP

#include "HEAR_core/Block.hpp"

namespace HEAR
{

class HoldVal : public Block {
private:
    InputPort<float>* inp;
    OutputPort<float>* out;
    float _val = 0;
    bool _hold = false;
public:
    enum IP{INPUT};
    enum OP{OUTPUT};
    HoldVal(int);
    void process();
    void update(UpdateMsg* u_msg) override;
};
    
} // namespace HEAR


#endif