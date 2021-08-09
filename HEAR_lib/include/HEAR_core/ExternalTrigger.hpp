
#ifndef EXTERNALTRIGGER_HPP
#define EXTERNALTRIGGER_HPP

#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Block.hpp"
#include <vector>
#include <atomic>
#include <mutex>

namespace HEAR{
class ExternalTrigger{
protected: 
    std::vector<Block*> _connected_blocks;
    std::atomic<bool> _state;
public:
    virtual TRIG_TYPE getType() = 0;
    virtual void process() = 0;
    void connect(Block* block){
        _connected_blocks.push_back(block);
    }
};

class UpdateTrigger : public ExternalTrigger{
private:
    UpdateMsg* msg_;
    std::mutex mtx_;
public:
    UpdateTrigger(){
        _state = false;
    }
    TRIG_TYPE getType(){
        return TRIG_TYPE::UPDATE;
    }
    void process(){
        if(_state){
            mtx_.lock();
            auto msg = msg_->copy();
            mtx_.unlock();
            for(auto &_connected_block : _connected_blocks){
            _connected_block->update(msg);
            }
            _state = false;
        }
    }
    void UpdateCallback(const UpdateMsg *msg){
        mtx_.lock();
        msg_ = msg->copy();
        mtx_.unlock();
        _state = true;
    }
};

class ResetTrigger : public ExternalTrigger{
public:
    ResetTrigger(){
        _state = false;
    }
    TRIG_TYPE getType(){
        return TRIG_TYPE::RESET;
    }
    void process(){
        if(_state){
            for(auto &_connected_block : _connected_blocks){
                _connected_block->reset();
            }
            _state = false;
        }
    }
    void resetCallback(){
        _state = true;
    }

};

}
#endif