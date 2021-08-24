
#ifndef EXTERNALTRIGGER_HPP
#define EXTERNALTRIGGER_HPP

#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Block.hpp"
#include <vector>
#include <queue>
#include <atomic>
#include <mutex>

namespace HEAR{
class ExternalTrigger{
protected: 
    std::vector<Block*> _connected_blocks;
    std::atomic<bool> _state;
public:
    virtual ~ExternalTrigger(){}
    virtual TRIG_TYPE getType() = 0;
    virtual void process() = 0;
    void connect(Block* block){
        _connected_blocks.push_back(block);
    }
};

class UpdateTrigger : public ExternalTrigger{
private:
    std::queue<UpdateMsg*> msgs_;
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
            while(true){
                mtx_.lock();
                if(msgs_.empty()){
                    _state = false;
                    mtx_.unlock();
                    break;
                }
                auto msg = msgs_.front();
                msgs_.pop();
                mtx_.unlock();
                for(auto &_connected_block : _connected_blocks){
                    _connected_block->update(msg);
                }
            }
        }
    }
    void UpdateCallback(const UpdateMsg *msg){
        mtx_.lock();
        msgs_.push(msg->copy());
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