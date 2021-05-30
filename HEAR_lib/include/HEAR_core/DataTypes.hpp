#ifndef DATATYPES_HPP
#define DATATYPES_HPP

namespace HEAR{

    enum TYPE{
        NA,
        Float,
        Float3
    };
    enum BLOCK_ID{
        EXT_IP,
        EXT_OP,
        PID,
        SUM,
        GAIN,
        MUX3,
        DEMUX3,
        ROSFLOATSUB,
        ROSFLOATPUB
    };
    enum IOTYPE{
        INPUT =0,
        OUTPUT =1
    };
    enum TRIG_TYPE{
        RESET,
        UPDATE
    };
    enum UPDATE_MSG_TYPE{
        PID_UPDATE,
        MRFT_UPDATE,
        BB_UPDATE,
        SWITCH_TRIG
    };
    enum SWITCH_STATE{
        OFF,
        ON,
        TOGGLE
    };

    class UpdateMsg{
        public:
            virtual UPDATE_MSG_TYPE getType()=0;
            virtual UpdateMsg* copy()=0;
    };

    class PID_UpdateMsg: UpdateMsg{
        public:
            float kp, ki, kd;
           UPDATE_MSG_TYPE getType(){return UPDATE_MSG_TYPE::PID_UPDATE;}
           UpdateMsg* copy(){
               auto Copy = new PID_UpdateMsg;
               *Copy = *this;
               return Copy;
           } 
    };

    class SwitchMsg : UpdateMsg{
        public:
            SWITCH_STATE sw_state = SWITCH_STATE::OFF;
            UPDATE_MSG_TYPE getType(){return UPDATE_MSG_TYPE::SWITCH_TRIG;}
            UpdateMsg* copy(){
                auto Copy = new SwitchMsg;
                *Copy = *this;
                return Copy;
            }
    };

    class float3{
    public:
        float x=0, y=0, z=0;
    };

}

#endif