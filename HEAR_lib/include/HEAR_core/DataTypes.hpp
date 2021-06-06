#ifndef DATATYPES_HPP
#define DATATYPES_HPP

namespace HEAR{

    enum TYPE{
        NA,
        Float,
        Float3,
        FloatVec,
        RotMat
    };
    enum BLOCK_ID{
        EXT_IP,
        EXT_OP,
        PID,
        SUM,
        GAIN,
        MUX3,
        DEMUX3,
        DIFFERENTIATOR,
        BW_FILT2,
        SATURATION,
        EUL2ROT,
        TOHORIZON,
        FROMHORIZON,
        FORCE2ROT,
        ROTDIFF2ROD,
        HEXAACTUATIONSYSTEM,
        ROSFLOATSUB,
        ROSPOINTSUB,
        ROSFLOATPUB,
        ROSFLOATARRPUB,
        ROSPOSPROV
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
        SWITCH_TRIG,
        BOOL_MSG
    };
    enum SWITCH_STATE{
        OFF,
        ON,
        TOGGLE
    };
    enum PID_ID{
        PID_X,
        PID_Y,
        PID_Z,
        PID_ROLL,
        PID_PITCH,
        PID_YAW,
        PID_YAW_RATE
    };

    class PID_parameters {
    public:
        int id;
        float kp=1, ki=0, kd=0, kdd=0, anti_windup=0;
        bool en_pv_derivation = false;
    };

    class UpdateMsg{
        public:
            virtual UPDATE_MSG_TYPE getType() const =0;
            virtual UpdateMsg* copy() const =0;
    };

    class PID_UpdateMsg: UpdateMsg{
        public:
            PID_parameters param;
            UPDATE_MSG_TYPE getType()const {return UPDATE_MSG_TYPE::PID_UPDATE;}
            UpdateMsg* copy() const{
               auto Copy = new PID_UpdateMsg;
               *Copy = *this;
               return Copy;
            } 
    };

    class SwitchMsg : UpdateMsg{
        public:
            SWITCH_STATE sw_state = SWITCH_STATE::OFF;
            UPDATE_MSG_TYPE getType() const {return UPDATE_MSG_TYPE::SWITCH_TRIG;}
            UpdateMsg* copy() const{
                auto Copy = new SwitchMsg;
                *Copy = *this;
                return Copy;
            }
    };

    class BoolMsg : UpdateMsg{
        public:
            bool data = false;
            UPDATE_MSG_TYPE getType()const {return UPDATE_MSG_TYPE::BOOL_MSG;}
            UpdateMsg* copy() const{
                auto Copy = new BoolMsg;
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