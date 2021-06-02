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
        PID_FREEZE,
        MRFT_UPDATE,
        BB_UPDATE,
        SWITCH_TRIG,
        ARM
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
        PID_ID id;
        float kp=1, ki=0, kd=0, kdd=0, anti_windup=0;
        bool en_pv_derivation = false;
    };

    class UpdateMsg{
        public:
            virtual UPDATE_MSG_TYPE getType()=0;
            virtual UpdateMsg* copy()=0;
    };

    class PID_UpdateMsg: UpdateMsg{
        public:
            PID_parameters param;
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
    class ArmMsg : UpdateMsg{
        public:
            bool arm = false;
            UPDATE_MSG_TYPE getType(){return UPDATE_MSG_TYPE::ARM;}
            UpdateMsg* copy(){
                auto Copy = new ArmMsg;
                *Copy = *this;
                return Copy;
            }
    };

    class PIDFreezeMsg : UpdateMsg{
        public:
            bool freeze = false;
            UPDATE_MSG_TYPE getType(){return UPDATE_MSG_TYPE::PID_FREEZE;}
            UpdateMsg* copy(){
                auto Copy = new PIDFreezeMsg;
                *Copy = *this;
                return Copy;
            }
    };

    class float3{
    public:
        float x=0, y=0, z=0;
    };

    struct BWFilt2_coeff{
        constexpr static float coeff_120Hz_2nd_butter_5hz[5] =  { -1.279632424997809,0.477592250072517,0.049489956268677,0.098979912537354,0.049489956268677 };
        constexpr static float coeff_200Hz_2nd_butter[5] = { -1.97779f, 0.97803f, 6.1006e-05f, 1.2201e-04f, 6.1006e-05f };
        constexpr static float coeff_200Hz_2nd_butter_50hz[5] = {-1.561018075800718, 0.641351538057563, 0.020083365564211, 0.040166731128423, 0.020083365564211};
        constexpr static float coeff_N200C90[5] = {1.5610, 0.6414, 0.8006, 1.6012, 0.8006};
        constexpr static float coeff_N200C60[5] = {0.3695, 0.1958, 0.3913,    0.7827,    0.3913};
        constexpr static float coeff_N200C50[5] = {-1.8e-16,    0.1716,    0.2929,    0.5858,    0.2929};
    };
    

}

#endif