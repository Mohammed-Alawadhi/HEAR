#ifndef LIBRARY_HPP
#define LIBRARY_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Vector3D.hpp"

#include "HEAR_control/BWFilter.hpp"
#include "HEAR_control/Constant.hpp"
#include "HEAR_control/Demux3.hpp"
#include "HEAR_control/Differentiator.hpp"
#include "HEAR_control/Eul2Rot.hpp"
#include "HEAR_control/FbLinearizer.hpp"
#include "HEAR_control/FromHorizon.hpp"
#include "HEAR_control/Gain.hpp"
#include "HEAR_control/HexaActuationSystem.hpp"
#include "HEAR_control/HoldVal.hpp"
#include "HEAR_control/InvertedSwitch.hpp"
#include "HEAR_control/KF3D.hpp"
#include "HEAR_control/MedianFilter.hpp"
#include "HEAR_control/Mrft.hpp"
#include "HEAR_control/Mux3.hpp"
#include "HEAR_control/PID_Block.hpp"
#include "HEAR_control/Rot2Eul.hpp"
#include "HEAR_control/Saturation.hpp"
#include "HEAR_control/Sum.hpp"
#include "HEAR_control/Switch.hpp"
#include "HEAR_control/ToHorizon.hpp"
#include "HEAR_control/Rot2Quat.hpp"
#include "HEAR_control/Quat2Rot.hpp"

#include <iostream>


namespace HEAR
{

class Library {
    public :
    static Block* createBlock(BLOCK_ID b_type, int b_uid, double _dt = 0, TYPE d_type=TYPE::NA);
};

Block* Library::createBlock(BLOCK_ID b_type, int b_uid, double _dt, TYPE d_type){
    Block* blk;
    switch (b_type)
    {
    case BLOCK_ID::BW_FILT2 :
        if(d_type == TYPE::Float3){
            blk = new BWFilter2<Vector3D<float>>(b_uid);
        }
        else{
            blk = new BWFilter2<float>(b_uid);
        }
        break;
    case BLOCK_ID::CONSTANT :
        if(d_type == TYPE::Float3){
            blk = new Constant<Vector3D<float>>(b_uid);
        }
        else{
            blk = new Constant<float>(b_uid);
        }
        break;
    case BLOCK_ID::DEMUX3 :
        blk = new Demux3(b_uid);
        break;
    case BLOCK_ID::DIFFERENTIATOR :
        if(d_type == TYPE::Float3){
            blk = new Differentiator<Vector3D<float>>(_dt, b_uid);
        }
        else{
            blk = new Differentiator<float>(_dt, b_uid);
        }
        break;
    case BLOCK_ID::EUL2ROT :
        blk = new Eul2Rot(b_uid);
        break;
    case BLOCK_ID::FORCE2ROT :
        blk = new FbLinearizer::Force2Rot(b_uid);
        break;
    case BLOCK_ID::FROMHORIZON :
        blk = new FromHorizon(b_uid);
        break;
    case BLOCK_ID::GAIN :
        blk = new Gain(b_uid);
        break;
    case BLOCK_ID::HEXAACTUATIONSYSTEM :
        blk = new HexaActuationSystem(b_uid);
        break;
    case BLOCK_ID::HOLDVAL :
        blk = new HoldVal(b_uid);
        break;
    case BLOCK_ID::INVERTED_SWITCH :
        blk = new InvertedSwitch(b_uid);
        break;
    case BLOCK_ID::KF :
        blk = new KF3D(b_uid, _dt);
        break;
    case BLOCK_ID::MEDIAN_FILTER :
        blk = new MedianFilter(b_uid);
        break;
    case BLOCK_ID::MRFT :
        blk = new MRFT_Block(b_uid);
        break;
    case BLOCK_ID::MUX3 :
        blk = new Mux3(b_uid);
        break;
    case BLOCK_ID::PID :
        blk = new PID_Block(_dt, b_uid);
        break;
    case BLOCK_ID::QUAT2ROT :
        blk = new Quat2Rot(b_uid);
        break;
    case BLOCK_ID::ROT2EUL :
        blk = new Rot2Eul(b_uid);
        break;
    case BLOCK_ID::ROT2QUAT :
        blk = new Rot2Quat(b_uid);
        break;
    case BLOCK_ID::ROTDIFF2ROD :
        blk = new FbLinearizer::RotDiff2Rod(b_uid);
        break;
    case BLOCK_ID::SATURATION :
        blk = new Saturation(b_uid);
        break;
    case BLOCK_ID::SUM :
        blk = new Sum(b_uid);
        break;
    case BLOCK_ID::SWITCH :
        blk = new Switch(b_uid);
        break;
    case BLOCK_ID::TOHORIZON:
        blk = new ToHorizon(b_uid);
        break; 
    default:
        std::cout << "Invalid BlockID \n";
        return NULL;
    }
    return blk;
}

}

#endif