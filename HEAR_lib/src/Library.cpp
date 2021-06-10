#include "HEAR_library/Library.hpp"

namespace HEAR
{
Block* Library::createBlock(BLOCK_ID b_type, int b_uid, float _dt = 0, TYPE d_type=TYPE::NA){
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
    case BLOCK_ID::MUX3 :
        blk = new Mux3(b_uid);
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
    case BLOCK_ID::TOHORIZON:
        blk = new ToHorizon(b_uid);
        break; 
    default:
        std::cout << "Invalid BlockID \n";
        return NULL;
    }
    return blk;
}    
} // namespace HEAR
