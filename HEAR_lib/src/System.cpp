#include "HEAR_core/System.hpp"
#include "Vector3D.hpp"

#include "HEAR_control/BWFilter.hpp"
#include "HEAR_control/Demux3.hpp"
#include "HEAR_control/Differentiator.hpp"
#include "HEAR_control/Eul2Rot.hpp"
#include "HEAR_control/FbLinearizer.hpp"
#include "HEAR_control/FromHorizon.hpp"
#include "HEAR_control/Gain.hpp"
#include "HEAR_control/HexaActuationSystem.hpp"
#include "HEAR_control/Mux3.hpp"
#include "HEAR_control/PID_Block.hpp"
#include "HEAR_control/Rot2Eul.hpp"
#include "HEAR_control/Saturation.hpp"
#include "HEAR_control/Sum.hpp"
#include "HEAR_control/ToHorizon.hpp"

namespace HEAR
{
Block* System::createBlock(BLOCK_ID b_type, const std::string& name, TYPE d_type=TYPE::NA){
    Block* blk;
    switch (b_type)
    {
    case BLOCK_ID::BW_FILT2 :
        if(d_type == TYPE::Float3){
            blk = new BWFilter2<Vector3D<float>>(num_blocks++);
        }
        else{
            blk = new BWFilter2<float>(num_blocks++);
        }
        break;
    case BLOCK_ID::DEMUX3 :
        blk = new Demux3(num_blocks++);
        break;
    case BLOCK_ID::DIFFERENTIATOR :
        if(d_type == TYPE::Float3){
            blk = new Differentiator<Vector3D<float>>(_dt, num_blocks++);
        }
        else{
            blk = new Differentiator<float>(_dt, num_blocks++);
        }
        break;
    case BLOCK_ID::EUL2ROT :
        blk = new Eul2Rot(num_blocks++);
        break;
    case BLOCK_ID::FORCE2ROT :
        blk = new FbLinearizer::Force2Rot(num_blocks++);
        break;
    case BLOCK_ID::FROMHORIZON :
        blk = new FromHorizon(num_blocks++);
        break;
    case BLOCK_ID::GAIN :
        blk = new Gain(num_blocks++);
        break;
    case BLOCK_ID::HEXAACTUATIONSYSTEM :
        blk = new HexaActuationSystem(num_blocks++);
        break;
    case BLOCK_ID::MUX3 :
        blk = new Mux3(num_blocks++);
    case BLOCK_ID::PID :
        blk = new PID_Block(_dt, num_blocks++);
        break;
    case BLOCK_ID::ROT2EUL :
        blk = new Rot2Eul(num_blocks++);
        break;
    case BLOCK_ID::ROTDIFF2ROD :
        blk = new FbLinearizer::RotDiff2Rod(num_blocks++);
        break;
    case BLOCK_ID::SATURATION :
        blk = new Saturation(num_blocks++);
        break;
    case BLOCK_ID::SUM :
        blk = new Sum(num_blocks++);
        break;
    case BLOCK_ID::TOHORIZON:
        blk = new ToHorizon(num_blocks++);
        break; 
    default:
        std::cout << "Invalid BlockID \n";
        return NULL;
    }

    _blocks.push_back(blk);
    _block_names.push_back(name);
    return blk;
}
    
} // namespace HEAR
