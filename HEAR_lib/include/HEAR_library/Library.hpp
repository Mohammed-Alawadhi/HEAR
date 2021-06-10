#ifndef LIBRARY_HPP
#define LIBRARY_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Vector3D.hpp"

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
#include "HEAR_control/Rot2Quat.hpp"
#include "HEAR_control/Quat2Rot.hpp"

#include <iostream>


namespace HEAR
{

class Library {
    public :
    static Block* createBlock(BLOCK_ID b_type, int b_uid, float _dt = 0, TYPE d_type=TYPE::NA);
};

}

#endif