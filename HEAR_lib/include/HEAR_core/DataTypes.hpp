#pragma once

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
        ROSFLOATSUB,
        ROSFLOATPUB
    };
    enum IOTYPE{
        INPUT =0,
        OUTPUT =1
    };

 }