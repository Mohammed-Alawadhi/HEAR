#ifndef MRFT_HPP
#define MRFT_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/Timer.hpp"
#include <iostream>
#include <cmath>

namespace HEAR{

class MRFT_Block : public Block{
private:
	Timer _timer;
	MRFT_ID _id;
    InputPort<float>* _input_port;
    InputPort<float>* _bias_port;
    OutputPort<float>* _output_port;
	bool _enable = false;
	bool first_run = true;
	float last_output;
	float e_max;
	float e_min;
	float has_reached_min;
	float has_reached_max;
	float peak_conf_counter;
    MRFT_parameters parameters;
	//
public:
	enum IP {INPUT, BIAS};
    enum OP{COMMAND};
	//---------------
	float mrft_anti_false_switching(float err);
	//---------------
    void setMRFT_ID(MRFT_ID id){
		_id = id;
	}
    void update_params(MRFT_parameters* para);

    MRFT_Block(int b_uid);
    ~MRFT_Block();
    void process();
    void update(UpdateMsg* u_msg) override;
    void reset() override;

};

}

#endif