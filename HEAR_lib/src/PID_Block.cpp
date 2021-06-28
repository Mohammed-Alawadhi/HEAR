#include "HEAR_control/PID_Block.hpp"

namespace HEAR{


PID_Block::PID_Block(double dt, int b_uid): _dt(dt), Block(BLOCK_ID::PID, b_uid){
    err_port = createInputPort<float>(IP::ERROR, "ERROR");
    pv_dot_port = createInputPort<float>(IP::PV_DOT, "PV_DOT");
    u_port = createOutputPort<float>(OP::COMMAND, "COMMAND");
}

void PID_Block::process(){
	if(_enable){
		float err, pv_dot;
		err_port->read(err);
		pv_dot_port->read(pv_dot);
		float u = pid_direct(err, pv_dot);
		// std::cout << "cont_id : " <<_id << " err " << err << " pv_dot " << pv_dot <<" u "<< u  <<" \n";
		u_port->write(u);
	}
}

void PID_Block::reset(){
    std::cout <<"reset function called from PID block " << (int)_id << std::endl;
    accum_I = 0;
    accum_u = 0;
    prev_err = 0;
}

void PID_Block::update(UpdateMsg* u_msg){
    if (u_msg->getType() == UPDATE_MSG_TYPE::PID_UPDATE){
	// std::cout << "updating parameters of controller " << ((PID_UpdateMsg*)u_msg)->param.id << " ...\n";
    	update_params(&((PID_UpdateMsg*)u_msg)->param);
    }
	else if (u_msg->getType() == UPDATE_MSG_TYPE::BOOL_MSG ){
		_enable = ((BoolMsg*)u_msg)->data;
	}
	else{
		std::cout <<"update message type mismatch for PID block\n";
	}
}

void PID_Block::update_params(PID_parameters* para){
    if(para->id != _id){
		return;
	}
	// std::cout << "updating parameters of controller " << para->id << " ...\n";
	if(para->kp >= 0.0){
		std::cout << "Kp = " << para->kp << std::endl;
		_parameters.kp = para->kp;
	}
	if(para->ki > 0.0){
		_parameters.ki = para->ki;
        i_term = true;
	}
    else{
        i_term = false;
    }
	_parameters.kd = para->kd;
	// if(para->kd > 0.0){
	// 	_parameters.kd = para->kd;
    //     d_term = true;
	// }
    // else{
    //     d_term = false;
    // }
	if(para->kdd > 0.0){
		_parameters.kdd = para->kdd;
	}
    else{
        dd_term =false;
    }
	_parameters.en_pv_derivation = para->en_pv_derivation;
	en_pv_derivation = para->en_pv_derivation;
    if(para->anti_windup > 0.0){
		_parameters.anti_windup = para->anti_windup;
        en_anti_windup = true;
	}
    else{
        en_anti_windup = false;
    }
}

float PID_Block::pid_direct(float err, float pv_first, float pv_second) { //Arbitrary large default value for pv_rate
	float u = 0;
	// ************************** P-term ***************************
	u = err *_parameters.kp;
	// ************************** I-term ***************************
	if (i_term)//&& os::is_flying) 
	{
		if (en_anti_windup) { //$$$$$$$$$$$$$$$$$$$$ TODO: Optimize! $$$$$$$$$$$$$$$$$$$$$
			if (fabs(accum_I) < _parameters.anti_windup) {
				accum_I += _parameters.ki*err*_dt;
			}
			else {
				//float buff_I = accum_I + _parameters.ki*err*_dt;
				//if (abs(buff_I) < _parameters.anti_windup) {
				//	accum_I = buff_I;
				//}
				if (((accum_I > 0) && (err < 0))||((accum_I < 0) && (err > 0))) {
					accum_I += _parameters.ki*err*_dt;
				}
			}
		}
		else {
			accum_I += _parameters.ki*err*_dt;
		}
	}
	u += accum_I;
	// ************************** D-term ***************************
	if (d_term) {
		if (en_pv_derivation) {
			u += _parameters.kd*(pv_first);
		}
		else {
			u += _parameters.kd*(err - prev_err) / _dt;
		}
	}
	// ************************* DD-term ***************************
	if (dd_term) {
		u+= _parameters.kdd*(-pv_second);
	}
	prev_err = err;
	return u;
}

}