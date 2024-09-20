/*
 * stepmotor_def.c
 *
 *  Created on: Apr 26, 2024
 *      Author: 15136
 */


#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "stepmotor_def.h"
#include "main.h"
#include "tim.h"
#include "adc.h"

step_param 	xstep_motor_info;
step_param 	ystep_motor_info;
step_param 	zstep_motor_info;
motor_cnt_param xmotor_cnt_info;
motor_cnt_param ymotor_cnt_info;
motor_cnt_param zmotor_cnt_info;
step_circle_param circle_info;

void step_init_info(step_param* step_motor_info, motor_cnt_param* motor_cnt_info){
	memset(step_motor_info, 0, sizeof(step_param));
	memset(motor_cnt_info, 0, sizeof(motor_cnt_param));

	step_motor_info->l_signal = Switch_Limit_Off;
	step_motor_info->r_signal = Switch_Limit_Off;
	step_motor_info->direction = 0;
	step_motor_info->muststop = 0;
	step_motor_info->pulse_cnt_max = 0;

	step_motor_info->cur_pulse = 0;
	step_motor_info->req_pulse = 0;
	step_motor_info->isrunning = 0;
	step_motor_info->req_freq = 0;
	step_motor_info->reqcnts = 0;

	step_motor_info->zisdown = ISUP;
	step_motor_info->drawarray = NULL;
	step_motor_info->dtype = LINE;
}

void cnt_array_init(step_param* step_motor_info, motor_cnt_param* motor_cnt_info, uint16_t reqFreq){
#if 1	// based on pulses
	double tmp_freq = 0;
	double nxt_cnt = 0.0;
	int t_tao = ACCE_SIZE / 4;
	for(int i = 1; i <= ACCE_SIZE; i++){
		tmp_freq = reqFreq - reqFreq * exp((double)(0 - i) / (double)t_tao);
		nxt_cnt = 1000000.0 / tmp_freq;
		motor_cnt_info->cnt_array[i-1] = (uint16_t)nxt_cnt;
	}
#endif
	step_motor_info->req_freq = reqFreq;
	double cal_cnts = 1000000 / reqFreq;
	step_motor_info->reqcnts = (uint16_t)cal_cnts;
}

void step_move(step_param* step_motor_info, motor_cnt_param* motor_cnt_info, uint32_t length_mm, uint8_t direction, uint32_t port, uint16_t pin, TIM_HandleTypeDef* htim){
	uint32_t pul_sum = (double)length_mm / MOVE_REV;
	step_motor_info->req_pulse = pul_sum;
	step_motor_info->direction = direction;
	step_motor_info->isrunning = 1;
	step_motor_info->cur_pulse = 0;
	HAL_GPIO_WritePin((GPIO_TypeDef* )port, pin, direction);

	if(htim->Instance == TIM2){
		TIM2->ARR = motor_cnt_info->cnt_array[0];
		TIM2->CCR4 = motor_cnt_info->cnt_array[0] / 2;
	} else if (htim->Instance == TIM3){
		TIM3->ARR = motor_cnt_info->cnt_array[0];
		TIM3->CCR1 = motor_cnt_info->cnt_array[0] / 2;
	} else {
		/* do nothing */
	}
}

void xstep_circle_handler(uint32_t curpuls){
	uint16_t circlecnt = xstep_motor_info.drawarray[curpuls / 10];
	TIM2->ARR = circlecnt;
	TIM2->CCR4 = circlecnt / 2;
}

void ystep_circle_handler(uint32_t curpuls){
	uint16_t circlecnt = ystep_motor_info.drawarray[curpuls / 10];
	TIM3->ARR = circlecnt;
	TIM3->CCR1 = circlecnt / 2;
}

void xstep_move_handler(step_param* step_motor_info, motor_cnt_param* motor_cnt_info, uint32_t curpuls){
	uint16_t req_freq = (uint16_t)step_motor_info->reqcnts;

	if(curpuls < (step_motor_info->req_pulse / 2) ){ // before midpoint

		if(curpuls < ACCE_SIZE){
		  TIM2->ARR = motor_cnt_info->cnt_array[curpuls];
		  TIM2->CCR4 = motor_cnt_info->cnt_array[curpuls] / 2;
		} else {
		  TIM2->ARR = req_freq;
		  TIM2->CCR4 = req_freq / 2;
		}

	} else { // after midpoint

		uint32_t back_index = step_motor_info->req_pulse - curpuls - 1;
		if(back_index < ACCE_SIZE){
		  TIM2->ARR = motor_cnt_info->cnt_array[back_index];
		  TIM2->CCR4 = motor_cnt_info->cnt_array[back_index] / 2;
		} else {
		  TIM2->ARR = req_freq;
		  TIM2->CCR4 = req_freq / 2;
		}

	}
}

void ystep_move_handler(step_param* step_motor_info, motor_cnt_param* motor_cnt_info, uint32_t curpuls){
	uint16_t req_freq = (uint16_t)step_motor_info->reqcnts;

	if(curpuls < (step_motor_info->req_pulse / 2) ){ // before midpoint

		if(curpuls < ACCE_SIZE){
		  TIM3->ARR = motor_cnt_info->cnt_array[curpuls];
		  TIM3->CCR1 = motor_cnt_info->cnt_array[curpuls] / 2;
		} else {
		  TIM3->ARR = req_freq;
		  TIM3->CCR1 = req_freq / 2;
		}

	} else { // after midpoint

		uint32_t back_index = step_motor_info->req_pulse - curpuls - 1;
		if(back_index < ACCE_SIZE){
		  TIM3->ARR = motor_cnt_info->cnt_array[back_index];
		  TIM3->CCR1 = motor_cnt_info->cnt_array[back_index] / 2;
		} else {
		  TIM3->ARR = req_freq;
		  TIM3->CCR1 = req_freq / 2;
		}

	}
}

void calibrate_xymotor(void){
#if 1
	cnt_array_init(&xstep_motor_info, &xmotor_cnt_info, XMOTORMAXV);
	cnt_array_init(&ystep_motor_info, &ymotor_cnt_info, YMOTORMAXV);

	step_move(&xstep_motor_info, &xmotor_cnt_info, 50000, XRight_To_LX2, (uint32_t)XDirection_GPIO_Port, XDirection_Pin, &htim2);
	step_move(&ystep_motor_info, &ymotor_cnt_info, 50000, YLeft_To_LX1_Back, (uint32_t)YDirection_GPIO_Port, YDirection_Pin, &htim3);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

	while((xstep_motor_info.muststop == 0) || (ystep_motor_info.muststop == 0)){
		HAL_Delay(500);
	}

	xstep_motor_info.muststop = 0;
	ystep_motor_info.muststop = 0;
	step_move(&xstep_motor_info, &xmotor_cnt_info, 13000, XLeft_To_LX1, (uint32_t)XDirection_GPIO_Port, XDirection_Pin, &htim2);
	step_move(&ystep_motor_info, &ymotor_cnt_info, 10000, YRight_To_LX2_Front, (uint32_t)YDirection_GPIO_Port, YDirection_Pin, &htim3);
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
#endif
}

void calibrate_pen(void){
	while((xstep_motor_info.isrunning) || (ystep_motor_info.isrunning)){
		HAL_Delay(500);
	}

	HAL_GPIO_WritePin(ZDirection_GPIO_Port, ZDirection_Pin, Z_to_Down);	/* Z_to_Up Z_to_Down */
	TIM4->ARR = 500;
	TIM4->CCR4 = 500 / 2;
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);

	uint32_t forcedata = 0;
	while(forcedata < 28000){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		forcedata = HAL_ADC_GetValue(&hadc1);
		HAL_Delay(10);
	}
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
	HAL_Delay(100);

#if 1
	zstep_motor_info.zisdown = ISUP;
	HAL_GPIO_WritePin(ZDirection_GPIO_Port, ZDirection_Pin, Z_to_Up);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
#endif
}

void pen_down(void){
	zstep_motor_info.zisdown = ISDOWN;

	HAL_GPIO_WritePin(ZDirection_GPIO_Port, ZDirection_Pin, Z_to_Down);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

void pen_up(void){
	zstep_motor_info.zisdown = ISUP;

	HAL_GPIO_WritePin(ZDirection_GPIO_Port, ZDirection_Pin, Z_to_Up);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
	HAL_Delay(400);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

/* brief: draw a line
 * unit is cm
 *
 * */
void line_move(float xlen, uint8_t xdir, float ylen, uint8_t ydir, uint8_t pendown){
	uint32_t xpuls = xlen * 1000.0;
	uint32_t ypuls = ylen * 1000.0;
	uint16_t xvel = (ylen > xlen) ? (float)(XMOTORMAXV) * (xlen / ylen) :  XMOTORMAXV;
	uint16_t yvel = (ylen > xlen) ? YMOTORMAXV : (float)(YMOTORMAXV) * (ylen / xlen);

	xstep_motor_info.dtype = LINE;
	ystep_motor_info.dtype = LINE;

	if(pendown == Z_to_Up){
		if(zstep_motor_info.zisdown == ISDOWN){
			pen_up();
		} else {
			/* do nothing */
		}
	}

	if(pendown == Z_to_Down){
		if(zstep_motor_info.zisdown == ISUP){
			pen_down();
		} else {
			/* do nothing */
		}
	}

	if(xlen > 0){
		cnt_array_init(&xstep_motor_info, &xmotor_cnt_info, xvel);
		step_move(&xstep_motor_info, &xmotor_cnt_info, xpuls, xdir, (uint32_t)XDirection_GPIO_Port, XDirection_Pin, &htim2);
	}

	if(ylen > 0){
		cnt_array_init(&ystep_motor_info, &ymotor_cnt_info, yvel);
		step_move(&ystep_motor_info, &ymotor_cnt_info, ypuls, ydir, (uint32_t)YDirection_GPIO_Port, YDirection_Pin, &htim3);
	}

	if(xlen > 0){ HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4); }
	if(ylen > 0){ HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1); }
}

/* brief: draw a circle
 *
 * */
void circle_move(uint8_t xdir, uint8_t ydir, uint8_t pendown){
#if 0
	uint32_t array_size = 2000;
	xstep_motor_info.dtype = CIRCLE;
	ystep_motor_info.dtype = CIRCLE;

	uint16_t* xdrawptr = malloc(sizeof(uint16_t) * array_size / 10);
	uint16_t* ydrawptr = malloc(sizeof(uint16_t) * array_size / 10);

	xstep_motor_info.drawarray = xdrawptr;
	ystep_motor_info.drawarray = ydrawptr;

	for(int i = 10; i <= array_size; i = i + 10){
		double w = M_PI * i / (array_size * 2) ;
		double xf = 4000 * sin(w);
		double yf = 4000 * cos(w);

		uint32_t xcnt = 1000000 / xf;
		uint32_t ycnt = 1000000 / yf;
		xcnt = (xcnt > 16000) ? 16000 : xcnt;
		ycnt = (ycnt > 16000) ? 16000 : ycnt;
		xdrawptr[(i / 10) - 1] = xcnt;
		ydrawptr[(i / 10) - 1] = ycnt;
	}

	xstep_motor_info.req_pulse = array_size;
	ystep_motor_info.req_pulse = array_size;
	xstep_motor_info.cur_pulse = 0;
	ystep_motor_info.cur_pulse = 0;
	xstep_motor_info.isrunning = 1;
	ystep_motor_info.isrunning = 1;
	xstep_motor_info.direction = xdir;
	ystep_motor_info.direction = ydir;
	HAL_GPIO_WritePin(XDirection_GPIO_Port, XDirection_Pin, xdir);
	HAL_GPIO_WritePin(YDirection_GPIO_Port, YDirection_Pin, ydir);

	TIM2->ARR = xdrawptr[0];
	TIM2->CCR4 = xdrawptr[0] / 2;

	TIM3->ARR = ydrawptr[0];
	TIM3->CCR1 = ydrawptr[0] / 2;

	if(pendown == Z_to_Up){
		if(zstep_motor_info.zisdown == ISDOWN){
			pen_up();
		} else {
			/* do nothing */
		}
	}

	if(pendown == Z_to_Down){
		if(zstep_motor_info.zisdown == ISUP){
			pen_down();
		} else {
			/* do nothing */
		}
	}

	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
#endif
}

static void enable_timx_stop_timy(void){
	TIM2->ARR = DRAWCIRCLEV;
	TIM2->CCR4 = DRAWCIRCLEV / 2;

	TIM3->ARR = 0;
	TIM3->CCR1 = 0;
}

static void enable_timy_stop_timx(void){
	TIM2->ARR = 0;
	TIM2->CCR4 = 0;

	TIM3->ARR = DRAWCIRCLEV;
	TIM3->CCR1 = DRAWCIRCLEV / 2;
}

void circle_move_method2(float xlen, uint8_t xdir, float ylen, uint8_t ydir, uint8_t pendown){

	if((xdir == XLeft_To_LX1) && (ydir == YRight_To_LX2_Front)){
		circle_info.dir = NR1;
		circle_info.xcordinate = xlen;
		circle_info.ycordinate = 0.0;
	} else if((xdir == XLeft_To_LX1) && (ydir == YLeft_To_LX1_Back)){
		circle_info.dir = NR2;
		circle_info.xcordinate = 0.0;
		circle_info.ycordinate = ylen;
	} else if((xdir == XRight_To_LX2) && (ydir == YLeft_To_LX1_Back)){
		circle_info.dir = NR3;
		circle_info.xcordinate = -xlen;
		circle_info.ycordinate = 0.0;
	} else if((xdir == XRight_To_LX2) && (ydir == YRight_To_LX2_Front)){
		circle_info.dir = NR4;
		circle_info.xcordinate = 0.0;
		circle_info.ycordinate = -ylen;
	} else {
		/* do nothing */
	}

	circle_info.nxterror = 0.0;
	circle_info.curpuls = 0;
	circle_info.totalpuls = xlen * 1600 + ylen * 1600;
	circle_info.xdir = xdir;
	circle_info.ydir = ydir;

	HAL_GPIO_WritePin(XDirection_GPIO_Port, XDirection_Pin, xdir);
	HAL_GPIO_WritePin(YDirection_GPIO_Port, YDirection_Pin, ydir);
	circle_info.circleisrunning = 1;
	circle_info.circle_lock = 1;

	switch(circle_info.dir){
		case NR1:
			circle_info.nxterror = circle_info.nxterror - 2 * circle_info.xcordinate + CIRCLE_REVOLUTION;
			circle_info.xcordinate = circle_info.xcordinate - CIRCLE_REVOLUTION;
			enable_timx_stop_timy();
			break;

		case NR2:
			circle_info.nxterror = circle_info.nxterror - 2 * circle_info.ycordinate + CIRCLE_REVOLUTION;
			circle_info.ycordinate = circle_info.ycordinate - CIRCLE_REVOLUTION;
			enable_timy_stop_timx();
			break;

		case NR3:
			circle_info.nxterror = circle_info.nxterror - 2 * (-circle_info.xcordinate) + CIRCLE_REVOLUTION;
			circle_info.xcordinate = circle_info.xcordinate + CIRCLE_REVOLUTION;
			enable_timx_stop_timy();
			break;

		case NR4:
			circle_info.nxterror = circle_info.nxterror - 2 * (-circle_info.ycordinate) + CIRCLE_REVOLUTION;
			circle_info.ycordinate = circle_info.ycordinate + CIRCLE_REVOLUTION;
			enable_timy_stop_timx();
			break;

		default:
			break;
	}

	if(pendown == Z_to_Up){
		if(zstep_motor_info.zisdown == ISDOWN){
			pen_up();
		} else {
			/* do nothing */
		}
	}

	if(pendown == Z_to_Down){
		if(zstep_motor_info.zisdown == ISUP){
			pen_down();
		} else {
			/* do nothing */
		}
	}

	TIM2->CNT = 0;
	TIM3->CNT = 0;
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
}

void circle_handler_method2(void){
	if(circle_info.nxterror >= 0){
		switch(circle_info.dir){
			case NR1:
				circle_info.nxterror = circle_info.nxterror - 2 * circle_info.xcordinate + CIRCLE_REVOLUTION;
				circle_info.xcordinate = circle_info.xcordinate - CIRCLE_REVOLUTION;
				enable_timx_stop_timy();
				break;

			case NR2:
				circle_info.nxterror = circle_info.nxterror - 2 * circle_info.ycordinate + CIRCLE_REVOLUTION;
				circle_info.ycordinate = circle_info.ycordinate - CIRCLE_REVOLUTION;
				enable_timy_stop_timx();
				break;

			case NR3:
				circle_info.nxterror = circle_info.nxterror - 2 * (-circle_info.xcordinate) + CIRCLE_REVOLUTION;
				circle_info.xcordinate = circle_info.xcordinate + CIRCLE_REVOLUTION;
				enable_timx_stop_timy();
				break;

			case NR4:
				circle_info.nxterror = circle_info.nxterror - 2 * (-circle_info.ycordinate) + CIRCLE_REVOLUTION;
				circle_info.ycordinate = circle_info.ycordinate + CIRCLE_REVOLUTION;
				enable_timy_stop_timx();
				break;

			default:
				break;
		}
	} else {
		switch(circle_info.dir){
			case NR1:
				circle_info.nxterror = circle_info.nxterror + 2 * circle_info.ycordinate + CIRCLE_REVOLUTION;
				circle_info.ycordinate = circle_info.ycordinate + CIRCLE_REVOLUTION;
				enable_timy_stop_timx();
				break;

			case NR2:
				circle_info.nxterror = circle_info.nxterror + 2 * (-circle_info.xcordinate) + CIRCLE_REVOLUTION;
				circle_info.xcordinate = circle_info.xcordinate - CIRCLE_REVOLUTION;
				enable_timx_stop_timy();
				break;

			case NR3:
				circle_info.nxterror = circle_info.nxterror + 2 * (-circle_info.ycordinate) + CIRCLE_REVOLUTION;
				circle_info.ycordinate = circle_info.ycordinate - CIRCLE_REVOLUTION;
				enable_timy_stop_timx();
				break;

			case NR4:
				circle_info.nxterror = circle_info.nxterror + 2 * circle_info.xcordinate + CIRCLE_REVOLUTION;
				circle_info.xcordinate = circle_info.xcordinate + CIRCLE_REVOLUTION;
				enable_timx_stop_timy();
				break;

			default:
				break;
		}
	}
}

void line_move_method2(float xlen, uint8_t xdir, float ylen, uint8_t ydir, uint8_t pendown){
	uint32_t xpuls = xlen * 1000.0;
	uint32_t ypuls = ylen * 1000.0;
	uint16_t xvel = (ylen > xlen) ? (float)(DRAWCIRCLEV) * (ylen / xlen) : DRAWCIRCLEV;
	uint16_t yvel = (ylen > xlen) ? DRAWCIRCLEV : (float)(DRAWCIRCLEV) * (xlen / ylen);

	xstep_motor_info.dtype = LINE;
	ystep_motor_info.dtype = LINE;

	if(xlen > 0){
		//cnt_array_init(&xstep_motor_info, &xmotor_cnt_info, xvel);
		step_move(&xstep_motor_info, &xmotor_cnt_info, xpuls, xdir, (uint32_t)XDirection_GPIO_Port, XDirection_Pin, &htim2);
		TIM2->ARR = xvel;
		TIM2->CCR4 = xvel / 2;
	}

	if(ylen > 0){
		//cnt_array_init(&ystep_motor_info, &ymotor_cnt_info, yvel);
		step_move(&ystep_motor_info, &ymotor_cnt_info, ypuls, ydir, (uint32_t)YDirection_GPIO_Port, YDirection_Pin, &htim3);
		TIM3->ARR = yvel;
		TIM3->CCR1 = yvel / 2;
	}

	if(pendown == Z_to_Up){
		if(zstep_motor_info.zisdown == ISDOWN){
			pen_up();
		} else {
			/* do nothing */
		}
	}

	if(pendown == Z_to_Down){
		if(zstep_motor_info.zisdown == ISUP){
			pen_down();
		} else {
			/* do nothing */
		}
	}

	if(xlen > 0){ HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4); }
	if(ylen > 0){ HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1); }
}



