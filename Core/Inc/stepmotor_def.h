/*
 * stepmotor_def.h
 *
 *  Created on: Apr 26, 2024
 *      Author: 15136
 */

#ifndef INC_STEPMOTOR_DEF_H_
#define INC_STEPMOTOR_DEF_H_

#include "main.h"

#define Motor_Enable					0
#define Motor_Disable					1
#define Switch_Limit_On					0
#define Switch_Limit_Off				1
#define XRight_To_LX2					0
#define XLeft_To_LX1					1
#define YRight_To_LX2_Front				1
#define YLeft_To_LX1_Back				0
#define Z_to_Down						0
#define Z_to_Up							1

#define Min_Velocity_ARR				16000
#define Test_Velocity_ARR				(500 >> 1)
#define Max_Velocity_ARR				300
#define XMOTORMAXV						13000
#define YMOTORMAXV						13000

#define ACCE_SIZE						500
#define MAXFREQ							3333.0
#define MINFREQ							500.0
#define MOVE_REV						0.625			//unit is mm

#define ISLINE							1
#define ISCIRCLE						0
#define ISDOWN							1
#define ISUP							0
#define CIRCLE_REVOLUTION				((double)1.0 / (double)1600.0)
#define DRAWCIRCLEV						300

#define CALIBRATION						1
#define STARTDRAW						!CALIBRATION

enum _draw_type{
	LINE = 0,
	CIRCLE = 1,
};
typedef enum _draw_type			draw_type;

enum _circle_direction{
	NR1 = 0,
	NR2 = 1,
	NR3 = 2,
	NR4 = 3,
};
typedef enum _circle_direction	circle_direction;


struct _step_param{
	uint8_t direction;
	uint8_t l_signal;			//LX2
	uint8_t r_signal;			//LX1
	uint32_t muststop;
	uint32_t pulse_cnt_max;

	uint32_t cur_pulse;			// record how many pulses are generated
	uint32_t req_pulse;			// the required pulse
	uint16_t req_freq;
	uint16_t reqcnts;
	uint8_t isrunning;

	uint8_t zisdown;
	uint16_t* drawarray;
	enum _draw_type dtype;
};
typedef struct _step_param			step_param;

struct _motor_cnt_param{
	uint16_t cnt_array[ACCE_SIZE];
};
typedef struct _motor_cnt_param		motor_cnt_param;

struct _move_param{
	uint16_t freq;
	uint16_t timeduration;
	uint8_t	 direction;
};
typedef struct _move_param		move_param;

struct _draw_move_param{
	float xlen;
	uint8_t xdir;
	float ylen;
	uint8_t ydir;
	uint8_t pendir;
	enum _draw_type dtype;
};
typedef struct _draw_move_param draw_move_param;

struct _step_circle_param{
	uint8_t xdir;
	uint8_t ydir;
	double xcordinate;
	double ycordinate;
	double nxterror;

	enum _circle_direction dir;
	uint32_t curpuls;
	uint32_t totalpuls;
	uint8_t circleisrunning;
	uint8_t circle_lock;
};
typedef struct _step_circle_param step_circle_param;

extern step_param xstep_motor_info;
extern step_param ystep_motor_info;
extern step_param zstep_motor_info;
extern motor_cnt_param xmotor_cnt_info;
extern motor_cnt_param ymotor_cnt_info;
extern motor_cnt_param zmotor_cnt_info;
extern step_circle_param circle_info;

void step_init_info(step_param* step_motor_info, motor_cnt_param* motor_cnt_info);
void cnt_array_init(step_param* step_motor_info, motor_cnt_param* motor_cnt_info, uint16_t reqFreq);
void step_move(step_param* step_motor_info, motor_cnt_param* motor_cnt_info, uint32_t length_mm, uint8_t direction, uint32_t port, uint16_t pin, TIM_HandleTypeDef* htim);
void xstep_move_handler(step_param* step_motor_info, motor_cnt_param* motor_cnt_info, uint32_t curpuls);
void ystep_move_handler(step_param* step_motor_info, motor_cnt_param* motor_cnt_info, uint32_t curpuls);
void calibrate_xymotor(void);
void calibrate_pen(void);
void pen_down(void);
void pen_up(void);
void line_move(float xlen, uint8_t xdir, float ylen, uint8_t ydir, uint8_t pendown);
void circle_move(uint8_t xdir, uint8_t ydir, uint8_t pendown);
void xstep_circle_handler(uint32_t curpuls);
void ystep_circle_handler(uint32_t curpuls);

void circle_handler_method2(void);
void circle_move_method2(float xlen, uint8_t xdir, float ylen, uint8_t ydir, uint8_t pendown);
void line_move_method2(float xlen, uint8_t xdir, float ylen, uint8_t ydir, uint8_t pendown);

#endif /* INC_STEPMOTOR_DEF_H_ */
