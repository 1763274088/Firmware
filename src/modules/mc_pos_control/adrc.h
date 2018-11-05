/*
 * File      : adrc.h
 *
 *
 * Change Logs:
 * Date           Author       	Notes
 * 2018-03-18     zoujiachi   	the first version
 */
 
#ifndef __ADRC_H__
#define __ADRC_H__



typedef struct
{
	float v1;
	float v2;
	float r2;
	float h2;
	float h;
}TD_Controller_Def;

typedef struct
{
	float v1;
	float v2;
	float r0;
	float h0;
	float h;
}ADRC_TD_Def;

typedef struct
{
	float h;
	float beta1;
	float beta2;
	float beta3;
	float alpha1;
	float alpha2;
	float delta;
	float u;
	float b0;
	/* ESO */
	float z1;
	float z2;
	float z3;
}ADRC_ESO_Def;

typedef struct
{
	float h;
	float beta1;
	float beta2;
	float u;
	float b0;
	/* LESO */
	float z1;
	float z2;
}ADRC_LESO_Def;	/* Linear ESO */

typedef struct
{
	float h;
	float h1;
	float r1;
	float c;
}ADRC_NLSEF_Def;

typedef struct
{
	uint16_t size;
	uint16_t head;
	float *data;
}Delay_Block;

float adrc_fhan(float v1, float v2, float r0, float h0);
float adrc_fal(float e, float alpha, float delta);
void adrc_td_init(ADRC_TD_Def* td_t, float h, float r0, float h0);
void adrc_td(ADRC_TD_Def* td, float v);
void adrc_td_control_init(TD_Controller_Def* td_controller, float h, float r2, float h2);
float adrc_td_control(TD_Controller_Def* td_controller, float err);
void adrc_eso_init(ADRC_ESO_Def* eso_t, float h, float beta1, float beta2,float beta3, float alpha1, float alpha2,float delta, float b0);
void adrc_eso(ADRC_ESO_Def* eso_t, float y);
void adrc_leso_init(ADRC_LESO_Def* leso_t, float h, float w, float b0);
void adrc_leso(ADRC_LESO_Def* leso_t, float y);
void adrc_nlsef_init(ADRC_NLSEF_Def* nlsef_t, float h, float r1, float h1, float c);
float adrc_nlsef(ADRC_NLSEF_Def* nlsef_t, float e1, float e2);

uint8_t _delay_block_create(Delay_Block *block, uint16_t size);
void _delay_block_flush(Delay_Block *block);
void _delay_block_push(Delay_Block *block, float val);
float _delay_block_pop(Delay_Block *block);


#endif

