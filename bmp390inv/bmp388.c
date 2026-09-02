#include <stdint.h>

#include "bmp388.h"

/*

  dP/P = -(Mg_0/R) dz/T

  R^{*} = universal gas constant: 8.3144598 J/(mol·K)
  g_{0} = gravitational acceleration: 9.80665 m/s^2
	  M = molar mass of Earth's air: 0.0289644 kg/mol

	  0.0289644 * 9.80665 / 8.3144598   [kg/mol * m/s^2 * mol.K / J] = 0.03416262031 [K/m]

	 dP/dz [Pa/m] = - 0.03416262031 [K/m] · P/T [Pa/K]

*/

// scaled integer arithmetic

static inline int8_t read_int8(uint8_t **pp) {
	uint8_t *p = *pp;
	int8_t	 r = p[0];
	(*pp)++;
	return r;
}
static inline int16_t read_int16(uint8_t **pp) {
	uint8_t *p = *pp;
	int16_t	 r = p[0];
	r |= ((int16_t)p[1]) << 8;
	(*pp) += 2;
	return r;
}
static inline uint16_t read_uint16(uint8_t **pp) {
	uint8_t *p = *pp;
	uint16_t r = p[0];
	r |= ((uint16_t)p[1]) << 8;
	(*pp) += 2;
	return r;
}

void bmp_decodeLinearisationParameters(struct LinearisationParameters *p, uint8_t *bufp) {
	p->T[0] = 0;				   // not used
	p->T[1] = read_uint16(&bufp);  // -8   t^0
	p->T[2] = read_uint16(&bufp);  // 30   t
	p->T[3] = read_int8(&bufp);	   // 48   t^2

	p->P[0] = 0;							  // not used
	p->P[1] = read_int16(&bufp) - (1 << 14);  // 20   t^0  p
	p->P[2] = read_int16(&bufp) - (1 << 14);  // 29   t    p
	p->P[3] = read_int8(&bufp);				  // 32   t^2  p
	p->P[4] = read_int8(&bufp);				  // 37   t^3  p

	p->P[5] = read_uint16(&bufp);  // -3   t^0
	p->P[6] = read_uint16(&bufp);  // 6    t
	p->P[7] = read_int8(&bufp);	   // 8    t^2
	p->P[8] = read_int8(&bufp);	   // 15   t^3

	p->P[9]	 = read_int16(&bufp);  // 48   t^0 p^2
	p->P[10] = read_int8(&bufp);   // 48   t   p^2

	p->P[11] = read_int8(&bufp);  // 65       p^3
}

void bmp_linearize(struct LinearisationParameters *par, uint32_t traw, uint32_t p, int32_t *t_mdegc, int32_t *p_mpa) {
	int64_t s = (int32_t)traw - (par->T[1] << 8);  //       24 bits significant
	int64_t t = s * par->T[3];					   // >>48  32 bits
	t += ((int64_t)par->T[2]) << 18;			   // >>48  34 bits
	t *= s;										   // >>48  58 bits
	t >>= 32;									   // >>16  26 bits

	// https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c#L2354
	// says t*25 >>14 is centidegrees C, or (t*100) >>16)

	*t_mdegc = (t * 1000) >> 16;  // millidegrees C  (~ 10.10 bits)

	// 36 ops
	// //              1+max( 16+41,  26 + 1+max(   8+23  ,     26 + 8)
	int64_t r1 = (((int64_t)par->P[6]) << 41) + t * ((par->P[7] << 23) + t * par->P[8]);  // 58 bits
	int64_t r2 = (((int64_t)par->P[2]) << 40) + t * ((par->P[3] << 21) + t * par->P[4]);  // 56 bits

	int64_t q1 = (((int64_t)par->P[5]) << 40) + t * (r1 >> 26);	 // p^0 <<26
	int64_t q2 = (((int64_t)par->P[1]) << 41) + t * (r2 >> 24);	 // p^1 <<24
	int64_t q3 = (((int64_t)par->P[9]) << 16) + t * par->P[10];	 // p^2
	int64_t q4 = par->P[11];									 // p^3

	int64_t v1 = p * ((q3 << 1) + p * q4);	// 58 bits

	int64_t w = q1 + (((p * ((q2 >> 20) + (v1 >> 24)))) >> 4);

	*p_mpa = (1000 * (w >> 16)) >> 21;
}
