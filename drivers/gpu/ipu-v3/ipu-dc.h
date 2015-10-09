/*	--*- c -*--
 * Copyright (C) 2015 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef H_LINUX_DRIVERS_GPU_IPUV3_IPU_DC_H
#define H_LINUX_DRIVERS_GPU_IPUV3_IPU_DC_H

#define NO_WAVE		0xffffffffu
#define NO_MAP		0xffffffffu

/* default BUILD_BUG_ON_ZERO() triggers a compilation error with non constant
 * symbols in the condition; define a modified version which works with
 * them */
#define _build_bug_on_zero(_cond) \
	(__builtin_constant_p(_cond) ? (sizeof(char[1 - 2*!!(_cond)]) - 1) : 0)

#define _mfield(_v, _pos, _max) \
	(_build_bug_on_zero((_v) > (_max)) + ((uint64_t)(_v) << (_pos)))

#define _mfield_s(_s)		_mfield(!!(_s), 41, 1)
#define _mfield_wave(_wave)	\
	(_mfield((_wave) == NO_WAVE ? 0 : ((_wave)+1), 11, 12))

#define _mfield_map(_map)	\
	(_mfield((_map) == NO_MAP ? 0 : ((_map)+1), 15, 0x1fu))

#define _mfield_glue(_glue)	_mfield(_glue, 4, 0x7fu)

#define _mfield_sync(_sync)	_mfield(_sync, 0, 10)

#define _mfield_code(_c, _pos)			\
	((uint64_t)(_c) << (_pos))

/* The IPU microcode of the iMX family */

/* Handle STOP bit in a special way; to reduce parameter count of macros below
 * do not make it a macro parameter. Caller has to OR it manually in the last
 * instruction.
 *
 * TODO: WSTS_I and WSTS_II do not support the stop bit; catch this at
 * runtime?
 */
#define MICROCODE_STOP		_mfield_s(true)

#define MICROCODE_HLG(_data)			\
	(_mfield(_data, 5, 0xffffffffu))

#define MICROCODE_WRG(_data, _wave, _glue, _sync)	\
	(_mfield_code(0x1, 39) |			\
	 _mfield(_data, 15, (1u << 25) - 1) |		\
	 _mfield_wave(_wave) |				\
	 _mfield_glue(_glue) |				\
	 _mfield_sync(_sync))

#define MICROCODE_HLOA(_af, _data, _map)	\
	(_mfield_code(0x14, 36) |		\
	 _mfield(_af, 36, 0x1u) |		\
	 _mfield(_data, 20, 0xffffu) |		\
	 _mfield_map(_map))

#define MICROCODE_WROA(_af, _data, _map, _wave, _glue, _sync)	\
	(_mfield_code(0x1c, 36) |				\
	 _mfield(_af, 36, 0x1u) |				\
	 _mfield(_data, 20, 0xffffu) |				\
	 _mfield_map(_map) |					\
	 _mfield_wave(_wave) |					\
	 _mfield_glue(_glue) |					\
	 _mfield_sync(_sync))

#define MICROCODE_HLOD(_data, _map)		\
	(_mfield_code(0x10, 36) |		\
	 _mfield(_data, 20, 0xffffu) |		\
	 _mfield_map(_map))

#define MICROCODE_WROD(_data, _map, _wave, _glue, _sync)	\
	(_mfield_code(0x18, 36) |				\
	 _mfield(_data, 20, 0xffffu) |				\
	 _mfield_map(_map) |					\
	 _mfield_wave(_wave) |					\
	 _mfield_glue(_glue) |					\
	 _mfield_sync(_sync))

#define MICROCODE_HLOAR(_af, _map)		\
	(_mfield_code(0x11, 36) |		\
	 _mfield_code(0x03, 34) |		\
	 _mfield(_af, 33, 0x1u) |		\
	 _mfield_map(_map))

#define MICROCODE_WROAR(_af, _map, _wave, _glue, _sync)	\
	(_mfield_code(0x19, 36) |			\
	 _mfield_code(0x03, 34) |			\
	 _mfield(_af, 33, 0x1u) |			\
	 _mfield_map(_map) |				\
	 _mfield_wave(_wave) |				\
	 _mfield_glue(_glue) |				\
	 _mfield_sync(_sync))

#define MICROCODE_HDLODR(_map)			\
	(_mfield_code(0x11, 36) |		\
	 _mfield_code(0x02, 34) |		\
	 _mfield_map(_map))

#define MICROCODE_WRODR(_m2, _m1, _m0, _map, _wave, _glue, _sync)	\
	(_mfield_code(0x19, 36) |					\
	 _mfield_code(0x02, 34) |					\
	 _mfield(_m2, 32, 0x1u) |					\
	 _mfield(_m1, 31, 0x1u) |					\
	 _mfield(_m0, 30, 0x1u) |					\
	 _mfield_map(_map) |						\
	 _mfield_wave(_wave) |						\
	 _mfield_glue(_glue) |						\
	 _mfield_sync(_sync))

#define MICROCODE_WRBC(_map, _wave, _glue, _sync)	\
	(_mfield_code(0x19, 36) |			\
	 _mfield_code(0x02, 34) |			\
	 _mfield_code(0x01, 33) |			\
	 _mfield_code(0x01, 32) |			\
	 _mfield_map(_map) |				\
	 _mfield_wave(_wave) |				\
	 _mfield_glue(_glue) |				\
	 _mfield_sync(_sync))

#define MICROCODE_WCLK(_nclk)			\
	(_mfield_code(0x19, 36) |		\
	 _mfield_code(0x00, 34) |		\
	 _mfield_code(0x01, 33) |		\
	 _mfield(_nclk, 20, 0x1fff))		\

/* TODO: catch unsupported STOP bit? */
#define MICROCODE_WSTS_I(_nclk, _map, _wave, _glue, _sync)	\
	(_mfield_code(0x11, 36) |				\
	 _mfield_code(0x01, 33) |				\
	 _mfield(_nclk, 20, 0x1fff) |				\
	 _mfield_map(_map) |					\
	 _mfield_wave(_wave) |					\
	 _mfield_glue(_glue) |					\
	 _mfield_sync(_sync))

/* TODO: catch unsupported STOP bit? */
#define MICROCODE_WSTS_II(_nclk, _map, _wave, _glue, _sync)	\
	(_mfield_code(0x11, 36) |				\
	 _mfield_code(0x02, 33) |				\
	 _mfield(_nclk, 20, 0x1fff) |				\
	 _mfield_map(_map) |					\
	 _mfield_wave(_wave) |					\
	 _mfield_glue(_glue) |					\
	 _mfield_sync(_sync))

#define MICROCODE_WSTS_III(_nclk, _map, _wave, _glue, _sync)	\
	(_mfield_code(0x11, 36) |				\
	 _mfield_code(0x03, 33) |				\
	 _mfield(_nclk, 20, 0x1fff) |				\
	 _mfield_map(_map) |					\
	 _mfield_wave(_wave) |					\
	 _mfield_glue(_glue) |					\
	 _mfield_sync(_sync))

#define MICROCODE_RD(_nclk, _map, _wave, _glue, _sync)	\
	(_mfield_code(0x11, 36) |			\
	 _mfield_code(0x00, 33) |			\
	 _mfield(_nclk, 20, 0x1fff) |			\
	 _mfield_map(_map) |				\
	 _mfield_wave(_wave) |				\
	 _mfield_glue(_glue) |				\
	 _mfield_sync(_sync))

/* TODO: accordingly ref manual, '_sync' is 3 bits only and some fields are
 * shifted right by one position; is this really correct or just a bug in the
 * doc? https://community.freescale.com/message/571388
 */
#define MICROCODE_WACK(_nclk, _wave, _glue, _sync)	\
	(_mfield_code(0x11, 36) |			\
	 _mfield_code(0x05, 33) |			\
	 ((_mfield(_nclk, 20, 0x1fff) |			\
	   _mfield_wave(_wave) |			\
	   _mfield_glue(_glue)) >> 1) |			\
	 _build_bug_on_zero((_sync) > 7) |		\
	 _mfield_sync(_sync))

#define MICROCODE_MSK(_mask)			\
	(_mfield_code(0x19, 36) |		\
	 _mfield_code(0x00, 34) |		\
	 _mfield(_mask, 15, (1u << 13) - 1))

#define MICROCODE_HMA(_addr)			\
	(_mfield_code(0x2, 37) |		\
	 _mfield(_addr, 5, 0xff))

#define MICROCODE_HMA1(_addr)			\
	(_mfield_code(0x1, 37) |		\
	 _mfield(_addr, 5, 0xff))

/* TODO: accordingly ref manual, '_sync' is 3 bits only; is this really
 * correct or just a bug in the doc?
 * https://community.freescale.com/message/571388
 */
#define MICROCODE_BMA(_lf, _af, _n, _sync)	\
	(_mfield_code(0x3, 37) |		\
	 _mfield(_lf, 36, 1) |			\
	 _mfield(_af, 35, 1) |			\
	 _mfield(_n,   5, 0xff) |		\
	 _build_bug_on_zero((_sync) > 7) |	\
	 _mfield_sync(_sync))

#endif	/* H_LINUX_DRIVERS_GPU_IPUV3_IPU_DC_H */
