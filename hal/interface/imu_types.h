/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * imu_types.h - Types used by IMU and releted functions.
 */
#ifndef IMU_TYPES_H_
#define IMU_TYPES_H_

#include <math.h>

 typedef struct {
         int16_t x;
         int16_t y;
         int16_t z;
 } Axis3i16;

 typedef struct {
         int32_t x;
         int32_t y;
         int32_t z;
 } Axis3i32;

 typedef struct {
         union {
		 struct {
                         float x;
                         float y;
                         float z;
                 };
		 float v[3];
        };
 } Axis3f;


 inline static void axis3fAdd(Axis3f *result, const Axis3f *a, const Axis3f *b)
 {
   result->x = a->x + b->x;
   result->y = a->y + b->y;
   result->z = a->z + b->z;
 }

 inline static void axis3fSub(Axis3f *result, const Axis3f *a, const Axis3f *b)
 {
   result->x = a->x - b->x;
   result->y = a->y - b->y;
   result->z = a->z - b->z;
 }

 inline static float axis3fLengthSq(const Axis3f *v)
 {
   return v->x * v->x + v->y * v->y + v->z * v->z;
 }

 inline static float axis3fLength(const Axis3f *v)
 {
   return sqrtf(axis3fLengthSq(v));
 }

 inline static float axis3fDistSq(const Axis3f *a, const Axis3f *b)
 {
   Axis3f aToB;
   axis3fSub(&aToB, b, a);
   return axis3fLengthSq(&aToB);
 }

 inline static float axis3fDist(const Axis3f *a, const Axis3f *b)
 {
   return sqrtf(axis3fDistSq(a, b));
 }

 inline static void axis3fScale(Axis3f *result, const Axis3f *a, float scale)
 {
   result->x = a->x * scale;
   result->y = a->y * scale;
   result->z = a->z * scale;
 }

#endif /* IMU_TYPES_H_ */
