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

#include <stdint.h>
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

 static inline Axis3f axis3f(float x, float y, float z)
 {
   Axis3f ret;
   ret.x = x;
   ret.y = y;
   ret.z = z;
   return ret;
 }

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

 typedef struct {
   // represents the quaternion a + b*i + c*j + d*k
   float a; // real
   float b; // i
   float c; // j
   float d; // k
 } Quatf;

 static inline Quatf quatf(float a, float b, float c, float d)
 {
   Quatf ret;
   ret.a = a;
   ret.b = b;
   ret.c = c;
   ret.d = d;
   return ret;
 }

 // note: I'm somewhat undecided as to whether these should pass by value or by const pointer,
 // and whether they should return by value and by pointer.  I'm hoping the compiler just does its magic either way.

 static inline Quatf quatfConjugate(Quatf q)
 {
   return quatf(q.a, -q.b, -q.c, -q.d);
 }

 static inline Quatf quatfHamiltonProduct(Quatf q1, Quatf q2)
 {
   return quatf(
       q1.a * q2.a - q1.b * q2.b - q1.c * q2.c - q1.d * q2.d,
       q1.a * q2.b + q1.b * q2.a + q1.c * q2.d - q1.d * q2.c,
       q1.a * q2.c - q1.b * q2.d + q1.c * q2.a + q1.d * q2.b,
       q1.a * q2.d + q1.b * q2.c - q1.c * q2.b + q1.d * q2.a
       );
 }

 static inline Quatf axis3fToQuatf(Axis3f v)
 {
   return quatf(0, v.x, v.y, v.z);
 }

 static inline Axis3f quatfPureQuatToAxis3f(Quatf q)
 {
   return axis3f(q.b, q.c, q.d);
 }

 static inline Axis3f quatfTransformAxis3f(Quatf q, Axis3f v)
 {
   Quatf p = axis3fToQuatf(v);
   Quatf qPrime = quatfConjugate(q);
   Quatf result = quatfHamiltonProduct(quatfHamiltonProduct(q, p), qPrime);
   return quatfPureQuatToAxis3f(result);
 }

 static inline Axis3f quatfInverseTransformAxis3f(Quatf q, Axis3f v)
 {
   Quatf p = axis3fToQuatf(v);
   Quatf qPrime = quatfConjugate(q);
   Quatf result = quatfHamiltonProduct(quatfHamiltonProduct(qPrime, p), q);
   return quatfPureQuatToAxis3f(result);
 }

#endif /* IMU_TYPES_H_ */
