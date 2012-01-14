/*
 * cu_ostypes.h
 *
 * Copyright 2001-2010 Texas Instruments, Inc. - http://www.ti.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/****************************************************************************/
/*                                                                          */
/*    MODULE:   cu_ostypes.h                                                */
/*    PURPOSE:                                                              */
/*                                                                          */
/****************************************************************************/

#ifndef _CUOSTYPES_H_
#define _CUOSTYPES_H_

/* types */
/*********/
#if !defined(VOID)
typedef void                    VOID,*PVOID;
#endif
typedef unsigned char           U8,*PU8;
typedef /*signed*/ char         S8,*PS8,**PPS8;
typedef unsigned short          U16,*PU16;
typedef signed short            S16,*PS16;
typedef unsigned long           U32,*PU32;
typedef signed long             S32,*PS32;
typedef float                   F32,*PF32;
typedef PVOID                   THandle;
typedef int                     TI_SIZE_T;

#endif

