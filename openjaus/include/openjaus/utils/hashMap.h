/*****************************************************************************
 *  Copyright (c) 2009, OpenJAUS.com
 *  All rights reserved.
 *
 *  This file is part of OpenJAUS.  OpenJAUS is distributed under the BSD
 *  license.  See the LICENSE file for details.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the University of Florida nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
// File Name: timeLib.h
//
// Written by: Danny Kent (jaus AT dannykent DOT com)
//
// Version: 3.3.0b
//
// Date: 09/08/09
//
// Description:	Provides HASH_MAP define depending on operating system / compiler flags

#ifndef HASHMAP_H_
#define HASHMAP_H_

#ifdef WIN32
	#include <hash_map>
	#define HASH_MAP stdext::hash_map
#elif defined(__GNUC__)
	#if defined(__QNX__)
		#include <hash_map>
		#ifndef _NAMESPACE_STLPORT
			#define _NAMESPACE_STLPORT std
		#endif
		#define HASH_MAP _NAMESPACE_STLPORT::hash_map   // QNX 6.3 - GNU 3.3.5
	#elif defined(__APPLE__)  && defined(__MACH__)
			#include <ext/hash_map>
			#define HASH_MAP __gnu_cxx::hash_map
	#else
		#include <features.h>
		#if __GNUC_PREREQ(4,0)
			#include <tr1/unordered_map>
			#define HASH_MAP std::tr1::unordered_map
		#else
			#include <ext/hash_map>
			#define HASH_MAP __gnu_cxx::hash_map
		#endif
	#endif
#else
	#error "No HASH_MAP equivalent defined for this system. Please define in .../libopenJaus/utils/hashMap.h"
#endif


#endif /* HASHMAP_H_ */
