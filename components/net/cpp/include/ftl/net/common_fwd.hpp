/**
 * @file common_fwd.hpp
 * @copyright Copyright (c) 2020 University of Turku, MIT License
 * @author Nicolas Pope
 */

#ifndef _FTL_NET_COMMON_FORWARD_HPP_
#define _FTL_NET_COMMON_FORWARD_HPP_

#ifndef WIN32
#define INVALID_SOCKET -1
#define SOCKET int

#else  // WIN32

#if defined(_WIN64)
    typedef unsigned __int64 UINT_PTR;
#else
    typedef unsigned int UINT_PTR, *PUINT_PTR;
#endif

typedef UINT_PTR SOCKET;
#define INVALID_SOCKET (SOCKET)(~0)

#endif

#endif
