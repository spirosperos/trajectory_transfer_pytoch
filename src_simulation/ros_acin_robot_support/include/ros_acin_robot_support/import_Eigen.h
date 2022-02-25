#pragma once
#ifndef IMPORT_EIGEN_H
#define IMPORT_EIGEN_H

#if defined(TWINCAT_COMPILE)
	#define TCMATH_BLOCK_STANDARDLIB (0)
	#include "TcDef.h"
	#include "TcBase.h"
	#include "TcError.h"
	#include "TcMath.h"
	#include "OsBase.h"
	#include <sstream>
	namespace std
    {
        /// initializer_list
        template<class _E>
        class initializer_list
        {
        public:
            typedef _E 		value_type;
            typedef const _E& reference;
            typedef const _E& const_reference;
            typedef size_t 		size_type;
            typedef const _E* iterator;
            typedef const _E* const_iterator;

        private:
            iterator			_M_array;
            size_type			_M_len;

            // The compiler can call a private constructor.
            constexpr initializer_list(const_iterator __a, size_type __l)
                : _M_array(__a), _M_len(__l) { }

        public:
            constexpr initializer_list() noexcept
                : _M_array(0), _M_len(0) { }

            // Number of elements.
            constexpr size_type
                size() const noexcept { return _M_len; }

            // First element.
            constexpr const_iterator
                begin() const noexcept { return _M_array; }

            // One past the last element.
            constexpr const_iterator
                end() const noexcept { return begin() + size(); }
        };
    }
	#define EIGEN_NO_CPUID
	#define _WIN32_WCE
	#define EIGEN_NO_IO
	//#define EIGEN_MAX_CPP_VER 17
	//Eigen compatilite definations
	#define eigen_assert(x)
	float rintf(const float& x);
	int rint(const float& x) ;
	long double fabsl(const long double& x);
	float fmaxf(const float& f1, const float& f2);
	float fminf(const float& f1, const float& f2);
	#include <Eigen/Dense>
	#include <Eigen/LU>
	#include <Eigen/Geometry>
	#include <Eigen/QR>
	typedef  Eigen::Matrix<double, 6, 1> v6_t;
	typedef  Eigen::Matrix<double, -1, 1> vx_1_t;

	static std::string stringf(const char* format, ...);
	static std::string EigenMatrixtoString(const Eigen::Ref < Eigen::Matrix<double,-1,-1> > & vect);
#else
	#include <cmath>
	#include <vector>
	#include <eigen3/Eigen/Dense>
	#include <eigen3/Eigen/LU>
	#include <eigen3/Eigen/Geometry>
	#include <eigen3/Eigen/QR>
#endif
#endif
