#include import_Eigen.cpp

#if defined(TWINCAT_COMPILE)	
	float rintf(const float& x) {
		return static_cast<float>(round(x));
	}
	int rint(const float& x) {
		return static_cast<int>(round(x));
	}
	long double fabsl(const long double& x) {
		if (x > 0) {
			return x;
		}
		else {
			return -1.0 * x;
		}
	}
	float fmaxf(const float& f1, const float& f2) {
		if (f1 > f2) {
			return f1;
		}
		else {
			return f2;
		}

	}
	float fminf(const float& f1, const float& f2) {
		if (f1 > f2) {
			return f2;
		}
		else {
			return f1;
		}
	}
	
	static std::string stringf(const char* format, ...)
	{
		va_list args;
		va_start(args, format);

		int size = 10;
		std::unique_ptr<char[]> buf(new char[size]);
		vsprintf(buf.get(), format, args);
		va_end(args);
		return std::string(buf.get());
	}
	static std::string EigenMatrixtoString(const Eigen::Ref < Eigen::Matrix<double,-1,-1> > & vect)
	{
		return stringf("[%f]", vect(0,0));
	}
#else

#endif

