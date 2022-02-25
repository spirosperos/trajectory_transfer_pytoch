#ifndef SQEXP_H
#define SQEXP_H
#include <acin_robot_control/interpolators/basis_fcn/basis_fcn.h>
class sqexp: public basis_fcn_C
{
public:
  sqexp():basis_fcn_C{}
  {
  }
  ~sqexp()
  {
    // std::cout<<"sqexp gets delted"<<std::endl<<std::endl;
  }
  uint64_t get_num_para(void) const{
    return 2;
  }
  double eval(double t, std::vector<double> &para) const{
    if (para.size() != 2){
      assert(1 == 0);
    }
    double sigma_sq = para[0];
    double center = para[1];
    double val = exp((-0.5*(t - center)*(t - center))/ (sigma_sq*sigma_sq));
    return val;
  }
  double evalDiff(double t, std::vector<double>  &para) const{
    if (para.size() != 2){
      assert(1 == 0);
    }
    double sigma_sq = para[0];
    double center = para[1];
    double val = (-(t - center))/ (sigma_sq*sigma_sq)* exp((-0.5*(t - center)*(t - center))/ (sigma_sq*sigma_sq));
    return val;
  }
  double evalDiffDiff(double t, std::vector<double> &para) const{
    if (para.size() != 2){
      assert(1 == 0);
    }
    double sigma_sq = para[0];
    double center = para[1];
    double sigma_square = sigma_sq*sigma_sq;
    double val = ((t*t-2*center*t-sigma_square+center*center)/ (sigma_square*sigma_square))* exp((-0.5*(t - center)*(t - center))/ (sigma_sq*sigma_sq));
    return val;
  }
};


#endif //SQEXP_H
