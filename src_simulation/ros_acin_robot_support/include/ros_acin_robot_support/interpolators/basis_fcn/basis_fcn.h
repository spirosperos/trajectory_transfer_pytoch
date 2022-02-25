#ifndef BASIS_FCN_H

#define BASIS_FCN_H

class basis_fcn_C
{
public:
  typedef basis_fcn_C basis_fcn_t;
  basis_fcn_C()
  {

  }
  ~basis_fcn_C()
  {

  }
  virtual double eval(double t, std::vector<double> &para) const=0;
  virtual double evalDiff(double t, std::vector<double>  &para) const=0;
  virtual double evalDiffDiff(double t, std::vector<double>  &para) const=0;
  virtual uint64_t get_num_para(void) const=0;
};


#endif //BASIS_FCN_H
