#ifndef PROMPINTERPOLATOR_H

#define PROMPINTERPOLATOR_H

#include <array>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <acin_robot_control/interpolators/basis_fcn/basis_fcn.h>
#include <acin_robot_control/interpolators/basis_fcn/sqexp.h>
#include <iostream>
#include <fstream>
typedef boost::shared_ptr<basis_fcn_C::basis_fcn_t> smart_bfcn_ptr_t;
typedef std::vector<smart_bfcn_ptr_t> bfcn_1D_t;
typedef std::vector<bfcn_1D_t> bfcn_2D_t;
typedef std::vector<std::vector<std::vector<double>>> bfcn_para_3D_t;

class ProMPInterpolator_C
{
public:
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Phi_t;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> TrajPoint_t;
  // typedef std::vector<basis_fcn_C::basis_fcn_t *> basis_fcn_vec_t;
  // typedef std::vector<std::vector<double>> basis_para_t;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> W_t;
  typedef std::vector<uint64_t> dim_W_t;
  ProMPInterpolator_C()
  {
    this->dim_W.resize(1);
    this->bfcn_2D.resize(1);
    this->bfcn_2D[0].resize(1);
    this->bfcn_2D[0][0] = smart_bfcn_ptr_t(new sqexp());
    this->bfcn_para_3D.resize(1);
    this->bfcn_para_3D[0].resize(1);
    this->bfcn_para_3D[0][0].resize(1);
    this->dim_ProMP = 1;
    this->dim_Wvec = 1;
    W = W_t(1,1);
  }

  ProMPInterpolator_C(std::vector<uint64_t> &dim_W, W_t W)
  {
    this->setup_ProMP(dim_W, W, true);
  }
  ~ProMPInterpolator_C(){
    // for(uint64_t ii = 0; ii < this->dim_W.size(); ii++){
    //   for(uint64_t jj = 0; jj < this->bfcn_2D[ii].size(); jj++){
    //     if (this->bfcn_2D[ii][jj] != NULL){
    //       delete this->bfcn_2D[ii][jj];
    //     }
    //   }
    // }
  }
  void default_basis(void){
    this->bfcn_2D.resize(this->dim_W.size());
    this->bfcn_para_3D.resize(this->dim_W.size());
    for(uint64_t ii = 0; ii < this->dim_W.size(); ii++){
      this->bfcn_2D[ii].resize(1);
      this->bfcn_para_3D[ii].resize(1);
      for(uint64_t jj = 0; jj < this->bfcn_2D[ii].size(); jj++){
        this->bfcn_2D[ii][jj] = smart_bfcn_ptr_t(new sqexp());
        std::vector<double> bfcn_para;
        double delta = 0;
        if(this->dim_W[ii] > 1){
         delta = (this->mu_ub - this->mu_lb)/(this->dim_W[ii]-1);
        }
        double sigma = delta/2;
        if (delta == 0){
          sigma = 1.0;
        }
        bfcn_para_3D[ii][jj].resize(this-> dim_W[ii]*2);
        for(uint64_t kk = 0; kk < this-> dim_W[ii]; kk++){
          double center = this->mu_lb + kk*delta;
          bfcn_para.push_back(sigma);
          bfcn_para.push_back(center);
        }
        bfcn_para_3D[ii][jj]=bfcn_para;
      }
    }
    this->bfunc_set=true;
  }
  void setup_ProMP(dim_W_t &dim_W, W_t &W, bfcn_2D_t &bfcn_2D,bfcn_para_3D_t &bfcn_para_3D){
    this->setup_ProMP(dim_W, W, false);
    this->set_bfcn(bfcn_2D, bfcn_para_3D);
  }
  void setup_ProMP(dim_W_t&dim_W, W_t &W, bool default_bfcn_flag = true){
    uint64_t temp = 0;
    if(this->dim_W.size() != dim_W.size()){
      this->dim_W.resize(dim_W.size());
    }
    for (uint64_t ii = 0; ii < dim_W.size(); ii++ ){
      temp += dim_W[ii];
      this->dim_W[ii] = dim_W[ii];
    }

    this->dim_Wvec = temp;
    this->dim_ProMP = dim_W.size();
    // Phi.resize(this->dim_ProMP, this->dim_Wvec);
    if(W.rows() == temp){
      this->W = W;
    }else{
      ROS_ERROR_STREAM(""<<"Wrong Dim. for the Weight Vector to setup the ProMP interpolatoper, specifed dim:="<< temp << "Vector w dim:="<<W.rows());
      this->W = W_t::Zero(temp,1);
    }
    this->q_offset = TrajPoint_t::Zero(dim_W.size(),1);
    this->qd_offset = TrajPoint_t::Zero(dim_W.size(),1);
    this->qdd_offset = TrajPoint_t::Zero(dim_W.size(),1);
    if(default_bfcn_flag){
      this->default_basis();
    }else{
      this->bfunc_set = false;
    }
  }
  void set_traj_offset(TrajPoint_t q, TrajPoint_t qd,TrajPoint_t qdd){
    if(q.rows() == this->dim_ProMP){
      this->q_offset = q;
    }
    if(qd.rows() == this->dim_ProMP){
      this->qd_offset = qd;
    }
    if(qdd.rows() == this->dim_ProMP){
      this->qdd_offset = qdd;
    }
  }
  void set_q_offset(TrajPoint_t q){
    if(q.rows() == this->dim_ProMP){
      this->q_offset = q;
    }
  }
  void set_qd_offset(TrajPoint_t qd){
    if(qd.rows() == this->dim_ProMP){
      this->qd_offset = qd;
    }
  }
  void set_qdd_offset(TrajPoint_t qdd){
    if(qdd.rows() == this->dim_ProMP){
      this->qdd_offset = qdd;
    }
  }
  void set_bfcn(bfcn_2D_t &bfcn_2D,bfcn_para_3D_t &bfcn_para_3D){
    // for(uint64_t ii = 0; ii < dim_W.size(); ii++){
    //   if this->bfcn_2D != Null
    // }
    this->bfcn_2D = bfcn_2D;
    this->bfcn_para_3D = bfcn_para_3D;
    this->bfunc_set = true;
    //TODO check consistened of the basis fcn and the t
  }

  uint64_t getProMPdim(void) const{
    return this -> dim_W.size();
  }
  std::vector<uint64_t> getWdim(void) const{
    return this->dim_W;
  }
  uint64_t getWdim_vec(void) const{
    return this->dim_W.size();
  }
  void printProMP(void) const{
    std::ostringstream stringStream;
    stringStream << "ProMP dim. of trajectory: "<< std::endl<<"\t"<<dim_W.size()<<std::endl;
    ROS_INFO_STREAM(stringStream.str());
    stringStream.str("");
    stringStream.clear();
    stringStream << "ProMP Weight vector dim.: "<< std::endl<<"\t"<<dim_Wvec<<std::endl;
    ROS_INFO_STREAM(stringStream.str());
    stringStream.str("");
    stringStream.clear();
    stringStream << "ProMP Weight dim. per dim: "<< std::endl<<"\t[";
    for (auto const& ele: dim_W) {
      stringStream << ele << ", ";
    }
    stringStream <<"]"<< std::endl;
    ROS_INFO_STREAM(stringStream.str());
    stringStream.str("");
    stringStream.clear();
    stringStream << "Weight vec: "<< std::endl<<"\t";
    stringStream <<W<< std::endl;
    ROS_INFO_STREAM(stringStream.str());
    return;
  }
  //void setTrajectory(std::array<std::array<double, df>, steps>& traj)
  // void setTrajectory(TrajMatrix& traj)
  // {
  //   traj_ = traj;
  // }
  void ProMP_accel(double t, TrajPoint_t &q_dd)
  {
    this->calc_(t, q_dd, this->bfcn_2D, this->bfcn_para_3D, 2);
  }

  void ProMP_vel(double t, TrajPoint_t &q_d)
  {
    this->calc_(t, q_d, this->bfcn_2D, this->bfcn_para_3D, 1);
  }

  void ProMP_traj(double t, TrajPoint_t &q)
  {
    this->calc_(t, q, this->bfcn_2D, this->bfcn_para_3D, 0);
  }

private:
    std::vector<uint64_t> dim_W;
    uint64_t dim_Wvec;
    uint64_t dim_ProMP;
    std::vector<double> basis_fnc_para;
    // TrajPoint_t q;
    W_t W;
    bfcn_2D_t bfcn_2D;
    bfcn_para_3D_t bfcn_para_3D;
    double t_0 = 0.0;
    double Tend = 1.0;
    double mu_ub = 1.1;
    double mu_lb = -0.1;
    bool bfunc_set = false;
    std::vector<double> default_para;
    std::string default_bfcn = "sqexp";
    TrajPoint_t q_offset;
    TrajPoint_t qd_offset;
    TrajPoint_t qdd_offset;

    void calc_(double t, TrajPoint_t &q, bfcn_2D_t &bfcn_2D, bfcn_para_3D_t &bfcn_para_3D, uint64_t dd = 0)
    {
      uint64_t W_offset = 0;
      assert(bfcn_2D.size() == dim_W.size());
      assert(bfcn_para_3D.size() == dim_W.size());
      Phi_t Phi = Phi_t::Zero(dim_ProMP, dim_Wvec);
      TrajPoint_t offset;
      for(uint64_t ii = 0; ii < this -> dim_W.size(); ii++){
        bfcn_1D_t *bfcn_1D = &(bfcn_2D[ii]);
        std::vector<std::vector<double>> *bfcn_para_2D = &(bfcn_para_3D[ii]);
        uint64_t num_w_for_DOF = 0;
        assert(bfcn_1D->size() == bfcn_para_2D->size());
        assert(bfcn_1D->size() >= 1);
        for(uint64_t ll = 0; ll < bfcn_para_2D->size(); ll++){
          std::vector<double> *basis_para = bfcn_para_2D->data();
          smart_bfcn_ptr_t bfnc_loc= bfcn_1D->operator[](ll);
          uint64_t num_para = bfnc_loc->get_num_para();
          uint64_t kk = 0;
          uint64_t num_W_slice = basis_para->size()/num_para; //number of weights for the basis function
          num_w_for_DOF +=num_W_slice;
          assert((basis_para->size() % num_para) == 0);

          // std::cout<<"basis_fnc_para[ii]"<<std::endl;
          // for (auto const& i: basis_fnc_para[ii]) {
          //  std::cout << i << " ";
          // }
          // std::cout<<std::endl;
          for(uint64_t jj =0; jj < num_W_slice; jj++){
            std::vector<double> para(num_para);
            for(uint16_t jj_para = 0; jj_para < num_para; jj_para++){
              assert(kk<basis_para->size());
              para[jj_para] = basis_para->operator[](kk);
              kk += 1;
            }
            double eval = 0.0;
            switch(dd){
              case 0:
                eval = bfnc_loc->eval(t, para);
                assert(q_offset.rows() == Phi.rows());
                offset = q_offset;
                break;
              case 1:
                eval = bfnc_loc->evalDiff(t, para);
                offset = qd_offset;
                break;
              case 2:
                eval = bfnc_loc->evalDiffDiff(t, para);
                offset = qdd_offset;
                break;
              default:
                eval = 0;
                break;
            }
            assert(W_offset < dim_Wvec);
            Phi(ii, W_offset + jj) = eval;
          }
          W_offset += num_W_slice;
        }
        assert(num_w_for_DOF == this->dim_W[ii]);
      }
      // std::cout << "Phi"<<std::endl;
      // std::cout << Phi<<std::endl;

      // static bool first =true;
      // // ROS_ERROR("print to file ? (%lu)",uint64_t(first));
      // if(first){
      //   std::ofstream outfile;
      //   ROS_WARN("print to file");
      //   outfile.open("/home/christoph/Phi.dat", std::ios::out | std::ios::trunc );
      //   outfile<<"t:= "<<t<<std::endl;
      //   outfile<<"Phi:="<<std::endl;
      //   outfile<<Phi<<std::endl;
      //   outfile.flush();
      //   outfile.close();
      //   first =false;
      // }
      // TrajPoint_t q_ = -Phi*this->W+offset;
      q = Phi*this->W+offset;
      // q.block<3,1>(0,0) = -q_.block<3,1>(0,0) + offset.block<3,1>(0,0);
      // q.block<3,1>(3,0) = q_.block<3,1>(3,0) + offset.block<3,1>(3,0);
    }
};

#endif // PROMPINTERPOLATOR_H
