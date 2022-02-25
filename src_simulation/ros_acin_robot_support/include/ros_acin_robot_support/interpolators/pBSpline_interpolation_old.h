#ifndef pBSpline_interpolator_H
#define pBSpline_interpolator_H
#define bool_eq(X,Y) ((X) == (Y) ? (true) : (false))
#define SAT_4(X) ((X) <= (4) ? (X) : (4))
#include <cmath>
#include <Eigen/Core>
#include <cstdlib>
#if defined(USE_SUPPORTED_SPLINE)
  #include <unsupported/Eigen/Splines>
#endif
#include <memory>
using namespace Eigen;
// class test_C{
//   static uint8_t ii;
//   test_C(){
//     printf("generated test_C\n");
//     ii++;
//   }
//   ~test_C(){
//     printf("destroyted test_C:= %u",(uint32_t) ii);
//   }
// };
// class quat_Class : public Eigen::Quaternion<double>(){
//   my_quat_Class() : Eigen::Quaternion<double>(){
//   }
//   ~my_quat_Class(): ~Eigen::Quaternion<double>(){
//   }
// };
template<uint16_t dim, uint16_t deg, uint16_t N_knots, bool QAUT_INTERPOLATION_FLAG = false>
class pBSpline_interpolator_C{
  public:
    /*
    void* operator new(size_t size)
    {
        void* p = std::aligned_alloc(32, size);
        if(p == 0)  throw std::bad_alloc();
        return p;
    }

    void operator delete(void *p)
    {
      pBSpline_interpolator_C<dim, deg, N_knots>* pc = static_cast<pBSpline_interpolator_C<dim, deg, N_knots> *>(p);
      std::aligned_free(p);
    }*/

    //in terms of quaternion ctrlpt_t is a 4D vectors in (x,y,z,w)!!
    //controll(support) point type for the interpolation
    typedef Eigen::Matrix<double, dim, N_knots> ctrlpts_t; //Eigen::Dym- resorves to number of curve points + degree of the polynom -1 assumin the curve have atleast 2 points !

    typedef Matrix<double, N_knots, 1> knot_t;

    typedef Matrix<double, deg, 1> pre_knot_t;
    typedef Matrix<double, dim, deg> pre_ctrlpts_t;

    typedef Eigen::Quaternion<double> ctrlpt_qa_t;
    typedef Eigen::Matrix<double, dim, 1> ctrlpt_t;
    /*type def for the knot points, */

    template<int16_t N>
    struct curve_C{
      private:
      public:
        typedef Eigen::Matrix<double, dim, N> pts_t;
        typedef Eigen::Matrix<double, N, 1> para_t;
        pts_t pts;
        para_t para;
        curve_C(){
          #if N == -1
            #ifdef EIGEN_NO_MALLOC
              throw std::range_error("no dynamic allocation is allowed, define the N template parameter of the spline not as -1");
            #endif
            pts.resize(dim,1);
            para.resize(1,1);
            // pts.col(0) = Eigen::Matrix<double, dim, 1>::Zero();
          #else
            pts = pts_t::Zero(dim,N);
            para = para_t::LinSpaced(N, 0.0, 1.0);
          #endif

        }
        void set_pts(Eigen::Ref<pts_t> p){
          pts = p;
        }
        void operator=(const curve_C<N> &curve_obj){
          #if N == -1
            this->pts.resize(dim, curve_obj.pts.cols());
            this->para.resize(curve_obj.pts.cols(), 1);
          #endif
          this->pts = curve_obj.pts;
          this->para = curve_obj.para;
        }
    };
    typedef struct curve_C< N_knots> curve_t;
    typedef struct curve_C< deg > pre_curve_t;

    // #if bool_eq(deg, 1)
      // typedef Matrix<double, dim,  2 + 2> ctrlptsloc_t; //TODO: i think we only need deg-1 + deg+1 points to define the spline will at the evaluation intervall
      // typedef Matrix<double, 2 + 2, 1> knotloc_t; //TODO: i think we only need deg-1 + deg+1 points to define the spline will at the evaluation intervall
    // #else

    typedef Matrix<double, dim, (deg)*2 + deg + 1> ctrlptsloc_t; // datatype for the controll points the local spline of deg, control vecotr with dim rows
    typedef Matrix<double, (deg)*2 + deg + 1, 1> knotloc_t; // datatype to define the knot element (time, x-Axsis,.... interpolation curve paramter) of the loacl splaine

    // #endif



    #if defined(USE_SUPPORTED_SPLINE)
      /*spline interpolator, Bsplines type, used from unsupported Eigen*/
      typedef Spline<double, dim, deg> SplineND_t; //first defines the dim of a controll Point, seceon defines the degree of the poly spline interpolation
      typedef SplineFitting<SplineND_t> SplineNDFitting_t;
      typedef std::shared_ptr<SplineND_t> SplineND_ptr_t;
    #endif
    pBSpline_interpolator_C(){
      if(QAUT_INTERPOLATION_FLAG && dim != 4){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("QAUT_INTERPOLATION_FLAG interpolation defined but dim. !=4, so ist cant not be a quaternion");
        #endif
      }
      reset();
    }
    /*@ctrlpts control points of the curve, matrix with dynmaic col. range, row dim is fixed
        /number of rows define the vector size
        /number of cols define the number of control points for the interpolation
      @tk knot points of the curve
    */
    pBSpline_interpolator_C(  const curve_t &curve,
                              pre_curve_t &pre_curve,
                              bool periodic = false){
      const ctrlpts_t &ctrlpts = curve.pts;
      const knot_t &tk = curve.para;

      if(QAUT_INTERPOLATION_FLAG && dim != 4){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("QAUT_INTERPOLATION_FLAG interpolation defined but dim. !=4, so ist cant not be a quaternion");
        #endif
      }
      //deg. should be atleast one, for linar spline interpolation linear
      if(deg < 1){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("spline deg. should be greater than 0");
        #endif
      }
      if (N_knots == -1){
        if(ctrlpts.cols() < 1){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("atleast one control point is need");
          #endif
        }
        if(ctrlpts.cols() != tk.rows()){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("number of control points and thier respective number of x values are not equal");
          #endif
        }
      }
      uint64_t N = ctrlpts.cols();
      if(N < deg + 1){
        #if defined(SPLINE_EXCEPTIONS)
          std::stringstream s_error;
          s_error << "number of control points of the curve which should be interpolated must be greater than deg + 1 (" << deg+  1 <<")" << ", N actuel:= "<<N;
          throw std::range_error(s_error.str());
        #endif
      }
      if(pre_curve.para(pre_curve.para.rows()-1) >= curve.para(0)){
        ACIN_CTRL_WARN("Init Interpolator, pre_curve parameter not well defined, redefine it");
        for (uint64_t ii = 0; ii < static_cast<uint64_t>(pre_curve.para.rows()); ii++){
          pre_curve.para(ii) = curve.para(0) - (curve.para(1) - curve.para(0))*(pre_curve.para.rows() - ii);
        }

      }

      set_curve(curve, pre_curve, periodic);
      //
      // double sp_knot_min = this->sp_knots(0);
      // double sp_knot_max = this->sp_knots(sp_knots.rows()-1);

    }
    ~pBSpline_interpolator_C(){
    }
    void ToEulerAngles(Eigen::Quaternion<double> &q, Vector3d &EulerAngles){
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
        EulerAngles(2) = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
        if (std::abs(sinp) >= 1)
            EulerAngles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            EulerAngles(1) = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        EulerAngles(0) = std::atan2(siny_cosp, cosy_cosp);
    }
    void reset(){
        curve_t curve;
        if (N_knots == -1){
          curve.pts = ctrlpts_t::Zero(dim, 2);
          curve.para = knot_t::Zero(2,1);
          curve.para(0,0) = 0.0;
          curve.para(1,0) = 1.0;
        }else{
          curve.pts = ctrlpts_t::Zero();
          curve.para = knot_t::Zero();
          curve.para(0,0) = 0.0;
          curve.para(N_knots-1,0) = 1.0;
        }
        pre_curve_t pre_curve;
        set_curve(curve, pre_curve, false, false);
        VALID_SPLINE = false;
    }
    void lin_interpolation( const Eigen::Ref<ctrlpt_t> q0,
                            const Eigen::Ref<ctrlpt_t> qend,
                            const pre_curve_t &pre_curve,
                            const double t0,
                            const double T
                            ){
      ACIN_CTRL_ASSERT(N_knots > 0);
      ctrlpts_t joint_knots = ctrlpts_t::Zero();
      knot_t t_vec = knot_t::LinSpaced(N_knots, t0, t0 + T);
      for (uint16_t ii = 0; ii < N_knots; ii++){
        joint_knots.col(ii) = q0 + (qend - q0)/(N_knots-1)*(ii);
      }
      curve_t joint_curve;
      joint_curve.pts = joint_knots;
      joint_curve.para = t_vec;
      set_curve(joint_curve, pre_curve, true, false, false);
    }

    void profiled_interpolation( const Eigen::Ref<ctrlpt_t> q0_,
                            const Eigen::Ref<ctrlpt_t> qe_,
                            pre_curve_t &pre_curve_,
                            const double t0,
                            const double T_,
                            const uint16_t Np_
                            ){
      ACIN_CTRL_ASSERT(N_knots > 0);
      ACIN_CTRL_ASSERT(N_knots > Np_);
      //ACIN_CTRL_ASSERT((double)Np_*2/(double)N_knots < 0.50);
      double p = (double)Np_/(double)N_knots;
      ACIN_CTRL_ASSERT(p >= 0.0 && p <= 1.0);
      double T0_ = 0;
      ctrlpt_t q1_ = q0_ + (qe_-q0_)*p;
      double T1_ = T0_ + T_*p;//phase of linear accel
      ctrlpt_t q2_ = qe_ - (qe_-q0_)*p;
      double T2_ = T0_ + T_ - T_*p;//phase of linear accel
      double T3_ = T0_ + T_;//phase of linear accel
      ctrlpt_t vc_ = (q2_ - q1_)/(T2_-T1_);
      ACIN_CTRL_ASSERT(T1_ >T0_);
      ACIN_CTRL_ASSERT(T2_ >T1_);
      ACIN_CTRL_ASSERT(T0_+T_ > T2_);
      double Ta = T_/N_knots;
      /*##################################*/
      bool set_pre_curve_para = false;
      bool set_pre_curve_pts = false;
      if (t0 - pre_curve_.pts(pre_curve_.pts.cols()-1) > Ta*3){
        set_pre_curve_para = true;
      }else{
        if(pre_curve_.pts(pre_curve_.pts.cols()-1) > t0){
          set_pre_curve_para = true;
          set_pre_curve_pts = true;
        }
      }
      set_pre_curve_para = true;
      if(set_pre_curve_pts || set_pre_curve_para){
        for(uint16_t ii = 0;  ii < pre_curve_.pts.cols(); ii++){
          if (set_pre_curve_para){
            pre_curve_.para(ii) = t0 - Ta*(pre_curve_.pts.cols()-ii);
          }
          if (set_pre_curve_pts){
            pre_curve_.pts.col(ii) = q0_;
          }
        }
      }
      /*##################################*/
      knot_t t_vec;
      knot_t t0_vec;
      ctrlpts_t joint_knots = ctrlpts_t::Zero();
      ctrlpt_t k0 = ctrlpt_t::Constant(0);
      ctrlpt_t k1 = ((-24 * T0_ + 24 * T1_) * vc_ + 60 * q0_ - 60 * q1_) *(double)pow((double)(T0_ - T1_), (double)(-3));
      ctrlpt_t k2 = ((-84 * T0_ + 84 * T1_) * vc_ + 180 * q0_ - 180 * q1_) *(double)pow( (double)(T0_ - T1_), (double)(-4));
      ctrlpt_t k3 = ((-60 * T0_ + 60 * T1_) * vc_ + 120 * q0_ - 120 * q1_) *(double)pow( (double)(T0_ - T1_),  (double)(-5));
      for (uint16_t ii = 0; ii < Np_; ii++){
          double t = T0_ + (T1_-T0_)/(Np_-1)*ii;
          t_vec(ii) = t0 + t;
          t0_vec(ii) = t0;
          joint_knots.col(ii) = -pow(T0_, 0.5e1) * k3 / 0.20e2 + (0.15e2 * k3 * t + (5 * k2)) * pow(T0_, 0.4e1) / 0.60e2 + (-0.30e2 * k3 * t * t - 0.20e2 *k2 * t -  (10 * k1)) * pow(T0_, 0.3e1) / 0.60e2 + (0.30e2 * k3 * pow(t, 0.3e1) + 0.30e2 *  k2 * t * t + 0.30e2 *  k1 * t +  (30 * k0)) * T0_ * T0_ / 0.60e2 - t * (k3 * pow(t, 0.3e1) + 0.4e1 / 0.3e1 *  k2 * t * t + 0.2e1 *  k1 * t +  (4 * k0)) * T0_ / 0.4e1 + k3 * pow(t, 0.5e1) / 0.20e2 +  k2 * pow(t, 0.4e1) / 0.12e2 +  k1 * pow(t, 0.3e1) / 0.6e1 +  k0 * t * t / 0.2e1 + q0_;
      }



      for (uint16_t ii = Np_; ii < N_knots-Np_; ii++){
        double t = T1_+Ta +(T2_-(T1_+Ta))/(N_knots-2*Np_-1)*(ii - Np_ );
        t_vec(ii) = t0 +  t;
        t0_vec(ii) = t0;
        joint_knots.col(ii) = -T1_*vc_ + t*vc_ + q1_;
      }
      k0 = ctrlpt_t::Constant(0);
      k1 = ((-36 * T2_ + 36 * T3_) * vc_ + 60 * q2_ - 60 * qe_) * (double) pow((double) (T2_ - T3_), (double) (-3));
      k2 = ((-96 * T2_ + 96 * T3_) * vc_ + 180 * q2_ - 180 * qe_) * (double) pow((double) (T2_ - T3_), (double) (-4));
      k3 = ((-60 * T2_ + 60 * T3_) * vc_ + 120 * q2_ - 120 * qe_) * (double) pow((double) (T2_ - T3_), (double) (-5));
      ctrlpt_t kv = vc_;

      for (uint16_t ii = N_knots-Np_; ii < N_knots; ii++){
        double t = T2_ + Ta + (T3_-(T2_+Ta))/(Np_-1)*(ii - (N_knots - Np_ ));
        t_vec(ii) = t0  + t;
        t0_vec(ii) = t0;
        joint_knots.col(ii) = -pow(T2_, 0.5e1) * k3 / 0.20e2 + (0.15e2 * k3 * t +  (5 * k2)) * pow(T2_, 0.4e1) / 0.60e2 + (-0.30e2 * k3 * t * t - 0.20e2 *  k2 * t -  (10 * k1)) * pow(T2_, 0.3e1) / 0.60e2 + (0.30e2 * k3 * pow(t, 0.3e1) + 0.30e2 *  k2 * t * t + 0.30e2 *  k1 * t +  (30 * k0)) * T2_ * T2_ / 0.60e2 + (-0.15e2 * k3 * pow(t, 0.4e1) - 0.20e2 *  k2 * pow(t, 0.3e1) - 0.30e2 *  k1 * t * t - 0.60e2 *  k0 * t -  (60 * kv)) * T2_ / 0.60e2 + k3 * pow(t, 0.5e1) / 0.20e2 +  k2 * pow(t, 0.4e1) / 0.12e2 +  k1 * pow(t, 0.3e1) / 0.6e1 +  k0 * t * t / 0.2e1 +  kv * t + q2_;

      }

      curve_t joint_curve;
      joint_curve.pts = joint_knots;
      joint_curve.para = t_vec;
      set_curve(joint_curve, pre_curve_, true, false, false);

      // std::cout<<"  konts dim.:= " <<   joint_knots.rows() <<std::endl;
      // std::cout<<"  t_vec of the profiled curve:= " <<  std::setprecision(8)<<  (t_vec -t0_vec).transpose() <<std::endl;
      // std::cout<<"Set first and last knot of the profiled curve:="<<std::endl;
      // std::cout<<"  \tprofile_knots.col(0):=" <<   joint_knots.col(0).transpose() <<std::endl;
      // std::cout<<"  \tprofile_knots.col(joint_knots.cols() -1 ):=" << joint_knots.col(joint_knots.cols() -1).transpose() <<std::endl;
      // Eigen::Matrix<double,deg,1> t0_pre = Eigen::Matrix<double,deg,1>::Constant(t0);
      // std::cout<<"pre curve:"
      // std::cout<<"  \tpre_curve_t:=" <<  std::setprecision(8)<<  (pre_curve_.para -t0_pre).transpose() <<std::endl;
      // std::cout<<"  \tpre_curve_pts:=" <<  std::setprecision(8)<<  (pre_curve_.pts).transpose() <<std::endl;
      // std::vector<uint8_t> dd_vec(3);
      // std::vector<double> t_tmp_vec(1);
      // t_tmp_vec[0] = t_vec(0);
      // eval_t peval_0 = eval_t(1,3);
      // evaluate(peval_0, t_tmp_vec, dd_vec);
      // ACIN_CTRL_WARN("Evaluate set curve at the beginning and the end");
      // std::cout<<"\tq 0:="<<q0_.transpose()<<std::endl;
      // std::cout<<"\tpoint_eval 0:= "<<peval_0(0,0).transpose()<<std::endl;
      // t_tmp_vec[0] = t_vec(N_knots-1);
      // evaluate(peval_0, t_tmp_vec, dd_vec);
      // std::cout<<"\tqe_ 0"<<qe_.transpose()<<std::endl;
      // std::cout<<"\tpeval_e 0"<<peval_0(0,0).transpose()<<std::endl;

    }
    double get_t_0(){
      return this->cu_tk_(0,0);
    }
    double get_t_end(){
      return this->cu_tk_(this->cu_tk_.rows() - 1,0);
    }
    ctrlpt_t get_ctrlpt_0(){
      return this->cu_ctrlpts_.col(0);
    }
    ctrlpt_t get_ctrlpt_end(){
      return this->cu_ctrlpts_.col(this->cu_ctrlpts_.cols() - 1);
    }
    uint64_t total_curve_knots(){
      return this->cu_ctrlpts_.cols();
    }
    uint64_t pre_curve_knots(){
      return deg;
    }
    uint64_t curve_knots(){
      return N_knots;
    }
    //# init_ctrlpts :=  controll points which are lay before the controll points of set_cu_ctrlpts ->  this can be used online interpolatation -> addaptiv to messauret actual curve,
    //   e.g.  this can be usefull to set the pre control points (set_cu_ctrlpts_0_) to the actuel/real curve(postion/pose) of some vector which should be interpolated (whatever should be interpolateded online)
    void set_curve( const curve_t &curve,
                    const pre_curve_t &pre_curve,
                    bool use_pre_curve = false,
                    bool periodic = false, bool verbose = false){
      //get refences to the controller point and the curve paramter as well as for the pre curve
      const ctrlpts_t &set_cu_ctrlpts = curve.pts;
      const knot_t &set_cu_tk = curve.para;
      const pre_ctrlpts_t &pre_ctrlpts = pre_curve.pts;
      const pre_knot_t &pre_tk = pre_curve.para;
      //std::cout<<"set curve new cu_tk "<<set_cu_tk.transpose()<<std::endl;
      //std::cout<<"set curve new pre curve tk "<<pre_tk.transpose()<<std::endl;
      if(set_cu_ctrlpts.rows() != dim){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("controll points have not the defined dims. for the interpolation object!");
        #endif
      }
      if(set_cu_tk.rows() < deg + 1){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("curve must consist of atleast 'deg + 1' elements !");
        #endif
      }
      if(set_cu_tk.rows() != set_cu_ctrlpts.cols()){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("tk.rows() != ctrlpts.cols() !");
        #endif
      }
      if (pre_ctrlpts.cols() != 0){
        if(pre_ctrlpts.rows() != dim){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("pre curve control points have not the defined dims. for the interpolation object!");
          #endif
        }
        if(pre_ctrlpts.cols() != pre_tk.rows()){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("pre curve paramter and pre curve controll poinst have not the same number of points");
          #endif
        }
        if(pre_tk(pre_tk.rows()-1) > set_cu_tk(0)){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("pre curve paramter greater than the actual curve paramter");
          #endif
        }
        // for (uint64_t ii = 0; ii < pre_tk.rows()-1; ii++){
        //   if(pre_tk(ii) > pre_tk(ii+1)){
        //     #if defined(SPLINE_EXCEPTIONS)
        //       throw std::range_error("curve paramter of pre curve not increasing");
        //     #endif
        //   }
        // }
      }
      // ctrl_pts0_t ctrl_pts0; //control points left of the control point of the spline which is valid,  define the derivatives at this point to get a C^(deg) spline at the interval
      // tk0_t tk0; //knots left of the control point of the spline which is valid, define the derivatives at first point to get a C^(deg) spline at the interval
      this-> periodic_curve = periodic;
      uint64_t cu_idx0 = deg;
      ACIN_CTRL_ASSERT(deg >=1 );
      if(cu_idx0 == 0 ){
        cu_idx0 = 1;
      }
      uint64_t cu_idxN = cu_tk_.rows() - 1;

      Eigen::Matrix<double, dim,  deg> set_cu_ctrlpts_0_ = Eigen::Matrix<double, dim,  deg>::Zero();
      Eigen::Matrix<double, deg, 1> set_cu_tk_0_ = Eigen::Matrix<double, deg,  1>::Zero();
      if(static_cast<uint64_t>(set_cu_tk_0_.rows()) != cu_idx0){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("set_cu_tk_0_.cols != cu_idx0");
        #endif
      }
      if(set_cu_ctrlpts_0_.cols() != cu_idx0){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("set_cu_ctrlpts_0_.cols != cu_idx0");
        #endif
      }
      uint64_t pre_N = pre_ctrlpts.cols();
      bool default_setup = false;
      //blending of curve if we have allready set a valid curve and spline, otherwise use the new curve to define the points left of the valid spline interval
      if(VALID_SPLINE && (use_pre_curve == false)){
        uint64_t idx_blend = 0;
        /**/
        if(set_cu_tk(0) >= this->cu_tk_(cu_idx0) && set_cu_tk(0) < this->cu_tk_(cu_idxN)){
          for(uint64_t ii = cu_idx0; ii < cu_idxN; ii++){
            if(set_cu_tk(0) >= this->cu_tk_(ii) && set_cu_tk(0) < this->cu_tk_(ii + 1)){
              idx_blend = ii;
              break;
            }
          }
          #if defined(SPLINE_EXCEPTIONS)
            if(verbose){
              std::cout<< "#####Set Curve with Blending####" <<std::endl;
              std::cout<<"\tnew curve start parameter (tk):=  "<< set_cu_tk(0)<<"\n";
              std::cout<<"\tidx_blend :=  "<<idx_blend<<"\n";
              std::cout<<"\tthis->cu_tk_(idx_blend):=  "<<this->cu_tk_(idx_blend)<<"\n";
              std::cout<<"\tthis->cu_tk_(idx_blend+1):= "<<this->cu_tk_(idx_blend+1)<<"\n";
              std::cout<<"\tpre curve is not defined, using this last curve data for blending"<<std::endl;
            }
          #endif
          for(uint8_t ii = 0; ii < cu_idx0; ii++ ){
            int64_t loc_idx = idx_blend - cu_idx0 + ii;
            if( loc_idx >= 0){
              set_cu_ctrlpts_0_.col(ii) = this->cu_ctrlpts_.col(loc_idx);
              set_cu_tk_0_(ii) = this->cu_tk_(loc_idx);
            }else{
              #if defined(SPLINE_EXCEPTIONS)
                throw std::range_error("NOT implementet and should not occour! blending which is over the begining of the curvre buffer,  frist index should be min. cu_idx0 and not smaller!");
              #endif
            }
          }

        }else{
          if(set_cu_tk(0) > this->cu_tk_(cu_idxN)){
            if((cu_idxN + 1) > static_cast<uint64_t>(cu_tk_.rows()) || cu_tk_.rows() == -1 ){
              #if defined(SPLINE_EXCEPTIONS)
                throw std::range_error("Should not be the case, (cu_idxN + 1)>cu_tk_.rows()");
              #endif
            }
            #if defined(SPLINE_EXCEPTIONS)
              if(verbose){
                std::cout << "set_cu_ctrlpts_0_: " << set_cu_ctrlpts.col(0) << std::endl;
              }
            #endif
            for(uint16_t ii = 0; ii < cu_idx0; ii++){
              if((int64_t)(cu_idxN - cu_idx0 + ii) >= 0){
                set_cu_ctrlpts_0_.col(ii) = set_cu_ctrlpts.col(0);
                set_cu_tk_0_(ii) = set_cu_tk(0) - (cu_tk_(cu_idxN) - cu_tk_(cu_idxN - cu_idx0 + ii));
              }else{
                #if defined(SPLINE_EXCEPTIONS)
                  throw std::range_error("NOT implementet and should not occour, blending which is over the begining of the curve buffer!");
                #endif
              }
            }
            // std::cout<<"temp  "<<set_cu_tk_0_<<"\n";
          }else{
            default_setup = true;
          }
        }
        // std::cout<< "idx_blend:= " << idx_blend << std::endl;
        // std::cout<< "set_cu_tk:= \n\t" << set_cu_tk << std::endl;
        // std::cout<< "cu_tk_:= \n\t" << cu_tk_ << std::endl;
      }else{
        default_setup = true;
      }
      if(default_setup){
        if(use_pre_curve){
          #if defined(SPLINE_EXCEPTIONS)
            if(verbose){
              std::cout<<"\tpre curve is defined, using this data for blending"<<std::endl;
              std::cout<<"\tpre_tk\n\t\t"<<pre_tk<<std::endl;
              std::cout<<"\tpre_ctrlpts\n\t\t"<<pre_ctrlpts<<std::endl;
            }
          #endif
          for(uint16_t ii = 0; ii < cu_idx0; ii++){
            //pre curve defined, using this for blending
            int64_t pre_idx = pre_N - cu_idx0 + ii;
            if(pre_idx >= 0){
              set_cu_ctrlpts_0_.col(ii) = pre_ctrlpts.col(pre_idx);
              set_cu_tk_0_(ii) = pre_tk(pre_idx);
            }else{
              //number of points in the pre curve excceded the number of point which are set before the actual curve
              set_cu_ctrlpts_0_.col(ii) = pre_ctrlpts.col(0);
              ACIN_CTRL_ASSERT(pre_idx < 0);
              double t_1 = set_cu_tk(0);
              if(pre_N > 1){
                t_1 = pre_tk(1);
              }
              set_cu_tk_0_(ii) = pre_tk(0) + (t_1 - pre_tk(0))*(pre_idx);
            }
          }
        }else{
          uint64_t N = set_cu_ctrlpts.cols(); // number of curve control points
          //here the new cu_ctrlpts and cu_tk_ are used to define the frist values of the spline
          for(uint16_t ii = 0; ii < cu_idx0; ii++){
            //for a periodic curve it is assumed that the N point of the curve repreents the frist point ! must be set in the control point matrix
            if(periodic){
              if((int32_t)(N - 1 - cu_idx0 + ii) < 0){
                set_cu_ctrlpts_0_.col(ii) = set_cu_ctrlpts.col(0);
                set_cu_tk_0_(ii) = set_cu_tk(0) - (set_cu_tk(1) - set_cu_tk(0))*(cu_idx0 - ii);
              }else{
                set_cu_ctrlpts_0_.col(ii) = set_cu_ctrlpts.col(N - 1 - cu_idx0 + ii);
                set_cu_tk_0_(ii) = set_cu_tk(0) - ( set_cu_tk(N - 1) - set_cu_tk(N - 1 - cu_idx0 + ii) );
              }
            }else{
              set_cu_ctrlpts_0_.col(ii) = set_cu_ctrlpts.col(0);
              //curve must consist of atleast 2 elementes!
              set_cu_tk_0_(ii) = set_cu_tk(0) - (set_cu_tk(1) - set_cu_tk(0))*(cu_idx0 - ii);
            }
          }
        }
      }
      this->idx_cu = cu_idx0;
      // std::cout<<"---------------Blending--------"<<"\n";
      // std::cout<<"set_cu_tk(0)  "<<set_cu_tk(0)<<"\n";
      // std::cout<<"set_cu_tk_0_(last)  "<<set_cu_tk_0_(set_cu_tk_0_.rows()-1)<<"\n\n\n";
      // std::cout<<"set_cu_tk_0_  "<<set_cu_tk_0_<<"\n";
      // std::cout<<"---------------********--------"<<"\n";
      if(set_cu_ctrlpts.cols() != set_cu_tk.rows()){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("set_cu_ctrlpts.cols() != set_cu_tk.rows() !");
        #endif
      }
      /*---------------------
        set up the new control points of the curve
      */
      /*---------------------
        set up the new knots of the curve, and control if the time is in weakly decreasing order, otherwise throw an error
      */
      #if !defined(EIGEN_NO_MALLOC)
        if(this->cu_ctrlpts_.cols() != (set_cu_ctrlpts.cols() + cu_idx0*2)){
          this->cu_ctrlpts_.resize(dim, set_cu_ctrlpts.cols() + (cu_idx0)*2);
          this->cu_tk_.resize(set_cu_tk.rows() + (cu_idx0)*2, 1);
        }
      #else
        if(this->cu_ctrlpts_.cols() != (set_cu_ctrlpts.cols() + cu_idx0*2)){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("this->cu_ctrlpts_.cols() != (set_cu_ctrlpts.cols() + cu_idx0*2,  the curve which should be saved in the intepolator must have N points, N must be a fixed number because no dynmaic eigen allocation is allowed!");
          #endif
        }
      #endif

      /*
      */
      for(uint32_t jj = 0; jj < cu_idx0; jj++){
        this->cu_ctrlpts_.col(jj) = set_cu_ctrlpts_0_.col(jj);
        this->cu_tk_(jj) = set_cu_tk_0_(jj);
        if((int64_t)(jj) > 0){
          if(cu_tk_(jj - 1) > cu_tk_(jj)){
            #if defined(SPLINE_EXCEPTIONS)
              std::cout<<"cu_tk_(jj - 1)" << cu_tk_(jj - 1)<<std::endl;
              std::cout<<"cu_tk_(jj)" << cu_tk_(jj)<<std::endl;
              std::cout<<"set_cu_tk_0_" << set_cu_tk_0_<<std::endl;
              throw std::range_error("curve  paramter is notweakly increasing, tk(jj-1) > tk(jj)");
            #endif
          }
        }
      }
      /*
      */
      for(uint32_t jj = 0; jj < set_cu_ctrlpts.cols(); jj++){
        this->cu_ctrlpts_.col(jj + cu_idx0) = set_cu_ctrlpts.col(jj);
        this->cu_tk_(jj + cu_idx0) = set_cu_tk(jj);
        if((int64_t)(jj + cu_idx0) > 0){
          if(cu_tk_(jj + cu_idx0 - 1 ) > cu_tk_(jj + cu_idx0)){
            #if defined(SPLINE_EXCEPTIONS)
              throw std::range_error("curve  paramter is notweakly increasing, tk(jj-1) > tk(jj)");
            #endif
          }
        }
      }
      /*
      */
      for(uint32_t jj = static_cast<uint32_t>(set_cu_ctrlpts.cols()); jj < set_cu_ctrlpts.cols() + cu_idx0; jj++){
        if(periodic_curve){
          if((jj - set_cu_ctrlpts.cols() + 1) >= set_cu_ctrlpts.cols() ){
            #if defined(SPLINE_EXCEPTIONS)
              throw std::range_error("(jj - set_cu_ctrlpts.cols() + 1) >= set_cu_ctrlpts.cols() ");
            #endif
          }
          this->cu_ctrlpts_.col(jj + cu_idx0) = set_cu_ctrlpts.col(jj - set_cu_ctrlpts.cols() + 1);
          this->cu_tk_(jj + cu_idx0) = set_cu_tk(set_cu_tk.rows()-1) + (set_cu_tk(jj - set_cu_ctrlpts.cols() + 1) - set_cu_tk(0));
        }else{
          this->cu_ctrlpts_.col(jj + cu_idx0) = set_cu_ctrlpts.col(set_cu_ctrlpts.cols()-1);
          this->cu_tk_(jj + cu_idx0) = set_cu_tk(set_cu_tk.rows()-1) + (set_cu_tk(set_cu_tk.rows()-1)-set_cu_tk(set_cu_tk.rows()-2))*(jj - set_cu_ctrlpts.cols() + 1);
        }
        if(cu_tk_(jj-1) > cu_tk_(jj)){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("curve  paramter is notweakly increasing, tk(jj-1) > tk(jj)");
          #endif
        }
      }
      set_spline();
    }
    /*end of */
    /***************************************************************************


    */
    typedef Eigen::Matrix<ctrlpt_t, 1, deg + 1> eval_t;
    typedef Eigen::Matrix<double, dim, deg + 1> eval_interpolation_t;
    unsigned int evaluate(Eigen::Ref<eval_t> peval, const std::vector<double> &t, const std::vector<uint8_t> &dd_vec = {0}){
      uint64_t cu_idx0 = deg;
      if(cu_idx0 == 0){
        cu_idx0 = 1;
      }
      uint64_t cu_idxN = static_cast<uint64_t>(cu_tk_.rows()) - 1 - cu_idx0; // we need at least deg*2+deg+1 point to define the spline
      if((static_cast<uint64_t>(cu_tk_.rows()) < deg + 1 + (cu_idx0)*2) || (static_cast<uint64_t>(cu_ctrlpts_.cols()) < deg + 1 + (cu_idx0)*2)){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("curve must consit of atleast 2 elements and deg elemente before the frist element and deg elements after last elemnt!");
        #endif
      }
      if ((static_cast<uint64_t>(peval.rows())!= t.size()) || static_cast<uint64_t>(peval.cols()) < dd_vec.size()){
        #if defined(SPLINE_EXCEPTIONS)
          printf("t.size() = %lu ,dd_vec.d() = %lu\n",t.size(),dd_vec.size());
          printf("peval.rows() = %lu ,peval.cols() = %lu\n",peval.rows(),peval.cols());
          throw std::range_error("peval should be set to the correct size before hand");
        #else

        #endif
      }
      if(!VALID_SPLINE){
        for(uint16_t kk=0; kk < t.size(); kk++){
          for(uint16_t dd=0; dd < dd_vec.size(); dd++){
            peval(kk, dd) = ctrlpt_t::Zero();
          }
        }
        #if defined(SPLINE_EXCEPTIONS)
          ROS_ERROR("eval curve with no valid spline set! Return zero for the evaluation!");
        #endif
        return 0;
      }
      for(uint16_t kk=0; kk < t.size(); kk++){
        double t_eval = t[kk]; //save the desired evaluation time, actuel evaluation time can be differ, in dependency of the curve settings (periodic) or if the time t lies outside the curve interval
        uint64_t idx_cu = this->idx_cu;
        if( t_eval < sp_knots(cu_idx0) ){
          // ROS_WARN("t for spline evaluation is smaller than actuel set spline, t not allways increasing, reset internal spline");
          idx_cu = cu_idx0;
          if(this->periodic_curve){
            double cu_knot_min = this->cu_tk_(cu_idx0);
            double cu_knot_max = this->cu_tk_(cu_idxN);
            double T = cu_knot_max - cu_knot_min;
            if (t[kk] < cu_knot_min){
              double temp = cu_knot_min - t[kk];
              uint64_t N_period = (uint16_t) floor(temp/T);
              double t_adjusted = T - (temp - N_period*T);
              t_eval = t_adjusted;
              #if defined(DEBUG_SPLINE)
                ROS_INFO_STREAM("\n\t ** periodic adjusting eval time because of curve settings and t smaller then the first curve knot **"<<std::endl);
                ROS_INFO_STREAM("\n\t T:= "<< T << std::endl);
                ROS_INFO_STREAM("\n\t N_period:= "<< N_period << std::endl);
                ROS_INFO_STREAM("\n\t t:= "<< t[kk] << std::endl);
                ROS_INFO_STREAM("\n\t t_eval:= "<<t_eval<<std::endl);
              #endif
            }
          }else{
            if(t_eval < cu_tk_(cu_idx0)){
              t_eval = cu_tk_(cu_idx0);
            }
          }
        }

        if(t_eval < cu_tk_(cu_idx0)){ /*[cu_tk_(0), cu_tk_(deg - 2)] point are defined in the curve buffer so that the derivatives are well defined at the frist cruve point*/
          #if defined(SPLINE_EXCEPTIONS)
            ROS_WARN("t smaller than t_min of the curve, eval the curve at t_min");
            throw std::range_error("t_eval < cu_tk_(deg) should not happen anymore ?");
          #endif
          return 0;//error
        }else{
          if(t_eval > cu_tk_(cu_idxN)){
            t_eval  = cu_tk_(cu_idxN);
            idx_cu = cu_tk_.rows() - 1 - (deg + 1);
            idx_cu = cu_idxN;
            if((int64_t)cu_idx0 >= (int64_t)(cu_tk_.rows() - 1 - (deg + 1))){
              #if defined(SPLINE_EXCEPTIONS)
                throw std::range_error("cu_idx0 >= (int64_t)(cu_tk_.rows() - 1 - (deg + 1))?");
              #endif
              return 0;//error
            }
            if(this->periodic_curve){
              double cu_knot_min = this->cu_tk_(cu_idx0);
              double cu_knot_max = this->cu_tk_(cu_idxN);
              double T = cu_knot_max - cu_knot_min;
              double temp = t[kk] - cu_knot_max;
              uint64_t N_period = (uint64_t) floor(temp/T);

              double t_adjusted = temp - N_period*T;
              if( cu_knot_min > t_adjusted || t_adjusted > cu_knot_max){
                #if defined(SPLINE_EXCEPTIONS)
                  throw std::range_error("t_adjusted not within the interval, overflow of N_period ???");
                #endif
                return 0;//error
              }
              t_eval = t_adjusted;
              #if defined(DEBUG_SPLINE)
                ROS_INFO_STREAM("\n\t ** periodic adjusting eval time because of curve settings and t greater last curve knot **"<<std::endl);
                ROS_INFO_STREAM("\n\t T:= "<< T << std::endl);
                ROS_INFO_STREAM("\n\t N_period:= "<< N_period << std::endl);
                ROS_INFO_STREAM("\n\t t:= "<< t[kk] << std::endl);
                ROS_INFO_STREAM("\n\t t_eval:= "<<t_eval<<std::endl);
               #endif
              idx_cu = cu_idx0;
            }else{

              if(t[kk] < cu_tk_(cu_idxN) ){
                t_eval = t[kk];
              }else{
                t_eval = cu_tk_(cu_idxN);
              }
            }

          }
          while(t_eval > cu_tk_(idx_cu+1) && (idx_cu) < (static_cast<uint64_t>(cu_tk_.rows()) - 1 - (deg))){
            idx_cu++;
          }


        }
        if(idx_cu != this->idx_cu){
           #if defined(DEBUG_SPLINE)
            ROS_INFO_STREAM("-----------------------------------------------" <<std::endl);
            ROS_INFO_STREAM("-----------------------------------------------" <<std::endl);
            ROS_INFO("\nsetup new piece wise spline");
            ROS_INFO("\n\t t asked for:= %f",t[kk]);
            ROS_INFO("\n\t t eval:= %f",t_eval);
            ROS_INFO("\n\t t spline valid interval begin:= %f", sp_knots(cu_idx0));
            ROS_INFO("\n\t t spline validt next:= %f", sp_knots(cu_idx0+1));
            ROS_INFO("\n\t t idx_cu old:= %llu", this->idx_cu);
            ACIN_CTRL_ASSERT(this->idx_cu <cu_ctrlpts_.cols() and  idx_cu<cu_ctrlpts_.cols());
            ROS_INFO_STREAM("\n\t curve ctrls(idx_cu_old):="<< cu_ctrlpts_.col(this->idx_cu)<<std::endl);
            ROS_INFO_STREAM("\n\t cu_tk_(idx_cu_old):="<< cu_tk_(this->idx_cu)<<std::endl);
            ROS_INFO("\n\t t idx_cu new:= %llu", idx_cu);
            ROS_INFO_STREAM("\n\t cu_tk_(idx_cu_new):="<< cu_tk_(idx_cu)<<std::endl);
            ROS_INFO_STREAM("\n\t curve ctrls(idx_cu_new):="<< cu_ctrlpts_.col(idx_cu)<<std::endl);
          #endif
          this->idx_cu = idx_cu;
          set_spline();
          #if defined(DEBUG_SPLINE)
            ROS_INFO_STREAM("-----------------------------------------------" <<std::endl);
          #endif
        }

        double sp_knot_min = this->sp_knots[0];
        double sp_knot_max = this->sp_knots[sp_knots.rows()-1];
        double scale = (double)1.0/(double)(sp_knot_max-sp_knot_min);

        // if(idx_cu != cu_idxN){
        if( (int64_t)((int64_t) cu_tk_.rows() - (int64_t)idx_cu) < deg + 1 ){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("(cu_tk_.rows() - idx_cu) < deg + 1");
          #endif
          return 0;//error
        }
        if(t_eval < this->sp_knots[cu_idx0] || t_eval > this->sp_knots[cu_idx0+1]){
          #if defined(SPLINE_EXCEPTIONS)
            std::setprecision(std::numeric_limits<double>::digits10 + 1);
            std::cout<<"t_eval:= "<<t_eval<<std::endl;
            std::cout<<"this->sp_knots(cu_idx0+1,0):= "<< this->sp_knots(cu_idx0+1,0)<<std::endl;
            throw std::range_error("t_eval < this->sp_knots[cu_idx0] || t_eval > this->sp_knots(cu_idx0+1,0");
          #endif
          return 0;
        }
        eval_interpolation_t eval_loc;
        if(QAUT_INTERPOLATION_FLAG){
          eval_interpolation_t eval_spline_;
          eval_spline(t_eval, eval_spline_, true);
          eval_loc =  eval_spline_;
          #if defined(DEBUG_SPLINE)
            std::cout << "eval_spline_:= "<< eval_spline_.col(0).transpose()<<std::endl;
            std::cout << "eval_spline_ qp:= "<< scale*eval_spline_.col(1).transpose()<<std::endl;
            std::cout << "eval_spline_ qpp:= "<< scale*scale*eval_spline_.col(2).transpose()<<std::endl;

            //get quaternion
            ctrlpt_t q_spline =  eval_spline_.col(0);
            ctrlpt_qa_t q_qa_spline;
            set_quat(q_spline, q_qa_spline);
            //get quaternion velocity, must be scaled, uniform bpline are used for interpilation in the interval [0.,1.0]
            ctrlpt_t qp_spline = scale*eval_spline_.col(1);
            ctrlpt_qa_t qp_qa_spline;
            set_quat(qp_spline, qp_qa_spline);
            //get quaternion accelerations, must be scaled, uniform bpline are used for interpilation in the interval [0.,1.0]
            ctrlpt_t qpp_spline = scale*scale*eval_spline_.col(2);
            ctrlpt_qa_t qpp_qa_spline;
            set_quat(qpp_spline, qpp_qa_spline);
            //calculation of omega from the spline velocity
            ctrlpt_qa_t q_tmp = q_qa_spline.inverse()*qp_qa_spline;
            ctrlpt_qa_t omega_qa_spline =ctrlpt_qa_t( 2.0*(q_tmp).coeffs());
            //calculation of omega_p from the spline velocity and accelerations
            ctrlpt_qa_t q_tmp_1 = ctrlpt_qa_t(2.0*(q_qa_spline.inverse()*qpp_qa_spline).coeffs());
            ctrlpt_qa_t omega_qa_p_spline = ctrlpt_qa_t(q_tmp_1.coeffs()- 0.5*(omega_qa_spline*omega_qa_spline).coeffs());

            std::cout << "omega spline:= "<<omega_qa_spline.vec().transpose()<<std::endl;
            std::cout << "omega_w spline (should be 0):= " << omega_qa_spline.w()<<std::endl;
            std::cout << "omega_p spline:= "<<omega_qa_p_spline.vec().transpose()<<std::endl;
            std::cout << "omega_w_p spline (should be 0):= " << omega_qa_p_spline.w()<<std::endl<<std::endl;
          #endif
        }else{
          eval_spline(t_eval, eval_loc, false);
        }

        for(uint16_t ii = 0; ii < dd_vec.size(); ii++){
          uint8_t dd =dd_vec[ii];
          ctrlpt_t tmp = eval_loc.col(ii);
          if(dd < deg + 1){
            if(dd !=0 && t[kk] > this->sp_knots(sp_knots.rows()-1,0)){
              peval(kk, ii) = ctrlpt_t::Zero();
            }else{
              peval(kk, ii) = tmp*pow(scale,(double)(dd));
            }
          }else{
            peval(kk, ii) = ctrlpt_t::Zero();
          }
        }
      }
      return 1;
    }
    double get_cu_t_last(){
      return cu_tk_(cu_tk_.rows()-1);
    }
    void get_curve(curve_t &ret_curve){
      ACIN_CTRL_ASSERT(ret_curve.pts.cols() == N_knots);
      if (cu_ctrlpts_.cols() == N_knots){
        ACIN_CTRL_ASSERT(this->cu_ctrlpts_.cols() == N_knots + deg*2);
        ACIN_CTRL_ASSERT(this->cu_tk_.size() == N_knots + deg*2);
        for(uint32_t ii = 0; ii< ret_curve.pts.cols(); ii++ ){
          ret_curve.pts.col(ii) = this->cu_ctrlpts_.col(ii + deg);
          ret_curve.para(ii) = this->cu_tk_(ii + deg);
        }
      }else{
        for(uint32_t ii = 0; ii< ret_curve.pts.cols(); ii++ ){
          ret_curve.pts.col(ii)= ctrlpt_t::Zero();
          ret_curve.para(ii) = 0;
        }
      }

    }
  private:
    typedef Matrix<ctrlpt_qa_t, 1, (deg )*2 + deg + 1> ctrlptloc_qa_t;
    typedef Matrix<ctrlpt_t, 1, (deg )*2 + deg + 1> ctrlptloc_s_t;
     /*[cu_tk_(0), cu_tk_(deg - 2)] point are defined in the curve buffer so that the derivatives are well defined at the frist cruve point
     points need in dependency of the desired local deg. of the spline!
     */
    /*trajcotry buffer control points*/
    //#ifdef EIGEN_NO_MALLOC
    Eigen::Matrix<double, dim, N_knots + deg*2>  cu_ctrlpts_; // control points (y vector evalues) of the current curve which should be interpolated, rows =  dim. of the vector y, colums=
    /*trajcotry buffer "time stemps" to the control poinbts points*/
    Eigen::Matrix<double,N_knots + deg*2, 1> cu_tk_ ; //knots of the curve, curve paramter knots, e.g. it could be the time
    //#else
      //Eigen::Matrix<double, dim, -1>  cu_ctrlpts_;
      //Eigen::Matrix<double, -1, 1>  cu_tk_;
    //#endif

    ctrlptsloc_t sp_ctrlpts = ctrlptsloc_t::Zero(); // control points of the current spline, Number of control points =  degree + 1 of the piece wise polynom and (degree -1) on both side, there for the differntaion on the side should also be defined
    knotloc_t sp_knots = knotloc_t::Zero(); //sp_knots -> valid intervall of knot is [sp_knots(deg-1), sp_knots(deg)], if evaluation of the curve interpolation is done we must check if the evaluation time is in this interval
    ctrlptloc_qa_t spline_ctrl_qa;
    bool VALID_SPLINE = false;
    #if defined(USE_SUPPORTED_SPLINE)
      SplineND_ptr_t spline_obj; // shared pointer to the current piece wise spline for the interpolation
    #endif

    uint64_t idx_cu = 0; // current curve knot index, knot range of the valid spline [cu_tk_(idx_cu), cu_tk_(idx_cu+1) if dx_cu+1 < N other wise extrapolation(clamping) or peridoic continuation must be done]
    bool periodic_curve = false; // defines if peridoic continuation can be peformed by the interpolator, TODO (revisid) at the moment the last point must be the frist point or it is assumed it is near it !
    //evaluating the spline, the x value must be normalized for the evaluation, Eigen implemnation of the spline
    // uint64_t num_B_spline_knots = 2 + (deg - 1)*2;
    // uint64_t num_B_spline_deg = deg;
    // typedef Eigen::Matrix<double, num_B_spline_knots, 1> B_t;
    // void b_spline_basis(double u, const knots, B_t &B){
    //
    // }
    double norm(double x, double x_min, double x_max) {
        double x_norm;
        x_norm = (x - x_min)/(x_max - x_min);
        return x_norm;
    }
    //normalize of the spline knots must be done for setting up the eigen spline interpolator
    template <uint64_t N> Eigen::Matrix<double,N,1> normalize(const Eigen::Ref<Eigen::Matrix<double,N,1>> x) {
        Eigen::Matrix<double,N,1> x_norm;
        #if defined(SPLINE_EXCEPTIONS)
          if (x.size() != x_norm.size()){
            throw std::range_error("(this->idx_cu > cu_idxN) || (this->idx_cu < cu_idx0)?");
          }
        #endif
        const double min = x.minCoeff();
        const double max = x.maxCoeff();
        for (int k = 0; k < x.size(); k++) {
          x_norm(k) = (x(k) - min)/(max - min);
        }
        return x_norm;
    }
    bool new_spline_=false;
    unsigned int set_spline(){
      uint64_t idx_cu = this->idx_cu;
      uint64_t cu_idx0 = deg;
      // if(cu_idx0 ==0){
      //   cu_idx0 = 1;
      // }
      // uint64_t cu_idxN = cu_tk_.rows() - 1 - (cu_idx0);

        if((this->idx_cu > (static_cast<uint64_t>(cu_tk_.rows()) - (deg + 1))) || (this->idx_cu < cu_idx0)){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("(this->idx_cu > cu_idxN) || (this->idx_cu < cu_idx0)?");
          #endif
          return 0;
        }
        if(cu_ctrlpts_.cols() != cu_tk_.rows()){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("cu_ctrlpts_.cols() != cu_tk_.rows()");
          #endif
          return 0;
        }
        if(sp_knots.rows() != cu_idx0*2 + deg+1){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("sp_knots.rows() == cu_idx0*2 + deg+1");
          #endif
          return 0;
        }
        if(sp_ctrlpts.cols() != sp_knots.rows()){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("sp_ctrlpts.cols() != sp_knots.rows()");
          #endif
          return 0;
        }
      //
      for(uint16_t ii = 0; ii < static_cast<uint16_t>(sp_knots.rows()); ii++){

        if(((int64_t)(idx_cu - cu_idx0 + ii) < 0)){
          #if defined(SPLINE_EXCEPTIONS)
            std::cout<<"idx_cu"<<idx_cu<<std::endl;
            throw std::range_error("(int64_t)(idx_cu - cu_idx0 + ii) < 0) || ((int64_t)(idx_cu - cu_idx0 + ii) >= cu_ctrlpts_.cols())");
          #endif
          return 0;
        }
        if((int64_t)(idx_cu - cu_idx0 + ii) >= (int64_t)cu_ctrlpts_.cols()){
          if(periodic_curve){
            int64_t kk = ( (idx_cu - cu_idx0 + ii)) - cu_ctrlpts_.cols() + 1;
            sp_knots(ii,0) = cu_tk_(cu_ctrlpts_.cols()-1) + (cu_tk_(kk) - cu_tk_(0));
            sp_ctrlpts.col(ii) = cu_ctrlpts_.col(kk);
          }else{
            int64_t kk = ( (idx_cu - cu_idx0 + ii)) - cu_ctrlpts_.cols() + 1;
            sp_ctrlpts.col(ii) = cu_ctrlpts_.col(cu_ctrlpts_.cols()-1);

            sp_knots(ii,0) = cu_tk_(cu_ctrlpts_.cols()-1) + (cu_tk_(cu_ctrlpts_.cols()-1)-cu_tk_(cu_ctrlpts_.cols()-2))*kk;
            // ROS_ERROR("idx_cu:= %d",idx_cu);
            // ROS_ERROR("last_= %d",cu_ctrlpts_.cols() - deg - 1);
            // ROS_ERROR("Should not happen, last points of the curve should be set correctly, Number of cruve points (set_cu_tk.rows() + (cu_idx0)*2)) := %ld (cu_idx0:=%ld)",cu_tk_.rows(),cu_idx0);
            // std::cout<<"kk:= "<<kk<<std::endl;
            // std::cout<<"sp_knots"<<sp_knots.transpose()<<std::endl;
          }
        }else{
          sp_ctrlpts.col(ii) = cu_ctrlpts_.col(idx_cu - cu_idx0 + ii);
          sp_knots(ii,0) = cu_tk_(idx_cu - cu_idx0 + ii);
        }
      }
      #if defined(DEBUG_SPLINE)
        ROS_INFO_STREAM("\n\t **new spline**");
        ROS_INFO_STREAM("\n\t spline knots (x values):= "<<std::endl);
        ROS_INFO_STREAM("\n\t "<< sp_knots.transpose()<<std::endl);
        ROS_INFO_STREAM("\n\t spline control pts.:="<<std::endl);
        ROS_INFO_STREAM("\n\t "<< this->sp_ctrlpts<<std::endl);
      #endif
      const auto knots_n = normalize<(deg)*2 + deg + 1>(sp_knots);  // Normalize x to be between 0 and 1
      #if defined(USE_SUPPORTED_SPLINE)
        SplineND_t spline_obj = SplineNDFitting_t::Interpolate(sp_ctrlpts, deg, knots_n);
        this->spline_obj = std::make_shared<SplineND_t>(spline_obj);
      #endif
      VALID_SPLINE = true;

      if(dim == 4){
        ctrlpt_qa_t q_ii;
        ctrlpt_qa_t q_lii;
        ctrlpt_qa_t q_rii;
        ctrlpt_qa_t Q_ii;
        for(uint64_t ii = 0; ii < sp_ctrlpts.cols(); ii++){
          set_quat(sp_ctrlpts.col(ii),Q_ii);
          Q_ii.normalize();
          spline_ctrl_qa(0, ii) = Q_ii;
        }
        new_spline_ =true;
      }
      return 1;
    }
    void quat_exp(ctrlpt_qa_t &q, ctrlpt_qa_t &q_exp){
      double a = q.w();
      double v_norm = q.vec().norm();
      double exp_a = std::exp(a);
      double cos_v_norm = std::cos(v_norm);
      double sin_v_norm = 0.0;
      if (v_norm != 0.0){
        sin_v_norm =  std::sin(v_norm)/v_norm;
        q_exp.vec() = exp_a*sin_v_norm*q.vec() ;
        q_exp.w() = exp_a*cos_v_norm;
      }else{
        q_exp = ctrlpt_qa_t(1.0, 0.0, 0.0, 0.0);
      }

    }
    void quat_ln(ctrlpt_qa_t &q, ctrlpt_qa_t &q_exp){
      q.normalize();
      double a = q.w();
      double v_norm = q.vec().norm();
      double q_norm = q.norm();
      double arcos_a_vnorm = 0.0;
      q_exp.w() = 0.0;
      double theta = std::atan2(v_norm, a);
      if(v_norm != 0.0){
        q_exp.vec() = theta/v_norm*q.vec();
      }else{
        q_exp.w() = 1.0;
        q_exp.vec() = 0.0*q.vec();
      }
    }
    void quat_pot(double r, ctrlpt_qa_t &q, ctrlpt_qa_t &q_exp){
      ctrlpt_qa_t q_ln;
      quat_ln(q, q_ln);
      ctrlpt_qa_t q_tmp;
      q_tmp = ctrlpt_qa_t(q_ln.coeffs()*r);
      quat_exp(q_tmp,q_exp);
    }
    void quat_pot_v2(double r, ctrlpt_qa_t &q, ctrlpt_qa_t &q_exp){
      q.normalize();
      double norm_v = q.vec().norm();
      if(norm_v == 0.0){
        q_exp.w() = 1.0;
        q_exp.vec() = 0.0*q.vec();
      }else{
        double phi = std::atan2(norm_v, q.w());
        q_exp.w() = std::cos(phi*r);
        q_exp.vec() = 1/norm_v*q.vec()*std::sin(phi*r);
      }
      // q_exp.normalize();
    }
    void quat_pot(double r, ctrlpt_t &q, ctrlpt_t &q_exp){
      double w = q(3,0);
      Vector3d v_tem(q(0,0),q(1,0),q(2,0));
      double norm_v = v_tem.norm();
      double phi = std::atan2(norm_v, w);
      if (norm_v == 0.0){
        q_exp(3,0) = 1.0;
        q_exp(2,0) = 0.0;
        q_exp(1,0) = 0.0;
        q_exp(0,0) = 0.0;
      }else{
        double sin_phi_norm = 1/norm_v*std::sin(phi*r);
        q_exp(3,0) = std::cos(phi*r); //quat w, realpart
        q_exp(2,0) = q(2,0)*sin_phi_norm; //quat z, im part
        q_exp(1,0) = q(1,0)*sin_phi_norm; //quat y, im part
        q_exp(0,0) = q(0,0)*sin_phi_norm; //quat x, im part
      }
    }

    typedef Eigen::Matrix<double, deg + 1, (deg)*2 + deg + 1> Basis_t;
    typedef Eigen::Matrix<double, deg + 1, deg + 1> Basis_loc_t;
    /* find span for the bspline basis_fcn, span index define the intervalls on which the basis function on the knots t_n for t  are non zeros
      span -> define intervall i in which t is t_n(ii) <= t < t_n(ii+1), therefore (i - deg) basis function are non zero for this interval
      Basis functicon of degree deg = p, B_(i-p, p), B_(i - p + 1,p), .....,B(i, p) -> for p = 3 (cubic BSpline), 4 bais function are non zero for the intervall i
      */
    uint64_t find_span( double t, knotloc_t &t_n, uint16_t Bspline_deg){
      uint64_t p = Bspline_deg;
      uint64_t N = t_n.rows(); // knot point of the curve
      if(N < deg + 1){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("find_span: number of knots should be N >= deg + 1 for Bsplines!");
        #endif
        return N + 1;
      }
      if (t == t_n(N - 1)){
           return N - 1;
      }
      uint64_t low = p;
      // if(t < t_n(low)){
      //   return p;
      // }
      uint64_t high  = N;
      uint64_t mid = (uint64_t)(((double)low+(double)high)/2.0);
      while(t < t_n(mid) || t >= t_n(mid+1)){
          if (t < t_n(mid)){
              high = mid;
          }else{
              low = mid;
          }
          mid = (uint64_t)(((double)low+(double)high)/2.0);
          if(high==low){
            ACIN_CTRL_WARN("<find_span> high == low (%lu),t:=%.5f, t_min:=%.5f, t_max:=%.5f",mid, t=0, t_n(0),t_n(N-1));
            break;
          }
      }
      return mid;
    }
    // see NURBS-Book for information how do construct
    // the type Basis_loc_t define a matrix of bais functions and there derivatives which are non zero
    // --- rows are the derivative of the bspline functions up to derivative deg = p,
    // B_ret.row() row zero (row_0) are the bspline function values, row one are the frist derivative and so on...
    // colums define the Bspline Basis function values ( degree p) or the derivate,
    // B_ret(0,0) =  N(i,p); B_ret(0,1) =  N(i-1,p);...; B_ret(0,p) =  N(i-p,p)
    // the basis function are  only in range of span non zero, (deg + 1) elements are non-zero,
    // span defines the knot idx ii in t_n in which t lies, t_n(ii)<=t<t_n(ii+1)
    // in which b spline basisfunction are non zero, t_n(span - 1) < t <= t_n(span),
    // Algorithm used for generation from the NURBS-BOOK (Algorithm A2.3; page: 72)
    void B_splinev3(double t, knotloc_t &t_n,
                    Basis_loc_t &B_ret)
    {
      typedef Eigen::Matrix<double, deg + 1, 1> lvec_t;

      uint64_t N = static_cast<uint64_t>(t_n.rows()); //number of knots of the curve, local curve of the whole curve where we fit Bsplines
      uint16_t M = static_cast<uint16_t>(B_ret.cols()) - 1;  //degree of the B splines
      Basis_loc_t B = Basis_loc_t::Zero();
      if(B.rows() != B.cols() || M != deg){

      }
      B(0,0) = 1.0;
      lvec_t uleft = lvec_t::Zero();
      lvec_t uright = lvec_t::Zero();

      uint64_t span = find_span(t,t_n, M);
      //calculate the value of  bslpline basis functions which are non zero in the current interval t_n(span-1) <t<= t_n(span)
      for(uint64_t jj = 1; jj < M +1; jj++){
          uleft(jj) = (double)t - (double)t_n(span + 1 - jj);
          uright(jj) = (double)t_n(span + jj) - (double)t;
          double saved = 0.0;
          for (uint64_t rr = 0; rr < jj; rr++){
              B(jj,rr) = uright(rr+1) + uleft(jj - rr);
              double temp = (double) B(rr, jj-1)/(double)B(jj, rr);
              B(rr, jj) = saved + uright(rr+1)*temp;
              saved =  uleft(jj - rr)*temp;
          }
          B(jj, jj) = saved;
          // return B[:,-1
      }
      //save basisfucntion values into row 0 of the return matrix
      for (uint64_t jj = 0; jj < M +1; jj++){
        B_ret(0,jj) = B(jj, M);
      }
      //tmp matrix
      typedef Eigen::Matrix<double, 2 , deg + 1> tmp_matA_t;
      tmp_matA_t a = tmp_matA_t::Zero();
      //calculate bslpline derivatives up to deg
      for (uint64_t rr = 0; rr < M + 1; rr++){
        uint8_t s1 = 0;
        uint8_t s2 = 1;
        a(0,0) = 1.0;
        for (uint64_t kk = 1; kk < M + 1; kk++){
          double d = 0.0;
          int64_t rk = rr - kk;
          int64_t pk = M - kk;
          int64_t j1 = 0;
          int64_t j2 = 0;
          if(rr >= kk){
              a(s2,0) = a(s1,0)/B(pk+1,rk);
              d = a(s2,0)*B(rk,pk);
          }
          if(rk >= (int32_t)-1){
              j1 = 1;
          }else{
              j1 = -rk;
          }
          if((int32_t)rr - (int32_t)1 <= pk){
              j2 = (int32_t)kk - 1;
          }else{
              j2 = (int32_t)M - (int32_t)rr;
          }
          for (int32_t jj = j1; jj < j2 + 1; jj++){
              a(s2,jj) = (a(s1,jj)-a(s1,jj-1))/B(pk+1,rk+jj);
              d = d + a(s2,jj)*B(rk+jj,pk);
          }
          if((int32_t)rr <= pk){
              a(s2,kk) = -a(s1, kk - 1)/B(pk + 1, rr);
              d = d + a(s2, kk)*B(rr, pk);
          }
          B_ret(kk,rr) = d;
          //swap indices
          uint8_t swap = s1;
          s1 = s2;
          s2 = swap;
        }
      }
      //mutiple the scaleing of the derivatives, based on recurvsive definiation/formular
      double rr = (double)M;
      for(uint64_t kk = 1; kk < M+1; kk++){
        for (uint64_t jj = 0; jj < M+1; jj++){
          B_ret(kk,jj) = B_ret(kk,jj)*rr;
        }
        rr = rr*(M-kk);
      }
    }
    Basis_loc_t basis = Basis_loc_t::Zero();
    Basis_t basis_ = Basis_t::Zero();
    uint8_t eval_spline(double t, Eigen::Ref< eval_interpolation_t > eval_loc, bool spherical = false){
      double sp_knot_min = this->sp_knots[0];
      double sp_knot_max = this->sp_knots[sp_knots.rows()-1];
      double t_norm = norm(t, sp_knot_min, sp_knot_max);
      //std::cout<< "t_norm:= " << std::setprecision(std::numeric_limits<long double>::digits10 + 1)<<t_norm<<std::endl;
      //std::cout<< "t:= " << std::setprecision(std::numeric_limits<long double>::digits10 + 1)<<t<<std::endl;
      //std::cout<< "sp_knot_min:= " << std::setprecision(std::numeric_limits<long double>::digits10 + 1)<<sp_knot_min<<std::endl;
      //std::cout<< "sp_knot_max:= " << std::setprecision(std::numeric_limits<long double>::digits10 + 1)<<sp_knot_max<<std::endl;
      #if defined(USE_SUPPORTED_SPLINE)
        SplineND_t &spline_obj_r = *this->spline_obj;
      #endif

      uint64_t sp_idx0 = deg;
      knotloc_t knots_n = normalize<(deg)*2 + deg + 1>(sp_knots);
      if(sp_idx0 == 0){
        sp_idx0 = 1;
      }
      #if defined(SPLINE_EXCEPTIONS)
        // if (t_norm < knots_n[sp_idx0]){
        //   printf("#####################\n");
        //   printf("#####################\n");
        //   printf("t_norm:=%f\n",t_norm);
        //   std::cout<<"knots_n"<<knots_n.transpose()<<std::endl;
        //   printf("#####################\n");
        //   printf("#####################\n");
        // }
        assert(t >= this->sp_knots[sp_idx0]);
      #endif
      //find span for the bspline basis_fcn, span index defines the intervalls on which the basis function on the knots t_n for t  are non zeros
      uint64_t span = find_span(t_norm, knots_n, (uint16_t)deg);
      B_splinev3(t_norm, knots_n, basis);


      //basis function enroll on all loacal spline interpolation points, spline interpolation points(well defined)=
      // (deg - 1)*2 + deg + 1, min. deg-1 points left evaluation point t (t >= this->sp_knots[deg-1])
      ACIN_CTRL_ASSERT(span >= deg);
      uint64_t idx_start = span;
      uint64_t idx_end  = span + deg + 1;
      for(uint64_t ii = 0; ii < deg + 1; ii++){
        for(uint64_t jj = 0; jj < deg + 1; jj++){
          basis_(jj, span + ii) = basis(jj, ii);
        }
      }
      if(spherical){
        #if defined(DEBUG_SPLINE_QAUT)
          std::cout<<"----- quat spline evaluation ------"<< t_norm <<"\n\n";
          // std::cout<<"span:= "<<  span <<"\n\n";
          // std::cout<<"idx_start:= "<<  idx_start <<"\n\n";
          // std::cout<<"idx_end:= "<<  idx_end <<"\n\n";
          // std::cout<<"t_norm= "<< t_norm <<"\n\n";
          // std::cout<<"knots_n= "<< knots_n.transpose() <<"\n\n";
          // std::cout<<"sp_knots= "<< spline_obj_r.knots()<<"\n\n";
          // std::cout<<"t= "<< t <<"\n\n";
          // std::cout<<"sp_knots= "<< sp_knots.transpose() <<"\n\n";
          // std::cout<<"sp_ctrlpts= "<< sp_ctrlpts <<"\n\n";
          std::cout<<"sp_ctrlpts angles= "<< sp_ctrlpts <<"\n\n";
          for(uint64_t ii=0; ii<sp_ctrlpts.cols()-1;ii++){
            ctrlpt_qa_t q_ii = spline_ctrl_qa(0,ii);
            ctrlpt_qa_t q_iin = spline_ctrl_qa(0,ii);
            ctrlpt_qa_t q_rel = q_ii.conjugate()*q_iin;
            double angle = 2*atan2(q_rel.vec().norm(), q_rel.w());
            printf("%f,",angle);
          }
          printf("\n");
         #endif
        if((int64_t) idx_start < 0 || (int64_t) idx_end > sp_knots.rows()){
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("eval_spline qauternion: start index of the first basis function or end index of last basis out of range!");
          #endif
          return 0;
        }

        #if defined(DEBUG_SPLINE_QAUT)
          if(new_spline_){
            std::cout<<"span"<< span<<"\n";
            std::cout<<"t_norm= "<< t_norm <<"\n\n";
            std::cout<<"knots_n= "<< knots_n.transpose() <<"\n\n";
            std::cout<<"basis.rows() "<< basis.rows()<<"\n";
            std::cout<<"basis.cols() "<< basis.cols()<<"\n";
            std::cout<<"basis:= \n\t"<< basis<<"\n";
            std::cout<<"basis_:= \n\t"<< basis_<<"\n";
          }
        #endif
        /*###########################*/
        /*###########################*/
        /* saving the interpolation values for the quaternion and there derivatives, q, qp, qpp*/
        ctrlpt_qa_t q_intepl__ (1,0,0,0);
        ctrlpt_qa_t q_intepl_dot_(0,0,0,0);
        ctrlpt_qa_t q_intepl_dot_dot_(0,0,0,0);
        /*###########################*/
        /*tmp variables for the quaternion interpolation */
        /*###########################*/
        ctrlpt_qa_t q_tmp;
        ctrlpt_qa_t q_temp_dot_(1,0,0,0);
        ctrlpt_qa_t q_temp_dot_dot_(1,0,0,0);
        //saves values allready calculatued during the interpolation proccess, quaternion interpolation calculates the potenz (q_ii^B_ii(t))(eval_temp(0,ii)) which is also need for the velocity of ther quaternion
        //in eval_temp(1,ii) the derivative of the quaternion potenz ii (diff(q_ii^B_ii(t),t)) are saved for the acceleration calculation
        Eigen::Matrix<ctrlpt_qa_t, 2, (deg)*2 + deg + 1 > eval_temp;
      //*********************************************************************
        // interpolation of the quaternion
        //*********************************************************************
        for(uint64_t ii = idx_start; (ii < idx_end); ii++){
          ctrlpt_qa_t q_ii = spline_ctrl_qa(0, ii);
          double b_ii_kk = basis_(0,ii);
          ctrlpt_qa_t q_ii_pot;
          quat_pot(b_ii_kk, q_ii, q_ii_pot);
          eval_temp(0,ii) = q_ii_pot;
          q_intepl__ = q_intepl__*q_ii_pot;
        }
        // std::cout<<"\nņq_intepl__\n\tw:="<<q_intepl__.w()<<"\n\tvec:="<<q_intepl__.vec()<<std::endl;
        //*********************************************************************
        // interpolation of the quaternion velocity
        //*********************************************************************
        for(uint64_t jj = idx_start; (jj < idx_end); jj++){
          q_temp_dot_ = ctrlpt_qa_t(1,0,0,0);
          for(uint64_t ii =idx_start; (ii < idx_end); ii++){
            if (ii == jj){
              double b_dot_ii_kk = basis_(1,jj);
              ctrlpt_qa_t q_ii_ln;
              ctrlpt_qa_t q_ii = spline_ctrl_qa(0, ii);
              quat_ln(q_ii, q_ii_ln);
              q_tmp = ctrlpt_qa_t((eval_temp(0,ii)*q_ii_ln).coeffs()*b_dot_ii_kk);
              eval_temp(1,ii) = q_tmp;
            }else{
              q_tmp = eval_temp(0,ii);
            }
            q_temp_dot_ = q_temp_dot_*q_tmp;
          }
          q_intepl_dot_ = ctrlpt_qa_t(q_intepl_dot_.coeffs() + q_temp_dot_.coeffs());
        }
        // std::cout<<"\nq_intepl_dot_\n\tw:="<<q_intepl_dot_.w()<<"\n\tvec:="<<q_intepl_dot_.vec()<<std::endl;
        //*********************************************************************
        //*********************************************************************
        // interpolation of the quaternion accelerations
        //*********************************************************************
        for(uint64_t jj = idx_start; (jj < idx_end); jj++){
          for(uint64_t ll = idx_start; (ll < idx_end); ll++){
            q_temp_dot_dot_ = ctrlpt_qa_t(1,0,0,0);
            for(uint64_t ii = idx_start; (ii < idx_end); ii++){
              if (ii == jj && ii == ll){
                double b_dot_ii_kk = basis_(1,ii);
                double b_dot_dot_ii_kk = basis_(2,ii);
                ctrlpt_qa_t q_ii_ln;
                ctrlpt_qa_t q_ii = spline_ctrl_qa(0, ii);
                quat_ln(q_ii, q_ii_ln);
                double b_dot_ii_kk_qaut = b_dot_ii_kk*b_dot_ii_kk;
                ctrlpt_qa_t qtmp_qa_1 = eval_temp(0,ii)*q_ii_ln;
                qtmp_qa_1 = qtmp_qa_1 * q_ii_ln;
                ctrlpt_qa_t qtmp_1 = ctrlpt_qa_t(b_dot_ii_kk_qaut*((qtmp_qa_1).coeffs()));
                ctrlpt_qa_t qtmp_2 = ctrlpt_qa_t(b_dot_dot_ii_kk*((eval_temp(0,ii)*q_ii_ln).coeffs()));
                q_tmp = ctrlpt_qa_t(qtmp_1.coeffs() + qtmp_2.coeffs());
              }else{
                if(ii == ll || ii == jj){
                  q_tmp = eval_temp(1,ii);
                }else{
                  q_tmp = eval_temp(0,ii);
                }
              }
              q_temp_dot_dot_ = q_temp_dot_dot_*q_tmp;
            }
            q_intepl_dot_dot_ = ctrlpt_qa_t(q_intepl_dot_dot_.coeffs() + q_temp_dot_dot_.coeffs());
          }
        }
        //*********************************************************************
        ctrlpt_t retun_q;
        get_quat(q_intepl__, retun_q);
        eval_loc.col(0) = retun_q;
        get_quat(q_intepl_dot_, retun_q);
        eval_loc.col(1) = retun_q;
        get_quat(q_intepl_dot_dot_, retun_q);
        eval_loc.col(2) = retun_q;
        new_spline_=false;
      }else{
        for(uint64_t ii_deg = 0; ii_deg < deg; ii_deg++){
          ctrlpt_t eval_tmp = ctrlpt_t::Zero();
          for(uint64_t ii = idx_start; (ii < idx_end); ii++){
            ctrlpt_t q_ii = sp_ctrlpts.col(ii);
            ACIN_CTRL_ASSERT(ii_deg < basis_.rows());
            double b_ii_kk = basis_(ii_deg, ii);
            eval_tmp = eval_tmp + b_ii_kk*q_ii;
          }
          eval_loc.col(ii_deg) = eval_tmp;
        }
        #if defined(USE_SUPPORTED_SPLINE)
          eval_interpolation_t eval_spline_obj = spline_obj_r.derivatives(t_norm, (int32_t) deg);
          double e_intrpl_norm = (eval_spline_obj.col(0)-eval_loc.col(0)).norm();
          if(e_intrpl_norm>0.001){
            //ACIN_CTRL_ERROR("e_intrpl_norm error:= %f",e_intrpl_norm);
          }
          eval_loc = eval_spline_obj;
        #endif


      }
      return 0;
    }
    void set_quat(const Eigen::Ref<ctrlpt_t> q, ctrlpt_qa_t &quat){
      Eigen::Matrix<double,4,1> tmp = Eigen::Matrix<double,4,1>::Zero();
      for(uint8_t ii = 0; ii < SAT_4(dim); ii++){
        tmp(ii,0) = q(ii,0);
      }
      quat = tmp;
    }
    void get_quat(const ctrlpt_qa_t &quat, Eigen::Ref<ctrlpt_t> q){
      Eigen::Matrix<double,4,1> tmp = quat.coeffs();
      for(uint8_t ii = 0; ii < dim; ii++){
        if(ii < 4){
          q(ii, 0) = tmp(ii,0);
        }else{
          q(ii, 0) = 0;
        }
      }
    }
    void eval_cubic_Bezier_interpolation(  double t,
                                      Eigen::Ref<eval_interpolation_t> eval,
                                      bool spherical = false)
    {
      uint64_t sp_idx0 = deg;
      double k = 0.999;//factor to print the addentional bezier_curve control points near the control points of the curve
      if(sp_idx0 == 0){
        sp_idx0 = 1;
      }
      uint64_t sp_idxN = sp_knots.rows();
      if(t < sp_knots(1) || t > sp_knots(sp_knots.rows() - 1)){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("cubic_Bezier_interpolation: t must be in range [sp_knots(1), sp_knots(sp_knots.rows()-1]");
        #endif
      }
      if(spherical){
        if(dim !=4){ //in terms of quaternion ctrlpt_t is a 4D vectors in (x,y,z,w)!!
          #if defined(SPLINE_EXCEPTIONS)
            throw std::range_error("cubic_Bezier_interpolation: sphreical interpolation only avaiable for quaternion (dim ==4)");
          #endif
        }
      }else{
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("cubic_Bezier_interpolation: only sphreical interpolation with Bezier_interpolation is implemented");
        #endif
      }
      uint64_t nn = 1;
      if((nn + 2) >= sp_knots.rows()){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("nn + 2 >= sp_knots.rows()");
        #endif
      }
      // ctrlpt_t q_n0 = sp_ctrlpts(sp_idx0 - 1);
      // ctrlpt_t q_n1 = sp_ctrlpts(sp_idx0);
      // ctrlpt_t q_n2 = sp_ctrlpts(sp_idx0 + 1);
      // ctrlpt_t a_n0 = ctrlpt_t::Zero();
      // ctrlpt_t a_n1 = ctrlpt_t::Zero();
      // ctrlpt_t b_n1 = ctrlpt_t::Zero();
      /***********************************************************************/

      ctrlpt_qa_t q_n0_qa;
      set_quat(sp_ctrlpts.col(nn), q_n0_qa);
      ctrlpt_qa_t q_n1_qa;
      set_quat(sp_ctrlpts.col(nn + 1), q_n1_qa);
      ctrlpt_qa_t q_n2_qa;
      set_quat(sp_ctrlpts.col(nn + 2), q_n2_qa);
      ctrlpt_qa_t a_n0_qa = ctrlpt_qa_t(1,0,0,0);
      ctrlpt_qa_t a_n1_qa = ctrlpt_qa_t(1,0,0,0);
      ctrlpt_qa_t b_n1_qa = ctrlpt_qa_t(1,0,0,0);

      // std::cout<<"\tq_n0_qa:="<<q_n0_qa.w()<<", "<<q_n0_qa.vec().transpose()<<"--phi:="<<std::acos(q_n0_qa.w())*2.0*180.0/3.14<<std::endl;
      // std::cout<<"\tq_n1_qa:="<<q_n1_qa.w()<<", "<<q_n1_qa.vec().transpose()<<"--phi:="<<std::acos(q_n1_qa.w())*2.0*180.0/3.14<<std::endl;
      // std::cout<<"\tq_n2_qa:="<<q_n2_qa.w()<<", "<<q_n2_qa.vec().transpose()<<"--phi:="<<std::acos(q_n2_qa.w())*2.0*180.0/3.14<<std::endl;
      ctrlpt_qa_t b_n1_qa_;
      /**/
      if(spherical){
          q_n0_qa.normalize();
          q_n1_qa.normalize();
          q_n2_qa.normalize();
          ctrlpt_qa_t q_tmp = q_n1_qa*q_n0_qa;//"q_[0]*q_[1] sum up the orentation quat mul"
          q_tmp.normalize();
          a_n1_qa = q_n2_qa.slerp(0.5, q_tmp);// averaging orentation to the next one
          a_n1_qa = q_n1_qa.slerp((1.0-k),a_n1_qa);
          a_n1_qa.normalize();
          double tmp_1 = 2*(a_n1_qa.dot(q_n1_qa));
          double tmp_2 = 2*(q_n2_qa.dot(q_n1_qa));
          ctrlpt_qa_t q_tmp_1 = ctrlpt_qa_t(tmp_1*q_n1_qa.coeffs ());
          ctrlpt_qa_t q_tmp_2 = ctrlpt_qa_t(tmp_2*q_n1_qa.coeffs ());
          b_n1_qa = ctrlpt_qa_t(q_tmp_1.coeffs () - a_n1_qa.coeffs ());
          b_n1_qa = b_n1_qa.slerp(k, q_n1_qa);
          a_n0_qa = ctrlpt_qa_t(q_tmp_2.coeffs () - q_n2_qa.coeffs ());
          a_n0_qa = q_n0_qa.slerp((1.0-k),a_n0_qa);
          b_n1_qa.normalize();
          a_n0_qa.normalize();
      }else{
          // a_n1 = (q_n0 + q_n1 + q_n2)/2
          // b_n1 = q_n1+(q_n1-a_n1)
          // b_n1 = q_n1 + k *(b_n1 - q_n1)
          // a_n1 = q_n1 + k *(a_n1 - q_n1)
          // a_n0 = q_n1 + (q_n1 - q_n2)
      }
      // std::cout<<"\ta_n0_qa:="<<a_n0_qa.w()<<", "<<a_n0_qa.vec().transpose()<<"--phi:="<<std::acos(a_n0_qa.w())*2.0*180.0/3.14<<std::endl;
      // std::cout<<"\ta_n1_qa:="<<a_n1_qa.w()<<", "<<a_n1_qa.vec().transpose()<<"--phi:="<<std::acos(a_n1_qa.w())*2.0*180.0/3.14<<std::endl;
      // std::cout<<"\tb_n1_qa:="<<b_n1_qa.w()<<", "<<b_n1_qa.vec().transpose()<<"--phi:="<<std::acos(b_n1_qa.w())*2.0*180.0/3.14<<std::endl;

      for(uint64_t ii = nn; ii < sp_knots.rows() - 2; ii++){
        if(t > sp_knots(ii + 1)){
          if(spherical){
            q_n0_qa = q_n1_qa;
            a_n0_qa = a_n1_qa;
            q_n1_qa = q_n2_qa;
            if (ii < sp_knots.rows() - 3){
              set_quat(sp_ctrlpts.col(ii + 2), q_n2_qa);
              ctrlpt_qa_t q_tmp = q_n0_qa*q_n1_qa;//#"q_[0]*q_[1] sum up the orentation"
              q_tmp.normalize();
              a_n1_qa = q_tmp.slerp(0.5, q_n2_qa);//#  averaging orentation to the next one
              a_n1_qa = q_n1_qa.slerp(1-k,a_n1_qa);
              b_n1_qa = ctrlpt_qa_t(2*a_n1_qa.dot(q_n1_qa)*q_n1_qa.coeffs() - a_n1_qa.coeffs());
              b_n1_qa = b_n1_qa.slerp(k,q_n1_qa);
              b_n1_qa.normalize();
              a_n1_qa.normalize();
            }else{
              //ii is sp_knots.rows() - 3, so next will be ii = sp_knots.rows() - 2 which leads to exit of the loop
              if(t > sp_knots(ii + 2)){
                #if defined(SPLINE_EXCEPTIONS)
                  throw std::range_error("cubic_Bezier_interpolation: ??");
                #endif
              }
              // q_n2not needed anymore, loop must terminte now ! after the last point there are no more points
              // a_n1_qa also dont matter
              b_n1_qa = a_n0_qa.slerp( (double)0.9, q_n1_qa);//#  averaging orentation to the next one
            }
          }else{
            // q_n0 = q_n1;
            // a_n0 = a_n1;
            // q_n1 = q_n2;
            if (ii < sp_knots.rows() - 3){

            }else{
              //ii is sp_knots.rows() - 3, so next will be ii = sp_knots.rows() - 2 which leads to exit of the loop
              if(t > sp_knots(ii + 2)){
                #if defined(SPLINE_EXCEPTIONS)
                  throw std::range_error("cubic_Bezier_interpolation: ??");
                #endif
              }
            }
          }
        }else{
          break;
        }
        nn++;
      }
      Eigen::Matrix<double, dim, -1> qB(dim,4);
      if((nn < 1 && t != sp_knots(sp_idx0)) || nn > sp_knots.rows() - 2){
        #if defined(SPLINE_EXCEPTIONS)
          std::cout<<"nn:="<<nn<<std::endl;
          std::cout<<"t:="<<t<<std::endl;
          std::cout<<"sp_knots"<<sp_knots<<std::endl;
          throw std::range_error("cubic_Bezier_interpolation: nn wrong range??");
        #endif
      }
      if(t < sp_knots(nn)|| t > sp_knots(nn+1)){
        #if defined(SPLINE_EXCEPTIONS)
          throw std::range_error("t < sp_knots(nn)|| t > sp_knots(nn+1)");
        #endif
      }
      double u =(t - sp_knots(nn))/((sp_knots(nn+1)-sp_knots(nn)));
      if (spherical){
        get_quat(q_n0_qa, qB.col(0));
        get_quat(a_n0_qa, qB.col(1));
        get_quat(b_n1_qa, qB.col(2));
        get_quat(q_n1_qa, qB.col(3));
      }else{

      }
      ctrlpt_t q_inter,qt_p,qt_pp;
      Bezier_curve(u, qB, q_inter , spherical);
      eval.col(0) = q_inter.col(0);
    }
    void Bezier_curve(  double u,
                        Eigen::Ref< Eigen::Matrix<double, dim, -1 > > qB,
                        Eigen::Ref<ctrlpt_t> q,
                        bool spherical)
    {
        uint64_t N = qB.cols();
        if(N == 0){
            #if defined(SPLINE_EXCEPTIONS)
              throw std::range_error("cannot generate a Bezierkurve for one curve point");
            #endif
        }
        if(u < 0  || u >1){
            #if defined(SPLINE_EXCEPTIONS)
              throw std::range_error("u for a Bezierkurve must between [0,1]");
            #endif
        }
        std::vector<ctrlpt_t> b_new(N);
        std::vector<ctrlpt_t> b_prv(N);
        std::vector<ctrlpt_t> bp_new(N);
        std::vector<ctrlpt_t> bp_prv(N);
        for(uint64_t rr = 0; rr < N; rr++){
          b_prv[rr] = qB.col(rr);
        }
        //interpolation by the Bezier curve interpolation, recurvsive contstruction, see pyhton code for better documentation,
        for(uint64_t rr = 1; rr < N; rr++){
            //# print("Recursion level:=(%i/%i)"%(rr,N))
            for(uint64_t ii = 0; ii< N - rr; ii++){
                //# print("Recursion level:=(%i/%i)"%(rr,N))
                //assert(len(b[rr-1]) == N - rr + 2)
                if(spherical){
                  //spheriacal interpolation controll points must be unit quaternions !, b_new == b_()
                  ctrlpt_qa_t b_ii_rr_prv; //# b_(ii)^(rr-1)
                  set_quat(b_prv[ii],b_ii_rr_prv); //convert coeffs vector to a quaternion to use slerp function of eigen library
                  ctrlpt_qa_t b_ii_nxt_rr_prv;//# b_(ii+1)^(rr-1)
                  set_quat(b_prv[ii+1],b_ii_nxt_rr_prv); //convert coeffs vector to a quaternion to use slerp function of eigen library (TODO better self implementation of slerp)
                  ctrlpt_qa_t b_new_quat = b_ii_rr_prv.slerp(u, b_ii_nxt_rr_prv);
                  b_new_quat.normalize();
                  get_quat(b_new_quat, b_new[ii]);
                  //spheriacal interpolation dot, d(b_rr[ii])/du
                }else{
                  ctrlpt_t &b_ii_rr_prv = b_prv[ii];//# b_(ii)^(rr-1)
                  ctrlpt_t &b_ii_nxt_rr_prv = b_prv[ii+1];//# b_(ii+1)^(rr-1)
                  b_new[ii] = (1-u)*b_ii_rr_prv + u*b_ii_nxt_rr_prv;
                }
            }
            for(uint64_t jj = 0; jj < N; jj++){
              if(jj >= N - rr){
                b_prv [jj] = ctrlpt_t::Zero();
              }else{
                b_prv [jj] = b_new[jj];
              }
            }
        }
        q = b_new[0];
    }
  };
#endif // EVALC_H
