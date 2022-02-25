import numpy as np
import ctypes
M_PI =3.1415926535897931
class robot_params_C(ctypes.Structure):
    def __init__(self):
        self.g = 9.81;
        self.alpha_base = 0;
        self.beta_base = 0;
        self.gamma_base = 0;
        self.dw0x = 0;
        self.dw0y = 0;
        self.dw0z = 0;
        self.d1 =0.1525
        self.d2 = 0.2075;
        self.d3 = 0.2325;
        self.d4 = 0.1875;
        self.d5 = 0.2125;
        self.d6 = 0.1875;
        self.d7 = 0.0796;
        self.d8 = 0.0724;
        self.m1 = 6.495;
        self.m2 = 8.807;
        self.m3 = 2.8;
        self.m4 = 5.283;
        self.m5 = 1.889;
        self.m6 = 2.32;
        self.m7 = 0.863;
        self.m7_ = 0.863;
        self.m7 = self.m7_;
        self.I1xx = 0.0690761441316632;
        self.I1xy = 0;
        self.I1xz = 0;
        self.I1yy = 0.0711085586914514;
        self.I1yz = 0;
        self.I1zz = 0.0200625854402119;
        self.I2xx = 0.0824701683534692;
        self.I2xy = 0;
        self.I2xz = 0;
        self.I2yy = 0.0164110968055191;
        self.I2yz = 0;
        self.I2zz = 0.0873510892661361;
        self.I3xx = 0.0230152941318817;
        self.I3xy = 0;
        self.I3xz = 0;
        self.I3yy = 0.0228661554143474;
        self.I3yz = 0;
        self.I3zz = 0.0554482283748983;

        self.I4xx = 0.0472789668341968;
        self.I4xy = 0;
        self.I4xz = 0;
        self.I4yy = 0.00964726804146909;
        self.I4yz = 0;
        self.I4zz = 0.0466529007761679;

        self.I5xx = 0.0138359953358589;
        self.I5xy = 0;
        self.I5xz = 0;
        self.I5yy = 0.0116859337751969;
        self.I5yz = 0;
        self.I5zz = 0.00571863785412578;

        self.I6xx = 0.00732758787216765;
        self.I6xy = 0;
        self.I6xz = 0;
        self.I6yy = 0.00477633755783711;
        self.I6yz = 0;
        self.I6zz = 0.00606797638550296;

        self.I7xx = 0.000884;
        self.I7xy = 0;
        self.I7xz = 0;
        self.I7yy = 0.000888;
        self.I7yz = 0;
        self.I7zz = 0.001105;
        self.sp1x = 0;
        self.sp1y = -0.01439;
        self.sp1z = 0.102503;

        self.sp2x = 1.2e-05;
        self.sp2y = 0.06472;
        self.sp2z = 0.004804;

        self.sp3x = -2.08e-04;
        self.sp3y = 0.01601;
        self.sp3z = 0.087283;

        self.sp4x = -2.03e-04;
        self.sp4y = 0.098639;
        self.sp4z = 0.011478;

        self.sp5x = 5.7e-05;
        self.sp5y = 0.014468;
        self.sp5z = 0.062938;

        self.sp6x = 3.63e-04;
        self.sp6y = 0.016982;
        self.sp6z = -0.019731;

        self.sp7x_ = 0.001904;
        self.sp7y_ = -9.3e-05;
        self.sp7z_ = 0.018862;
        self.sp7x = self.sp7x_;
        self.sp7y = self.sp7y_;
        self.sp7z = self.sp7z_;

        self.n__1=160;
        self.n__2=160;
        self.n__3=160;
        self.n__4=160;
        self.n__5=100;
        self.n__6=160;
        self.n__7=160;

        self.I__r1xx=0.000185;
        self.I__r1yy=self.I__r1xx;
        self.I__r1zz=0.000185+0.000238166953125;
        self.I__r2xx=0.000185;
        self.I__r2yy=self.I__r2xx;
        self.I__r2zz=0.000185+0.000238166953125;
        self.I__r3xx=0.000129;
        self.I__r3yy=self.I__r3xx;
        self.I__r3zz=0.000129+6.20384375e-005;
        self.I__r4xx=0.000129;
        self.I__r4yy=self.I__r4xx;
        self.I__r4zz=0.000129+6.20384375e-005;
        self.I__r5xx=7.5e-05;
        self.I__r5yy=self.I__r5xx;
        self.I__r5zz=7.5e-05+7.20968e-005;
        self.I__r6xx=1.5e-05;
        self.I__r6yy=self.I__r6xx;
        self.I__r6zz=1.5e-05+3.51125e-006;
        self.I__r7xx=1.5e-05;
        self.I__r7yy=self.I__r7xx;
        self.I__r7zz = 1.5e-05+3.51125e-006;

        self.B = {self.n__1*self.n__1*self.I__r1zz, self.n__2*self.n__2*self.I__r2zz,self.n__3*self.n__3*self.I__r3zz,self.n__4*self.n__4*self.I__r4zz,self.n__5*self.n__5*self.I__r5zz,self.n__6*self.n__6*self.I__r6zz, self.n__7*self.n__7*self.I__r7zz};
        self.deg_to_rad = M_PI / 180.0;
        self.tau_max =[320, 320, 176, 176, 110, 40, 40];
        self.Tau_lb = [-320, -320, -176, -176, -110, -40, -40];
        self.Tau_ub = [320, 320, 176, 176, 110, 40, 40];
        self.q_limit_upper = [170 * self.deg_to_rad, 120 * self.deg_to_rad, 170 * self.deg_to_rad, 120 * self.deg_to_rad, 170 * self.deg_to_rad, 120 * self.deg_to_rad, 175 * self.deg_to_rad];
        self.q_limit_lower = [-170 * self.deg_to_rad, -120 * self.deg_to_rad, -170 * self.deg_to_rad, -120 * self.deg_to_rad, -170 * self.deg_to_rad, -120 * self.deg_to_rad, -175 * self.deg_to_rad];

    _fields_= [ ("g",ctypes.c_double),
                ("alpha_base",ctypes.c_double),("beta_base",ctypes.c_double),("gamma_base",ctypes.c_double),
                ("dw0x",ctypes.c_double),("dw0y",ctypes.c_double),("dw0z",ctypes.c_double),
                ("d1",ctypes.c_double),("d2",ctypes.c_double),("d3",ctypes.c_double),
                ("d4",ctypes.c_double),("d5",ctypes.c_double),("d6",ctypes.c_double),
                ("d7",ctypes.c_double),("d8",ctypes.c_double),
                ("m1",ctypes.c_double),("m2",ctypes.c_double),("m3",ctypes.c_double),
                ("m4",ctypes.c_double),("m5",ctypes.c_double),("m6",ctypes.c_double),
                ("m7",ctypes.c_double),
                ("m7_",ctypes.c_double),
                ("I1xx",ctypes.c_double),("I1xy",ctypes.c_double),("I1xz",ctypes.c_double),
                ("I1yy",ctypes.c_double),("I1yz",ctypes.c_double),("I1zz",ctypes.c_double),
                ("I2xx",ctypes.c_double),("I2xy",ctypes.c_double),("I2xz",ctypes.c_double),
                ("I2yy",ctypes.c_double),("I2yz",ctypes.c_double),("I2zz",ctypes.c_double),
                ("I3xx",ctypes.c_double),("I3xy",ctypes.c_double),("I3xz",ctypes.c_double),
                ("I3yy",ctypes.c_double),("I3yz",ctypes.c_double),("I3zz",ctypes.c_double),
                ("I4xx",ctypes.c_double),("I4xy",ctypes.c_double),("I4xz",ctypes.c_double),
                ("I4yy",ctypes.c_double),("I4yz",ctypes.c_double),("I4zz",ctypes.c_double),
                ("I5xx",ctypes.c_double),("I5xy",ctypes.c_double),("I5xz",ctypes.c_double),
                ("I5yy",ctypes.c_double),("I5yz",ctypes.c_double),("I5zz",ctypes.c_double),
                ("I6xx",ctypes.c_double),("I6xy",ctypes.c_double),("I6xz",ctypes.c_double),
                ("I6yy",ctypes.c_double),("I6yz",ctypes.c_double),("I6zz",ctypes.c_double),
                ("I7xx",ctypes.c_double),("I7xy",ctypes.c_double),("I7xz",ctypes.c_double),
                ("I7yy",ctypes.c_double),("I7yz",ctypes.c_double),("I7zz",ctypes.c_double),
                ("sp1x",ctypes.c_double),("sp1y",ctypes.c_double),("sp1z",ctypes.c_double),
                ("sp2x",ctypes.c_double),("sp2y",ctypes.c_double),("sp2z",ctypes.c_double),
                ("sp3x",ctypes.c_double),("sp3y",ctypes.c_double),("sp3z",ctypes.c_double),
                ("sp4x",ctypes.c_double),("sp4y",ctypes.c_double),("sp4z",ctypes.c_double),
                ("sp5x",ctypes.c_double),("sp5y",ctypes.c_double),("sp5z",ctypes.c_double),
                ("sp6x",ctypes.c_double),("sp6y",ctypes.c_double),("sp6z",ctypes.c_double),
                ("sp7x",ctypes.c_double),("sp7y",ctypes.c_double),("sp7z",ctypes.c_double),
                ("sp7x_",ctypes.c_double),("sp7y_",ctypes.c_double),("sp7z_",ctypes.c_double),
                ("n__1",ctypes.c_double),("n__2",ctypes.c_double),("n__3",ctypes.c_double),
                ("n__4",ctypes.c_double),("n__5",ctypes.c_double),("n__6",ctypes.c_double),
                ("n__7",ctypes.c_double),
                ("I__r1xx",ctypes.c_double),("I__r1yy",ctypes.c_double),("I__r1zz",ctypes.c_double),
                ("I__r2xx",ctypes.c_double),("I__r2yy",ctypes.c_double),("I__r2zz",ctypes.c_double),
                ("I__r3xx",ctypes.c_double),("I__r3yy",ctypes.c_double),("I__r3zz",ctypes.c_double),
                ("I__r4xx",ctypes.c_double),("I__r4yy",ctypes.c_double),("I__r4zz",ctypes.c_double),
                ("I__r5xx",ctypes.c_double),("I__r5yy",ctypes.c_double),("I__r5zz",ctypes.c_double),
                ("I__r6xx",ctypes.c_double),("I__r6yy",ctypes.c_double),("I__r6zz",ctypes.c_double),
                ("deg_to_rad",ctypes.c_double)
                ]
