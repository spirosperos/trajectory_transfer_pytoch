#! /usr/bin/env python3
import os
import sys
import rospy
import rospkg
import numpy as np
import matplotlib
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import FormatStrFormatter
import ctypes
from ctypes.util import find_library
import vispy
import tkinter as tk
from vispy.plot import Fig as vispyfig
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Time
from inspect import getmembers, isfunction, isclass
sys.path.append(os.path.dirname('..'))
import random

import subprocess
import importlib
model_lock = threading.Lock()

class acin_robot_model_C():
    __param = None;
    __q_dim = 0;
    __robot_name = ""
    __module_dir = ""
    __package_dir = ""
    def __init__(self,**kwargs):
        print("#######################  ###################################")
        print("################ Loading acin_robot_model.... ##############")
        print("#######################  ###################################")
        rospack = rospkg.RosPack()
        kwargs.setdefault("num_test",10);
#        print("Loading acin_robot_model....")

        self.__package_dir = rospack.get_path('ros_acin_robot_support');
        if "dim" in  kwargs:
            self.__q_dim = kwargs["dim"];
        if "robot_name" in  kwargs:
            self.robot_name = kwargs["robot_name"];# module dir will be set by the setter function for the robot_name parameter
        if "param" in  kwargs:
            self.__param = kwargs["param"];
        else:
            if not self.__robot_name.strip():
                raise ValueError("<acin_robot_model --__init__()>_: robot name not given")
            if not os.path.isdir(self.__module_dir):
                raise ValueError("<acin_robot_support --__init__()>: model dir not the robot not set" )
            try:
                spec = importlib.util.spec_from_file_location("robot_params", self.__module_dir+os.sep+"python"+os.sep+"__init__.py")
                module = spec.loader.load_module()
                functions_list = [o for o in getmembers(module) if isclass(o[1])]
                self.__param = module.robot_params_C()
            except Exception as ec:
                print(ec)

        print("robot name:= \"%s\""% self.robot_name)
        print("module_dir:= \"%s\""% self.__module_dir)
        print("robot configuration space dim.:= %i"% self.__q_dim)
        print("Performing %i test with random vectors (q, q_p)\n\t->check if the parameter sturct in python is well defined (number of paramters)"%(kwargs["num_test"]))
        print("sizeof python parameter struct: ", sys.getsizeof(self.__param))

        q_ub = self.__param.q_limit_upper;
        q_lb = self.__param.q_limit_lower;
        q_test_vec = np.zeros((kwargs["num_test"],self.__q_dim));
        for jj_test in range(kwargs["num_test"]):
            q_test_vec[jj_test,:] = np.array([random.uniform(q_lb[ii],q_ub[ii]) for ii in range(self.__q_dim)]);

        if find_library("evalJ") != None:
            self.__Jlib = ctypes.CDLL(find_library("evalJ")) # searches $LD_LIBRARY_PATH, the path to the lib must in this variable
            functions_list = [o for o in getmembers(self.__Jlib) if isclass(o[1])]
            #define the arguments type of the C function (forward kinematics), py wrapper interface done in c
            #1.argument: pointer to the q coord.
            #2.argument: pointer to the jacobian matrix of the robot
            #3.argument: pointer to the paramter struct of the robot used for evaluation, if NONE is given the paramter defined in the corresponding c-header file is used for the evaluation
            self.__Jlib.py_acin_evalJ.argtypes = [np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),
                                            np.ctypeslib.ndpointer(dtype=np.float64, ndim=2, flags='WRITEABLE'),
                                            ctypes.POINTER(type(self.__param))]
        else:
            raise ImportError("Can't find lib. \"evalJ\", ensure the lib can be found on your $LD_LIBRARY_PATH")
        if find_library("evalT0E") != None:
            self.__T0Elib = ctypes.CDLL(find_library("evalT0E")) # searches $LD_LIBRARY_PATH, the path to the lib must in this variable
            functions_list = [o for o in getmembers(self.__T0Elib) if isclass(o[1])]

            #define the arguments type of the C function (forward kinematics), py wrapper interface done in c
            #1.argument: pointer to the q coord.
            #2.argument: pointer to the homogenoues transformation from endeffectro coord. to the basis coord system
            #3.argument: pointer to the paramter struct of the robot used for evaluation
            self.__T0Elib.py_acin_evalT0E.argtypes = [np.ctypeslib.ndpointer(dtype=np.float64, ndim=1),
                                            np.ctypeslib.ndpointer(dtype=np.float64, ndim=2, flags='WRITEABLE'),
                                            ctypes.POINTER(type(self.__param))]



        else:
            raise ImportError("Can't find lib. \"evalT0E\", ensure the lib can be found on your $LD_LIBRARY_PATH")

        for jj_test in range(kwargs["num_test"]):
            q_test_jj =q_test_vec[jj_test]
            H_python_para = self.evalT0E(q_test_jj, default_param=False)
            H_default_para = self.evalT0E(q_test_jj, default_param=True)
            if np.linalg.norm(H_python_para - H_default_para) > 0.001:
                raise ValueError("evaluation error for the homogeneous transformation, wrong model loaded or the parameter structes are not equal \n \t-> the c++ and python struct must have the same number of elementes (both values and number of elemente can cause problems, only pointer are given to the c function lib)")
        print("#######################  ###################################")
        print("#### desired robot_model successfully loaded ###")
        print("#######################  ###################################")
        return;
    def evalJ(self, q, grade = np.empty((0)), **kwargs):
        assert(len(q) == self.__q_dim)
        J_ = np.zeros((6,self.__q_dim), dtype=np.float64, order='F') # placeholder for return value of the c function, dot of jacobian matrix
        q_ = np.zeros((self.__q_dim), dtype=np.float64, order='F') # placeholder for return value of the c function, dot of jacobian matrix
        if len(q.shape) == 1:
            q = q[np.newaxis,:]
        for ii in  range(len(q)):
            q_[ii] = q[0,ii];
        if grade.size > 0:
            assert(1==0)
            assert(q.size==grade.size)
        if "default_param" in kwargs and kwargs["default_param"]:
            print("none")
            self.__Jlib.py_acin_evalJ(q_, J_, None)
        else:
            self.__Jlib.py_acin_evalJ(q_, J_, self.__param)

        return J_;
    def evalT0E(self, q, grade = np.empty((0)), **kwargs):
        H_ = np.zeros((4,4), dtype=np.float64, order='F') # placeholder for return value of the c function, homogen transfromation
        q_ = np.zeros((self.__q_dim), dtype=np.float64, order='F') # placeholder for return value of the c function, homogen transfromation
        if len(q.shape) == 1:
            q = q[np.newaxis,:]
        else:
            if q.shape[0] != 1:
                q = q.transpose()
        for ii in  range(q.shape[1]):
            q_[ii] = q[0,ii];
        if grade.size > 0:
            assert(1==0)
            assert(q.size==grade.size)
        if "default_param" in kwargs and kwargs["default_param"]:
            self.__T0Elib.py_acin_evalT0E(q_, H_, None)
        else:
            self.__T0Elib.py_acin_evalT0E(q_, H_, self.__param)
        return H_;
    @property
    def param(self):
        return self.__param;
    @param.setter
    def param(self, arg):
        self.__param = arg;
    @property
    def robot_name(self):
        return self.__robot_name

    @robot_name.setter
    def robot_name(self, robot_name):
        self.__robot_name = robot_name;
        self.__module_dir = os.path.join(self.__package_dir, "include","ros_acin_robot_support","models",self.__robot_name );
        self.__module = self.__module_dir.replace(os.sep,'.');

    def get_robot_model(self,q_dim):
        robot_model_mod = importlib.import_module("ros_acin_robot_model")
        param = self.get_robot_params()
        return robot_model_mod.acin_robot_model_C(q_dim, param);
if __name__ == '__main__':

    #functions_list = [o for o in getmembers(ros_acin_robot_support) if isclass(o[1])]
    #print(functions_list)
    print(os.path.dirname(os.path.abspath(__file__)))
    sys.path.append(os.path.dirname('../..'))
    try:
        robot = acin_robot_model_C(dim = 7, robot_name = "kukalbriiwa_model")

    except rospy.ROSInterruptException:
        pass
