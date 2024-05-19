/*********************************************************************************
  *FileName:		PID.hpp
  *Author:  		qianwan
  *Detail: 			PID template

  *Version:  		2.0
  *Date:  			2023/12/24
  *Describe:		Rebuild with template

  *Version:  		1.3
  *Date:  			2023/05/17
  *Describe:		Add forward loop
  
  *Version:  		1.2
  *Date:  			2023/03/13
  *Describe:		修补BUG
  
  *Version:  		1.1
  *Date:  			2023/02/26
  *Describe:		调整积分微分范围判定条件表达式

  *Version:  		1.0
  *Date:  			2023/02/10
  *Describe:		基于测试数据,取消对CMSIS-DSP的依赖
**********************************************************************************/
/*Version:  2.0*/
/*Stepper:  0.2*/
#pragma once
#ifndef PID_H_
#define PID_H_
namespace PID {

    template<typename T>
    class cPID {
    public:
        virtual void SetRef(T ref) = 0;

        virtual T Calculate(T fdb, T dt) = 0;

        virtual void Rst() = 0;

        virtual T Out() = 0;
    };

    template<typename T>
    class cPID_Inc : public cPID<T> {
    protected:
        /*PID Parameters*/
        T _kp; /*Ratio*/
        T _ki; /*Integral*/
        T _kd; /*Differentiation*/
        T _kf; /*Forward*/
        T _dt; /*Timing difference*/
        T _out; /*Frequency*/
        T _max_out;
        T _min_out;
        bool _en_inter_separation;  /*Enable integral separation*/
        bool _en_differ_separation; /*Enable differentiation separation*/
        T _inter_range;  /*Integral Separation Range*/
        T _differ_range; /*Differentiation Separation Range*/

        /*PID Template Value*/
        T _feedback;
        T _ref;
        T _last_ref;
        T _error;
        T _derror;
        T _dderror;
        T _preerror;
        T _prederror;

        inline T ForwardLoop() {
            /* 1阶惯性环节前馈环 G(s)= (1 + s) */
            return _ref + (_ref - _last_ref) / _dt;
        }

    public:
        cPID_Inc(
                T kp,
                T ki,
                T kd,
                T kf,
                T dt,
                T max_out,
                T min_out,
                bool en_inter_separation,
                T inter_range,
                bool en_differ_separation,
                T differ_range
        ) :
                _kp(kp),
                _ki(ki),
                _kd(kd),
                _kf(kf),
                _dt(dt),
                _max_out(max_out),
                _min_out(min_out),
                _en_inter_separation(en_inter_separation),
                _en_differ_separation(en_differ_separation),
                _inter_range(inter_range),
                _differ_range(differ_range) {
            Rst();
        }

        cPID_Inc() {}; /*Default Constructor*/

        void SetParam(
                T kp,
                T ki,
                T kd,
                T kf,
                T dt,
                T max_out,
                T min_out,
                bool en_inter_separation,
                T inter_range,
                bool en_differ_separation,
                T differ_range
        ) {
            _kp = kp;
            _ki = ki;
            _kd = kd;
            _kf = kf;
            _dt = dt;
            _max_out = max_out;
            _min_out = min_out;
            _en_inter_separation = en_inter_separation;
            _inter_range = inter_range;
            _en_differ_separation = en_differ_separation;
            _differ_range = differ_range;
            Rst();
        }

        void Rst() override {
            _feedback = 0;
            _ref = 0;
            _last_ref = 0;
            _error = 0;
            _derror = 0;
            _dderror = 0;
            _prederror = 0;
            _preerror = 0;
            _out = 0;
        }

        void SetRef(T ref) override {
            _last_ref = _ref;
            _ref = ref;
        }

        T Calculate(T fdb, T dt) override {
            /*中间量*/
            T tmp[4] = {0};

            /*前期准备*/
            _feedback = fdb;
            _dt = dt;
            _error = (_ref - _feedback) / _dt;
            _derror = _error - _preerror;
            _dderror = _derror - _prederror;
            _preerror = _error;
            _prederror = _derror;

            /*比例积分微分运算*/
            //pid->Out = pid->Out + (pid->Kp * pid->DError + pid->Ki * pid->Error + pid->Kd * pid->DDError);
            tmp[0] = _kp * _derror;
            //I 积分分离
            if (_en_inter_separation ? (fabs(_error) < _inter_range) : 1) {
                tmp[1] = _ki * _error;
            }
            //D 微分分离
            if (_en_differ_separation ? (fabs(_error) < _differ_range) : 1) {
                tmp[2] = _kd * _dderror;
            }

            //求和
            tmp[3] = _out + tmp[0] + tmp[1] + tmp[2] + ForwardLoop() * _kf;

            /*后期处理*/
            //输出限幅
            tmp[3] = (tmp[3] > _max_out) ? _max_out : tmp[3];
            tmp[3] = (tmp[3] < _min_out) ? _min_out : tmp[3];
            //赋值
            _out = tmp[3];
            return tmp[3];
        }

        T Out() override {
            return _out;
        }
    };

    template<typename T>
    class cPID_Pst : public cPID<T> {
    protected:
        /*PID Parameters*/
        T _kp; /*Ratio*/
        T _ki; /*Integral*/
        T _kd; /*Differentiation*/
        T _kf; /*Forward*/
        T _dt; /*Timing difference*/
        T _out; /*Frequency*/
        T _max_out;
        T _min_out;
        T _integral_max;
        T _integral_min;
        bool _en_inter_separation;  /*Enable integral separation*/
        bool _en_differ_separation; /*Enable differentiation separation*/
        T _inter_range;  /*Integral Separation Range*/
        T _differ_range; /*Differentiation Separation Range*/

        /*PID Template Value*/
        T _feedback;
        T _ref;
        T _last_ref;
        T _error;
        T _last_error;
        T _integral;


        inline T ForwardLoop() {
            /* 1阶惯性环节前馈环 G(s)= (1 + s) */
            return _ref + (_ref - _last_ref) / _dt;
        }

    public:
        cPID_Pst(
                T kp,
                T ki,
                T kd,
                T kf,
                T dt,
                T max_out,
                T min_out,
                T integral_max,
                T integral_min,
                bool en_inter_separation,
                T inter_range,
                bool en_differ_separation,
                T differ_range
        ) :
                _kp(kp),
                _ki(ki),
                _kd(kd),
                _kf(kf),
                _dt(dt),
                _max_out(max_out),
                _min_out(min_out),
                _integral_max(integral_max),
                _integral_min(integral_min),
                _en_inter_separation(en_inter_separation),
                _en_differ_separation(en_differ_separation),
                _inter_range(inter_range),
                _differ_range(differ_range) {
            Rst();
        }

        cPID_Pst() {}; /*Default Constructor*/

        void SetParam(
                T kp,
                T ki,
                T kd,
                T kf,
                T dt,
                T max_out,
                T min_out,
                T integral_max,
                T integral_min,
                bool en_inter_separation,
                T inter_range,
                bool en_differ_separation,
                T differ_range
        ) {
            _kp = kp;
            _ki = ki;
            _kd = kd;
            _kf = kf;
            _dt = dt;
            _max_out = max_out;
            _min_out = min_out;
            _integral_max = integral_max;
            _integral_min = integral_min;
            _en_inter_separation = en_inter_separation;
            _inter_range = inter_range;
            _en_differ_separation = en_differ_separation;
            _differ_range = differ_range;
            Rst();
        }

        void Rst() override {
            _feedback = 0;
            _ref = 0;
            _last_ref = 0;
            _error = 0;
            _last_error = 0;
            _integral = 0;
            _out = 0;
        }

        void SetRef(T ref) override {
            _last_ref = _ref;
            _ref = ref;
        }

        T Calculate(T fdb, T dt) override {
            /*中间量*/
            T tmp[4] = {0};

            /*前期准备*/
            _feedback = fdb;
            _dt = dt;
            _error = (_ref - _feedback) / _dt;
            _integral += _error;

            /*Limit value of integral*/
            _integral = _integral > _integral_max ? _integral_max : _integral;
            _integral = _integral < _integral_min ? _integral_min : _integral;

            tmp[0] = _kp * _error;

            //I 积分分离
            if (_en_inter_separation ? (fabs(_error) < _inter_range) : 1) {
                tmp[1] = _ki * _integral;
            }
            //D 微分分离
            if (_en_differ_separation ? (fabs(_error) < _differ_range) : 1) {
                tmp[2] = _kd * (_error - _last_error);
            }

            tmp[3] = tmp[0] + tmp[1] + tmp[2] + ForwardLoop() * _kf;
            //输出限幅
            tmp[3] = (tmp[3] > _max_out) ? _max_out : tmp[3];
            tmp[3] = (tmp[3] < _min_out) ? _min_out : tmp[3];
            //赋值
            _out = tmp[3];
            return _out;
        }

        T Out() override {
            return _out;
        }

    };

    /*Integer Incremental*/
    using PID_Inc_i = cPID_Inc<int>;
    /*Integer Position*/
    using PID_Pst_i = cPID_Pst<int>;
    /*Float Incremental*/
    using PID_Inc_f = cPID_Inc<float>;
    /*Float Position*/
    using PID_Pst_f = cPID_Pst<float>;
    /*Double Incremental*/
    using PID_Inc_d = cPID_Inc<double>;
    /*Double Position*/
    using PID_Pst_d = cPID_Pst<double>;
}
#endif