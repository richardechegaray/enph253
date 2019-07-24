class pid{
    public:
        pid();
        float output_pid(float error);
        float kp, ki, kd; //coefficients to calculate p, i, d
        float p, i, d; //real values to apply
        //float p_limit, i_limit, d_limit;
        float i_limit;
        //float i_sum;
        float prev_error;
        float speed;
};