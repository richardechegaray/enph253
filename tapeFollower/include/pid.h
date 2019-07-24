class pid{
    public:
        pid();
        float output_pid(float error);
        float kp, ki, kd; //coefficients to calculate p, i, d
        float p, i, d; //real values to apply
        float i_limit;
        float prev_error;
        float speed;
};