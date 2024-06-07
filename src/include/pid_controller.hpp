#include <sys/time.h>

class PID {
public:
    /**
     * @param P - 比例增益 Proportion
     * @param I - 微分增益 Integration
     * @param D - 积分增益 Derivative
     * @param Omax  - 输出幅值 : 角速度 包括 积分限幅
     * @param Vrate - 角速度变化率
     */
    PID(float P, float I, float D, float Omax, float Vrate = 0)
        : P(P), I(I), D(D), Omax(Omax), Vrate(Vrate) {
            ts_error_last = get_tick();
    }
    ~PID() noexcept = default;
    float operator()(float error) { // 误差输入计算
        unsigned long long ts_error_now = get_tick();
        float delta_ts = 1e-6f * float(ts_error_now - ts_error_last); // Δt : s
        if (delta_ts <= 0) delta_ts = 1e-3f; // get_tick溢出快速修复(暂定1ms,待定)
        /**
        * @proportion  比例部分 = Kp * e(t)
        * @integration 积分部分 = Ki * ∫e(τ)d(τ) =离散化=> Ki * Σe(t) => sum_last + Ki * 1/2Δt( e(t) + e(t-1) )
        * @derivative  微分部分 = Kd * de(t)/dt  =离散化=> Kd * e(Δt)/Δt
        * @Imax        积分限幅 : 超出设定的(-Vmax,Vmax)范围则矫正
        * @Vmax        输出限幅 : 超出设定的(-Vmax,Vmax)范围则矫正
        */
        float proportion  = P * error;
        float integration = I * 0.5f * delta_ts * (error - error_last) + integration_sum;
        integration = (integration<-Omax)?-Omax:(integration>Omax?Omax:integration);
        float derivative  = D * (error - error_last) / delta_ts;
        float output = proportion + integration + derivative;
        output = (output<-Omax)?-Omax:(output>Omax?Omax:output);
        /**
        * @Orate 输出变化率 : output_rate = (当前output - 上次output) / Δt
        *                   如果PID矫正率高于设定极限, 则不可达到, 选取最大可达变化率
        */
        if (Vrate > 0) {
            float Orate = (output - output_last) / delta_ts;
            if (Orate > Vrate)
                output = output_last + Vrate * delta_ts;
            else if (Orate < -Vrate)
                output = output_last - Vrate * delta_ts;
        }
        error_last      = error;
        integration_sum = integration;
        ts_error_last   = ts_error_now;
        output_last     = output;
        return output;
    }
private:
    unsigned long long get_tick() __attribute__((always_inline)) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        unsigned long long now = tv.tv_sec * 1000 * 1000 + tv.tv_usec;
        return now;
    }
public:
    float P, I, D, Omax, Vrate;
private:
    float error_last      = 0; // 上次误差
    float integration_sum = 0; // 先前积分总和
    float output_last     = 0; // 上次输出值
    unsigned long long ts_error_last; // 上次误差采样时间 : us
};