#include "foc.h"
#include "main.h"
#include "tim.h"
#include "as5048a.h"
#include <math.h>

#define SQRT3_2     0.86602540378f
#define TWO_PI_3    2.09439510239f
#define DUTY_MIN    0.03f
#define DUTY_MAX    0.95f

volatile float g_pp                 = (float)FOC_POLE_PAIRS;
volatile int32_t g_dir              = FOC_ENCODER_DIR;
volatile float g_enc_off_counts     = 0.0f;
volatile float g_theta_e            = 0.0f;
volatile int32_t g_detect_dir       = 0;
volatile float g_detect_pp          = 0.0f;
volatile uint8_t g_calib_err        = 0xFF;
volatile uint32_t g_dgb_ccr1 = 0, g_dbg_ccr2 = 0, g_dbg_ccr3 = 0;



/* ---------------------------------------------------------------- */
/*                      SetPhaseDuty, Init, Stop                    */
/* ---------------------------------------------------------------- */
void SetPhaseDuty(float ua, float ub, float uc)
{
    float vmax = ua, vmin = ua;
    if (ub > vmax) vmax = ub;
    if (uc > vmax) vmax = uc;
    if (ub < vmin) vmin = ub;
    if (uc < vmin) vmin = uc;
    float vn = -0.5f * (vmax + vmin);


    /*-----------------------SVPWM---------------------*/
    float da = 0.5f + ua + vn; // DUTY A,B,C
    float db = 0.5f + ub + vn;
    float dc = 0.5f + uc + vn;

    if (da < DUTY_MIN) da = DUTY_MIN; else if (da > DUTY_MAX) da = DUTY_MAX;
    if (db < DUTY_MIN) db = DUTY_MIN; else if (db > DUTY_MAX) db = DUTY_MAX;
    if (dc < DUTY_MIN) dc = DUTY_MIN; else if (dc > DUTY_MAX) dc = DUTY_MAX;

    float arr = (float)TIM8->ARR;
    TIM8->CCR1 = (uint32_t)(da * arr);
    TIM8->CCR2 = (uint32_t)(db * arr);
    TIM8->CCR3 = (uint32_t)(dc * arr);
}

void FOC_M1_EmergencyStop(void)
{
    TIM8->BDTR &= ~TIM_BDTR_MOE;                             /* High-Z */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);   /* EN_GATE = LOW */
}

void FOC_M1_Init(void)
{
    g_pp = (float)FOC_POLE_PAIRS;
    g_dir = FOC_ENCODER_DIR;
    g_enc_off_counts = 0.0f;
}

/* ---------------------------------------------------------------- */
/*                                       */
/* ---------------------------------------------------------------- */

static void SetVecAngle(float v, float th)
{
    SetPhaseDuty(v * cosf(th),
                 v * cosf(th - TWO_PI_3),  
                 v * cosf(th + TWO_PI_3));
}

static float ReadCountsBlocking(void)
{
    for(int t = 0; t < 50; t++)
    {
        uint16_t r = AS5048A_ReadRaw();
        if(r != AS5048A_ERR) 
        {
            return (float)r;
        }
    }
    return -1.0f;
}

static float ReadCountsAvg(int n)
{
    float s = 0.0f;
    float c = 0.0f;
    int ok = 0;
    
    for(int i = 0; i < n; i++)
    {
        float a = ReadCountsBlocking();
        if(a >= 0.0f)
        {
            float rad = a * (FOC_2PI / FOC_ENCODER_CPR);
            s += sinf(rad);
            c += cosf(rad);
            ok++;
        }
        HAL_Delay(2);
    }

    if(!ok) return -1.0f;

    float m = atan2f(s,c);
    if(m < 0.0f) 
    {
        m += FOC_2PI;
    }

    return m * (FOC_ENCODER_CPR / FOC_2PI);
}


uint8_t FOC_AutoCalibrate(float v)
{
    g_calib_err = 0xFF;

    // V_d = 0
    for(int i = 1; i <= 100; i++)
    {
        SetVecAngle(v * (float)i / 100.0f, 0.0f);
        HAL_Delay(10);
    }
    HAL_Delay(300);

    float prev = ReadCountsAvg(16);
    if(prev < 0.0f)
    {
        SetPhaseDuty(0,0,0);
        g_calib_err = 1;
        return 1;
    }
    
    //Openloop 
    float total = 0.0f;
    for (int i = 1; i <= 360; i++) {
        SetVecAngle(v, (float)i * (FOC_2PI / 360.0f));
        HAL_Delay(8);
        float cur = ReadCountsBlocking();
        if (cur < 0.0f) { SetPhaseDuty(0,0,0); g_calib_err = 1; return 1; }
        float d = cur - prev;
        if (d >  8192.0f) d -= FOC_ENCODER_CPR;
        if (d < -8192.0f) d += FOC_ENCODER_CPR;
        total += d;
        prev = cur;
    }
    HAL_Delay(500);   /* rotor về đúng theta = 2*pi = 0 */
 
    /* 3. Kiểm tra hợp lệ */
    if (fabsf(total) < 100.0f) { SetPhaseDuty(0,0,0); g_calib_err = 2; return 2; }
 
    float pp  = FOC_ENCODER_CPR / fabsf(total);
    float ppr = roundf(pp);
    g_detect_pp  = pp;
    g_detect_dir = (total > 0.0f) ? 1 : -1;
    if (ppr < 1.0f || fabsf(pp - ppr) > 0.3f) { SetPhaseDuty(0,0,0); g_calib_err = 3; return 3; }
 
    /* 4. Chốt offset tại theta_e = 0 */
    float off = ReadCountsAvg(32);
    if (off < 0.0f) { SetPhaseDuty(0,0,0); g_calib_err = 1; return 1; }
 
    g_pp  = ppr;
    g_dir = g_detect_dir;
    g_enc_off_counts = off;
 
    SetPhaseDuty(0.0f, 0.0f, 0.0f);   /* 0 V */
    g_calib_err = 0;
    return 0;
}




/* ---------------------------------------------------------------- */
void FOC_VoltageMode_Step(float enc_counts, float vq)
{
    if (vq >  FOC_VQ_LIMIT) vq =  FOC_VQ_LIMIT;
    if (vq < -FOC_VQ_LIMIT) vq = -FOC_VQ_LIMIT;
 
    /* Góc điện tính theo count để giữ độ chính xác float */
    float mech = enc_counts - g_enc_off_counts;
    float te = fmodf(mech * g_pp * (float)g_dir, FOC_ENCODER_CPR);
    if (te < 0.0f) te += FOC_ENCODER_CPR;
 
    float theta = te * (FOC_2PI / FOC_ENCODER_CPR);
    g_theta_e = theta;
 
    float s = sinf(theta);
    float c = cosf(theta);
 
    /* Inverse Park với Vd = 0 */
    float v_alpha = -vq * s;
    float v_beta  =  vq * c;
 
    /* Inverse Clarke */
    float v_a = v_alpha;
    float v_b = -0.5f * v_alpha + SQRT3_2 * v_beta;
    float v_c = -0.5f * v_alpha - SQRT3_2 * v_beta;
 
    SetPhaseDuty(v_a, v_b, v_c);
}