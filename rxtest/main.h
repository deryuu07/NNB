#define FLD_SUCCESS 1 
#define FLD_ERROR 2
#define FLD_BLANK 3
#define FLD_OK 4
#define FLD_TMOUT 5
#define FLD_NOBLANK 6
#define BUZZ_permit 1
#define LED_ON 0
#define LED_OFF 1 
#define ON 1
#define OFF 0 
#define SW_ON 0
#define SW_OFF 1
#define corner 6
#define corner2 7
#define goal 8
#define START_A 1
#define STOP 0
#define ST 1
#define curve 0
#define TRED 125
#define R 100.0
//#define mmpp 0.00934524
//#define cmpp 0.000934524
//#define mmpp 0.0086019   //径23mm rate 42:8
//#define cmpp 0.00086019
#define mmpp 0.014717     //径24mm rate 64:20

#define max_thre 400.0
#define PWM_MAX 1499
#define window_size 30

/**********ポートマクロ***********/
#define SW PORTE.PORT.BIT.B0
#define LED1 PORT7.DR.BIT.B4	//○●○○
#define LED2 PORT7.DR.BIT.B5	//●○○○ 
#define SensLED0 PORT7.DR.BIT.B0
#define SensLED1 PORT7.DR.BIT.B1
#define SensLED2 PORT7.DR.BIT.B2
#define SensLED3 PORT7.DR.BIT.B3
#define SensLED4 PORT9.DR.BIT.B5
#define SensLEDCorner PORT1.DR.BIT.B0
#define SensLEDGoal PORT7.DR.BIT.B6
#define MOT_ctr_R1 PORTA.DR.BIT.B0
#define	MOT_ctr_R2 PORTA.DR.BIT.B1
#define MOT_ctr_L1 PORTA.DR.BIT.B2
#define	MOT_ctr_L2 PORTA.DR.BIT.B3
#define MOT_STBY   PORTA.DR.BIT.B4		
#define ENC_R MTU1.TCNT
#define ENC_L MTU2.TCNT
#define MOT_R MTU0.TGRA
#define MOT_L MTU0.TGRC
#define SensSize 10
#define SENS_THRE 380
#define CORNER 8
#define GOAL 9
#define COURSE_SIZE 200

struct{
	short Original[SensSize];  //生値
	short MaxValue[SensSize];  //最大値
	short MinValue[SensSize];  //最小値
	short Max_Min[SensSize];   //最大-最小値
	short Calibrated[SensSize];//=(Original-MinValue)/(MaxValue-MinValue)
	short Pre;
	short Err;
	short PreErr;
	short Center;
}SENSOR;	

struct{
	long Right;
	long Left;
	long R_L;
	long Local;	
	short Count;
}ENC;

struct{
	long Right[COURSE_SIZE];
	long Left[COURSE_SIZE];
	short Dis[COURSE_SIZE];
	short Course[COURSE_SIZE];
	short AccelDis[COURSE_SIZE];
}COURSE_DATA;

struct{
	float Right;
	float Left;
	float R_L;
	float Desire;
	float BasicDesire;
	float Err;
	float Pre;
	float PreErr;
	float SumErr;
	char  Now[COURSE_SIZE];
}VEL;

struct{
	float SensP;
	float SensD;
	float VelP;
	float VelI;
	float VelD;
}GAIN;

struct{
	float SensP;
}FGAIN;

struct{
	short Count;
	char  Timing;
	unsigned long LocalTime;
}LOG;

struct{
	char Stop;
	char GetSens;
	char Goal;
	char Corner;
	char Circle;
	char FirstAccel;
}FLAG;

float accel=6000.0;
short Battery=0;

/***************カウンタ***************/
unsigned long TimeCount=0;
unsigned short stop_count=0;

/***************床センサ***************/
short thre_max=350;	//300
short sens_dis[5]={1680, 300, 0 , -300, -1680};
short max_ERR=300;
short min_ERR=-300;

/***************ジャイロ***************/
short zero_gyro=0;
short sum_gyro=0;
short ave_gyro=0;
short curve_check=0;
long gyro_data_sum=0;
long data_count=0;
short MAX_gyro=0;
short MIN_gyro=32000;
char gyro_count=0;
float Desire_gyro=0;
float gyro_rate=0.0;
float AngVelRate=0.0;
/***************PIDパラメータ**************/
/***角速度成分***/
float K_G=1.0;
float first_K_G=0.0;
float K_P_GYRO=1.0;
float K_I_GYRO=0.0005;//0.0001;
float K_D_GYRO=0.95;
float GYRO_ERR=0.0;
float GYRO_SUM_ERR=0.0;
float PRE_GYRO=0.0;
float PRE_GYRO_ERR=0.0;

float speed_R=200.0;
float speed_L=200.0;
/***************フラグ*****************/
char stop_frag=0;
char start0=0;
char max_frag=0;
char min_frag=0;
short EG_stop_count=0;
char debug_mode_flag=0;
char fail_safe_frag=0;
short battery=0;
char emergency_frag=0;

/*************プロトタイプ宣言**********/
void LED_manager(char mode);
void reset();
void calc_sens();
void mot_STB(short start_stop);
void search_max();
void init_ENC();
void STB_MODE();
void mot_brake();
void EG_stop();
void stop_manager();
void goal_check();
void goal_check2();
void TIMER_WAIT(unsigned short ms);
void set();
void get_senc();
short get_Batt();
void Battery_Check();
short select_debug_mode();
short select_mode();
void calc_accel_dis(float Initial_velocity);
void ch_para();
void trace2();
void trace();
void out_sens();
void out_course();
void out_ERR();
void out_speed();
void init_thre();
void out_enc();
void out_PWM();
void ch_debug_mode(int mode);
void ch_mode(int mode);
void para_save();
void para_load();
