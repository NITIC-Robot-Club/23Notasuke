#include "mbed.h"
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <ratio>
#include "PIDcontroller.h"

UnbufferedSerial 	raspPico(PB_6,PB_7);
UnbufferedSerial	pc(USBTX, USBRX);

// LED光らせるぜ
PwmOut	LED	(PB_1);

// シザー停止リミットスイッチ
DigitalIn	ueLimit		(PA_3); //上限
DigitalIn	sitaLimit	(PA_4); //下限

// 電源基板停止
DigitalOut	emergency(PB_0);

// パタパタ接続確認LED
DigitalIn	PataPataState(PA_0);

PwmOut 		huta_servo(PA_7);

// CAN周り
RawCAN  can(PA_11, PA_12, 1000000);
CircularBuffer<CANMessage, 32> queue;
Ticker  getter;
char RcvData[8] = {0x00};

// CANMessage sendmsg(0x000, TxData, 8, CANData, CANStandard);

struct C610Data{
    unsigned ID;
    int32_t counts;
    int32_t rpm;
    int32_t current;
};

// PID用
const float	kp = 1.0;
const float	ki = 0.001;
const float	kd = 2.0;
const int 	slow_targetRPM = 1000; 	// 手動昇降目標値[rpm]
const int 	fast_targetRPM = 4000; 	// 自動一定上げ目標値[rpm]
Ticker		calculater;				// pid.conpute()を一定間隔でアレしたい
PID			pid(kp, ki, kd, 0.05);

// CAN受信用
void canListen(void);

// 受信データの形式の変更
void datachange(unsigned ID, struct C610Data *C610, CANMessage *Rcvmsg);

// 電流値を送信用に変換
void TorqueToBytes(uint16_t torqu, unsigned char *upper, unsigned char *lower);

// ESCにデータ送信
void sendData(const int32_t torqu0);

// 一定昇降とか用の時間制限
std::chrono::microseconds TIMELIMIT = 1s;

// 一定上げ
void tryer(void);

// 最高-最低上下
void nullpo(void);

void hutaPakaPaka(void);

void goHome(void);

// シリアル読み
void reader(void);
char are;	// シリアルのバッファ

// pid計算機
void pid_calculater(void);

int main(void){
    can.attach(&canListen, CAN::RxIrq);
    struct C610Data M1;
    M1.ID = 0x201;
    CANMessage Rxmsg;

	// PID各種設定
	pid.setOutputLimits(-8000, 8000);
	pid.setInputLimits(-18000, 18000);
	pid.setSetPoint(0);
	calculater.attach(pid_calculater ,10ms);

	// LED点灯、電源オフ
	LED.write(1);
	emergency.write(0);

	// ラズピコから来たコマンド文字列置き場
	char command_from_raspPico[64];

	// コマンド文字列の添字
	int index = 0;

    while(true){
        while(!queue.empty()){
            queue.pop(Rxmsg);
            datachange(M1.ID, &M1, &Rxmsg);
        }
        printf("%d %d %d\n", M1.counts, M1.rpm, M1.current);  

		pid.setProcessValue(M1.rpm);

		// 下半身から照射きたら電源オン
		if(!PataPataState) 	emergency.write(0);
		else				emergency.write(1);

		if(raspPico.read(&are, 1) > 0){
			if(are == '\n'){
				command_from_raspPico[index] = '\0';
			}
			switch(command_from_raspPico[0]){
				case 'u':	// ゆっくり上げる
					// 速度を受け取ったパラメータに合わせる
					pid.setSetPoint(atoi(&command_from_raspPico[1]));
					break;
				case 'd':	// ゆっくり下げる
					// 速度を受け取ったパラメータに合わせる
					pid.setSetPoint(atoi(&command_from_raspPico[1]));
					break;
				case 't':	// 自動収穫（一定時間上げ下げ）
					tryer();
					break;
				case 'n':	// 一定まで上げる
					nullpo();
					break;
				case 'g': 	// フタ開閉
					hutaPakaPaka();
				case 'h':	// 最低点まで下げる
					goHome();
				default:
					pid.setSetPoint(0);
					break;
			}
			index = 0;
			memset(command_from_raspPico, 0, sizeof(command_from_raspPico));
		}else{
			command_from_raspPico[index] = are;
			index++;
		}
    }	
}

void canListen(){
    CANMessage Rcvmsg;
    if (can.read(Rcvmsg)){
        queue.push(Rcvmsg);
    }
}

void datachange(unsigned ID, struct C610Data *C610, CANMessage *Rcvmsg){
    if(ID == Rcvmsg->id){
        C610->counts = uint16_t((Rcvmsg->data[0] << 8) | Rcvmsg->data[1]);
        C610->rpm = int16_t((Rcvmsg->data[2] << 8) | Rcvmsg->data[3]);
        C610->current = int16_t((Rcvmsg->data[4] << 8) | Rcvmsg->data[5]);
    }
}

void TorqueToBytes(uint16_t torqu, unsigned char *upper, unsigned char *lower){
    *upper = (torqu >> 8) & 0xFF;
    *lower = torqu & 0XFF;
}

void sendData(const int32_t torqu0){
    int16_t t0,t1;
    if(torqu0>32000){
        t0 = 32000;
    }else if(torqu0<-32000){
        t0 = -32000;
    }else{
        t0 = torqu0;
    }

    CANMessage msg;
    msg.id = 0x200;
    TorqueToBytes(t0, &msg.data[0], &msg.data[1]);
    for(int i=2; i<8; i++){
        msg.data[i] = 0x00;
    }
    can.write(msg);
}

void tryer(void){
	Timer time;

	// 上限に到達していた場合、ちょっと下げてから
	if(!ueLimit){
		time.start();
		while(time.elapsed_time() <= TIMELIMIT && sitaLimit){
			pid.setSetPoint(-1 * fast_targetRPM);
		}
		time.reset();
		pid.setSetPoint(0);
		ThisThread::sleep_for(10ms);
	}

	// 一定時間上げる or 上限まで上げる
	time.start();
	while(time.elapsed_time() <= TIMELIMIT && ueLimit){
		pid.setSetPoint(fast_targetRPM);
	}
	time.reset();

	// 一定時間下げる or 下限まで下げる
	time.start();
	while(time.elapsed_time() <= TIMELIMIT && sitaLimit){
		pid.setSetPoint(-1 * fast_targetRPM);
	}
	time.reset();

	pid.setSetPoint(0);
}

void nullpo(void){ // ガッ
	Timer time;

	time.start();
	while(time.elapsed_time() <= TIMELIMIT && ueLimit){
		pid.setSetPoint(fast_targetRPM);
	}
	time.reset();

	pid.setSetPoint(0);
}

void hutaPakaPaka(void){
	huta_servo.pulsewidth_us(700);
	ThisThread::sleep_for(1s);
	huta_servo.pulsewidth_us(2300);
}

void goHome(void){
	Timer time;

	time.start();

	// リミス死亡時のときのために一応時間制御もつけておく
	// これは収穫時のTIMELIMITとは異なるため注意
	while(time.elapsed_time() <= 3s && sitaLimit){
		pid.setSetPoint(-1 * fast_targetRPM);
	}
	time.reset();

	pid.setSetPoint(0);
}

void pid_calculater(void){
	sendData(pid.compute());
}