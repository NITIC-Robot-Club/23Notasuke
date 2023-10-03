#include "mbed.h"
#include <cstdint>
#include <cstdio>
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
PID			pid(kp, ki, kd, 0.05);
const int 	slow_targetRPM = 1000; 	// 手動昇降目標値[rpm]
const int 	fast_targetRPM = 4000; 	// 自動一定上げ目標値[rpm]
Ticker		calculater;				// pid.conpute()を一定間隔でアレしたい

// CAN受信用
void canListen(void);

// 受信データの形式の変更
void datachange(unsigned ID, struct C610Data *C610, CANMessage *Rcvmsg);

// 電流値を送信用に変換
void TorqueToBytes(uint16_t torqu, unsigned char *upper, unsigned char *lower);

// ESCにデータ送信
void sendData(const int32_t torqu0);

// 一定上げ
void autotry(void);

// 最高-最低上下
void fulltry(void);

// シリアル読み
void reader(void);
char are;	// 信号入れるやつ

// pid計算機
void pid_calculater(void);

int main(void){
    can.attach(&canListen, CAN::RxIrq);

	int 	OutPutCurrent;

    struct C610Data M1;
    M1.ID = 0x201;
    CANMessage Rxmsg;

	pid.setOutputLimits(-8000, 8000);
	pid.setInputLimits(-18000, 18000);
	calculater.attach(pid_calculater ,10ms);

    while(true){
        while(!queue.empty()){
            queue.pop(Rxmsg);
            datachange(M1.ID, &M1, &Rxmsg);
        }
        printf("%d %d %d\n", M1.counts, M1.rpm, M1.current);  

		pid.setProcessValue(M1.rpm);


		pc.attach(reader, UnbufferedSerial::RxIrq);
		// pc.read(&are,1);
		// raspPico.read(&are,1);

		float power;

		switch(are){
			case 'u':	// ゆっくり上げる
				pid.setSetPoint(slow_targetRPM);
				break;
			case 'd':	// ゆっくり下げる
				pid.setSetPoint(-1 * slow_targetRPM);
				break;
			case 't':	// 一定上げ
				autotry();
			case 'f':	// ごっつりオート収穫
				fulltry();
				break;
			case 'g': 	// フタ開閉
			case 'h':	// 最低点まで下げる
			default:
				// pid.setSetPoint(0);
				break;
		}
        // printf("%f\n",power);
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
        // printf("%d %d %d\n",C610->counts, C610->rpm, C610->current);
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

void autotry(void){
	Timer time;
	if(!ueLimit){
		time.start();
		while(time.elapsed_time() <= 2s && sitaLimit){
			pid.setSetPoint(-1 * fast_targetRPM);
		}
		time.reset();
		pid.setSetPoint(0);
		ThisThread::sleep_for(10ms);
	}
	time.start();
	while(time.elapsed_time() <= 2s && ueLimit){
		pid.setSetPoint(fast_targetRPM);
	}
	time.reset();
	pid.setSetPoint(0);
}

void fulltry(void){

	// まずは初期位置に戻す
	if(!ueLimit){	// 上限設定が押されていたら下限までいちど降ろす
		while(sitaLimit){
			pid.setSetPoint(-1 * fast_targetRPM);
		}
	}

	while(ueLimit){	// 上限まで上げる
		pid.setSetPoint(fast_targetRPM);
	}

	// 逆起電力防止のアレ
	pid.setSetPoint(0);
	ThisThread::sleep_for(50ms);

	while(sitaLimit){	// 下限まで戻す
		pid.setSetPoint(-1 * fast_targetRPM);
	}
	pid.setSetPoint(0);
}

void reader(void){
	pc.read(&are, 1);
}

void pid_calculater(void){
	sendData(pid.compute());
}