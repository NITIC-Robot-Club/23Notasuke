#include "mbed.h"
#include <cstdint>
#include <cstdio>

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

// CAN受信用
void canListen(void);

// 受信データの形式の変更
void datachange(unsigned ID, struct C610Data *C610, CANMessage *Rcvmsg);

// 電流値を送信用に変換
void TorqueToBytes(uint16_t torqu, unsigned char *upper, unsigned char *lower);

// ESCにデータ送信
void sendData(const int32_t torqu0);

// 自動昇降
void autoUpDown(void);

int main(void){
    can.attach(&canListen, CAN::RxIrq);

	char 	are;
	int 	OutPutCurrent;

    struct C610Data M1;
    M1.ID = 0x201;
    CANMessage Rxmsg;

	// 電源をオフにしておく
	emergency.write(0);

    while(true){
        while(!queue.empty()){
            queue.pop(Rxmsg);
            datachange(M1.ID, &M1, &Rxmsg);
        }
        printf("%d %d %d\n", M1.counts, M1.rpm, M1.current);  

		// 信号が来たら電源オン
		if(!PataPataState){
			emergency.write(0);
		}else{
			emergency.write(1);
		}

		pc.read(&are,1);
		raspPico.read(&are,1);

		switch(are){
			case 'a':	// ゆっくり上げる
				OutPutCurrent = 3000 * ueLimit;
				break;
			case 'b':	// ゆっくり下げる
				OutPutCurrent = -3000 * sitaLimit;
				break;
			case 'q':	// ごっつりオート収穫
				autoUpDown();
				break;
			default:
				OutPutCurrent = 0;
				break;
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

void autoUpDown(void){

	// まずは初期位置に戻す
	if(!ueLimit){	// 上限設定が押されていたら下限までいちど降ろす
		while(sitaLimit){
			sendData(-4000);
		}
	}

	while(ueLimit){	// 上限まで上げる
		sendData(4000);
	}

	while(sitaLimit){	// 下限まで戻す
		sendData(-4000);
	}
}