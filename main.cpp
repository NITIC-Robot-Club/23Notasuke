#include "mbed.h"
#include <cstdint>
#include <cstdio>

// UnbufferedSerial 	raspPico	(PB_6,PB_7);
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


//  受信割り込み関数
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

void sendData(const int32_t torqu0, const int32_t torqu1){
    int16_t t0,t1;
    if(torqu0>32000){
        t0 = 32000;
    }else if(torqu0<-32000){
        t0 = -32000;
    }else{
        t0 = torqu0;
    }
    if(torqu1>32000){
        t1 = 32000;
    }else if(torqu1<-32000){
        t1 = -32000;
    }else{
        t1 = torqu1;
    }

    CANMessage msg;
    msg.id = 0x200;
    TorqueToBytes(t0, &msg.data[0], &msg.data[1]);
    TorqueToBytes(t1, &msg.data[2], &msg.data[4]);
    for(int i=5; i<8; i++){
        msg.data[i] = 0x00;
    }
    can.write(msg);
}

int main(void){
    can.attach(&canListen, CAN::RxIrq);

    int pulse = 0;
    int newpulse = 0;

    // 1で上昇、2で下降（ホンマか？）
    int wise = 0;
    int circle = 0;

    bool pid_flag = true;

    struct C610Data M1;
    M1.ID = 0x201;
    CANMessage Rxmsg;
    while(true){
        while(!queue.empty()){
            queue.pop(Rxmsg);
            datachange(M1.ID, &M1, &Rxmsg);
        }
        printf("%d %d %d\n", M1.counts, M1.rpm, M1.current);  

		char are;
		float data;
		pc.read(&are,1);
		if(are == 'a'){
			sendData(4000, 0);
			wise = 1;
		}
		else if(are == 'b'){
			sendData(-4000, 0);
			wise = 2;
		}else if(are == 'z'){
			sendData(0,0);
		}
    }
}
