#include "mbed.h"
#include "PS3.h"
#include <cstdint>
#include <cstdio>
#include "PIDcontroller.h"

const float kp = 0.1;
const float ki = 0.001;
const float kd = 0.0;

// 上昇に必要な回転数
const int N = 10;

PS3     ps3(A0, A1);
RawCAN  can(D4, D10, 1000000);
PID     pid(kp, ki, kd, 0.050);
Ticker  getter;

CircularBuffer<CANMessage, 32> queue;


void getdata(void);

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

void datachange(unsigned ID, struct C610Data *C610, CANMessage *msg){
    if(ID == msg->id){
        C610->counts = uint16_t((msg->data[0] << 8) | msg->data[1]);
        C610->rpm = int16_t((msg->data[2] << 8) | msg->data[3]);
        C610->current = int16_t((msg->data[4] << 8) | msg->data[5]);
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


bool maru,batu,ue,sita;

int main(void){
    can.attach(&canListen, CAN::RxIrq);
    getter.attach(getdata, 10ms);

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
        printf("%d %d %d", M1.counts, M1.rpm, M1.current);  
        pulse = newpulse;
        newpulse = M1.counts;


        if(newpulse == 0){
            if(wise == 0){
                
            }else if(wise == 1){
                circle++;
            }else{
                circle--;
            }
        }

        if(circle == N){
            int pidcounts;

            // 最大値がわかんなくて泣いてる
            // 最大値に近かったら引いてPIDに無理無理突っ込め
            while(pid_flag){
                if(batu) break;
                if(M1.counts >= 8000 - 500){
                    pidcounts = M1.counts - 8000;
                }else{
                    pidcounts = M1.counts;
                }

                pid.setInputLimits(M1.counts-500, pidcounts+500);
                pid.setOutputLimits(8000, 16000);
                pid.setSetPoint(newpulse);
                pid.setProcessValue(pidcounts);

                sendData(pid.compute(),0);
            }

        }else{
            if(ue){
                sendData(32000, 0);
                wise = 1;
            }
            else if(sita){
                sendData(-32000, 0);
                wise = 2;
            }
            else{
                wise = 0;
                int pidcounts;

                // 最大値がわかんなくて泣いてる
                // 最大値に近かったら引いてPIDに無理無理突っ込め
                while(pid_flag){
                    if(batu) break;
                    if(M1.counts >= 8000 - 500){
                        pidcounts = M1.counts - 8000;
                    }else{
                        pidcounts = M1.counts;
                    }

                    pid.setInputLimits(pidcounts-500, pidcounts+500);
                    pid.setOutputLimits(-16000, 16000);
                    pid.setSetPoint(newpulse);
                    pid.setProcessValue(pidcounts);

                    sendData(pid.compute(),0);
                }
            }

        }
    }
}

void getdata(void){
    maru = ps3.getButtonState(PS3::maru);
    batu = ps3.getButtonState(PS3::batu);

    ue = ps3.getButtonState(PS3::ue);
    sita = ps3.getButtonState(PS3::sita);
}