#include "mbed.h"
#include "PIDcontroller.h"


UnbufferedSerial raspPico(PB_6, PB_7, 9600);
bool recv = false;

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
char RcvData[8] = {0x00};

// CANMessage sendmsg(0x000, TxData, 8, CANData, CANStandard);

struct C610Data{
    unsigned ID;
    int32_t counts;
    int32_t rpm;
    int32_t current;
}M1;


// 速度を固定する PIDでも全速になるくらいなはず
const int speed_u = 1000000;
const int speed_d = 1000000;

// PID用
const float	kp = 4.0;
const float	ki = 0.05;
const float	kd = 0.05;
const int 	slow_targetRPM = 300000; 	// 手動昇降目標値[rpm]
const int 	fast_targetRPM = 100000; 	// 自動一定上げ目標値[rpm]
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


// 最高 - 最低
void tryer();

// 最高まで
void nullpo();

void hutaPakaPaka(void);
bool isHutaClose = true;

void goHome(void);

// シリアル読み
void	reader(void);
char	are;	// シリアルのバッファ
char 	old_are;
char 	command_from_raspPico[64] = {};
int		index = 0;

// pid計算機
void pid_calculater(void);
int target = 0;
int turn_direction = 0;
int torqu = 0;

int main(void){

	// CAN周り設定
    M1.ID = 0x201;
    CANMessage Rxmsg;
    can.attach(&canListen, CAN::RxIrq);

	// PIDいろいろ設定
	pid.setInputLimits(0, 18000);
	pid.setOutputLimits(0, 32000);

	// LED点灯、電源オフ
    LED.write(1);
    emergency.write(1);

	calculater.attach(pid_calculater ,50ms);
    raspPico.attach(reader,SerialBase::RxIrq);
    while(true){
        while(!queue.empty()){
            queue.pop(Rxmsg);
            datachange(M1.ID, &M1, &Rxmsg);
        }

		// printf("%d %d %d\nue: %d\tsita:%d\nrpm:%d\n-----\n\n", M1.counts, M1.rpm, M1.current,ueLimit.read(),sitaLimit.read(),M1.rpm); 

		// 下半身から照射きたら電源オン
		// if(!PataPataState) 	emergency.write(0);
		// else				emergency.write(1);
		// if(!PataPataState) 	printf("0:ON\n");
		// else				printf("1:off\n");
        emergency.write(0);


        if(recv){
            recv = false;
            printf("%s\n",command_from_raspPico);
			switch(command_from_raspPico[0]){
				case 'u':	// ゆっくり上げる
					// 速度を受け取ったパラメータに合わせる
					target = atoi(&command_from_raspPico[1]);
					turn_direction = 1;
                    printf("UP Done\n");    
					break;
				case 'd':	// ゆっくり下げる
					// 速度を受け取ったパラメータに合わせる
					target = atoi(&command_from_raspPico[1]);
					turn_direction = -1;
                    printf("Down Done\n");
					break;
                case 's':
                    target = 0;
                    turn_direction = 0;
                    printf("stop Done\n");
                    break;
				case 't':	// 自動収穫（一定時間上げ下げ）
					tryer();
                    printf("t:Auot Done\n");
					break;
				case 'n':	// 一定まで上げる
					nullpo();
                    printf("n: to limit Done\n");
					break;
				case 'g': 	// フタ開閉
					hutaPakaPaka();
                    printf("open Done\n");
					break;
				case 'h':	// 最低点まで下げる
					goHome();
                    printf("Home Done\n");
					break;
			}
        }
        if(!ueLimit.read()      && turn_direction == 1)     target = 0;
        if(!sitaLimit.read()    && turn_direction == -1)    target = 0;
		sendData(torqu);
        char command_from_raspPico[64] ="";
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
    //暴走対策
    if(!ueLimit.read()      && turn_direction == 1)     t0 = 0;
    if(!sitaLimit.read()    && turn_direction == -1)    t0 = 0;
    CANMessage msg;
    msg.id = 0x200;
    TorqueToBytes(t0, &msg.data[0], &msg.data[1]);
    for(int i=2; i<8; i++){
        msg.data[i] = 0x00;
    }
    if(msg.data[0]!='\0')printf("msg.data:%d,%d\n",msg.data[0],msg.data[1]);
    can.write(msg);
}

void tryer(){

	// 上限に到達していた場合、ちょっと下げてから
	if(!ueLimit.read()){
		while(sitaLimit.read()){
			target = fast_targetRPM;
			turn_direction = -1;
		}
        target = 0;
		turn_direction = 0;
        //つまり対策?
		// ThisThread::sleep_for(10ms);
	}

	// 上限まで上げる
	if (ueLimit.read()){
		target = fast_targetRPM;
		turn_direction = 1;
	}
    target = 0;
	turn_direction = 0;
    // ThisThread::sleep_for(100ms);

	// 一定時間下げる or 下限まで下げる
	if(sitaLimit.read()){
		target = fast_targetRPM;
		turn_direction = -1;
	}
    target = 0;
	turn_direction = 0;
}

void nullpo(){ // ガッ
	Timer time;

	while(ueLimit.read()){
		target = fast_targetRPM;
		turn_direction = 1;
	}
    target = 0;
	turn_direction = 0;
}

void hutaPakaPaka(void){
    if(isHutaClose){
        isHutaClose = false;
        huta_servo.pulsewidth_us(2300);
        ThisThread::sleep_for(2s);
        huta_servo.pulsewidth_us(750);
        isHutaClose = true;
    }
}

void goHome(void){
	Timer time;
	while(sitaLimit){
		target = fast_targetRPM;
		turn_direction = -1;
	}
    target = 0;
	turn_direction = 0;
}

void reader(){
	old_are = are;
    raspPico.read(&are,1);
    command_from_raspPico[index] = are;
    index++;
    raspPico.write(&are,1);
    if(command_from_raspPico[index - 1] == '\n'){
        command_from_raspPico[index] = '\0';
        index = 0;
        recv = true;
    }
}

void pid_calculater(void){
	int nowRPM = M1.rpm;
	int error_sign = 1;
	pid.setProcessValue(abs(nowRPM));
	pid.setSetPoint(target);
	if((target - nowRPM) < 0) error_sign = -1;
	torqu = pid.compute() * error_sign * turn_direction;
}