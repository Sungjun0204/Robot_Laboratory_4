/*
 * GccApplicatikhyojon1.c
 *
 * Author : KHS
 */ 

#include "mcu_init.h"
#include "dataType.h"

#include <math.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////


volatile int32_t g_Cnt, g_preCnt;

volatile double g_Pdes = 0., g_Ppre;
volatile double g_Pcur, g_Pvcur;
volatile double g_Perr;

volatile double g_Vcur, g_Vpre;
volatile double g_Vdes = 0.2;
volatile double g_Verr;
volatile double g_Vlimit = 10.;


volatile double g_Ccur;
volatile double g_Cdes;
volatile double g_Cerr;
volatile double g_Cerr_sum;
volatile double g_Climit = 0.1;

volatile double g_ADC;
volatile int g_SendFlag = 0;
volatile int g_Direction;

volatile int cur_control = 0;
volatile double g_vel_control;
volatile double g_pos_control;
volatile unsigned char g_TimerCnt;

volatile unsigned char checkSize;
volatile unsigned char g_buf[256], g_BufWriteCnt, g_BufReadCnt;

volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID = 1;


// 추가한 변수들
volatile double g_Pderr;
volatile double g_Verr_sum;

volatile double Kp_p = 4.5;
volatile double Kd_p = 0.15;

volatile double Kp_s = 1.5042;
volatile double Ki_s = 46.660104;

//volatile double Kp_p = 12.5664;
//volatile double Kd_p = 0.1;

//volatile double Kp_s = 0.0324;
//volatile double Ki_s = 8.6274;

volatile double Kp_c = 0.826867;
volatile double Ki_c = 2211.7;




// 모터에 PWM 값을 지정해 주는 함수 
void SetDutyCW(double v){
	
	while(TCNT1  == 0);

	int ocr = v * (200. / 24.) + 200;
	
	if(ocr > OCR_MAX)	ocr = OCR_MAX;
	else if(ocr < OCR_MIN)	ocr = OCR_MIN;	// OCR값을 10~390으로 제한
	//OCR1A = OCR1B = ocr;
	
	OCR1A = OCR3B = ocr + 8;		//1 H
	OCR1B = OCR3A = ocr - 8;		//1 L
}


void InitLS7366(){
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR0 | WR_REG);
	SPI_MasterSend(X4_QUAD | FREE_RUN | DISABLE_INDEX | SYNCHRONOUS_INDEX |FILTER_CDF_1);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR1 | WR_REG);
	SPI_MasterSend(FOUR_BYTE_COUNT_MODE | ENABLE_COUNTING);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_CNTR | CLR_REG);
	PORTB = 0x01;
}


// ADC를 받아오는 함수: 전류센서 값
int getADC(char ch){

	ADMUX = (ADMUX & 0xf0) + ch;
	ADCSRA |= 0x40;
	while(!(ADCSRA & 0x10));
	return ADC;
}



// USART0번 인터럽트 >> UART0번을 통해 MFC로부터 값이 들어오면 발동
// 이제 여기서부터 ATMega128의 역할 시작
ISR(USART0_RX_vect){

	g_buf[g_BufWriteCnt++] = UDR0;
	// 값이 들어오면 값이 들어오는 대로 버퍼에 값들 저장
}



// 타이머 인터럽트 >> 본격적인 메인 코드 내용
//ISR(TIMER3_OVF_vect){
ISR(TIMER0_OVF_vect){
			
	TCNT0 = 256 - 125;			// 131로 설정함: 제어주기 2ms
	//TCNT3 = 65536 - 125;		
	//Read LS7366
	int32_t cnt;
	
	PORTC = 0x01;
	
	g_ADC = getADC(0);			// ADC 0번 값을 읽어 옴: ADC 0번 (PF0)은 전류센서 연결
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | LOAD_REG);	// 0010 1000 | 1100 0000 = 1110 1000
	PORTB = 0x01;
			
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | RD_REG);
	cnt = SPI_MasterRecv();		cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();
	PORTB = 0x01;
	g_Cnt = -cnt;
	
	PORTC = 0x03;
	
	g_Pcur = (g_Cnt / (4096. * 81.)) * 2 * M_PI;		// 엔코더로 받은 현재 shaft의 위치 정보
	
	
	////////////////////////////// 제어 시작 /////////////////////////////
	//TO DO
	
	/*
	//Position Control
	if((g_TimerCnt % 100) == 0){
		
		g_TimerCnt = 0;
		
	}
	
	//Velocity Control
	if((g_TimerCnt % 10) == 0){
		
		g_Vcur = (g_Pcur - g_Pvcur) / 0.005;
		g_Pvcur = g_Pcur;
	}
	g_TimerCnt++;
	
	//Current Control
	g_Cdes = -0.1; //cascade시 변경 필요.
	
	g_Ccur = -( ((g_ADC / 1024. * 5.) - 2.5) * 10.); 
	g_Cerr = g_Cdes - g_Ccur;				

	cur_control = g_Cerr * 0.1 + g_Cerr_sum * 1.5;
	cur_control += g_Vcur * 0.0683;
	
	g_Cerr_sum += g_Cerr;
	
	//I-term anti windup
	if(cur_control >= 24){
		g_Cerr_sum -= (cur_control - 24.) * 1. / 0.1 / 3.;
		cur_control = 24;
	}
	else if(cur_control < -24){
		g_Cerr_sum -= (cur_control + 24.) * 1. / 0.1 / 3.;
		cur_control = -24;
	}
	
	SetDutyCW(cur_control);				// 최종 PWM 보냄
	
	
	*/
	
	
	
	
	
	if((g_TimerCnt % 100) == 0){		// position control
		g_TimerCnt = 0;
		//g_Pdes = -1.62;
		if(g_Pdes < 0) g_Pdes + 2*M_PI;	// 음수 target시 0~360으로 표현
		
		g_Perr = g_Pdes - (g_Pcur);		// 목표 위치 값 - 현재 위치 값 = position error 
		g_Pderr = g_Perr - g_Ppre;		// 현재 에러 값 - 이전 에러 값 = position error_dot
		
		/*
		// 효율적인 도는 방향 정하기
		if(g_Perr > M_PI) {
			g_Perr -= 2*M_PI;
		}
		else if(g_Perr < -M_PI) {
			g_Perr += 2*M_PI;
		}
		*/
		
		g_pos_control = (double) g_Perr * Kp_p +  g_Pderr* Kd_p;		//PD 제어기
		// 위치 제어의 결과 값으로 속도 값이 도출됨

		// gear가 적용된 motor의 최대 속도는 642.65/81 [rad/sec]
		// 모터의 최대/최소 속도에 대한 saturation 설정
		if(g_pos_control > 642.65/81.){
			g_pos_control = 642.65/81.;
		}
		else if(g_pos_control < -642.65/81.){
			g_pos_control = -642.65/81.;
		}
		
		g_Ppre = g_Perr;		// 현재 위치 에러 값을 이전 위치 에러 값으로 저장
		
	}
	
	
	
	if((g_TimerCnt % 10) == 0){			// speed control
		
		// speed limit -Vlimit ~ +Vlimit
		// 목표 속도 값을 기준으로 saturation 설정
		
		if(g_pos_control > g_Vlimit){
			g_pos_control = g_Vlimit;
		}
		else if(g_pos_control < -g_Vlimit){
			g_pos_control = -g_Vlimit;
		}
		
		// saturation 적용 된 최종 위치 제어 값을 목표 속도 값으로 저장 
		g_Vdes = g_pos_control; // [rad/sec]
		
		
		g_Vcur = (g_Pcur - g_Pvcur) / 0.005;	// (현재 엔코더 값 - 이전 엔코더 값) / 0.005초 -> 현재속도 [rad/sec]
		g_Pvcur = g_Pcur;						// 현재 엔코더 값을 이전 엔코더 값으로 저장
		
		g_Verr = g_Vdes - g_Vcur;  // (목표 속도 값 - 현재 속도 값) = 속도 값 에러
		
		g_vel_control = g_Verr * Kp_s + g_Verr_sum * Ki_s * 0.005;		// PI 제어기
		// 속도 제어의 결과 값으로 전류 값이 도출됨
		
		
		g_Verr_sum += g_Verr;	// 속도 값에 대한 에러 값을 계속 누적
		
		// 최대 허용 전류 값에 대한 saturation & anti-windup
		if(g_vel_control > 2.08){
			g_Verr_sum -= (g_vel_control - 2.08) * 1. / 2.5042;	//  anti windup gain은 1/Kps
			g_vel_control = 2.08;
		}
		else if(g_vel_control < -2.08){
			g_Verr_sum -= (g_vel_control + 2.08) * 1. / 2.5042; //  anti windup gain은 1/Kps
			g_vel_control = -2.08;
		}

	}
	g_TimerCnt++;
	
	// torque control 		2000Hz
	//g_Cdes = -0.1;	// current target
	
	g_Cdes = -g_vel_control;		// 속도 제어 값의 부호를 반전시켜 목표 전류 값으로 다시 저장 

	// 전류 값에 대한 saturation
	if(g_Cdes > g_Climit)
		g_Cdes = g_Climit;
	else if(g_Cdes < -g_Climit)
		g_Cdes = -g_Climit;				
	
	
	g_Ccur = -( ((g_ADC / 1024. * 5.) - 2.488 + 0.005) * 10.);		// 전류 센서를 통해 받은 전류 값(ADC)을 통해 현재 전류 값 계산
	g_Cerr = g_Cdes - g_Ccur;				// (목표 전류 값 - 현재 전류 값) = 전류 값 에러

	cur_control = g_Cerr * Kp_c + g_Cerr_sum * Ki_c * 0.0005;	// PI 제어기:  200hz이므로 0.0005초에 한번 이므로 적분부에 0.005를 곱해줌
	cur_control += g_Vcur * 0.0683;				// 역기전력 보상
	
	g_Cerr_sum += g_Cerr;		// current error sum(I-term)
	
	
	//I-term anti-wind up   누적 오차 해소
	if(cur_control >= 24){									//최대 출력 24V
		g_Cerr_sum -= (cur_control - 24.) * 1. /  Kp_c;		// anti windup 계수 1/3kp
		cur_control = 24;
	}
	else if(cur_control < -24){
		g_Cerr_sum -= (cur_control + 24.) * 1. / Kp_c;		// anti windup 계수 1/3kp
		cur_control = -24;
	}
	

	// 최종적으로 전류 제어 값으로 전압 값을 계산하기 위해 값을 넘겨 줌
	SetDutyCW(cur_control);				// target Voltage
	
	
	

	////////////////////////////// 제어 종료 //////////////////////////
	
	g_SendFlag++;

}



int main(void){
	
	Packet_t packet;
	packet.data.header[0] = packet.data.header[1] = packet.data.header[2] = packet.data.header[3] = 0xFE;
				// 패킷을 보낼 때 맨 앞 header 데이터 설정
	
	InitIO();
	
	//Uart
	InitUart0();	// atmega128에서 MFC로 serial 통신을 위한 USART 레지스터 설정 함수
	InitUart1();	// atmega128에서 시리얼 통신1.9b로 serial 통신을 위한 USART 레지스터 설정 함수
	
	//SPI
	InitSPI();
	
	//Timer
	InitTimer0();
	InitTimer1();
	InitTimer3();


	TCNT1 = TCNT3 = 0;
	SetDutyCW(0.);
	
	//ADC
	InitADC();
	
	//LS7366
	InitLS7366();
	
	//TCNT3 = 65536 - 125;
	TCNT0 = 256 - 125;
	sei();

	unsigned char check = 0;
	
    while (1) {
		
		// 패킷 통신 해석 코드 >> 참고로 해당 코드를 작성한 13승민좌께서 AX-12모터의 패킷 통신 방식을 참고하여 만드셨다고 함
		for(;g_BufReadCnt != g_BufWriteCnt; g_BufReadCnt++){	
			// 반복문 선언: 버퍼로 받은 패킷 데이터 다 읽을때 까지 for문 반복 
			// SendShortUART1(g_PacketMode); TransUart1(32); TransUart1(13);
			// Packet 모드에 따라 switch문 발동
			switch(g_PacketMode){
			
			///////					
			case 0:									// 프로그램 처음 시작할 때는 g_PacketMode = 0
			///////									// 패킷 통신 시작을 알리는 0xFF 4개가 들어오는지 판단하는 case
				// SendShortUART1(g_buf[g_BufReadCnt]); TransUart1(32); TransUart1(13);
				if (g_buf[g_BufReadCnt] == 0xFE) {	// 버퍼에 0xFE가 들어오면
					checkSize++;					// checkSize 1 증가
					if (checkSize == 4) {			// 증가하다가 checkSize가 가 되면 
						g_PacketMode = 1;			// 패킷 모드 1로 변경
					}
				}
				else {								// 만약 checkSize가 0xFF가 아니면
					checkSize = 0;					// 패킷 통신 start 부분이 아니라고 판단하여 다시 checkSize = 0으로 초기화
				}
				break;
				
				
			////////
			case 1:									// case 1로 넘어왔다는 의미는 패킷 통신의 실제 내용을 읽기 시작한다는 의미
			///////									// 즉, 패킷으로 받은 데이터의 본격적인 분석 시작이지만, 해당 case에서는
													// 로실 모터를 돌리기 위한 패킷 데이터인지 확인하는 작업을 진행한다. 

				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];	
													// 버퍼에 저장되어 있는 패킷의 본 데이터들을 분리하기 위해 별도의 배열로 이동
				
				if (checkSize == 8) {				// 패킷의 실제 데이터는 7개이므로, checkSize = 8이 되는 순간 패킷 내용 분석 시작
					if(g_PacketBuffer.data.id == g_ID){		// 해당 패킷 데이터의 ID가 일치하면

						g_PacketMode = 2;					// g_PacketMode = 2로 변경 후 다음 작업 진행
					}
					else{									// 만약 g_PacketMode = 2가 아니라면
						g_PacketMode = 0;					// 해당 모터를 돌리기 위한 패킷이 아니라고 판단하고 처음부터 다시 시작
						checkSize = 0;						// checkSize도 다시 0으로 초기화
					}
				}

				break;
			
			///////
			case 2:									// case 2까지 왔다는 것은 해당 로실 모터를 돌리기 위한 패킷이 제대로 넘어왔다는 의미
			///////									// 이제부터 패킷으로 받은 데이터의 본격적인 분석 시작
				
				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				check += g_buf[g_BufReadCnt];
				
				if (checkSize == g_PacketBuffer.data.size) {		// 패킷 데이터 크기가 일치 여부 확인

					if(check == g_PacketBuffer.data.check){			// 마지막으로 체크 데이터가 일치하는지 확인 

						switch(g_PacketBuffer.data.mode){			// 패킷 데이터로 받은 모드 비트 값 확인

							case 2:									// 모드 비트 값이 2일 때 
							g_Pdes = g_PacketBuffer.data.pos / 1000.;
							g_Vlimit = g_PacketBuffer.data.velo / 1000.;
							g_Climit = g_PacketBuffer.data.cur / 1000.;		// 본 패킷의 진짜 데이터 세 개를 추출하여 각 변수에 저장

							//SendShortUART1(g_Pdes); TransUart1(32);
							//SendShortUART1(g_Vlimit); TransUart1(32);
							//SendShortUART1(g_Climit); TransUart1(32);		// MFC(UART0)에서 ATMega128로 넘어오는 값dmf UART1로 확인
							//TransUart1(13);
							break;											// 해당 변수들은 제어주기문에서 각종 계산을 통해 다시 
							}												// 패킷으로 포장되어 MFC로 송신할 것이다. 
					}
					
					check = 0;
					g_PacketMode = 0;
					checkSize = 0;											// 패킷에서 데이터 추출을 완료했다면 패킷을 분석하기 위한
				}															// 각종 파라미터들을 0으로 초기화. 이후 다음 통신을 대긴
				
				else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)) {
																			// 그런데 데이터 크기가 일치하지 않으면
					TransUart0('f');										// 문자 f 출력 후 파라미터들을 0으로 초기화 후
					check = 0;												// 다음 통신을 대기
					g_PacketMode = 0;
					checkSize = 0;
				}
			}
		}
		
		
		
		
		/////////////////////////////////////////////////////////////
		// 여기부터는 반대로 ATMega128에서 MFC로 데이터를 보내는 코드 //
		////////////////////////////////////////////////////////////
		
		if(g_SendFlag > 19){			// 데이터 송신 플래그 g_SendFlag 가 20 이상이면
			g_SendFlag = 0;				// g_SendFlag 를 0으로 초기화한 후 데이터 송신 패킷 제작 시작
										// 참고로 g_SendFlag 변수는 제어주기 코드에서 증가됨

		
		// 패킷에 들어갈 데이터들을 각각의 위치에 저장
			packet.data.size = sizeof(Packet_data_t);		// size = 20
			packet.data.id = g_ID;							// g_ID = 1
			packet.data.mode = 3;							// mode = 3
			packet.data.check = 0;							// checksum 저장할 곳을 0으로 초기화
			
			/*
			packet.data.pos = g_Pdes * 1000;		
			packet.data.velo = g_Vlimit * 1000;		
			packet.data.cur = g_Climit * 1000;				// 소수점 아래 값 손실 방지를 위해 1000을 곱한 후 송신
			*/
			
			packet.data.pos = g_Pcur * 1000;
			packet.data.velo = g_Vcur * 1000;
			packet.data.cur = g_Ccur * 1000;
			
			
			for (int i = 8; i < sizeof(Packet_t); i++)
			packet.data.check += packet.buffer[i];			// checksum 제작: pos~cur데이터를 다 더하고 저장
			
			for(int i=0; i<packet.data.size; i++)
				TransUart0(packet.buffer[i]);				// 최종 패킷 발송
				
			for(int i=0; i<packet.data.size; i++){			// 얘는 MFC로 받은 값이 ATMega로 잘 받아졌는지 확인하는 용도
				TransNumUart1(packet.buffer[i]);			// 시리얼 통신 1.9b로 쏴 주는 코드
				TransUart1(32);	// 스페이스바
			}
			TransUart1(13);		// 줄 바꿈
			
		}
	}
		
}

