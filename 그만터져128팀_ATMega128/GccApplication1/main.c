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


// �߰��� ������
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




// ���Ϳ� PWM ���� ������ �ִ� �Լ� 
void SetDutyCW(double v){
	
	while(TCNT1  == 0);

	int ocr = v * (200. / 24.) + 200;
	
	if(ocr > OCR_MAX)	ocr = OCR_MAX;
	else if(ocr < OCR_MIN)	ocr = OCR_MIN;	// OCR���� 10~390���� ����
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


// ADC�� �޾ƿ��� �Լ�: �������� ��
int getADC(char ch){

	ADMUX = (ADMUX & 0xf0) + ch;
	ADCSRA |= 0x40;
	while(!(ADCSRA & 0x10));
	return ADC;
}



// USART0�� ���ͷ�Ʈ >> UART0���� ���� MFC�κ��� ���� ������ �ߵ�
// ���� ���⼭���� ATMega128�� ���� ����
ISR(USART0_RX_vect){

	g_buf[g_BufWriteCnt++] = UDR0;
	// ���� ������ ���� ������ ��� ���ۿ� ���� ����
}



// Ÿ�̸� ���ͷ�Ʈ >> �������� ���� �ڵ� ����
//ISR(TIMER3_OVF_vect){
ISR(TIMER0_OVF_vect){
			
	TCNT0 = 256 - 125;			// 131�� ������: �����ֱ� 2ms
	//TCNT3 = 65536 - 125;		
	//Read LS7366
	int32_t cnt;
	
	PORTC = 0x01;
	
	g_ADC = getADC(0);			// ADC 0�� ���� �о� ��: ADC 0�� (PF0)�� �������� ����
	
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
	
	g_Pcur = (g_Cnt / (4096. * 81.)) * 2 * M_PI;		// ���ڴ��� ���� ���� shaft�� ��ġ ����
	
	
	////////////////////////////// ���� ���� /////////////////////////////
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
	g_Cdes = -0.1; //cascade�� ���� �ʿ�.
	
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
	
	SetDutyCW(cur_control);				// ���� PWM ����
	
	
	*/
	
	
	
	
	
	if((g_TimerCnt % 100) == 0){		// position control
		g_TimerCnt = 0;
		//g_Pdes = -1.62;
		if(g_Pdes < 0) g_Pdes + 2*M_PI;	// ���� target�� 0~360���� ǥ��
		
		g_Perr = g_Pdes - (g_Pcur);		// ��ǥ ��ġ �� - ���� ��ġ �� = position error 
		g_Pderr = g_Perr - g_Ppre;		// ���� ���� �� - ���� ���� �� = position error_dot
		
		/*
		// ȿ������ ���� ���� ���ϱ�
		if(g_Perr > M_PI) {
			g_Perr -= 2*M_PI;
		}
		else if(g_Perr < -M_PI) {
			g_Perr += 2*M_PI;
		}
		*/
		
		g_pos_control = (double) g_Perr * Kp_p +  g_Pderr* Kd_p;		//PD �����
		// ��ġ ������ ��� ������ �ӵ� ���� �����

		// gear�� ����� motor�� �ִ� �ӵ��� 642.65/81 [rad/sec]
		// ������ �ִ�/�ּ� �ӵ��� ���� saturation ����
		if(g_pos_control > 642.65/81.){
			g_pos_control = 642.65/81.;
		}
		else if(g_pos_control < -642.65/81.){
			g_pos_control = -642.65/81.;
		}
		
		g_Ppre = g_Perr;		// ���� ��ġ ���� ���� ���� ��ġ ���� ������ ����
		
	}
	
	
	
	if((g_TimerCnt % 10) == 0){			// speed control
		
		// speed limit -Vlimit ~ +Vlimit
		// ��ǥ �ӵ� ���� �������� saturation ����
		
		if(g_pos_control > g_Vlimit){
			g_pos_control = g_Vlimit;
		}
		else if(g_pos_control < -g_Vlimit){
			g_pos_control = -g_Vlimit;
		}
		
		// saturation ���� �� ���� ��ġ ���� ���� ��ǥ �ӵ� ������ ���� 
		g_Vdes = g_pos_control; // [rad/sec]
		
		
		g_Vcur = (g_Pcur - g_Pvcur) / 0.005;	// (���� ���ڴ� �� - ���� ���ڴ� ��) / 0.005�� -> ����ӵ� [rad/sec]
		g_Pvcur = g_Pcur;						// ���� ���ڴ� ���� ���� ���ڴ� ������ ����
		
		g_Verr = g_Vdes - g_Vcur;  // (��ǥ �ӵ� �� - ���� �ӵ� ��) = �ӵ� �� ����
		
		g_vel_control = g_Verr * Kp_s + g_Verr_sum * Ki_s * 0.005;		// PI �����
		// �ӵ� ������ ��� ������ ���� ���� �����
		
		
		g_Verr_sum += g_Verr;	// �ӵ� ���� ���� ���� ���� ��� ����
		
		// �ִ� ��� ���� ���� ���� saturation & anti-windup
		if(g_vel_control > 2.08){
			g_Verr_sum -= (g_vel_control - 2.08) * 1. / 2.5042;	//  anti windup gain�� 1/Kps
			g_vel_control = 2.08;
		}
		else if(g_vel_control < -2.08){
			g_Verr_sum -= (g_vel_control + 2.08) * 1. / 2.5042; //  anti windup gain�� 1/Kps
			g_vel_control = -2.08;
		}

	}
	g_TimerCnt++;
	
	// torque control 		2000Hz
	//g_Cdes = -0.1;	// current target
	
	g_Cdes = -g_vel_control;		// �ӵ� ���� ���� ��ȣ�� �������� ��ǥ ���� ������ �ٽ� ���� 

	// ���� ���� ���� saturation
	if(g_Cdes > g_Climit)
		g_Cdes = g_Climit;
	else if(g_Cdes < -g_Climit)
		g_Cdes = -g_Climit;				
	
	
	g_Ccur = -( ((g_ADC / 1024. * 5.) - 2.488 + 0.005) * 10.);		// ���� ������ ���� ���� ���� ��(ADC)�� ���� ���� ���� �� ���
	g_Cerr = g_Cdes - g_Ccur;				// (��ǥ ���� �� - ���� ���� ��) = ���� �� ����

	cur_control = g_Cerr * Kp_c + g_Cerr_sum * Ki_c * 0.0005;	// PI �����:  200hz�̹Ƿ� 0.0005�ʿ� �ѹ� �̹Ƿ� ���кο� 0.005�� ������
	cur_control += g_Vcur * 0.0683;				// �������� ����
	
	g_Cerr_sum += g_Cerr;		// current error sum(I-term)
	
	
	//I-term anti-wind up   ���� ���� �ؼ�
	if(cur_control >= 24){									//�ִ� ��� 24V
		g_Cerr_sum -= (cur_control - 24.) * 1. /  Kp_c;		// anti windup ��� 1/3kp
		cur_control = 24;
	}
	else if(cur_control < -24){
		g_Cerr_sum -= (cur_control + 24.) * 1. / Kp_c;		// anti windup ��� 1/3kp
		cur_control = -24;
	}
	

	// ���������� ���� ���� ������ ���� ���� ����ϱ� ���� ���� �Ѱ� ��
	SetDutyCW(cur_control);				// target Voltage
	
	
	

	////////////////////////////// ���� ���� //////////////////////////
	
	g_SendFlag++;

}



int main(void){
	
	Packet_t packet;
	packet.data.header[0] = packet.data.header[1] = packet.data.header[2] = packet.data.header[3] = 0xFE;
				// ��Ŷ�� ���� �� �� �� header ������ ����
	
	InitIO();
	
	//Uart
	InitUart0();	// atmega128���� MFC�� serial ����� ���� USART �������� ���� �Լ�
	InitUart1();	// atmega128���� �ø��� ���1.9b�� serial ����� ���� USART �������� ���� �Լ�
	
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
		
		// ��Ŷ ��� �ؼ� �ڵ� >> ����� �ش� �ڵ带 �ۼ��� 13�¹��²��� AX-12������ ��Ŷ ��� ����� �����Ͽ� ����̴ٰ� ��
		for(;g_BufReadCnt != g_BufWriteCnt; g_BufReadCnt++){	
			// �ݺ��� ����: ���۷� ���� ��Ŷ ������ �� ������ ���� for�� �ݺ� 
			// SendShortUART1(g_PacketMode); TransUart1(32); TransUart1(13);
			// Packet ��忡 ���� switch�� �ߵ�
			switch(g_PacketMode){
			
			///////					
			case 0:									// ���α׷� ó�� ������ ���� g_PacketMode = 0
			///////									// ��Ŷ ��� ������ �˸��� 0xFF 4���� �������� �Ǵ��ϴ� case
				// SendShortUART1(g_buf[g_BufReadCnt]); TransUart1(32); TransUart1(13);
				if (g_buf[g_BufReadCnt] == 0xFE) {	// ���ۿ� 0xFE�� ������
					checkSize++;					// checkSize 1 ����
					if (checkSize == 4) {			// �����ϴٰ� checkSize�� �� �Ǹ� 
						g_PacketMode = 1;			// ��Ŷ ��� 1�� ����
					}
				}
				else {								// ���� checkSize�� 0xFF�� �ƴϸ�
					checkSize = 0;					// ��Ŷ ��� start �κ��� �ƴ϶�� �Ǵ��Ͽ� �ٽ� checkSize = 0���� �ʱ�ȭ
				}
				break;
				
				
			////////
			case 1:									// case 1�� �Ѿ�Դٴ� �ǹ̴� ��Ŷ ����� ���� ������ �б� �����Ѵٴ� �ǹ�
			///////									// ��, ��Ŷ���� ���� �������� �������� �м� ����������, �ش� case������
													// �ν� ���͸� ������ ���� ��Ŷ ���������� Ȯ���ϴ� �۾��� �����Ѵ�. 

				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];	
													// ���ۿ� ����Ǿ� �ִ� ��Ŷ�� �� �����͵��� �и��ϱ� ���� ������ �迭�� �̵�
				
				if (checkSize == 8) {				// ��Ŷ�� ���� �����ʹ� 7���̹Ƿ�, checkSize = 8�� �Ǵ� ���� ��Ŷ ���� �м� ����
					if(g_PacketBuffer.data.id == g_ID){		// �ش� ��Ŷ �������� ID�� ��ġ�ϸ�

						g_PacketMode = 2;					// g_PacketMode = 2�� ���� �� ���� �۾� ����
					}
					else{									// ���� g_PacketMode = 2�� �ƴ϶��
						g_PacketMode = 0;					// �ش� ���͸� ������ ���� ��Ŷ�� �ƴ϶�� �Ǵ��ϰ� ó������ �ٽ� ����
						checkSize = 0;						// checkSize�� �ٽ� 0���� �ʱ�ȭ
					}
				}

				break;
			
			///////
			case 2:									// case 2���� �Դٴ� ���� �ش� �ν� ���͸� ������ ���� ��Ŷ�� ����� �Ѿ�Դٴ� �ǹ�
			///////									// �������� ��Ŷ���� ���� �������� �������� �м� ����
				
				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				check += g_buf[g_BufReadCnt];
				
				if (checkSize == g_PacketBuffer.data.size) {		// ��Ŷ ������ ũ�Ⱑ ��ġ ���� Ȯ��

					if(check == g_PacketBuffer.data.check){			// ���������� üũ �����Ͱ� ��ġ�ϴ��� Ȯ�� 

						switch(g_PacketBuffer.data.mode){			// ��Ŷ �����ͷ� ���� ��� ��Ʈ �� Ȯ��

							case 2:									// ��� ��Ʈ ���� 2�� �� 
							g_Pdes = g_PacketBuffer.data.pos / 1000.;
							g_Vlimit = g_PacketBuffer.data.velo / 1000.;
							g_Climit = g_PacketBuffer.data.cur / 1000.;		// �� ��Ŷ�� ��¥ ������ �� ���� �����Ͽ� �� ������ ����

							//SendShortUART1(g_Pdes); TransUart1(32);
							//SendShortUART1(g_Vlimit); TransUart1(32);
							//SendShortUART1(g_Climit); TransUart1(32);		// MFC(UART0)���� ATMega128�� �Ѿ���� ��dmf UART1�� Ȯ��
							//TransUart1(13);
							break;											// �ش� �������� �����ֱ⹮���� ���� ����� ���� �ٽ� 
							}												// ��Ŷ���� ����Ǿ� MFC�� �۽��� ���̴�. 
					}
					
					check = 0;
					g_PacketMode = 0;
					checkSize = 0;											// ��Ŷ���� ������ ������ �Ϸ��ߴٸ� ��Ŷ�� �м��ϱ� ����
				}															// ���� �Ķ���͵��� 0���� �ʱ�ȭ. ���� ���� ����� ���
				
				else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)) {
																			// �׷��� ������ ũ�Ⱑ ��ġ���� ������
					TransUart0('f');										// ���� f ��� �� �Ķ���͵��� 0���� �ʱ�ȭ ��
					check = 0;												// ���� ����� ���
					g_PacketMode = 0;
					checkSize = 0;
				}
			}
		}
		
		
		
		
		/////////////////////////////////////////////////////////////
		// ������ʹ� �ݴ�� ATMega128���� MFC�� �����͸� ������ �ڵ� //
		////////////////////////////////////////////////////////////
		
		if(g_SendFlag > 19){			// ������ �۽� �÷��� g_SendFlag �� 20 �̻��̸�
			g_SendFlag = 0;				// g_SendFlag �� 0���� �ʱ�ȭ�� �� ������ �۽� ��Ŷ ���� ����
										// ����� g_SendFlag ������ �����ֱ� �ڵ忡�� ������

		
		// ��Ŷ�� �� �����͵��� ������ ��ġ�� ����
			packet.data.size = sizeof(Packet_data_t);		// size = 20
			packet.data.id = g_ID;							// g_ID = 1
			packet.data.mode = 3;							// mode = 3
			packet.data.check = 0;							// checksum ������ ���� 0���� �ʱ�ȭ
			
			/*
			packet.data.pos = g_Pdes * 1000;		
			packet.data.velo = g_Vlimit * 1000;		
			packet.data.cur = g_Climit * 1000;				// �Ҽ��� �Ʒ� �� �ս� ������ ���� 1000�� ���� �� �۽�
			*/
			
			packet.data.pos = g_Pcur * 1000;
			packet.data.velo = g_Vcur * 1000;
			packet.data.cur = g_Ccur * 1000;
			
			
			for (int i = 8; i < sizeof(Packet_t); i++)
			packet.data.check += packet.buffer[i];			// checksum ����: pos~cur�����͸� �� ���ϰ� ����
			
			for(int i=0; i<packet.data.size; i++)
				TransUart0(packet.buffer[i]);				// ���� ��Ŷ �߼�
				
			for(int i=0; i<packet.data.size; i++){			// ��� MFC�� ���� ���� ATMega�� �� �޾������� Ȯ���ϴ� �뵵
				TransNumUart1(packet.buffer[i]);			// �ø��� ��� 1.9b�� �� �ִ� �ڵ�
				TransUart1(32);	// �����̽���
			}
			TransUart1(13);		// �� �ٲ�
			
		}
	}
		
}

