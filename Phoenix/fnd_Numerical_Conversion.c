#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

unsigned char Font[18] = { 0x3F, 0x06, 0x5B, 0x4F,
							0x66, 0x6D, 0x7C, 0x07,
							0x7F, 0x67, 0x77, 0x7C,
							0x39, 0x5E, 0x79, 0x71,
							0x08, 0x80};
unsigned int adc_data =0;

void Segment (int N)
{
	int i;
	unsigned char N1000, N100, N10, N1;
	int Buff;

	N1000= N / 1000; // 세그먼트 천의자리 추출
	Buff = N % 1000;

	N100= N / 100; // 세그먼트 천의자리 추출
	Buff = N % 100;

	N10= N / 10; // 세그먼트 천의자리 추출
	Buff = N % 10;


	for( i=0; i < 30; i++){
	
		PORTC = 0x0e;  //왼쪽 첫 번째 세그먼트 ON
		PORTA = Font[N1000]; //천의 자리 숫자 출력
		_delay_ms(1);

		PORTC = 0x0d;  //왼쪽 두 번째 세그먼트 ON
		PORTA = Font[N100]; //백의 자리 숫자 출력
		_delay_ms(1);

		PORTC = 0x0b;  //왼쪽 세 번째 세그먼트 ON
		PORTA = Font[N10]; //십의 자리 숫자 출력
		_delay_ms(1);

		PORTC = 0x07;  //왼쪽 네 번째 세그먼트 ON
		PORTA = Font[N1]; //일의 자리 숫자 출력
		_delay_ms(1);
	}
}

ISR(ADC_vect)
{
	
	adc_data = ADCW; //AD 값을 읽어 온다
	ADCSRA = ADCSRA | 0x40 ;//ad 변환 시작
}


int main(void){
	DDRA = 0xff;
	DDRC= 0x0f;

	PORTA= 0xff;
	PORTC= 0x0f;

	DDRF= 0x00;  // PF0 을 입력 핀으로 사용

	ADMUX =0x00;
	ADCSRA = 0x8b;	// AD 변환 종료 인터럽트 인에이블


	
	EIMSK=0x03;
	EIFR=0x03;
	EICRA=0x0f;

/*
	UCSR0A=0x00;
	UCSR0B=0x98;
	UCSR0C=0x06;
	UBRR0H=0x00;
	UBRR0L=0x07;
*/

	_delay_us(10);

	sei();
	
	ADCSRA |= 0x40; 

	

	while(1){
		
	Segment(adc_data);
	}
return 0;
}
