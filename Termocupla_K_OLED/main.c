/*
 * Termocupla_K_OLED.c
 *
 * Created: 12/5/2019 19:06:10
 * Author : Jandry O. Banegas
 */ 
#define F_CPU 8000000UL
#define BAUD0 115200

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "Mi_CoopScheduler.h"
#include "Mi_USART.h"
#include "SSD1306.h"
#include "Tahoma13x14.h"

#define T_MAS PIND2
#define T_MAS_PULSADO PIND & (1<<T_MAS)
#define T_MINUS PIND3
#define T_MINUS_PULSADO PIND & (1<<T_MINUS)

#define PWM_CALDERA OCR0A
#define SETPOINT_DEFECTO 40

#define SS 2
#define MOSI 3
#define MISO 4
#define SCK 5
#define TRUE 1
#define FALSE 0

volatile uint8_t Cada2=0;
volatile uint32_t msTick=0;

typedef enum{OK,OC,VCC_SC,GND_SC}MAX31855_STATUS;
typedef struct{
	volatile uint8_t *_CSDir;
	volatile uint8_t _CSPin;
	uint32_t _Trama;
	float _TerTempC;
	float _RJTempC;
	MAX31855_STATUS _STATUS;
}MAX31855;
MAX31855 TermoK={
	._CSDir=&PORTB,
	._CSPin=SS,
	._STATUS=OK,
	._Trama=0,
	._TerTempC=0,
	._RJTempC=0	
};
typedef enum{PARALELO,INCREMENTAL}TIPO_PID;// variable para cambiar entre los tipos de PID
typedef struct PID{
	TIPO_PID _Tipo;
	float _kp;
	float _ki;
	float _kd;
	float _P;
	float _I;
	float _D;
	float _Medida;
	float _Setpoint;
	float _Error[3];
	float _U;
	unsigned char _PWM_MAX;
	unsigned char _PWM_MIN;
	float _Delta_t;
	/*
		Variables Back-calculation Anti-reset wind up
	*/
	float _Anti_windup;
	float _ka;
	float _Error_sat;
	float _U_sat;
}PID;
PID PID_Caldera={
	._Tipo=PARALELO,
	._kp=0.69661,
	._ki=0.0073939,
	._kd=16.4076,
	._ka=1/0.69661,//ka=1/kp
	._Setpoint=SETPOINT_DEFECTO,
	._P=0,
	._I=0,
	._D=0,
	._Anti_windup=0,
	._Error={0,0,0},
	._PWM_MAX=255,
	._PWM_MIN=0,
	._Delta_t=0.1
};
void ConfPulsadores();
void ConfTimers();
void ConfSPIMaestro();
uint8_t MAX31855Leer(MAX31855 *);
void CargarPlantilla();
void Implementar_PID(PID *,float);

void LecturaTermo();
void EjecutarPID();

void ActualizarGLCD();

static STareas Tareas[]={
	{100 ,0 , LecturaTermo},//Lee la temperatura de la termocupla cada 100ms
	{100 ,0 , EjecutarPID},//Ejecuta el control PID cada 100ms
	{2000 ,0 , ActualizarGLCD},//Actualiza el GLCD cada 2 segundos
};
/*
*/
int main(void)
{
	ConfPulsadores();
	ConfTimers();
	ConfSPIMaestro();
	Inicia_Usart(USART0,round(MiUBRR0));
	Env_Usart_P(USART0,PSTR("Iniciando\r\n"));
	GLCD_Setup();
	CargarPlantilla();
	sei();//Activar interrupciones
	
	NroTareas=sizeof(Tareas)/sizeof(*Tareas);//Especificar el numero de tareas
    for(;;)
    {
		EjecutarScheduler(Tareas,msTick);
    }
}
/************************************************************************/
/*				INTERRUPCIONES                                                  */
/************************************************************************/
ISR(TIMER2_OVF_vect){
	Cada2++;// Configurado el micro a 8Mhz cada dos overflows equivalen a 1ms
	if(Cada2==2){
		msTick++;
		Cada2=0;
	}
}
ISR(PCINT2_vect){
	cli();
	if (T_MAS_PULSADO)
	{
		PID_Caldera._Setpoint=PID_Caldera._Setpoint+5;//Incremeta el Setpoint
	}
	if (T_MINUS_PULSADO)
	{
		PID_Caldera._Setpoint=(PID_Caldera._Setpoint>0)?PID_Caldera._Setpoint-5:0;//Disminuye el Setpoint pero evita valores menores a 0
	}
	sei();
}
/*
*/
void ConfPulsadores(){
	DDRD&=~((1<<T_MAS)|(1<<T_MINUS));
	
	EICRA=(1<<ISC11);//Lanza interrupcion en flanco ascendente
	PCICR=(1<<PCIE2);
	PCMSK2=(1<<PCINT18)|(1<<PCINT19);//Activa la interrupcion de cambio de estado en los pines de los pulsadores
}
/*
*/
void ConfTimers(){
	/**********************Timer 2 SysClock*************************************************************/
	TCCR2A|=(1<<COM2A1)|(1<<COM2B1)|(1<<WGM20);//modo PWM, Phase correct
	TCCR2B=(1<<CS21);//preescala de 8
	TIMSK2=(1<<TOIE2);//interrupcion de overflow activada
	TCNT2=0;
	/**********************Timer 0 PWM*************************************************************/
	TCCR0A|=(1<<COM0A1)|(1<<COM0B1)|(1<<WGM00);	//modo PWM,Phase correct
	TCCR0B|=(1<<CS00)|(1<<CS01);//preescala de 64
	DDRD|=(1<<PIND6);// PIND6 PWM (OCR0A) de la caldera
}
/*
*/
void ConfSPIMaestro(){
	DDRB|=(1<<MOSI) | (1<<SCK) | (1<<SS); // Configura como salidas MOSI,SCK y SS
	SPCR= (1<<SPE) | (1<<MSTR) | (1<<SPR0)|(1<<SPR1); //SPI modo maestro FClk/64
	//SPSR= (1<<SPI2X);
	PORTB|=(1<<SS);
}
/*
*/
uint8_t  MAX31855Leer(MAX31855 *M)
{
    uint8_t n;
	uint16_t Temp1;
	uint16_t Temp2;
	//Leer datos
    *M->_CSDir&=~(1<<M->_CSPin);    // Inica transmison
	M->_Trama=0;
    for (n=0; n<4; n++)
    {
        SPDR = 0;                         // Enviamos un dato vacio
        while ((SPSR & (1<<SPIF)) == 0)  ;    // esperamos a que termine de leer
        M->_Trama = (M->_Trama<<8) + SPDR;                // Agregamos el paquete a la trama
    }
    *M->_CSDir|=(1<<M->_CSPin);         // Terminamos la transmision
	
	Temp1=(uint16_t)(M->_Trama>>16);
	Temp2=(uint16_t)(M->_Trama);
	n= FALSE;
	if (Temp1 & 0x8000)
	{
		n=TRUE;
	}
	//Verificar Error
	if(Temp1 & 0x1){
		Env_Usart_P(USART0,PSTR("Error...\r\n"));
		if (Temp2 & 0x1)
		{
			Env_Usart_P(USART0,PSTR("Circuito Abierto\r\n"));
			M->_STATUS=OC;
		}
		if (Temp2 & 0x2)
		{
			Env_Usart_P(USART0,PSTR("Corto VCC\r\n"));
			M->_STATUS=VCC_SC;
		}
		if (Temp2 & 0x4)
		{
			Env_Usart_P(USART0,PSTR("Corto GND\r\n"));
			M->_STATUS=GND_SC;
		}
	}else{
		M->_STATUS=OK;
	}
	//Extraes solo la temperatura de la termocupla
	Temp1&=0x7FFC;
	//Desplazas hacia la derecha
	Temp1>>=2;
	//Almacena el valor de Temperatura de la termocupla
	M->_TerTempC=Temp1;
	M->_TerTempC*=0.25;
	if (n==TRUE)
	{
		M->_TerTempC*=-1;
	}
	return M->_STATUS;
}
/*
*/
void CargarPlantilla(){
	GLCD_SetFont(Tahoma13x14,13,14,GLCD_Overwrite);
	GLCD_GotoXY(1,5);
	GLCD_PrintString_P(PSTR("Td: 0"));
	GLCD_GotoXY(60,5);
	GLCD_PrintString_P(PSTR("Tm: 0"));
	//Eje de Graficacion
	GLCD_DrawRectangle(1,20,127,63,GLCD_Black);
	GLCD_Render();
}
/*

*/
 void Implementar_PID(PID *Pntr, float medida){
	 float _PWM;
	 Pntr->_Medida=medida;
	 Pntr->_Error[0]=Pntr->_Setpoint-Pntr->_Medida;//error
	 
	 if (Pntr->_Tipo == PARALELO)// modo paralelo 
	 {
		 Pntr->_P=Pntr->_kp*Pntr->_Error[0];// parte proporcional
		 Pntr->_I+=Pntr->_ki*(Pntr->_Error[0]-Pntr->_Anti_windup)*Pntr->_Delta_t;//parte integral - bc Anti windup
		 Pntr->_D=Pntr->_kd*(Pntr->_Error[0]-Pntr->_Error[1])/Pntr->_Delta_t;//parte derivatica
		 Pntr->_Error[1]=Pntr->_Error[0];
		 /*salida_CONTROL*/
		 Pntr->_U=Pntr->_P + Pntr->_I + Pntr->_D;
		 _PWM=Pntr->_U;
		 //Limita la salida al maximo del PWM
		 if (Pntr->_U > Pntr->_PWM_MAX)
		 {
			 _PWM=Pntr->_PWM_MAX;
		 }else if(Pntr->_U < -Pntr->_PWM_MAX){
			 _PWM=-Pntr->_PWM_MAX;
		 }
		 //En este SISTEMA si la salida es positiva se envia al PWM, caso contrario se apaga la caldera.
		if (_PWM >= 0)
		{
			PWM_CALDERA=round(_PWM);
		}else{
			PWM_CALDERA=0;
		}
		Pntr->_U_sat=round(_PWM);
		 /*
		 Antireset Windup back calculation
		 */
		 Pntr->_Error_sat=Pntr->_U-Pntr->_U_sat;
		 Pntr->_Anti_windup=Pntr->_Error_sat*Pntr->_ka;

	 }else{//modo Incremental
		Pntr->_P=Pntr->_kp*(Pntr->_Error[0]-Pntr->_Error[1]);
		Pntr->_I=Pntr->_ki*Pntr->_Error[0]*Pntr->_Delta_t;
		Pntr->_D=(Pntr->_kd*(Pntr->_Error[0]-2*Pntr->_Error[1]+Pntr->_Error[2]))/Pntr->_Delta_t;
		//Salida incremental
		Pntr->_U+=Pntr->_P+Pntr->_I+Pntr->_D;
		//Desplazamos el error
		Pntr->_Error[2]=Pntr->_Error[1];
		Pntr->_Error[1]=Pntr->_Error[0];

		_PWM=Pntr->_U;
		//Limita la salida al maximo del PWM
		if (Pntr->_U > Pntr->_PWM_MAX)
		{
			_PWM=Pntr->_PWM_MAX;
			}else if(Pntr->_U < -Pntr->_PWM_MAX){
			_PWM=-Pntr->_PWM_MAX;
		}
		//En este SISTEMA si la salida es positiva se envia al PWM, caso contrario se apaga la caldera.
		if (_PWM >= 0)
		{
			PWM_CALDERA=round(_PWM);
			}else{
			PWM_CALDERA=0;
		}
		Pntr->_U_sat=round(_PWM);
	 }
}
/*
*/
void LecturaTermo(){
	char cadena[100];
	MAX31855Leer(&TermoK);
	sprintf_P(cadena,PSTR("$$d%.2f#d%.2f#QQ\r\n"),TermoK._TerTempC,PID_Caldera._Setpoint);
	Env_Usart(USART0,cadena);
}
/*
*/
void EjecutarPID(){
	Implementar_PID(&PID_Caldera,TermoK._TerTempC);
	//PWM_CALDERA=255;
}
/*
*/
void ActualizarGLCD(){
	char cadena[100];
	GLCD_GotoXY(22,5);
	sprintf_P(cadena,PSTR("%d "),(uint16_t)PID_Caldera._Setpoint);
	GLCD_PrintString(cadena);
	GLCD_GotoXY(85,5);
	sprintf_P(cadena,PSTR("%.2f "),TermoK._TerTempC);
	GLCD_PrintString(cadena);
	GLCD_GotoXY(50,32);
	sprintf_P(cadena,PSTR("Er: %d   "),(uint16_t)PID_Caldera._Error[0]);
	GLCD_PrintString(cadena);
	GLCD_Render();
}