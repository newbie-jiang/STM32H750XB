#include "paj7620u2.h"
#include "paj7620u2_cfg.h"

static void paj7620u2_selectBank(bank_e bank);//选择PAJ7620U2 BANK区域
static u8 paj7620u2_wakeup(void);//PAJ7620U2唤醒

//PAJ7620U2初始化
//返回值：0:失败 1:成功
u8 paj7620u2_init()
{
	u8 i;
	u8 status;
	
	GS_i2c_init();//传感器I2C初始化							
	status = paj7620u2_wakeup();		//唤醒PAJ7620U2
	if(!status) 
		return 0;
	paj7620u2_selectBank(BANK0);	//进入BANK0寄存器区域
	for(i=0;i<INIT_SIZE;i++)						//初始化模块
	{
		GS_Write_Byte(init_Array[i][0], init_Array[i][1]);//初始化PAJ7620U2
	}
  paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域
	return 1;
}

GestureData *gesture;
void Gesture_Init(void)
{
	u8 i;
	paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
	for(i=0;i<GESTURE_SIZE;i++)
	{
		GS_Write_Byte(gesture_arry[i][0],gesture_arry[i][1]);//手势识别模式初始化
	}
	paj7620u2_selectBank(BANK0);//切换回BANK0寄存器区域

	gesture = (GestureData *)malloc(sizeof(GestureData));
	if(NULL == gesture){
		//
		printf("Error: struct \"GESTURE_DATA\" malloc failed\r\n");
	}
	memset(gesture, 0, sizeof(GestureData));
	
}

//选择PAJ7620U2 BANK区域
void paj7620u2_selectBank(bank_e bank)
{
	switch(bank)
	{
		case BANK0: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK0);break;//BANK0寄存器区域
		case BANK1: GS_Write_Byte(PAJ_REGITER_BANK_SEL,PAJ_BANK1);break;//BANK1寄存器区域
	}		
}

//PAJ7620U2唤醒
u8 paj7620u2_wakeup()
{ 
	u8 data=0x0a;
	GS_WakeUp();//唤醒PAJ7620U2
	delay_ms(5);//唤醒时间>400us
	GS_WakeUp();//唤醒PAJ7620U2
	delay_ms(5);//唤醒时间>400us
	paj7620u2_selectBank(BANK0);//进入BANK0寄存器区域
	data = GS_Read_Byte(0x00);//读取状态
	if(data!=0x20) return 0; //唤醒失败
	
	return 1;
}
