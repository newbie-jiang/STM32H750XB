#include "irda_nec.h"
#include "stm32h7xx_it.h"
#include "tim.h"




/**********************根据定时器计数可调（可调参数）***************************************/
#define   TIM_INTERRUPT_TIME        10                     /* 计一次数时间(us)*/
#define   OVERFLOW                  65535                 /* 溢出值 (变量类型相关这里无符号16位) */
/******************NEC协议解码(适用于NEC协议，非必要勿改（可动态增加范围）)***********************/
#define   START_MIN               13000/TIM_INTERRUPT_TIME    /* 引导码最小计数次数 */
#define   START_MAX               14000/TIM_INTERRUPT_TIME    /* 引导码最大计数次数 */
  
#define   SURE_0_MIN              1000/TIM_INTERRUPT_TIME     /* 0最小计数次数 */           
#define   SURE_0_MAX              1200/TIM_INTERRUPT_TIME     /* 0最大计数次数 */            
 
#define   SURE_1_MIN              2100/TIM_INTERRUPT_TIME      /* 1最小计数次数 */ 
#define   SURE_1_MAX              2350/TIM_INTERRUPT_TIME      /* 1最大计数次数 */            
/*******************************************************************************************/  



uint32_t IC_IRDA_NEC(void)
{
	static uint16_t get_new_tim; /*获取最新时间*/
	static uint16_t get_old_tim; /*获取上次时间*/
	static uint16_t think_tim;/*间隔时间*/
	static char irda_state;/*解码状态*/
	static uint32_t irda_decoder,get_ir_code;/*解码值*/
	static uint8_t code_flag; /*解码成功标志位*/
  static uint8_t irda_data_bit_count=32;/*红外的32个bit*/

	if(nec_ValueChanged)/*产生下降沿中断*/
	{
	
		/*获取计数值*/
		get_new_tim=HAL_TIM_ReadCapturedValue(&htim5,TIM_CHANNEL_3);
		/**************向上溢出，考虑溢出情况**************************/	
		if(get_new_tim > get_old_tim) 
			think_tim = get_new_tim - get_old_tim;          /*无溢出*/
		else   
			think_tim = get_new_tim + OVERFLOW - get_old_tim;    /*溢出*/
		
		get_old_tim = get_new_tim;
    /***************************************************************/
		
		
	switch (irda_state)
	{
		case 0: 
			/*引导码*/
			if((think_tim>=START_MIN&&think_tim<=START_MAX))
					irda_state=1; /*引导码正确进入下一阶段*/
			else 
				//HAL_TIM_Base_Stop(&htim6);
			 break; /*非引导码直接跳出*/
	  
		case 1:
			/*判断0和1*/
	   if((think_tim>=SURE_0_MIN&&think_tim<=SURE_0_MAX))   /*判断为0   1.12ms*/
			{
				irda_decoder|=0<<(irda_data_bit_count-1); 
			  irda_data_bit_count-=1; /*红外32位数据*/
				irda_state=2;
				
			}
			else if(think_tim>=SURE_1_MIN&&think_tim<=SURE_1_MAX) /*判断为1   2.24ms*/
			{
			  irda_decoder|=1<<(irda_data_bit_count-1);
				irda_data_bit_count-=1;
				irda_state=2;
			}
			
			else /*非0和1*/
			{
			 irda_state=0; /*重新判断引导码*/
			
			}
			
			
		/*判断是否接受满32位数据，接受满则完成解码，未解码完成重新判断0和1*/	
		case 2:
			
		if(irda_data_bit_count==0)   /*是否满足32位数据*/
		{
		  code_flag=1; /*解码完成解码标志位置1*/
		  irda_data_bit_count=32; /*重新设置32bit数据*/
		  irda_state=0; /*重新判断引导码*/
	  }
	 else
			irda_state=1; /*继续判断0和1*/
		  	
	
	}
	
	 
	 nec_ValueChanged=false;/*恢复中断标志位*/
	
	}
	
	if(code_flag==1)
	{
	  
	  get_ir_code=irda_decoder;
    code_flag=0;	
		irda_decoder=0;	
		/***********************校验******************************/
		
		unsigned int address = (get_ir_code >> 24) & 0xFF;/*获取地址码*/
    unsigned int address_complement = (get_ir_code >> 16) & 0xFF;// 获取地址反码
    unsigned int data = (get_ir_code >> 8) & 0xFF;  // 获取数据码
    unsigned int data_complement = get_ir_code & 0xFF;// 获取数据反码
		
		//    printf("地址码: 0x%02X\n", address);
    //    printf("地址反码: 0x%02X\n", address_complement);
    //    printf("数据码: 0x%02X\n", data);
    //    printf("数据反码: 0x%02X\r\n", data_complement);
		
		
		if((address+address_complement==0xff)&&(data+data_complement==0xff))
		{
		 return get_ir_code;/*所有码*/
		 //return data;/*数据码*/
		}
/**************************************************************************/
//	return get_ir_code;
		
	}
	
	 return 0x00;
	
	
	
}





