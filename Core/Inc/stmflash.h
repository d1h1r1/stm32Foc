#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "main.h"  
//=========================�������ͺ궨��
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define __IO    volatile 
typedef __IO u16 vu16;
 
//=========================�û������Լ�����Ҫ����
#define STM32_FLASH_SIZE 	64 	 	//��ѡSTM32��FLASH������С(��λΪK)
    #if     STM32_FLASH_SIZE < 256      //����������С
    #define STM_SECTOR_SIZE     1024    //1K�ֽ�
    #else 
    #define STM_SECTOR_SIZE	    2048    //2K�ֽ�
    #endif	
#define STM32_FLASH_BASE    0x08000000 		//STM32 FLASH����ʼ��ַ
#define FLASH_SAVE_ADDR     STM32_FLASH_BASE+STM_SECTOR_SIZE*62	//дFlash�ĵ�ַ������ӵ����ڶ�ҳ��ʼ
#define STM32_FLASH_WREN 	1              	//ʹ��FLASHд��(0��������;1��ʹ��)
#define FLASH_WAITETIME  	50000          	//FLASH�ȴ���ʱʱ��
 
 
 
 
u8 STMFLASH_GetStatus(void);				  //���״̬
u8 STMFLASH_WaitDone(u16 time);				  //�ȴ���������
u8 STMFLASH_ErasePage(u32 paddr);			  //����ҳ
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);//д�����
u16 STMFLASH_ReadHalfWord(u32 faddr);		  //��������  
void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//ָ����ַ��ʼд��ָ�����ȵ�����
u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//ָ����ַ��ʼ��ȡָ����������
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����
void Flash_PageErase(uint32_t PageAddress);     //��������
						   
#endif
 