

//******************************************************************************        
//name:         infrared.h        
//introduce:    ���ⷢ��    
//author:       lihao      
//changetime:   2017.06.02     
//email:        lihao198262@163.com    
//******************************************************************************


#ifndef __INFRARED_H
#define __INFRARED_H

#include "stm8s.h"

#define CHANNEL1

#define HAIER_CON 0xA1

#define MEDIA_CON 0xA2

#define AIRCON_MEDIA
//#define AIRCON_HAIER
#ifdef AIRCON_MEDIA
extern char mediaoff[4];
extern char media_last_on_status[4];
#endif
#ifdef AIRCON_HAIER
extern char haieroff[15];
extern char haier_last_on_status[15];
#endif



/*******************************************************************************
 * ����: Haier_Infrared_Send
 * ����: ���ⷢ�� ���ͺ���
 * �β�: unsigned long data
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void Haier_Infrared_Send(uint8_t data[], int len);

/*******************************************************************************
 * ����: NEC_Infrared_Send
 * ����: ���ⷢ�� ����NEC
 * �β�: unsigned long data
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void NEC_Infrared_Send(unsigned long data);

/*******************************************************************************
 * ����: Media_Infrared_Send
 * ����: ���ⷢ��
 * �β�: uint8_t base 
 * �β�: uint8_t high 
 * �β�: uint8_t low 
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void Media_Infrared_Send(uint8_t base, uint8_t high, uint8_t low);


/*******************************************************************************
 * ����: Infrared_Send
 * ����: ���ⷢ��
 * �β�: unsigned long data
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void Infrared_Send(unsigned long data);

/*******************************************************************************
 * ����: Infrared_Init()
 * ����: ��ʼ������
 * �β�: ��
 * ����: ��
 * ˵��: �� 
 ******************************************************************************/
void Infrared_Init(void);

extern u16 ir_send_delay;

bool Set_Send_Buf(u32 *buf, u8 len);

bool Set_AC_Buf(uint8_t *buf, u8 len);

bool Set_AC_Media_Buf(uint8_t *buf, u8 len);

void IR_Send();

#endif


