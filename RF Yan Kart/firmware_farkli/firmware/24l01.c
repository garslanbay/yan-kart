#include "24l01.h"
//#include "lcd.h"
#include "delay.h"
#include "spi.h"



const uint8_t TX_ADDRESS[2][TX_ADR_WIDTH]={{0xE7,0xE7,0xE7,0xE7,0xE7},{0xE7,0xE7,0xE7,0xE7,0xE7}}; //���͵�ַ
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xE7,0xE7,0xE7,0xE7,0xE7}; //���յ�ַ

void delay_us1(uint8_t delay);

bitt read;
bitt status3;

extern uint8_t bitval;

uint8_t mySPI_GetData(void);

void NRF24L01_SPI_Init(void)
{
	
 	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_SPI1EN,ENABLE);
	
	
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft | SPI_NSSInternalSoft_Set ;
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_Low;
	SPI_InitStructure.SPI_CRCPolynomial=7;
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;
	
	SPI_Init(SPI1,&SPI_InitStructure);
	
	SPI_RxFIFOThresholdConfig(SPI1,SPI_RxFIFOThreshold_QF); 
	
	SPI_Cmd(SPI1,ENABLE);
	
}
 
//��ʼ��24L01��IO��
void NRF24L01_Init(void)
{  
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//CSN
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//�������
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��
	
 	SPI1_Init();    		//��ʼ��SPI1  
	
	NRF24L01_SPI_Init();//���NRF���ص��޸�SPI������

	GPIO_ResetBits(GPIOA,GPIO_Pin_8); 			//ʹ��24L01 CE=0
	GPIO_SetBits(GPIOA,GPIO_Pin_4);			//SPIƬѡȡ��	CSN=1 		 	 
}
//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t i;
	SPI1_SetSpeed(SPI_SPEED_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   	 
	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	buf[0]=buf[1]=buf[2]=buf[3]=buf[4]=0;
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++){
		if(buf[i]!=0XA5)break;	
	}		
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	 	 
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ

bitt temp1,temp2;
uint8_t NRF24L01_Write_Reg(uint8_t reg,uint8_t value)
{
	uint8_t status;	
	
		temp1.bytee=reg;
		temp2.bytee=value;
		//NRF24L01_CE_CLR;
   	NRF24L01_CSN_CLR;    
		delay_us1(1);//ʹ��SPI����
		//status=spi_yaz1(temp1);
  	status =SPI1_ReadWriteByte(reg);//���ͼĴ����� 
		//delay_us1(1);
		//spi_yaz1(temp2);
  	SPI1_ReadWriteByte(value);      //д��Ĵ�����ֵ
		delay_us1(1);
  	NRF24L01_CSN_SET;                 //��ֹSPI����	  
		delay_us1(1);								//Very Very Important!!!!!!
  	return(status);       			//����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	temp1.bytee=reg;
		NRF24L01_CSN_CLR;          //ʹ��SPI����
		delay_us1(1);	
		//spi_yaz1(temp1);
  	SPI1_ReadWriteByte(reg);   //���ͼĴ�����
		//delay_us1(1);
		//reg_val=spi_oku1();
  	reg_val=mySPI_GetData();
		delay_us1(1);
  	NRF24L01_CSN_SET;          //��ֹSPI����
		delay_us1(1);
  	return(reg_val);           //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
uint8_t NRF24L01_Read_Buf(uint8_t reg,uint8_t *pBuf,uint8_t len)
{
	uint8_t status,uint8_t_ctr;	
	temp1.bytee=reg;
		//NRF24L01_CE_CLR;
  	NRF24L01_CSN_CLR;           //ʹ��SPI����
		//status=spi_yaz1(temp1);
  	status=SPI1_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(uint8_t_ctr=0;uint8_t_ctr<len;uint8_t_ctr++)
			pBuf[uint8_t_ctr]= mySPI_GetData();
  	NRF24L01_CSN_SET;       //�ر�SPI����
	  delay_us1(1);
  	return status;        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
		uint8_t status,uint8_t_ctr;	 
		temp1.bytee=reg;
		NRF24L01_CE_CLR;
		NRF24L01_CSN_CLR;          //ʹ��SPI����
		//status=spi_yaz1(temp1);
  	status = SPI1_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(uint8_t_ctr=0; uint8_t_ctr<len; uint8_t_ctr++)
		{
			//temp2.bytee=*pBuf++;
			//spi_yaz1(temp2);
			SPI1_ReadWriteByte(*pBuf++); //д������	 
			
		}
  	NRF24L01_CSN_SET;       //�ر�SPI����
		delay_us1(1);						//important
  	return status;          //���ض�����״ֵ̬
}				   
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
	temp1.bytee=STATUS;
 	//SPI1_SetSpeed(SPI_BaudRatePrescaler_8);//spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	NRF24L01_CE_CLR;
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	NRF24L01_CE_SET;//��������	   
	delay_us1(130);//�ȴ��������
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
}
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;		    							   
	//SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(WRITE_REG_NRF+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		bitval=GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_3);
		GPIO_WriteBit(GPIOB,GPIO_Pin_3,!bitval);
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}					    
//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void NRF24L01_RX_Mode(void)
{
		NRF24L01_CE_CLR;	  
  	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	  
  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x00);    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,0x02);	     //����RFͨ��Ƶ��		  
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x07);//����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG, 0x3B);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	NRF24L01_CE_SET; //CEΪ��,�������ģʽ 
		delay_us1(150);	//Another bug
}						 
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void NRF24L01_TX_Mode(void)
{														 
		NRF24L01_CE_CLR;	 
  	NRF24L01_Write_Buf(WRITE_REG_NRF+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(WRITE_REG_NRF+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_AA,0x00);     //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(WRITE_REG_NRF+EN_RXADDR,0x00); //ʹ��ͨ��0�Ľ��յ�ַ  
  	NRF24L01_Write_Reg(WRITE_REG_NRF+SETUP_RETR,0x00);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_CH,0x02);       //����RFͨ��Ϊ40
  	NRF24L01_Write_Reg(WRITE_REG_NRF+RF_SETUP,0x07);  //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG,0x3A);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
		NRF24L01_CE_SET;//CEΪ��,10us����������
	  delay_us1(130);

}		  

void NRF24L01_SLEEP_Mode(void)
{
	NRF24L01_CE_CLR;
	NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG,0x39);
	//NRF24L01_CE_SET; //CEΪ��,�������ģʽ 
	//delay_us1(150);	//Another bug
}

void NRF24L01_WAKEUP_Mode(void)
{
	NRF24L01_CE_CLR;
	NRF24L01_Write_Reg(WRITE_REG_NRF+CONFIG,0x3B);
	NRF24L01_CE_SET; //CEΪ��,�������ģʽ 
	delay_us1(150);	//Another bug
}
void delay_us1(uint8_t delay)
{
	uint16_t temp=16000;
	
	while(temp--)
	{
		__NOP();

	}
	__NOP();

}

uint8_t spi_oku1(void)
{
		
	
		GPIO_SetBits(GPIOA,GPIO_Pin_5);			
           
    read.aa.bit7 =GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
    
		GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
	

		GPIO_SetBits(GPIOA,GPIO_Pin_5);			
           
    read.aa.bit6 =GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
    
		GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
	

		GPIO_SetBits(GPIOA,GPIO_Pin_5);			
           
    read.aa.bit5 =GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
    
		GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
	

		GPIO_SetBits(GPIOA,GPIO_Pin_5);			
           
    read.aa.bit4 =GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
    
		GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
		

		GPIO_SetBits(GPIOA,GPIO_Pin_5);			
           
    read.aa.bit3 =GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
    
		GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
		

		GPIO_SetBits(GPIOA,GPIO_Pin_5);			
           
    read.aa.bit2 =GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
    
		GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
		

		GPIO_SetBits(GPIOA,GPIO_Pin_5);			
           
    read.aa.bit1 =GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
    
		GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
		

		GPIO_SetBits(GPIOA,GPIO_Pin_5);			
           
    read.aa.bit0 =GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
    
		GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
		



		return read.bytee;

}

uint8_t spi_yaz1(bitt v)
{
				
	
	
				GPIO_WriteBit(GPIOA,GPIO_Pin_7,v.aa.bit7);
	
				GPIO_SetBits(GPIOA,GPIO_Pin_5);	
	
				status3.aa.bit7=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
				
        GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
	

				GPIO_WriteBit(GPIOA,GPIO_Pin_7,v.aa.bit6);
	
				GPIO_SetBits(GPIOA,GPIO_Pin_5);	
	
				status3.aa.bit6=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
				
        GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
	

				GPIO_WriteBit(GPIOA,GPIO_Pin_7,v.aa.bit5);
	
				GPIO_SetBits(GPIOA,GPIO_Pin_5);	
	
				status3.aa.bit5=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
				
        GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
				

				GPIO_WriteBit(GPIOA,GPIO_Pin_7,v.aa.bit4);
	
				GPIO_SetBits(GPIOA,GPIO_Pin_5);	
	
				status3.aa.bit4=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
				
        GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
				

				GPIO_WriteBit(GPIOA,GPIO_Pin_7,v.aa.bit3);
	
				GPIO_SetBits(GPIOA,GPIO_Pin_5);	
	
				status3.aa.bit3=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
				
        GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
				

				GPIO_WriteBit(GPIOA,GPIO_Pin_7,v.aa.bit2);
	
				GPIO_SetBits(GPIOA,GPIO_Pin_5);	
	
				status3.aa.bit2=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
				
        GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
				

				GPIO_WriteBit(GPIOA,GPIO_Pin_7,v.aa.bit1);
	
				GPIO_SetBits(GPIOA,GPIO_Pin_5);	
	
				status3.aa.bit1=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
				
        GPIO_ResetBits(GPIOA,GPIO_Pin_5); 
				

				GPIO_WriteBit(GPIOA,GPIO_Pin_7,v.aa.bit0);
	
				GPIO_SetBits(GPIOA,GPIO_Pin_5);	
	
				status3.aa.bit0=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
				
        GPIO_ResetBits(GPIOA,GPIO_Pin_5);

				return status3.bytee;
}



