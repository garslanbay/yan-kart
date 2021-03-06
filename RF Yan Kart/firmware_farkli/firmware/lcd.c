/******************************************************************
*Dosya          :lcd.c
*Yazar          :Ayhan KORKMAZ - AyhanKorkmaz.Net
*Iletisim           :info@ayhankorkmaz.net
*Versiyon       :V1.0
*******************************************************************/
 
#include "stm32f0xx.h"
#include "lcd.h"
 unsigned char sayac2,s,adres=64;
 
 #define RS		GPIO_Pin_9
 #define En		GPIO_Pin_8
 

#define  D4		GPIO_Pin_4
#define	 D5		GPIO_Pin_5
#define  D6		GPIO_Pin_6
#define	 D7		GPIO_Pin_7 
 
 
/*****************************************************************************
Kullanim  : Eklemek istediginiz �zel karakterleri ekleyebiliriniz
            Bunun i�in LCD karakter olusturma programlarini kulanabilirsiniz
******************************************************************************/
 unsigned char karakter_[8][8]=
{
  /* TR Karakterler */
{ 0,14,16,16,17,14, 4, 0},//�
{ 0, 0,12, 4, 4, 4,14, 0},//I
{10, 0,14,17,17,17,14, 0},//�
{ 0,15,16,14, 1,30, 4, 0},//$
{10, 0,17,17,17,17,14, 0},//�
/* �zel isaretler */
{2, 6,14,30,14, 6,  2, 0},//<
{ 0, 4, 4, 4, 4, 4, 4, 0},//|
{ 0, 16, 8, 4, 2, 1, 0,0} //\//
};
/********************************************************
Fonksiyon : delay
Amac      : Gecikme Fonksiyonudur.
Kullanim  : delay(0x0000FFFF);
*********************************************************/
 void delay(unsigned long delay)
{
 while(delay--);
}
/********************************************************
Fonksiyon : lcd_komut
Amac      : LCD �alismasi i�in gerekli komutlari verir
Kullanim  : lcd_komut(temizle);
*********************************************************/
void lcd_komut(char komut)
{
		GPIO_WriteBit(GPIOC,RS,0); //GPIOC->BRR =  0x00000001;            //RS=0

		GPIO_WriteBit(GPIOC,D7,0);
		GPIO_WriteBit(GPIOC,D6,0);
		GPIO_WriteBit(GPIOC,D5,0);
		GPIO_WriteBit(GPIOC,D4,0);
		if(komut & 0x80)
				GPIO_WriteBit(GPIOC,D7,1);
		if(komut & 0x40)
				GPIO_WriteBit(GPIOC,D6,1);
		if(komut & 0x20)
				GPIO_WriteBit(GPIOC,D5,1);
		if(komut & 0x10)
				GPIO_WriteBit(GPIOC,D4,1);			//GPIOC->ODR |= (komut & 0x000000F0);  

		GPIO_WriteBit(GPIOC,En,1);			//GPIOC->ODR |= 0x00000002;            //E=1
    lcd_gecikme();
		lcd_gecikme();
    GPIO_WriteBit(GPIOC,En,0);					//GPIOC->BRR = 0x00000002;         //E=0
		lcd_gecikme();
		lcd_gecikme();
		
		GPIO_WriteBit(GPIOC,D7,0);
		GPIO_WriteBit(GPIOC,D6,0);
		GPIO_WriteBit(GPIOC,D5,0);
		GPIO_WriteBit(GPIOC,D4,0);
 
    lcd_gecikme();
		
    if(komut & 0x08)
				GPIO_WriteBit(GPIOC,D7,1);
		if(komut & 0x04)
				GPIO_WriteBit(GPIOC,D6,1);
		if(komut & 0x02)
				GPIO_WriteBit(GPIOC,D5,1);
		if(komut & 0x01)
				GPIO_WriteBit(GPIOC,D4,1);
		
		GPIO_WriteBit(GPIOC,En,1);			//GPIOC->ODR |= 0x00000002;            //E=1
    lcd_gecikme();
		lcd_gecikme();
    GPIO_WriteBit(GPIOC,En,0);					//GPIOC->BRR = 0x00000002;         //E=0
		lcd_gecikme();
		lcd_gecikme();
		
		GPIO_WriteBit(GPIOC,D7,0);
		GPIO_WriteBit(GPIOC,D6,0);
		GPIO_WriteBit(GPIOC,D5,0);
		GPIO_WriteBit(GPIOC,D4,0);
		
		lcd_gecikme();
}
/********************************************************
Fonksiyon : lcd_karakter_yaz
Amac      : LCD Ekrana bir karakter basar
Kullanim  : lcd_karakter_yaz('A');
*********************************************************/
void lcd_karakter_yaz(char veri)
{
	char veri1;
	veri1=veri;
switch (veri) {
case '�' : veri=0x00; break;
case 'I' : veri=0x01; break;
case '�' : veri=0x02; break;
case 'S' : veri=0x03; break;
case '�' : veri=0x04; break;
 
case '�' : veri=0x00; break;
case 'i' : veri=0x01; break;
case '�' : veri=0x02; break;
case 's' : veri=0x03; break;
case '�' : veri=0x04; break;
 
default : break;
}
 
		GPIO_WriteBit(GPIOC,RS,1); //GPIOC->BRR =  0x00000001;            //RS=0


		GPIO_WriteBit(GPIOC,D7,0);
		GPIO_WriteBit(GPIOC,D6,0);
		GPIO_WriteBit(GPIOC,D5,0);
		GPIO_WriteBit(GPIOC,D4,0);

		if(veri & 0x80)
				GPIO_WriteBit(GPIOC,D7,1);
		if(veri & 0x40)
				GPIO_WriteBit(GPIOC,D6,1);
		if(veri & 0x20)
				GPIO_WriteBit(GPIOC,D5,1);
		if(veri & 0x10)
				GPIO_WriteBit(GPIOC,D4,1);
	
		GPIO_WriteBit(GPIOC,En,1);//GPIOC->ODR |= 0x00000002;            //E=1
    lcd_gecikme();
		lcd_gecikme();
    GPIO_WriteBit(GPIOC,En,0);					//GPIOC->BRR = 0x00000002;         //E=0
		lcd_gecikme();
		lcd_gecikme();
		
		
		
		GPIO_WriteBit(GPIOC,D7,0);
		GPIO_WriteBit(GPIOC,D6,0);
		GPIO_WriteBit(GPIOC,D5,0);
		GPIO_WriteBit(GPIOC,D4,0);
 
    lcd_gecikme();
		
		

		
    if(veri & 0x08)
				GPIO_WriteBit(GPIOC,D7,1);
		if(veri & 0x04)
				GPIO_WriteBit(GPIOC,D6,1);
		if(veri & 0x02)
				GPIO_WriteBit(GPIOC,D5,1);
		if(veri & 0x01)
				GPIO_WriteBit(GPIOC,D4,1);
		
		GPIO_WriteBit(GPIOC,En,1);			//GPIOC->ODR |= 0x00000002;            //E=1
    lcd_gecikme();
		lcd_gecikme();
    GPIO_WriteBit(GPIOC,En,0);					//GPIOC->BRR = 0x00000002;         //E=0
		lcd_gecikme();
		lcd_gecikme();
		
		
		GPIO_WriteBit(GPIOC,D7,0);
		GPIO_WriteBit(GPIOC,D6,0);
		GPIO_WriteBit(GPIOC,D5,0);
		GPIO_WriteBit(GPIOC,D4,0);
		
    lcd_gecikme();
}
/********************************************************
Fonksiyon : lcd_yazi_yaz
Amac      : LCD Ekrana yazi yazar
Kullanim  : lcd_yazi_yaz("AyhanKorkmaz.Net");
*********************************************************/
void lcd_yazi_yaz(char *veri)
{
 
    while(*veri)
    {
        lcd_gecikme();
				lcd_gecikme();
        lcd_karakter_yaz(*veri++);
 
    }
          delay(0x0000FFFF);
}
/**************************************************************
Fonksiyon : lcd_git_xy
Amac      : LCD ekranin hangi b�lmesine yazilacagini ayarlariz
Kullanim  : lcd_git_xy(2,1); 2. satir 1. S�tun
***************************************************************/
void lcd_git_xy(unsigned char satir,unsigned char sutun)
{
    if(satir==1)
    {
        lcd_komut(0x00000080 + (sutun-1));                  //1.satir 1.s�tun i�in cgram adresi 0x80 dir
    }
    else if(satir==2)
    {
        lcd_komut(0x000000C0 +(sutun-1));                   //2.satir 1.s�tun i�in chram adresi 0x80+0x40=0xC0 dir
    }
}
/********************************************************
Fonksiyon : lcd_hazirla
Amac      : LCD �alismasi i�in gerekli ilk ayarlar
Kullanim  : lcd_hazirla();
*********************************************************/
void lcd_hazirla(void)
{
	

	
    delay(0x0000FFFF);
    delay(0x0000FFFF);

    delay(0x0000FFFF);
 
    //RCC_DeInit();


 
    lcd_komut(zorunlu);
          delay(0x0000FFFF);
    lcd_komut(ikisatir4bit5x8);
          delay(0x0000FFFF);
    lcd_komut(imlecsagakay);
          delay(0x0000FFFF);
    lcd_komut(dayansonig);
          delay(0x0000FFFF);
    lcd_komut(dayansonyok);
          delay(0x0000FFFF);
    lcd_komut(temizle);
          delay(0x0000FFFF);
    lcd_gecikme();
 
		lcd_temizle();
 
    for(sayac2=0;sayac2<=7;sayac2++){  // T�rk�e karakterler tanitiliyor
    lcd_komut(adres);
    for(s=0;s<=7;s++){
        lcd_karakter_yaz(karakter_[sayac2][s]);
                     }
        adres=adres+8;
                 }
        lcd_temizle();
         delay(0x0000FFFF);
 
}
/********************************************************
Fonksiyon : lcd_temizle
Amac      : LCD ekran tamamen silinir
Kullanim  : lcd_temizle();
*********************************************************/
void lcd_temizle(void)
{
    lcd_komut(temizle);
    lcd_gecikme();
}
/********************************************************
Fonksiyon : lcd_gecikme
Amac      : LCD �alismasi i�in gerekli gecikmedir
Kullanim  : lcd_gecikme();
*********************************************************/
void lcd_gecikme(void)
{
    unsigned long delay=0x00000080;
    while(delay--);
}