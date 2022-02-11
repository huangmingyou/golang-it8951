package main

// 0. 初始化it8915控制器
// 1. 上传图片
// 2. 墨水屏上显示图片

// huangmingyou@gmail.com
// 2022.01.04

/*
#cgo CFLAGS:   -I/root/testgo/lib
#cgo LDFLAGS: -lbcm2835 -lm -lrt -lpthread

#include "e-Paper/EPD_IT8951.h"
#include "GUI/GUI_Paint.h"
#include "Config/Debug.h"
#include "Config/DEV_Config.h"
#include "GUI/GUI_BMPfile.h"
#include "GUI/GUI_Paint.h"



#include "example.h"
#include <time.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>


UBYTE *bmp_dst_buf = NULL;
UBYTE *bmp_src_buf = NULL;
UDOUBLE bmp_width, bmp_height;
UBYTE  bmp_BitCount;
UDOUBLE bytesPerLine;
UDOUBLE imageSize;
UDOUBLE skip;
BMPRGBQUAD  palette[256];

UBYTE GC16_Mode = 2;
UBYTE A2_Mode = 6;



#define Enhance false

#define USE_Factory_Test false

#define USE_Normal_Demo true

#define USE_Touch_Panel false

UWORD VCOM = 2510;

PAINT Paint;
IT8951_Dev_Info Dev_Info;
UWORD Panel_Width;
UWORD Panel_Height;
UDOUBLE Init_Target_Memory_Addr;
int epd_mode = 0;	//0: no rotate, no mirror
					//1: no rotate, horizontal mirror, for 10.3inch
					//2: no totate, horizontal mirror, for 5.17inch
					//3: no rotate, no mirror, isColor, for 6inch color
					

UBYTE *Refresh_Frame_Buf = NULL;

UBYTE *Panel_Frame_Buf = NULL;
UBYTE *Panel_Area_Frame_Buf = NULL;

bool Four_Byte_Align = false;

extern int epd_mode;




void DEV_Digital_Write(UWORD Pin, UBYTE Value)
{
	bcm2835_gpio_write(Pin, Value);
}

UBYTE DEV_Digital_Read(UWORD Pin)
{
	UBYTE Read_Value = 0;
	Read_Value = bcm2835_gpio_lev(Pin);
	return Read_Value;
}

void DEV_SPI_WriteByte(UBYTE Value)
{
	bcm2835_spi_transfer(Value);
}

UBYTE DEV_SPI_ReadByte()
{
	UBYTE Read_Value = 0x00;
	Read_Value = bcm2835_spi_transfer(0x00);
	return Read_Value;
}

void DEV_Delay_ms(UDOUBLE xms)
{
	bcm2835_delay(xms);
}








static void DEV_GPIO_Mode(UWORD Pin, UWORD Mode)
{
	if(Mode == 0 || Mode == BCM2835_GPIO_FSEL_INPT) {
		bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_INPT);
	} else {
		bcm2835_gpio_fsel(Pin, BCM2835_GPIO_FSEL_OUTP);
	}
}


static void DEV_GPIO_Init(void)
{
	DEV_GPIO_Mode(EPD_RST_PIN, BCM2835_GPIO_FSEL_OUTP);
	DEV_GPIO_Mode(EPD_CS_PIN, BCM2835_GPIO_FSEL_OUTP);
	DEV_GPIO_Mode(EPD_BUSY_PIN, BCM2835_GPIO_FSEL_INPT);

	DEV_Digital_Write(EPD_CS_PIN, HIGH);
}



UBYTE DEV_Module_Init(void)
{

	if(!bcm2835_init()) {
		Debug("bcm2835 init failed  !!! \r\n");
		return 1;
	} else {
		Debug("bcm2835 init success !!! \r\n");
	}

	bcm2835_spi_begin();                                         //Start spi interface, set spi pin for the reuse function
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);     //High first transmission
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                  //spi mode 0
	//bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);   //For RPi3/3B/3B+
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);   //For RPi 4

	//GPIO Config
	DEV_GPIO_Init();

	return 0;
}



void DEV_Module_Exit(void)
{
	DEV_Digital_Write(EPD_CS_PIN, LOW);
	DEV_Digital_Write(EPD_RST_PIN, LOW);

	bcm2835_spi_end();
	bcm2835_close();
}


static void EPD_IT8951_Reset(void)
{
    DEV_Digital_Write(EPD_RST_PIN, HIGH);
    DEV_Delay_ms(200);
    DEV_Digital_Write(EPD_RST_PIN, LOW);
    DEV_Delay_ms(10);
    DEV_Digital_Write(EPD_RST_PIN, HIGH);
    DEV_Delay_ms(200);
}


static void EPD_IT8951_ReadBusy(void)
{
    UBYTE Busy_State = DEV_Digital_Read(EPD_BUSY_PIN);
    //0: busy, 1: idle
    while(Busy_State == 0) {
        Busy_State = DEV_Digital_Read(EPD_BUSY_PIN);
    }
}


static void EPD_IT8951_WriteCommand(UWORD Command)
{
	//Set Preamble for Write Command
	UWORD Write_Preamble = 0x6000;
	
	EPD_IT8951_ReadBusy();

    DEV_Digital_Write(EPD_CS_PIN, LOW);
	
	DEV_SPI_WriteByte(Write_Preamble>>8);
	DEV_SPI_WriteByte(Write_Preamble);
	
	EPD_IT8951_ReadBusy();	
	
	DEV_SPI_WriteByte(Command>>8);
	DEV_SPI_WriteByte(Command);
	
	DEV_Digital_Write(EPD_CS_PIN, HIGH);
}


static void EPD_IT8951_WriteData(UWORD Data)
{
    //Set Preamble for Write Command
	UWORD Write_Preamble = 0x0000;

    EPD_IT8951_ReadBusy();

    DEV_Digital_Write(EPD_CS_PIN, LOW);

	DEV_SPI_WriteByte(Write_Preamble>>8);
	DEV_SPI_WriteByte(Write_Preamble);

    EPD_IT8951_ReadBusy();

	DEV_SPI_WriteByte(Data>>8);
	DEV_SPI_WriteByte(Data);

    DEV_Digital_Write(EPD_CS_PIN, HIGH);
}


static void EPD_IT8951_WriteMuitiData(UWORD* Data_Buf, UDOUBLE Length)
{
    //Set Preamble for Write Command
	UWORD Write_Preamble = 0x0000;

    EPD_IT8951_ReadBusy();

    DEV_Digital_Write(EPD_CS_PIN, LOW);

	DEV_SPI_WriteByte(Write_Preamble>>8);
	DEV_SPI_WriteByte(Write_Preamble);

    EPD_IT8951_ReadBusy();

    for(UDOUBLE i = 0; i<Length; i++)
    {
	    DEV_SPI_WriteByte(Data_Buf[i]>>8);
	    DEV_SPI_WriteByte(Data_Buf[i]);
    }
    DEV_Digital_Write(EPD_CS_PIN, HIGH);
}



static UWORD EPD_IT8951_ReadData()
{
    UWORD ReadData;
	UWORD Write_Preamble = 0x1000;
    UWORD Read_Dummy;

    EPD_IT8951_ReadBusy();

    DEV_Digital_Write(EPD_CS_PIN, LOW);

	DEV_SPI_WriteByte(Write_Preamble>>8);
	DEV_SPI_WriteByte(Write_Preamble);

    EPD_IT8951_ReadBusy();

    //dummy
    Read_Dummy = DEV_SPI_ReadByte()<<8;
    Read_Dummy |= DEV_SPI_ReadByte();

    EPD_IT8951_ReadBusy();

    ReadData = DEV_SPI_ReadByte()<<8;
    ReadData |= DEV_SPI_ReadByte();

    DEV_Digital_Write(EPD_CS_PIN, HIGH);

    return ReadData;
}




static void EPD_IT8951_ReadMultiData(UWORD* Data_Buf, UDOUBLE Length)
{
	UWORD Write_Preamble = 0x1000;
    UWORD Read_Dummy;

    EPD_IT8951_ReadBusy();

    DEV_Digital_Write(EPD_CS_PIN, LOW);

	DEV_SPI_WriteByte(Write_Preamble>>8);
	DEV_SPI_WriteByte(Write_Preamble);

    EPD_IT8951_ReadBusy();

    //dummy
    Read_Dummy = DEV_SPI_ReadByte()<<8;
    Read_Dummy |= DEV_SPI_ReadByte();

    EPD_IT8951_ReadBusy();

    for(UDOUBLE i = 0; i<Length; i++)
    {
	    Data_Buf[i] = DEV_SPI_ReadByte()<<8;
	    Data_Buf[i] |= DEV_SPI_ReadByte();
    }

    DEV_Digital_Write(EPD_CS_PIN, HIGH);
}



static void EPD_IT8951_WriteMultiArg(UWORD Arg_Cmd, UWORD* Arg_Buf, UWORD Arg_Num)
{
     //Send Cmd code
     EPD_IT8951_WriteCommand(Arg_Cmd);
     //Send Data
     for(UWORD i=0; i<Arg_Num; i++)
     {
         EPD_IT8951_WriteData(Arg_Buf[i]);
     }
}


static UWORD EPD_IT8951_ReadReg(UWORD Reg_Address)
{
    UWORD Reg_Value;
    EPD_IT8951_WriteCommand(IT8951_TCON_REG_RD);
    EPD_IT8951_WriteData(Reg_Address);
    Reg_Value =  EPD_IT8951_ReadData();
    return Reg_Value;
}



static void EPD_IT8951_WriteReg(UWORD Reg_Address,UWORD Reg_Value)
{
    EPD_IT8951_WriteCommand(IT8951_TCON_REG_WR);
    EPD_IT8951_WriteData(Reg_Address);
    EPD_IT8951_WriteData(Reg_Value);
}



static UWORD EPD_IT8951_GetVCOM(void)
{
    UWORD VCOM;
    EPD_IT8951_WriteCommand(USDEF_I80_CMD_VCOM);
    EPD_IT8951_WriteData(0x0000);
    VCOM =  EPD_IT8951_ReadData();
    return VCOM;
}



static void EPD_IT8951_SetVCOM(UWORD VCOM)
{
    EPD_IT8951_WriteCommand(USDEF_I80_CMD_VCOM);
    EPD_IT8951_WriteData(0x0001);
    EPD_IT8951_WriteData(VCOM);
}



static void EPD_IT8951_LoadImgStart( IT8951_Load_Img_Info* Load_Img_Info )
{
    UWORD Args;
    Args = (\
        Load_Img_Info->Endian_Type<<8 | \
        Load_Img_Info->Pixel_Format<<4 | \
        Load_Img_Info->Rotate\
    );
    EPD_IT8951_WriteCommand(IT8951_TCON_LD_IMG);
    EPD_IT8951_WriteData(Args);
}


static void EPD_IT8951_LoadImgAreaStart( IT8951_Load_Img_Info* Load_Img_Info, IT8951_Area_Img_Info* Area_Img_Info )
{
    UWORD Args[5];
    Args[0] = (\
        Load_Img_Info->Endian_Type<<8 | \
        Load_Img_Info->Pixel_Format<<4 | \
        Load_Img_Info->Rotate\
    );
    Args[1] = Area_Img_Info->Area_X;
    Args[2] = Area_Img_Info->Area_Y;
    Args[3] = Area_Img_Info->Area_W;
    Args[4] = Area_Img_Info->Area_H;
    EPD_IT8951_WriteMultiArg(IT8951_TCON_LD_IMG_AREA, Args,5);
}

static void EPD_IT8951_LoadImgEnd(void)
{
    EPD_IT8951_WriteCommand(IT8951_TCON_LD_IMG_END);
}


static void EPD_IT8951_GetSystemInfo(void* Buf)
{
    IT8951_Dev_Info* Dev_Info; 

    EPD_IT8951_WriteCommand(USDEF_I80_CMD_GET_DEV_INFO);

    EPD_IT8951_ReadMultiData((UWORD*)Buf, sizeof(IT8951_Dev_Info)/2);

    Dev_Info = (IT8951_Dev_Info*)Buf;
	Debug("Panel(W,H) = (%d,%d)\r\n",Dev_Info->Panel_W, Dev_Info->Panel_H );
	Debug("Memory Address = %X\r\n",Dev_Info->Memory_Addr_L | (Dev_Info->Memory_Addr_H << 16));
	Debug("FW Version = %s\r\n", (UBYTE*)Dev_Info->FW_Version);
	Debug("LUT Version = %s\r\n", (UBYTE*)Dev_Info->LUT_Version);
}


static void EPD_IT8951_SetTargetMemoryAddr(UDOUBLE Target_Memory_Addr)
{
	UWORD WordH = (UWORD)((Target_Memory_Addr >> 16) & 0x0000FFFF);
	UWORD WordL = (UWORD)( Target_Memory_Addr & 0x0000FFFF);

    EPD_IT8951_WriteReg(LISAR+2, WordH);
    EPD_IT8951_WriteReg(LISAR  , WordL);
}


static void EPD_IT8951_WaitForDisplayReady(void)
{
    //Check IT8951 Register LUTAFSR => NonZero Busy, Zero - Free
    while( EPD_IT8951_ReadReg(LUTAFSR) )
    {
        //wait in idle state
    }
}





static void EPD_IT8951_HostAreaPackedPixelWrite_1bp(IT8951_Load_Img_Info*Load_Img_Info,IT8951_Area_Img_Info*Area_Img_Info, bool Packed_Write)
{
    UWORD Source_Buffer_Width, Source_Buffer_Height;
    UWORD Source_Buffer_Length;

    UWORD* Source_Buffer = (UWORD*)Load_Img_Info->Source_Buffer_Addr;
    EPD_IT8951_SetTargetMemoryAddr(Load_Img_Info->Target_Memory_Addr);
    EPD_IT8951_LoadImgAreaStart(Load_Img_Info,Area_Img_Info);

    //from byte to word
    //use 8bp to display 1bp, so here, divide by 2, because every byte has full bit.
    Source_Buffer_Width = Area_Img_Info->Area_W/2;
    Source_Buffer_Height = Area_Img_Info->Area_H;
    Source_Buffer_Length = Source_Buffer_Width * Source_Buffer_Height;
    
    if(Packed_Write == true)
    {
        EPD_IT8951_WriteMuitiData(Source_Buffer, Source_Buffer_Length);
    }
    else
    {
        for(UDOUBLE i=0; i<Source_Buffer_Height; i++)
        {
            for(UDOUBLE j=0; j<Source_Buffer_Width; j++)
            {
                EPD_IT8951_WriteData(*Source_Buffer);
                Source_Buffer++;
            }
        }
    }

    EPD_IT8951_LoadImgEnd();
}





static void EPD_IT8951_HostAreaPackedPixelWrite_2bp(IT8951_Load_Img_Info*Load_Img_Info, IT8951_Area_Img_Info*Area_Img_Info, bool Packed_Write)
{
    UWORD Source_Buffer_Width, Source_Buffer_Height;
    UWORD Source_Buffer_Length;

    UWORD* Source_Buffer = (UWORD*)Load_Img_Info->Source_Buffer_Addr;
    EPD_IT8951_SetTargetMemoryAddr(Load_Img_Info->Target_Memory_Addr);
    EPD_IT8951_LoadImgAreaStart(Load_Img_Info,Area_Img_Info);

    //from byte to word
    Source_Buffer_Width = (Area_Img_Info->Area_W*2/8)/2;
    Source_Buffer_Height = Area_Img_Info->Area_H;
    Source_Buffer_Length = Source_Buffer_Width * Source_Buffer_Height;

    if(Packed_Write == true)
    {
        EPD_IT8951_WriteMuitiData(Source_Buffer, Source_Buffer_Length);
    }
    else
    {
        for(UDOUBLE i=0; i<Source_Buffer_Height; i++)
        {
            for(UDOUBLE j=0; j<Source_Buffer_Width; j++)
            {
                EPD_IT8951_WriteData(*Source_Buffer);
                Source_Buffer++;
            }
        }
    }

    EPD_IT8951_LoadImgEnd();
}





static void EPD_IT8951_HostAreaPackedPixelWrite_4bp(IT8951_Load_Img_Info*Load_Img_Info, IT8951_Area_Img_Info*Area_Img_Info, bool Packed_Write)
{
    UWORD Source_Buffer_Width, Source_Buffer_Height;
    UWORD Source_Buffer_Length;

    UWORD* Source_Buffer = (UWORD*)Load_Img_Info->Source_Buffer_Addr;
    EPD_IT8951_SetTargetMemoryAddr(Load_Img_Info->Target_Memory_Addr);
    EPD_IT8951_LoadImgAreaStart(Load_Img_Info,Area_Img_Info);

    //from byte to word
    Source_Buffer_Width = (Area_Img_Info->Area_W*4/8)/2;
    Source_Buffer_Height = Area_Img_Info->Area_H;
    Source_Buffer_Length = Source_Buffer_Width * Source_Buffer_Height;

    if(Packed_Write == true)
    {
        EPD_IT8951_WriteMuitiData(Source_Buffer, Source_Buffer_Length);
    }
    else
    {
        for(UDOUBLE i=0; i<Source_Buffer_Height; i++)
        {
            for(UDOUBLE j=0; j<Source_Buffer_Width; j++)
            {
                EPD_IT8951_WriteData(*Source_Buffer);
                Source_Buffer++;
            }
        }
    }

    EPD_IT8951_LoadImgEnd();
}







static void EPD_IT8951_HostAreaPackedPixelWrite_8bp(IT8951_Load_Img_Info*Load_Img_Info,IT8951_Area_Img_Info*Area_Img_Info)
{
    UWORD Source_Buffer_Width, Source_Buffer_Height;

    UWORD* Source_Buffer = (UWORD*)Load_Img_Info->Source_Buffer_Addr;
    EPD_IT8951_SetTargetMemoryAddr(Load_Img_Info->Target_Memory_Addr);
    EPD_IT8951_LoadImgAreaStart(Load_Img_Info,Area_Img_Info);

    //from byte to word
    Source_Buffer_Width = (Area_Img_Info->Area_W*8/8)/2;
    Source_Buffer_Height = Area_Img_Info->Area_H;

    for(UDOUBLE i=0; i<Source_Buffer_Height; i++)
    {
        for(UDOUBLE j=0; j<Source_Buffer_Width; j++)
        {
            EPD_IT8951_WriteData(*Source_Buffer);
            Source_Buffer++;
        }
    }
    EPD_IT8951_LoadImgEnd();
}






static void EPD_IT8951_Display_Area(UWORD X,UWORD Y,UWORD W,UWORD H,UWORD Mode)
{
    UWORD Args[5];
    Args[0] = X;
    Args[1] = Y;
    Args[2] = W;
    Args[3] = H;
    Args[4] = Mode;
    //0x0034
    EPD_IT8951_WriteMultiArg(USDEF_I80_CMD_DPY_AREA, Args,5);
}



static void EPD_IT8951_Display_AreaBuf(UWORD X,UWORD Y,UWORD W,UWORD H,UWORD Mode, UDOUBLE Target_Memory_Addr)
{
    UWORD Args[7];
    Args[0] = X;
    Args[1] = Y;
    Args[2] = W;
    Args[3] = H;
    Args[4] = Mode;
    Args[5] = (UWORD)Target_Memory_Addr;
    Args[6] = (UWORD)(Target_Memory_Addr>>16);
    //0x0037
    EPD_IT8951_WriteMultiArg(USDEF_I80_CMD_DPY_BUF_AREA, Args,7); 
}



static void EPD_IT8951_Display_1bp(UWORD X, UWORD Y, UWORD W, UWORD H, UWORD Mode,UDOUBLE Target_Memory_Addr, UBYTE Back_Gray_Val,UBYTE Front_Gray_Val)
{
    //Set Display mode to 1 bpp mode - Set 0x18001138 Bit[18](0x1800113A Bit[2])to 1
    EPD_IT8951_WriteReg(UP1SR+2, EPD_IT8951_ReadReg(UP1SR+2) | (1<<2) );

    EPD_IT8951_WriteReg(BGVR, (Front_Gray_Val<<8) | Back_Gray_Val);

    if(Target_Memory_Addr == 0)
    {
        EPD_IT8951_Display_Area(X,Y,W,H,Mode);
    }
    else
    {
        EPD_IT8951_Display_AreaBuf(X,Y,W,H,Mode,Target_Memory_Addr);
    }
    
    EPD_IT8951_WaitForDisplayReady();

    EPD_IT8951_WriteReg(UP1SR+2, EPD_IT8951_ReadReg(UP1SR+2) & ~(1<<2) );
}


void Enhance_Driving_Capability(void)
{
    UWORD RegValue = EPD_IT8951_ReadReg(0x0038);
    Debug("The reg value before writing is %x\r\n", RegValue);

    EPD_IT8951_WriteReg(0x0038, 0x0602);

    RegValue = EPD_IT8951_ReadReg(0x0038);
    Debug("The reg value after writing is %x\r\n", RegValue);
}




void EPD_IT8951_SystemRun(void)
{
    EPD_IT8951_WriteCommand(IT8951_TCON_SYS_RUN);
}


void EPD_IT8951_Standby(void)
{
    EPD_IT8951_WriteCommand(IT8951_TCON_STANDBY);
}


void EPD_IT8951_Sleep(void)
{
    EPD_IT8951_WriteCommand(IT8951_TCON_SLEEP);
}


IT8951_Dev_Info EPD_IT8951_Init(UWORD VCOM)
{
    IT8951_Dev_Info Dev_Info;

    EPD_IT8951_Reset();

    EPD_IT8951_SystemRun();

    EPD_IT8951_GetSystemInfo(&Dev_Info);
    
    //Enable Pack write
    EPD_IT8951_WriteReg(I80CPCR,0x0001);

    //Set VCOM by handle
    if(VCOM != EPD_IT8951_GetVCOM())
    {
        EPD_IT8951_SetVCOM(VCOM);
        Debug("VCOM = -%.02fV\n",(float)EPD_IT8951_GetVCOM()/1000);
    }
    return Dev_Info;
}


void EPD_IT8951_Clear_Refresh(IT8951_Dev_Info Dev_Info,UDOUBLE Target_Memory_Addr, UWORD Mode)
{
    UDOUBLE ImageSize = ((Dev_Info.Panel_W * 4 % 8 == 0)? (Dev_Info.Panel_W * 4 / 8 ): (Dev_Info.Panel_W * 4 / 8 + 1)) * Dev_Info.Panel_H;
    UBYTE* Frame_Buf = malloc (ImageSize);
    memset(Frame_Buf, 0xFF, ImageSize);


    IT8951_Load_Img_Info Load_Img_Info;
    IT8951_Area_Img_Info Area_Img_Info;

    EPD_IT8951_WaitForDisplayReady();

    Load_Img_Info.Source_Buffer_Addr = (UDOUBLE)Frame_Buf;
    Load_Img_Info.Endian_Type = IT8951_LDIMG_L_ENDIAN;
    Load_Img_Info.Pixel_Format = IT8951_4BPP;
    Load_Img_Info.Rotate =  IT8951_ROTATE_0;
    Load_Img_Info.Target_Memory_Addr = Target_Memory_Addr;

    Area_Img_Info.Area_X = 0;
    Area_Img_Info.Area_Y = 0;
    Area_Img_Info.Area_W = Dev_Info.Panel_W;
    Area_Img_Info.Area_H = Dev_Info.Panel_H;

    EPD_IT8951_HostAreaPackedPixelWrite_4bp(&Load_Img_Info, &Area_Img_Info, false);

    EPD_IT8951_Display_Area(0, 0, Dev_Info.Panel_W, Dev_Info.Panel_H, Mode);

    free(Frame_Buf);
    Frame_Buf = NULL;
}


void EPD_IT8951_1bp_Refresh(UBYTE* Frame_Buf, UWORD X, UWORD Y, UWORD W, UWORD H, UBYTE Mode, UDOUBLE Target_Memory_Addr, bool Packed_Write)
{
    IT8951_Load_Img_Info Load_Img_Info;
    IT8951_Area_Img_Info Area_Img_Info;

    EPD_IT8951_WaitForDisplayReady();

    Load_Img_Info.Source_Buffer_Addr = (UDOUBLE)Frame_Buf;
    Load_Img_Info.Endian_Type = IT8951_LDIMG_L_ENDIAN;
    //Use 8bpp to set 1bpp
    Load_Img_Info.Pixel_Format = IT8951_8BPP;
    Load_Img_Info.Rotate =  IT8951_ROTATE_0;
    Load_Img_Info.Target_Memory_Addr = Target_Memory_Addr;

    Area_Img_Info.Area_X = X/8;
    Area_Img_Info.Area_Y = Y;
    Area_Img_Info.Area_W = W/8;
    Area_Img_Info.Area_H = H;


    //clock_t start, finish;
    //double duration;

    //start = clock();

    EPD_IT8951_HostAreaPackedPixelWrite_1bp(&Load_Img_Info, &Area_Img_Info, Packed_Write);

    //finish = clock();
    //duration = (double)(finish - start) / CLOCKS_PER_SEC;
	//Debug( "Write occupy %f second\n", duration );

    //start = clock();

    EPD_IT8951_Display_1bp(X,Y,W,H,Mode,Target_Memory_Addr,0xF0,0x00);

    //finish = clock();
    //duration = (double)(finish - start) / CLOCKS_PER_SEC;
	//Debug( "Show occupy %f second\n", duration );
}



void EPD_IT8951_1bp_Multi_Frame_Write(UBYTE* Frame_Buf, UWORD X, UWORD Y, UWORD W, UWORD H,UDOUBLE Target_Memory_Addr, bool Packed_Write)
{
    IT8951_Load_Img_Info Load_Img_Info;
    IT8951_Area_Img_Info Area_Img_Info;

    EPD_IT8951_WaitForDisplayReady();

    Load_Img_Info.Source_Buffer_Addr = (UDOUBLE)Frame_Buf;
    Load_Img_Info.Endian_Type = IT8951_LDIMG_L_ENDIAN;
    //Use 8bpp to set 1bpp
    Load_Img_Info.Pixel_Format = IT8951_8BPP;
    Load_Img_Info.Rotate =  IT8951_ROTATE_0;
    Load_Img_Info.Target_Memory_Addr = Target_Memory_Addr;

    Area_Img_Info.Area_X = X/8;
    Area_Img_Info.Area_Y = Y;
    Area_Img_Info.Area_W = W/8;
    Area_Img_Info.Area_H = H;

    EPD_IT8951_HostAreaPackedPixelWrite_1bp(&Load_Img_Info, &Area_Img_Info,Packed_Write);
}




void EPD_IT8951_1bp_Multi_Frame_Refresh(UWORD X, UWORD Y, UWORD W, UWORD H,UDOUBLE Target_Memory_Addr)
{
    EPD_IT8951_WaitForDisplayReady();

    EPD_IT8951_Display_1bp(X,Y,W,H, A2_Mode,Target_Memory_Addr,0xF0,0x00);
}




void EPD_IT8951_2bp_Refresh(UBYTE* Frame_Buf, UWORD X, UWORD Y, UWORD W, UWORD H, bool Hold, UDOUBLE Target_Memory_Addr, bool Packed_Write)
{
    IT8951_Load_Img_Info Load_Img_Info;
    IT8951_Area_Img_Info Area_Img_Info;

    EPD_IT8951_WaitForDisplayReady();

    Load_Img_Info.Source_Buffer_Addr = (UDOUBLE)Frame_Buf;
    Load_Img_Info.Endian_Type = IT8951_LDIMG_L_ENDIAN;
    Load_Img_Info.Pixel_Format = IT8951_2BPP;
    Load_Img_Info.Rotate =  IT8951_ROTATE_0;
    Load_Img_Info.Target_Memory_Addr = Target_Memory_Addr;

    Area_Img_Info.Area_X = X;
    Area_Img_Info.Area_Y = Y;
    Area_Img_Info.Area_W = W;
    Area_Img_Info.Area_H = H;

    EPD_IT8951_HostAreaPackedPixelWrite_2bp(&Load_Img_Info, &Area_Img_Info,Packed_Write);

    if(Hold == true)
    {
        EPD_IT8951_Display_Area(X,Y,W,H, GC16_Mode);
    }
    else
    {
        EPD_IT8951_Display_AreaBuf(X,Y,W,H, GC16_Mode,Target_Memory_Addr);
    }
}




void EPD_IT8951_4bp_Refresh(UBYTE* Frame_Buf, UWORD X, UWORD Y, UWORD W, UWORD H, bool Hold, UDOUBLE Target_Memory_Addr, bool Packed_Write)
{
    IT8951_Load_Img_Info Load_Img_Info;
    IT8951_Area_Img_Info Area_Img_Info;

    EPD_IT8951_WaitForDisplayReady();

    Load_Img_Info.Source_Buffer_Addr = (UDOUBLE)Frame_Buf;
    Load_Img_Info.Endian_Type = IT8951_LDIMG_L_ENDIAN;
    Load_Img_Info.Pixel_Format = IT8951_4BPP;
    Load_Img_Info.Rotate =  IT8951_ROTATE_0;
    Load_Img_Info.Target_Memory_Addr = Target_Memory_Addr;

    Area_Img_Info.Area_X = X;
    Area_Img_Info.Area_Y = Y;
    Area_Img_Info.Area_W = W;
    Area_Img_Info.Area_H = H;

    EPD_IT8951_HostAreaPackedPixelWrite_4bp(&Load_Img_Info, &Area_Img_Info, Packed_Write);

    if(Hold == true)
    {
        EPD_IT8951_Display_Area(X,Y,W,H, GC16_Mode);
    }
    else
    {
        EPD_IT8951_Display_AreaBuf(X,Y,W,H, GC16_Mode,Target_Memory_Addr);
    }
}


void EPD_IT8951_8bp_Refresh(UBYTE *Frame_Buf, UWORD X, UWORD Y, UWORD W, UWORD H, bool Hold, UDOUBLE Target_Memory_Addr)
{
    IT8951_Load_Img_Info Load_Img_Info;
    IT8951_Area_Img_Info Area_Img_Info;

    EPD_IT8951_WaitForDisplayReady();

    Load_Img_Info.Source_Buffer_Addr = (UDOUBLE)Frame_Buf;
    Load_Img_Info.Endian_Type = IT8951_LDIMG_L_ENDIAN;
    Load_Img_Info.Pixel_Format = IT8951_8BPP;
    Load_Img_Info.Rotate =  IT8951_ROTATE_0;
    Load_Img_Info.Target_Memory_Addr = Target_Memory_Addr;

    Area_Img_Info.Area_X = X;
    Area_Img_Info.Area_Y = Y;
    Area_Img_Info.Area_W = W;
    Area_Img_Info.Area_H = H;

    EPD_IT8951_HostAreaPackedPixelWrite_8bp(&Load_Img_Info, &Area_Img_Info);

    if(Hold == true)
    {
        EPD_IT8951_Display_Area(X, Y, W, H, GC16_Mode);
    }
    else
    {
        EPD_IT8951_Display_AreaBuf(X, Y, W, H, GC16_Mode, Target_Memory_Addr);
    }
}

void Paint_SetPixel(UWORD Xpoint, UWORD Ypoint, UWORD Color)
{
    if(Xpoint > Paint.Width || Ypoint > Paint.Height){
        //Debug("Exceeding display boundaries\r\n");
        return;
    }      
    UWORD X, Y;

    switch(Paint.Rotate) {
    case 0:
        X = Xpoint;
        Y = Ypoint;  
        break;
    case 90:
        X = Paint.WidthMemory - Ypoint - 1;
        Y = Xpoint;
        break;
    case 180:
        X = Paint.WidthMemory - Xpoint - 1;
        Y = Paint.HeightMemory - Ypoint - 1;
        break;
    case 270:
        X = Ypoint;
        Y = Paint.HeightMemory - Xpoint - 1;
        break;
    default:
        return;
    }
    
    switch(Paint.Mirror) {
    case MIRROR_NONE:
        break;
    case MIRROR_HORIZONTAL:
        X = Paint.WidthMemory - X - 1;
        break;
    case MIRROR_VERTICAL:
        Y = Paint.HeightMemory - Y - 1;
        break;
    case MIRROR_ORIGIN:
        X = Paint.WidthMemory - X - 1;
        Y = Paint.HeightMemory - Y - 1;
        break;
    default:
        return;
    }

    if(X > Paint.WidthMemory || Y > Paint.HeightMemory){
        Debug("Exceeding display boundaries\r\n");
        return;
    }

    UDOUBLE Addr = X * (Paint.BitsPerPixel) / 8 + Y * Paint.WidthByte;

    switch( Paint.BitsPerPixel ){
        case 8:{
            Paint.Image[Addr] = Color & 0xF0;
            break;
        }
        case 4:{
            Paint.Image[Addr] &= ~( (0xF0) >> (7 - (X*4+3)%8 ) );
            Paint.Image[Addr] |= (Color & 0xF0) >> (7 - (X*4+3)%8 );
            break;
        }
        case 2:{
            Paint.Image[Addr] &= ~( (0xC0) >> (7 - (X*2+1)%8 ) );
            Paint.Image[Addr] |= (Color & 0xC0) >> (7 - (X*2+1)%8 );
            break;
        }
        case 1:{
            Paint.Image[Addr] &= ~( (0x80) >> (7 - X%8) );
            Paint.Image[Addr] |= (Color & 0x80) >> (7 - X%8);
            break;
        }
    }

}

static void Bitmap_format_Matrix(UBYTE *dst,UBYTE *src){
	UDOUBLE i,j,k;
    UBYTE *psrc = src;
    UBYTE *pdst = dst;
    UBYTE *p = psrc;
	UBYTE temp;
	UDOUBLE count;
	
	//Since the bmp storage is from the back to the front, it needs to be converted in reverse order.
	switch(bmp_BitCount)
	{
		case 1:
			pdst += (bmp_width * bmp_height);
			
			for(i=0;i<bmp_height;i++)
			{
				pdst -= bmp_width;
				count = 0;
				for (j=0;j<(bmp_width+7)/8;j++)
				{
					temp = p[j];
					
					for (k=0;k<8;k++)
					{
						pdst[0]= ((temp & (0x80>>k)) >> (7-k));
						count++;
						pdst++;
						if (count == bmp_width)
						{
							break;
						}
					}
				}
				pdst -= bmp_width;
				p += bytesPerLine;
			}
		break;
		case 4:
			pdst += (bmp_width * bmp_height);

			for(i=0;i<bmp_height;i++)
			{
				pdst -= bmp_width;
				count = 0;
				for (j=0;j<(bmp_width+1)/2;j++)
				{
					temp = p[j];
					pdst[0]= ((temp & 0xf0) >> 4);
					count++;
					pdst++;
					if (count == bmp_width)
					{
						break;
					}

					pdst[0] = temp & 0x0f;
					count++;
					pdst++;
					if (count == bmp_width)
					{
						break;
					}
				}
				pdst -= bmp_width;
				p += bytesPerLine;
			}
		break;
		case 8:
			pdst += (bmp_width*bmp_height);
			for(i=0;i<bmp_height;i++)
			{
				p = psrc+(i+1)*bytesPerLine;
				p -= skip;
				for(j=0;j<bmp_width;j++)
				{
					pdst -= 1;
					p -= 1;
					pdst[0] = p[0];
				}
			}
		break;
		case 16:
			pdst += (bmp_width*bmp_height*2);
			for(i=0;i<bmp_height;i++)
			{
				p = psrc+(i+1)*bytesPerLine;
				p -= skip;
				for(j=0;j<bmp_width;j++)
				{
					pdst -= 2;
					p -= 2;
					pdst[0] = p[1];
					pdst[1] = p[0];
				}
			}
		break;
		case 24:
			pdst += (bmp_width*bmp_height*3);
			for(i=0;i<bmp_height;i++)
			{
				p = psrc+(i+1)*bytesPerLine;
				p -= skip;
				for(j=0;j<bmp_width;j++)
				{
					pdst -= 3;
					p -= 3;
					pdst[0] = p[2];
					pdst[1] = p[1];
					pdst[2] = p[0];
				}
			}
		break;
		case 32:
			pdst += (bmp_width*bmp_height*4);
			for(i=0;i<bmp_height;i++)
			{
				p = psrc+(i+1)*bmp_width*4;
				for(j=0;j<bmp_width;j++)
				{
					pdst -= 4;
					p -= 4;
					pdst[0] = p[2];
					pdst[1] = p[1];
					pdst[2] = p[0];
					pdst[3] = p[3];
				}
			}
		break;
		
		default:
		break;
	}	

}



static void DrawMatrix(UWORD Xpos, UWORD Ypos,UWORD Width, UWORD High,const UBYTE* Matrix)
{
	UWORD i,j,x,y;
	UBYTE R,G,B;
	UBYTE temp1,temp2;
	double Gray;
	
	for (y=0,j=Ypos;y<High;y++,j++)
	{
 		for (x=0,i=Xpos;x<Width;x++,i++)
		{
			switch(bmp_BitCount)
			{
				case 1:
				case 4:
				case 8:
					R = palette[Matrix[(y*Width+x)]].rgbRed;
					G = palette[Matrix[(y*Width+x)]].rgbGreen;
					B = palette[Matrix[(y*Width+x)]].rgbBlue;
				break;
				
				case 16:
					temp1 = Matrix[(y*Width+x)*2];
					temp2 = Matrix[(y*Width+x)*2+1];
					R = (temp1 & 0x7c)<<1;
					G = (((temp1 & 0x03) << 3 ) | ((temp2&0xe0) >> 5))<<3;
					B = (temp2 & 0x1f)<<3;
				break;
				
				case 24:
					R = Matrix[(y*Width+x)*3];
					G = Matrix[(y*Width+x)*3+1];
					B = Matrix[(y*Width+x)*3+2];
				break;
				
				case 32:
					R = Matrix[(y*Width+x)*4];
					G = Matrix[(y*Width+x)*4+1];
					B = Matrix[(y*Width+x)*4+2];
				break;
				
				default:
				break;
			}
		
			Gray = (R*299 + G*587 + B*114 + 500) / 1000;
            Paint_SetPixel(i, j, Gray);
		}
	}
}



void Paint_SetMirroring(UBYTE mirror)
{
    if(mirror == MIRROR_NONE || mirror == MIRROR_HORIZONTAL ||
        mirror == MIRROR_VERTICAL || mirror == MIRROR_ORIGIN) {
        Debug("mirror image x:%s, y:%s\r\n",(mirror & 0x01)? "mirror":"none", ((mirror >> 1) & 0x01)? "mirror":"none");
        Paint.Mirror = mirror;
    } else {
        Debug("mirror should be MIRROR_NONE, MIRROR_HORIZONTAL, \
        MIRROR_VERTICAL or MIRROR_ORIGIN\r\n");
    }
}
void Paint_SelectImage(UBYTE *image)
{
    Paint.Image = image;
}

void Paint_Clear(UWORD Color)
{
    UDOUBLE ImageSize = Paint.WidthByte * Paint.HeightByte;
    memset( Paint.Image, Color,  ImageSize);
}


void Paint_SetBitsPerPixel(UBYTE bpp)
{
    if(bpp == 8 || bpp == 4 || bpp == 2 || bpp == 1){
            Paint.BitsPerPixel = bpp;
            Paint.GrayScale = pow(2, Paint.BitsPerPixel);
            Paint.WidthByte = (Paint.WidthMemory * bpp % 8 == 0)? (Paint.WidthMemory * bpp / 8 ) : (Paint.WidthMemory * bpp / 8 + 1);
    }
    else{
        Debug("Set BitsPerPixel Input parameter error\r\n");
        Debug("BitsPerPixel Only support: 1 2 4 8 \r\n");
    }
}


void Paint_NewImage(UBYTE *image, UWORD Width, UWORD Height, UWORD Rotate, UWORD Color)
{
    Paint.Image = NULL;
    Paint.Image = image;

    Paint.WidthMemory = Width;
    Paint.HeightMemory = Height;
    Paint.Color = Color;
    Paint.BitsPerPixel = 8;
    Paint.GrayScale = pow(2, Paint.BitsPerPixel);
    Paint.WidthByte = Width;
    Paint.HeightByte = Height;

    Paint.Rotate = Rotate;
    Paint.Mirror = MIRROR_NONE;

    if(Rotate == ROTATE_0 || Rotate == ROTATE_180) {
        Paint.Width = Width;
        Paint.Height = Height;
    } else {
        Paint.Width = Height;
        Paint.Height = Width;
    }
}

void DEV_Delay_us(UDOUBLE xus)
{
        bcm2835_delayMicroseconds(xus);
}


UBYTE GUI_ReadBmp(const char *path, UWORD x, UWORD y)
{
	//bmp file pointer
	FILE *fp;
	BMPFILEHEADER FileHead;
	BMPINFOHEADER InfoHead;
	UDOUBLE total_length;
	UBYTE *buf = NULL;
	UDOUBLE ret = -1;
	 
	fp = fopen(path,"rb");
	if (fp == NULL)
	{
		return(-1);
	}
 
	ret = fread(&FileHead, sizeof(BMPFILEHEADER),1, fp);
	if (ret != 1)
	{
		Debug("Read header error!\n");
		fclose(fp);
		return(-2);
	}

	//Detect if it is a bmp image, since BMP file type is "BM"(0x4D42)
	if (FileHead.bType != 0x4D42)
	{
		Debug("It's not a BMP file\n");
		fclose(fp);
		return(-3);
	}
	
	Debug("BMP_bSize:%d \n", FileHead.bSize);
 	Debug("BMP_bOffset:%d \n", FileHead.bOffset);
	
	ret = fread((char *)&InfoHead, sizeof(BMPINFOHEADER),1, fp);
	if (ret != 1)
	{
		Debug("Read infoheader error!\n");
		fclose(fp);
		return(-4);
	}
	
	Debug("BMP_biInfoSize:%d \n", InfoHead.biInfoSize);
 	Debug("BMP_biWidth:%d \n", InfoHead.biWidth);
	Debug("BMP_biHeight:%d \n", InfoHead.biHeight);
	Debug("BMP_biPlanes:%d \n", InfoHead.biPlanes);
	Debug("BMP_biBitCount:%d \n", InfoHead.biBitCount);
	Debug("BMP_biCompression:%d \n", InfoHead.biCompression);
	Debug("BMP_bimpImageSize:%d \n", InfoHead.bimpImageSize);
	Debug("BMP_biXPelsPerMeter:%d \n", InfoHead.biXPelsPerMeter);
	Debug("BMP_biYPelsPerMeter:%d \n", InfoHead.biYPelsPerMeter);
	Debug("BMP_biClrUsed:%d \n", InfoHead.biClrUsed);
	Debug("BMP_biClrImportant:%d \n", InfoHead.biClrImportant);
	
	total_length = FileHead.bSize-FileHead.bOffset;
	bytesPerLine=((InfoHead.biWidth*InfoHead.biBitCount+31)>>5)<<2;
	imageSize=bytesPerLine*InfoHead.biHeight;
	skip=(4-((InfoHead.biWidth*InfoHead.biBitCount)>>3))&3;
	
	Debug("bimpImageSize:%d\n", InfoHead.bimpImageSize);
	Debug("total_length:%d\n", total_length);
	Debug("bytesPerLine = %d\n", bytesPerLine);
	Debug("imageSize = %d\n", imageSize);
	Debug("skip = %d\n", skip);
	
    bmp_width = InfoHead.biWidth;
    bmp_height = InfoHead.biHeight;
	bmp_BitCount = InfoHead.biBitCount;
	
	//This is old code, but allocate imageSize byte memory is more reasonable
    bmp_src_buf = (UBYTE*)calloc(1,total_length);
	//bmp_src_buf = (UBYTE*)calloc(1,imageSize);
    if(bmp_src_buf == NULL){
        Debug("Load > malloc bmp out of memory!\n");
        return -1;
    }
	//This is old code, but allocate imageSize byte memory is more reasonable
	bmp_dst_buf = (UBYTE*)calloc(1,total_length);
	//bmp_dst_buf = (UBYTE*)calloc(1,imageSize);
    if(bmp_dst_buf == NULL){
        Debug("Load > malloc bmp out of memory!\n");
        return -2;
    }

	 //Jump to data area
    fseek(fp, FileHead.bOffset, SEEK_SET);
	
	//Bytes per line
    buf = bmp_src_buf;
    while ((ret = fread(buf,1,total_length,fp)) >= 0) 
	{
        if (ret == 0) 
		{
            DEV_Delay_us(100);
            continue;
        }
		buf = ((UBYTE*)buf) + ret;
        total_length = total_length - ret;
        if(total_length == 0)
            break;
    }
	
	//Jump to color pattern board
	switch(bmp_BitCount)
	{	
		case 1:
			fseek(fp, 54, SEEK_SET);
			ret = fread(palette,1,4*2,fp);
			if (ret != 8) 
			{
				Debug("Error: fread != 8\n");
				return -5;
			}

			//this is old code, will likely result in memory leak if use 1bp source bmp image
			 
			bmp_dst_buf = (UBYTE*)calloc(1,InfoHead.biWidth * InfoHead.biHeight);
			if(bmp_dst_buf == NULL)
			{
				Debug("Load > malloc bmp out of memory!\n");
				return -5;
			}
			
		break;
		
		case 4:
			fseek(fp, 54, SEEK_SET);
			ret = fread(palette,1,4*16,fp);
			if (ret != 64) 
			{
				Debug("Error: fread != 64\n");
				return -5;
			}
			//this is old code, will likely result in memory leak if use 4bp source bmp image
			
			bmp_dst_buf = (UBYTE*)calloc(1,InfoHead.biWidth * InfoHead.biHeight);
			if(bmp_dst_buf == NULL)
			{
				Debug("Load > malloc bmp out of memory!\n");
				return -5;
			}
			
		break;
		
		case 8:
			fseek(fp, 54, SEEK_SET);

			ret = fread(palette,1,4*256,fp);

			if (ret != 1024) 
			{
				Debug("Error: fread != 1024\n");
				return -5;
			}
		break;
		
		default:
		break;
	}

	Bitmap_format_Matrix(bmp_dst_buf,bmp_src_buf);
	DrawMatrix(x, y,InfoHead.biWidth, InfoHead.biHeight, bmp_dst_buf);

    free(bmp_src_buf);
    free(bmp_dst_buf);

	bmp_src_buf = NULL;
	bmp_dst_buf = NULL;

	fclose(fp);
	return(0);
}
static void Epd_Mode(int mode)
{
	if(mode == 1) {
		Paint_SetRotate(ROTATE_0);
		Paint_SetMirroring(MIRROR_HORIZONTAL);
	}else {
		Paint_SetRotate(ROTATE_0);
		Paint_SetMirroring(MIRROR_NONE);
	}
}

void Paint_SetRotate(UWORD Rotate)
{
    if(Rotate == ROTATE_0 || Rotate == ROTATE_90 || Rotate == ROTATE_180 || Rotate == ROTATE_270) {
        Debug("Set image Rotate %d\r\n", Rotate);
        Paint.Rotate = Rotate;
    } else {
        Debug("rotate = 0, 90, 180, 270\r\n");
    }
}


UBYTE Display_BMP_Example(UWORD Panel_Width, UWORD Panel_Height, UDOUBLE Init_Target_Memory_Addr, UBYTE BitsPerPixel){
    UWORD WIDTH;
    if(Four_Byte_Align == true){
        WIDTH  = Panel_Width - (Panel_Width % 32);
    }else{
        WIDTH = Panel_Width;
    }
    UWORD HEIGHT = Panel_Height;

    UDOUBLE Imagesize;

    Imagesize = ((WIDTH * BitsPerPixel % 8 == 0)? (WIDTH * BitsPerPixel / 8 ): (WIDTH * BitsPerPixel / 8 + 1)) * HEIGHT;
    if((Refresh_Frame_Buf = (UBYTE *)malloc(Imagesize)) == NULL) {
        Debug("Failed to apply for black memory...\r\n");
        return -1;
    }

    Paint_NewImage(Refresh_Frame_Buf, WIDTH, HEIGHT, 0, BLACK);
    Paint_SelectImage(Refresh_Frame_Buf);
	Epd_Mode(epd_mode);
    Paint_SetBitsPerPixel(BitsPerPixel);
    Paint_Clear(WHITE);

    char Path[30];
    sprintf(Path,"/home/pi/1.bmp", WIDTH, HEIGHT);

    GUI_ReadBmp(Path, 0, 0);

    //you can draw your character and pattern on the image, for color definition of all BitsPerPixel, you can refer to GUI_Paint.h, 
    //Paint_DrawRectangle(50, 50, WIDTH/2, HEIGHT/2, 0x30, DOT_PIXEL_3X3, DRAW_FILL_EMPTY);
    //Paint_DrawCircle(WIDTH*3/4, HEIGHT/4, 100, 0xF0, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
    //Paint_DrawNum(WIDTH/4, HEIGHT/5, 709, &Font20, 0x30, 0xB0);

    switch(BitsPerPixel){
        case BitsPerPixel_8:{
         //   Paint_DrawString_EN(10, 10, "8 bits per pixel 16 grayscale", &Font24, 0xF0, 0x00);
            EPD_IT8951_8bp_Refresh(Refresh_Frame_Buf, 0, 0, WIDTH,  HEIGHT, false, Init_Target_Memory_Addr);
            break;
        }
        case BitsPerPixel_4:{
          //  Paint_DrawString_EN(10, 10, "4 bits per pixel 16 grayscale", &Font24, 0xF0, 0x00);
            EPD_IT8951_4bp_Refresh(Refresh_Frame_Buf, 0, 0, WIDTH,  HEIGHT, false, Init_Target_Memory_Addr,false);
            break;
        }
        case BitsPerPixel_2:{
           // Paint_DrawString_EN(10, 10, "2 bits per pixel 4 grayscale", &Font24, 0xC0, 0x00);
            EPD_IT8951_2bp_Refresh(Refresh_Frame_Buf, 0, 0, WIDTH,  HEIGHT, false, Init_Target_Memory_Addr,false);
            break;
        }
        case BitsPerPixel_1:{
            //Paint_DrawString_EN(10, 10, "1 bit per pixel 2 grayscale", &Font24, 0x80, 0x00);
            EPD_IT8951_1bp_Refresh(Refresh_Frame_Buf, 0, 0, WIDTH,  HEIGHT, A2_Mode, Init_Target_Memory_Addr,false);
            break;
        }
    }

    if(Refresh_Frame_Buf != NULL){
        free(Refresh_Frame_Buf);
        Refresh_Frame_Buf = NULL;
    }
    return 0;
}
*/
import "C"
import (
	//"fmt"
	//"os"
	"github.com/gin-gonic/gin"
	//"net/http"
	//"path/filepath"
	//"strconv"
)

func main() {
	r := gin.Default()
	r.Run("0.0.0.0:9988")
}
