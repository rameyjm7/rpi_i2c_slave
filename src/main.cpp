#include <pigpio.h>
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#include "unistd.h"
#include <stdarg.h>

// global variables
static const uint8_t     max_register  = 8;          // the number of registers, after this we return 0xff
const static bool debug         = false;             // set true for extra print statements
const uint8_t     slaveAddress  = 0x62;              // 8 bit address for convenience
const uint8_t     slaveAddress7 = slaveAddress >> 1; // 7 bit address without R/W bit (0)
const uint32_t    microsleep    = 1000;              // number of micro seconds to sleep if no i2c transaction received
bsc_xfer_t xfer;                                     // Struct to control data flow
uint8_t regsi[max_register];

// function prototypes
void runSlave();
void closeSlave();
int getControlBits(int, bool);


uint8_t dbg_fprintf (FILE *out,char const* format, ...)
{
    if(debug)
    {
       va_list args;
       va_start (args, format);
       if (out) {
           if (int r = vfprintf (out, format, args))
               return r;
           if(out!= stdout) fflush (out);
       }
       va_end (args);
    }
    return 0;
}

uint8_t slave_init()
{
    gpioInitialise();
    printf("# %s : Initialized GPIOs\n", __func__);
    // Close old device (if any)
    xfer.control = getControlBits(slaveAddress7, false); // To avoid conflicts when restart$
    bscXfer(&xfer);
    // Set I2C slave Address to 0x0A
    printf("# %s : Setting I2C slave address to 0x%02x\n",__func__, slaveAddress);
    xfer.control = getControlBits(slaveAddress7, true);
    int r = bscXfer(&xfer); // Should now be visible in I2C-Scanners
    regsi[0] =  0x01;;
    regsi[1] =  0x02;
    regsi[2] =  0x03;
    regsi[3] =  0x04;
    regsi[4] =  0x05;
    regsi[5] =  0x06;
    regsi[6] =  0x07;
    regsi[7] =  0x08;
    return r;
}

void show()
{
  printf("%s \n",__func__);
  printf("regs ");
  for(int i =0 ; i < max_register ; i++)
     printf("%02x ", i);
  printf("\n");
  printf("vals ");
  for(int i =0 ; i < max_register ; i++)
     printf("%02x ", regsi[i]);
  printf("\n");
}

int main(int argc, char* argv[]){
    runSlave();
    closeSlave();
    return 0;
}

void runSlave()
{
    int status = 0;
    status = slave_init();
    if (status >= 0)
    {
        printf("# %s : Opened slave\n", __func__);
        xfer.rxCnt = 0;
        while(1)
        {
            bscXfer(&xfer);
            if(xfer.rxCnt == 0 )
            {
                dbg_fprintf(stdout,"Waiting...\n");
                memset( xfer.txBuf, '\0', sizeof(char)*BSC_FIFO_SIZE );
                usleep(microsleep);
            }
            else
            {
               // this will be I2C read function, return the register number rxBuf[0]
               if(xfer.rxCnt == 1)
               {
                   printf("# Received %d bytes\n", xfer.rxCnt);
                   printf("# Byte %d : 0x%02x\n", 0, xfer.rxBuf[0]);
                   uint8_t reg_to_read = xfer.rxBuf[0];
                   printf("# Master requested to read register %d, value %02x \n", reg_to_read,regsi[reg_to_read]);
                   xfer.txCnt = 1;
                   memset( xfer.txBuf, regsi[reg_to_read], 1);
                   int r = bscXfer(&xfer);
                   xfer.rxCnt = 0;
                   xfer.txCnt = 0;
                   memset( xfer.rxBuf, '\0', sizeof(char)*BSC_FIFO_SIZE);
                   memset( xfer.txBuf, '\0', sizeof(char)*BSC_FIFO_SIZE);
                   printf("# Transfer returned %d \n",r);
                   show();
               }
               else if(xfer.rxCnt == 2)
               {
                   printf("# Received %d bytes\n", xfer.rxCnt);
                   int reg = xfer.rxBuf[0];
                   int val = xfer.rxBuf[1];
                   printf("# Writing register %d with value %x\n",reg,val);
                   regsi[reg] = val;
//                   printf("# Reading register %d : %x\n",reg,regsi[reg]);
                   show();
               }
            }
        }
    }
    else
    {
       printf("Failed to open slave!!!\n");
       exit(1);
    }
}

void closeSlave() {
    gpioInitialise();
    printf("Initialized GPIOs\n");
    xfer.control = getControlBits(slaveAddress, false);
    bscXfer(&xfer);
    printf("Closed slave.\n");
    gpioTerminate();
    printf("Terminated GPIOs.\n");
}


int getControlBits(int address /* max 127 */, bool open) {
    /*
    Excerpt from http://abyz.me.uk/rpi/pigpio/cif.html#bscXfer regarding the control bits:

    22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    a  a  a  a  a  a  a  -  -  IT HC TF IR RE TE BK EC ES PL PH I2 SP EN

    Bits 0-13 are copied unchanged to the BSC CR register. See pages 163-165 of the Broadcom 
    peripherals document for full details. 

    aaaaaaa defines the I2C slave address (only relevant in I2C mode)
    IT  invert transmit status flags
    HC  enable host control
    TF  enable test FIFO
    IR  invert receive status flags
    RE  enable receive
    TE  enable transmit
    BK  abort operation and clear FIFOs
    EC  send control register as first I2C byte
    ES  send status register as first I2C byte
    PL  set SPI polarity high
    PH  set SPI phase high
    I2  enable I2C mode
    SP  enable SPI mode
    EN  enable BSC peripheral
    */

    // Flags like this: 0b/*IT:*/0/*HC:*/0/*TF:*/0/*IR:*/0/*RE:*/0/*TE:*/0/*BK:*/0/*EC:*/0/*ES:*/0/*PL:*/0/*PH:*/0/*I2:*/0/*SP:*/0/*EN:*/0;

    int flags;
    if(open)
        flags = /*RE:*/ (1 << 9) | /*TE:*/ (1 << 8) | /*I2:*/ (1 << 2) | /*EN:*/ (1 << 0);
    else // Close/Abort
        flags = /*BK:*/ (1 << 7) | /*I2:*/ (0 << 2) | /*EN:*/ (0 << 0);

    return (address << 16 /*= to the start of significant bits*/) | flags;
}
