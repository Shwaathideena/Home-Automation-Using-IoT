// Stop Go C Example (Basic)
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include <string.h>
#include "enc28j60.h"
#include "ether.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define RESET        (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // A2

#define MAX_SYNC_MESSAGE_SIZE 8

uint8_t rxMessage[100];
uint8_t slotSize;
uint8_t deadTime;
uint8_t slotNumber;
uint8_t frameSize;
uint8_t frameNumber;
uint8_t txFramenumber = 0xFF;
uint8_t txSlot = 38;
uint32_t time;
uint32_t prevtime;
uint8_t nodeId = 0x02;
uint8_t hwAddress = 00;
uint8_t processPack[10];
bool flag = false;
uint32_t time1;
uint32_t time2;
/////////////////////////////////////////////////////////////////////////////
char str[100];
# define MAX_CHARS 80
#define MAX_MESSAGES 10
#define MAX_RXMESSAGES 20
uint8_t txMessages[MAX_MESSAGES][256];
bool txValid[MAX_MESSAGES];
uint8_t txSize[MAX_MESSAGES];
uint8_t rxMessages[MAX_RXMESSAGES][256];
bool rxValid[MAX_RXMESSAGES];
uint8_t receivedSlot[MAX_RXMESSAGES];
uint8_t receivedFramenum[MAX_RXMESSAGES];
/////////////////////////////////////////////////////////////////////////////

//Added by Shivang
#define HARDWARE_ADDRESS_BLOCK 0x0
#define HARDWARE_ADDRESS_OFFSET 0x0
#define RULE_BLOCK 0x1
#define RULE_OFFSET 0x0
uint32_t hardwareAddress=0x0;
uint32_t deviceAddress=0x0;
uint8_t data_send[50];
uint8_t seqnum=0;
uint16_t tenmin=0;
uint8_t sendp=0;
uint8_t temperature;
uint8_t humidity;
////////////////////////////////////////////////////////////////////////////////
struct accept
{
    uint8_t type;
    uint8_t size;
    uint8_t destAdd;
    uint8_t nodeId;
    uint8_t deviceId;
    uint8_t seqId;
    uint8_t crc;
}*accepts;

struct control
{
    uint8_t type;
    uint8_t size;
    uint8_t destAdd;
    uint8_t sourceAdd;
    uint8_t channel;
    uint8_t action;
    uint8_t value[10];//Size = 10
    uint8_t seqId;
    uint8_t crc;
}*ctrl;

struct event
{
    uint8_t type;
    uint8_t size;
    uint8_t broadAdd;
    uint8_t sourceAdd;
    //uint8_t eventId;
    uint8_t eventType;
    uint8_t eventMsg1;
    uint8_t eventMsg2;
    uint8_t seqId;
    uint8_t crc;
}events[2];

struct rule
{
    uint8_t type;
    uint8_t size;
    uint8_t destAdd;
    uint8_t rule[20];//size=20
    uint8_t seqId;
    uint8_t crc;
}*rules;

struct discovery
{
    uint8_t type;
    uint8_t size;
    uint8_t broadAdd;
    uint8_t sourceAdd;
    uint8_t netId;
    uint8_t seqId;
    uint8_t crc;
}*disc;

struct discoveryResponse
{
    uint8_t type;
    uint8_t size;
    uint8_t destAdd;
    uint8_t hardwareAdd;
}discRes;

struct acknowledge
{
    uint8_t type;
    uint8_t size;
    uint8_t destAdd;
    uint8_t sourceAdd;
    uint8_t seqId;
    uint8_t crc;
}*ack;

struct get
{
    uint8_t type;
    uint8_t size;
    uint8_t destAdd;
    uint8_t subAdd;
    uint8_t parameterType;
    uint8_t seqId;
    uint8_t crc;
}*gets;

struct sync
{
    uint8_t type;
    uint8_t size;
    uint8_t signature;
    uint8_t slotSize;
    uint8_t deadTime;
    uint8_t frameSize;
    uint8_t frameNum;
    uint8_t crc;
}*sync;

struct join
{
    uint8_t type;
    uint8_t size;
    uint8_t broadAdd;
    uint8_t sourceAdd;
    uint8_t hardwareAdd;
    uint8_t deviceId;
    uint8_t frameNum;
    uint8_t slot;
    uint8_t nodeId;
    uint8_t seqId;
    uint8_t crc;
}*joins;

struct joinResponse
{
    uint8_t type;
    uint8_t size;
    uint8_t destAdd;
    uint8_t deviceId;
    uint8_t deviceType;
    uint8_t deviceName;
    uint8_t capabilities;
    uint8_t seqId;
    uint8_t crc;

}*joinRes;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------


// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOD;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x1E;  // enable internal pull-up for push button

    //////////////////////// Additions by services team
    GPIO_PORTD_DIR_R = 0x0;  // bits 1-3 are outputs, other pins are inputs
    GPIO_PORTD_DEN_R = 0x0;  // enable LEDs and pushbuttons

    // Configure RESET for ENC28J60
    GPIO_PORTA_DIR_R = 0x04;  // make bit 1 an output
    GPIO_PORTA_DR2R_R = 0x04; // set drive strength to 2mA
    GPIO_PORTA_DEN_R = 0x04;  // enable bits 1 for digital

    // Configure ~CS for ENC28J60
    GPIO_PORTB_DIR_R = 0x02;  // make bit 1 an output
    GPIO_PORTB_DR2R_R = 0x02; // set drive strength to 2mA
    GPIO_PORTB_DEN_R = 0x02;  // enable bits 1 for digital

    // Configure SSI2 pins for SPI configuration
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R2;           // turn-on SSI2 clocking
    GPIO_PORTB_DIR_R |= 0x90;                        // make bits 4 and 7 outputs
    GPIO_PORTB_DR2R_R |= 0x90;                       // set drive strength to 2mA
    GPIO_PORTB_AFSEL_R |= 0xD0;                      // select alternative functions for MOSI, MISO, SCLK pins
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB7_SSI2TX | GPIO_PCTL_PB6_SSI2RX | GPIO_PCTL_PB4_SSI2CLK; // map alt fns to SSI2
    GPIO_PORTB_DEN_R |= 0xD0;                        // enable digital operation on TX, RX, CLK pins

    // Configure the SSI2 as a SPI master, mode 3, 8bit operation, 1 MHz bit rate
    SSI2_CR1_R &= ~SSI_CR1_SSE;                      // turn off SSI2 to allow re-configuration
    SSI2_CR1_R = 0;                                  // select master mode
    SSI2_CC_R = 0;                                   // select system clock as the clock source
    SSI2_CPSR_R = 40;                                // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI2_CR0_R = SSI_CR0_FRF_MOTO | SSI_CR0_DSS_8;   // set SR=0, mode 0 (SPH=0, SPO=0), 8-bit
    SSI2_CR1_R |= SSI_CR1_SSE;                       // turn on SSI2
    //

    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;                           // turn-on UART1

    // configure UART1 pins
    GPIO_PORTC_DIR_R |= 0x60;                                          // define TX(PC5),D_EN(PC6) as output and RX(PC4) as input
    GPIO_PORTC_DEN_R |= 0x70;                                          // enable TX,D_EN and RX as digital pins
    GPIO_PORTC_PDR_R |= 0x70;                                          // pull down TX,D_EN and RX as digital pins
    GPIO_PORTC_AFSEL_R |= 0x30;                                        // enable alternate function on PC4 and PC5
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;      // define peripheral control for TX and RX of UART1

    // Configure UART1 to 38400 baud
    UART1_CTL_R = 0;                                                                                   // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                                                                    // use system clock (40 MHz)
    UART1_IBRD_R = 21;                                                                                 // r = 40 MHz / (Nx38400Hz), set floor(r)=65, where N=16
    UART1_FBRD_R = 45;                                                                                  // round(fract(r)*64)=7
    UART1_LCRH_R = UART_LCRH_WLEN_8;// | UART_LCRH_FEN ;                   // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;                                       // enable TX, RX and module
    UART1_IM_R = UART_IM_RXIM;
    NVIC_EN0_R |= 1 << (INT_UART1-16);
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;                                       // enable TX, RX and module

    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;                           // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                                             // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                                           // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;       // define peripheral control for TX and RX of UART0

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                                   // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                                    // use system clock (40 MHz)
    UART0_IBRD_R = 21;                                                 // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                                                 // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;                   // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;

}

void initEEPROM()
{
     //Enable EEPROM peripheral for writing Source address to flash memory in response to SET address command
       SYSCTL_RCGCEEPROM_R = SYSCTL_RCGCEEPROM_R0;                                   // wait 6 clocks
       __asm("   NOP");__asm("   NOP");__asm("   NOP");__asm("   NOP");
       while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
       if((EEPROM_EESUPP_R & 0x0C) != 0)
       {
           putsUart0("EEPROM ERROR!!");
       }
       EEPROM_EEBLOCK_R = RULE_BLOCK;
       EEPROM_EEOFFSET_R = RULE_OFFSET;
}


void initTimer1()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = (slotSize+deadTime)*40000;
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

}

void initTimer2()
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;       // turn-on timer
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER2_TAILR_R = 0xFFFFFFFF;
    TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

void putcUart1(uint8_t c)
{
    while (UART1_FR_R & UART_FR_TXFF);    // wait till the TX FIFO is empty
    UART1_DR_R = c;                        // put the character in the FIFO
}

void putsUart1(char* str)
{
    uint8_t i;                            // define a counter
    for (i = 0; i < strlen(str); i++)     // increment the counter till the end of the string
      putcUart1(str[i]);                  // put individual characters on the FIFO

}

uint8_t getcUart1()
{
    uint16_t i;
    i = 0;
    while (UART1_FR_R & UART_FR_RXFE);    // wait till the TX FIFO is empty
    return UART1_DR_R & 0xFF;                        // put the character in the FIFO
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);     // wait till the TX FIFO is empty
    UART0_DR_R = c;                        // put the character in the FIFO
}

//
void nextLine()
{
    putcUart0('\n');
    putcUart0('\r');
}

void putsUart0(char* str)
{
    uint8_t i;                            // define a counter
    uint8_t s;
    s = stringLength(str);
    for (i = 0; i < s; i++)               // increment the counter till the end of the string
      putcUart0(str[i]);                  // put individual characters on the FIFO
}

uint16_t stringLength(char st[])
{
    uint16_t i;
    i = 0;
    while(st[i] != 0)
    {
        i++;
    }
    i++;
    return i;
}

// Blocking function that writes a number when the UART buffer is not full
void putnUart0(uint8_t c)
{
    char snum[10];
    uint8_t a = c;
    uint8_t n,i = 0,j = 0;
    while(a!=0)
    {
        //a = a%10;
        a = a/10;
        i++;
    }
    snum[i] = 0;
    a = c;
    if(c == 0)
    putsUart0("0");
    while(a != 0)
    {
        n = a%10;
        a = a/10;
        snum[i-1-j] = n+48;
        j++;
    }
    putsUart0(snum);                   //prints a number after convertig it to ASCII
}


// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);    // wait till the RX FIFO is full
    return UART0_DR_R & 0xFF;             // return the received character after masking the control information
}

//Blocking function that reads a string when the UART buffer is full
void getsUart0()
{
    uint32_t count = 0;                  // define counter
    char c;                              // define a temporary storage variable
    get : c = getcUart0();               // receive a character
    if(c == 8)                           //check if character entered is a backspace
    {
        if(count > 0)                    // check if count is greater than 0
            count--;                     // decrement count if count is greater than 0
        goto get;                        // get another character
    }

    else if(c == 13)                     //check if the character entered is a carriage return
    {
       done : str[count++] = 0;          // end the string if a carriage return is entered
       return;                           //return from the loop
    }
    else if(c >= ' ')                    //check if the entered character is valid
    {
        str[count++] = c;                // store the character
        if(count >= MAX_CHARS)           // check if the count exceeds the max number of characters allowed
            goto done;                   // end the string and return from loop
        else
            goto get;                    // get another character
    }
    else
        goto get;                        // get another character
}

uint16_t getNumber()
{
    uint16_t number;
    char* c;
    c = &str[0];                                         // obtain the position of the number if valid
    number = atoi(c);                                             // convert the string to an integer
    return number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void uart1Isr()
{
    uint8_t i,j;
    i = 0;
    j = 0;
    uint8_t CRC;
    bool received_true = false;
    if((slotNumber != txSlot) && (slotNumber != (frameSize*1000/(slotSize+deadTime))))
    {
        while(rxValid[i])
        {
            i++;
            if(i>MAX_RXMESSAGES)
            {
                i = 0;
                goto receive;
            }
        }
    receive:while(TIMER2_TAV_R > time1 + 1000 - ((slotSize+deadTime)*40000))       //1000 is added to exit the isr slightly before end of slot
        {
            if(!(UART1_FR_R & UART_FR_RXFE))
            {
               rxMessages[i][j++] = UART1_DR_R & 0xFF;
               received_true = true;
            }
        }
        if(received_true)
          rxValid[i] = checkMessage(i);
        if(rxValid[i])
        {
            receivedSlot[i] = slotNumber;
            receivedFramenum[i] = frameNumber;
        }
    }
    UART1_ICR_R = UART_ICR_RXIC;
}

void timer1Isr()
{

    int k = 0;
    TIMER2_TAV_R = 0xFFFFFFFF;
    time1 = TIMER2_TAV_R;
    prevtime = time1;
    uint8_t i,j;
    uint8_t CRC;
    bool received_true = false;
    slotNumber++;
    if(slotNumber >= (frameSize*1000/(slotSize+deadTime)))
    {
        while(slotNumber != 0xFF)
        {
            rxMessage[0] = getcUart1();
               if(rxMessage[0] == 0xFF)
               {
                    rxMessage[1] = getcUart1();
                   for(i=0;i<rxMessage[1];i++)
                   {
                       rxMessage[i+2] = getcUart1();
                   }
                   CRC = 0;
                   for(i=0;i<rxMessage[1]+1;i++)
                   {
                       CRC += rxMessage[i];
                   }
                   CRC = ~CRC;
                   if(CRC == rxMessage[rxMessage[1]+1])
                   {
                       tenmin++;    //ten minute wait for server hit weather
                       sendp++;     //one minute to send the data packet for weather
                       RED_LED = 0;
                       if(!((rxMessage[rxMessage[1]])&0x01))
                           BLUE_LED = 1;
                       else
                           BLUE_LED = 0;
                       slotSize = rxMessage[6];
                       deadTime = rxMessage[7];
                       frameSize = rxMessage[8];
                       frameNumber = rxMessage[9];
                       slotNumber = 0xFF;
                       TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
                       TIMER1_TAILR_R = 1000;
                       TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
                   }
               }
              if(TIMER2_TAV_R < 0xFFF00000)
               {
                   RED_LED = 1;
                   BLUE_LED = 0;
               }
        }

    }
    else if(slotNumber == txSlot)
     //if(txValid[0] == true)
   {

        i = 0;
        j = 0;
        while(!txValid[i])
        {
            i++;
            if(i >= MAX_MESSAGES)
                goto here;
        }
        while((TIMER2_TAV_R > time1 + 1000 - (slotSize*40000)) && (j < txSize[i]))
        {
            if(!(UART1_FR_R & UART_FR_TXFF))
            {
                UART1_DR_R = txMessages[i][j++];
            }
        }
        txValid[i] = false;
    here:while(TIMER2_TAV_R > time1 + 1000 - (slotSize*40000));     //1000 is added to exit the isr slightly before end of slot
        time2 = TIMER2_TAV_R;
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_TAILR_R = ((slotSize+deadTime)*40000)-(time1-time2)-10000;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    }
    else if((slotNumber != txSlot) && (slotNumber != (frameSize*1000/(slotSize+deadTime))))
    {
        time2 = TIMER2_TAV_R;
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_TAILR_R = ((slotSize+deadTime)*40000)-(time1-time2)-10000;
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}



bool checkMessage(uint8_t num)
{
    uint8_t i,CRC;
    CRC = 0;
    for(i=0;i<rxMessages[num][1]+2-1;i++)
    {
        CRC += rxMessages[num][i];
    }
    CRC = ~CRC;
    if(CRC == rxMessages[num][rxMessages[num][1]+2-1])
        return true;
    else
        return false;
}

void discoveryProcess()
{
    //int flag2 = 0;
    static uint8_t l = 0;
    uint8_t crc1 = 0;
    uint8_t i,j,m;
    i = 0;
    m = 0;
    uint8_t k = 0;
    uint8_t crc1_check = 0;
    while(!rxValid[l])
    {
        l++;
        if(l > MAX_RXMESSAGES)
        {
            l = 0;
            return;
        }
    }
    for( i = 0 ; i<=MAX_MESSAGES ; i++)
    {
        if (txValid[i] == true)
               {
                   continue;
               }
        else if ( flag == 0)
        {
            switch (rxMessages[l][0])
            {
            case 0xAE:                                        // DISCOVERY MESSAGE
            {
                crc1_check = 0;
                if(nodeId == 0x00)
                {
                    txMessages[i][0] = 0xAF;                       // command
                    txMessages[i][1] = 0x04;                       // size
                    txMessages[i][2] = rxMessages[l][3];             // dest Address
                    txMessages[i][3] = hwAddress;                  // source Address
                    txMessages[i][4] = rxMessages[l][5];             // sequence Id
                    for(k = 0; k < 5 ; k++)
                    {
                       crc1 += txMessages[i][k];
                    }
                    crc1 = ~crc1 ;
                    txMessages[i][5] = crc1;                        //CRC
                    txSize[i] = 0x06;
                    txValid[i] = true;
                    rxValid[l] = false;
                    l++;
                }
               return;
            }
            case 0xBE:                                          // JOIN RESPONSE
            {
                crc1_check = 0;
                crc1 = 0;
                //if(nodeId == 0)                                // validate the message first then change it to new address and then do not make it enter this loop by adding the condition of the assigned address.

                 if ((hwAddress  == rxMessages[l][4]) &&  (nodeId == rxMessages[l][2]))
                 {
               nodeId = rxMessages[l][5];
               EEPROM_EEBLOCK_R=0x00;
               EEPROM_EEOFFSET_R=0x00;
               EEPROM_EERDWR_R &= 0xFFFF00FF ;
               __asm("   NOP");__asm("   NOP");__asm("   NOP");__asm("   NOP");
               EEPROM_EEBLOCK_R=0x00;
               EEPROM_EEOFFSET_R=0x00;
               EEPROM_EERDWR_R |= (nodeId<<8);
               __asm("   NOP");__asm("   NOP");__asm("   NOP");__asm("   NOP");
               txMessages[i][0] = 0xBF;                          // COMMAND TYPE
               txMessages[i][1] = 0x04;                          // SIZE
               txMessages[i][2] = rxMessages[l][3];                // DEST ADDRESS
               txMessages[i][3] = nodeId;                        // SOURCE ADDRESS
               txMessages[i][4] = 0x01;                          // SEQ ID
               for(k = 0; k < 5 ; k++)
               {
                   crc1 += txMessages[i][k];
               }
               crc1 = ~crc1 ;
               txMessages[i][5] = crc1;    //crc
               txSize[i] = 0x06;
               txSlot = rxMessages[l][6];
               EEPROM_EEBLOCK_R=0x00;
               EEPROM_EEOFFSET_R=0x00;
               EEPROM_EERDWR_R &= 0x00FFFFFF ;
               __asm("   NOP");__asm("   NOP");__asm("   NOP");__asm("   NOP");
               EEPROM_EEBLOCK_R=0x00;
               EEPROM_EEOFFSET_R=0x00;
               EEPROM_EERDWR_R |= (txSlot<<24);
               __asm("   NOP");__asm("   NOP");__asm("   NOP");__asm("   NOP");
               txValid[i] = true;
               rxValid[l] = false;
               l++;
                 }

               return;
            }
            default:
            {
                l++;
                return;
            }
            }
        }
        else
            break;
    }

}
                            //time macro    //seqID     chksm
//packet = 0c 07 0xff 0x03 0x02 0xHHMM      0x0     0x0
//        type  data size, broadcast addr

void sendPacket(uint8_t *data)
{
    uint8_t i,j;
    i = 0;
    j = 0;
    uint8_t size = 0;
    while(txValid[i])
    {
        i++;
        if(i > MAX_MESSAGES)
            return;
    }

    txMessages[i][j++] = (*data);
    data++;
    size = (*data);
    txMessages[i][j++] = (*data);
    data++;
    for(j=0;j<size;j++)
    {
        txMessages[i][j+2] = (*data);
        data++;
    }
    txSize[i] = size + 2;
    txValid[i] = true;
}

void checkReset()
{
    waitMicrosecond(1000000);
    if(PUSH_BUTTON == 0)
    {
        GREEN_LED = 1;
        waitMicrosecond(1000000);
        GREEN_LED = 0;
        EEPROM_EEBLOCK_R=0x00;
        EEPROM_EEOFFSET_R=0x00;
        EEPROM_EERDWR_R &= 0xFFFF00FF ;
        __asm("   NOP");__asm("   NOP");__asm("   NOP");__asm("   NOP");
        EEPROM_EEBLOCK_R=0x00;
        EEPROM_EEOFFSET_R=0x00;
        EEPROM_EERDWR_R &= 0x00FFFFFF ;
        __asm("   NOP");__asm("   NOP");__asm("   NOP");__asm("   NOP");
        nodeId = 0;
        txSlot = 0;
    }
    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    //Added by services - Weather
    uint8_t* udpData;
    uint8_t* tcpData;
    uint8_t tcpStart=1;          //to start tcp connection
    uint8_t http=0;             //to start http connection
    uint8_t parse=0;            //to parse data
    uint8_t finack=0;           //to close connection
    uint8_t sendtcp=1;          //to send tcp syn

    initHw();
    initTimer2();
    hwAddress = 2;
    nodeId = 4;
    txSlot = 2;
    uint8_t i,CRC;
    RED_LED = 1;
    for(i=0;i<MAX_MESSAGES;i++)
        txValid[i] = false;
    for(i=0;i<MAX_RXMESSAGES;i++)
        rxValid[i] = false;
        bool gotSync = false;
        while(!gotSync)
        {
            rxMessage[0] = getcUart1();
               if(rxMessage[0] == 0xFF)
               {
                    rxMessage[1] = getcUart1();
                   for(i=0;i<rxMessage[1];i++)
                   {
                       rxMessage[i+2] = getcUart1();
                   }
                   CRC = 0;
                   for(i=0;i<rxMessage[1]+1;i++)
                   {
                       CRC += rxMessage[i];
                   }
                   CRC = ~CRC;
                   if(CRC == rxMessage[rxMessage[1]+1])
                   {

                       RED_LED = 0;
                       if(!((rxMessage[rxMessage[1]])&0x01))
                           BLUE_LED = 1;
                       else
                           BLUE_LED = 0;
                       slotSize = rxMessage[6];
                       deadTime = rxMessage[7];
                       frameSize = rxMessage[8];
                       frameNumber = rxMessage[9];
                       slotNumber = 0;
                       initTimer1();
                       gotSync = true;
                   }
               }
        }
        while(nodeId == 0)
        {
           discoveryProcess();
        }
        // init ethernet interface
        etherInit(ETHER_UNICAST | ETHER_BROADCAST | ETHER_HALFDUPLEX);
        etherSetIpAddress(192,168,137,202);

        // flash phy leds
        etherWritePhy(PHLCON, 0x0880);
        RED_LED = 1;
        waitMicrosecond(500000);
        etherWritePhy(PHLCON, 0x0990);
        RED_LED = 0;
        waitMicrosecond(500000);
        //send a tcp data connection to start the process
        uint8_t data[1530]={0x46,0x00,0xC0,0x00,0x02,0x03,0x04,0x05,0x06,0x07,0xC8,0xD3,0xFF,0xD3,0x57,0x59,0x08,0x00,0x45,0x00,0x00,0x34,0x4F,0xE6,0x40,0x00,0x80,0x06,0x16,0xC1,0xC0,0xA8,0x89,0x01,0xC0,0xA8,0x89,0xCA,0xC3,0xA8,0x04,0x00,0xA3,0xF3,0x9F,0x6E,0x00,0x00,0x00,0x00,0x80,0x02,0x44,0x70,0x8B,0x78,0x00,0x00,0x02,0x04,0x05,0xB4,0x01,0x03,0x03,0x08,0x01,0x01,0x04,0x02,0x00};
        while(true)
        {
            if(PUSH_BUTTON == 0)
            {
                checkReset();
            }
            if (etherKbhit())
            {
                if (etherIsOverflow())
                {
                    RED_LED = 1;
                    waitMicrosecond(100000);
                    RED_LED = 0;
                }
                if(sendtcp==1)      //send tcp data for the first time
                {
                    if(etherIsTcp(data) && (tcpStart==1))
                    {
                        ethersendTcpSyn();
                        tcpStart=0;
                        http=1;
                        sendtcp=0;
                    }
                }
                if(sendtcp==1 && tenmin>=600)       //send tcp data for every 10 minutes
                {
                    if(tcpStart==1)
                    {
                        ethersendTcpSyn();
                        tcpStart=0;
                        http=1;
                        sendtcp=0;
                    }
                }
                // get packet
                etherGetPacket(data, 1530);
                // handle arp request
                if (etherIsArp(data))
                {
                    etherSendArpResp(data);
                    RED_LED = 1;
                    GREEN_LED = 1;
                    waitMicrosecond(50000);
                    RED_LED = 0;
                    GREEN_LED = 0;
                }
                // handle ip datagram
                if (etherIsIp(data))
                {
                    if (etherIsIpUnicast(data))
                    {
                        // handle icmp ping request
                        if (etherIsPingReq(data))
                        {
                            etherSendPingResp(data);
                            RED_LED = 1;
                            BLUE_LED = 1;
                            waitMicrosecond(50000);
                            RED_LED = 0;
                            BLUE_LED = 0;
                        }
                        //handle (fin,ack) message to send ack
                        if(etherIsFinAck(data) && (finack==1))
                        {
                            etherSendTcpAckback(data);
                            finack=0;
                            tcpStart=1;
                            sendtcp=1;
                            parse=0;
                            http=0;
                            tenmin=0;
                        }
                        //parse the data stored in ack
                        if(etherIsTcpAck(data) && (parse==1) && (http==0))
                        {
                            temperature = parsetemp(data);
                            humidity = parsehum(data);
                        }
                        //handle 200 OK message to send ack
                        if (etherIsTcp(data) && (http==0) && (tcpStart==0))
                        {
                            if(humidity==0x00)
                            {
                                humidity = parsehum1(data);
                            }
                            etherSendHttpAckback(data);
                            finackcomplete(data);
                            finack=1;
                        }
                        //handle (syn,ack) message to send ack and http message
                        if (etherIsTcp(data) && (tcpStart==0) && (http==1))
                        {
                            etherSendTcpAckback(data);
                            etherSendhttpget(data);
                            http=0;
                            parse=1;
                        }
                    }
                }
            }

            //store send packet data
            struct event *e;
            e = &events[0];
            e->type = 0x0C;
            e->size = 7;
            e->broadAdd = 0xFF;
            e->sourceAdd = 0x02;
            e->eventType = 0x04;
            e->eventMsg1 = temperature;
            e->eventMsg2 = humidity;
            e->seqId = seqnum++;
            e->crc = ((e->type + e->size +  e->broadAdd + e->sourceAdd + e->eventType + e->eventMsg1 + e->eventMsg2 + e->seqId));
            e->crc = ~(e->crc);

            //send packet data for every 1 minute
            if(sendp>=60)
            {
                sendPacket(e);
                putsUart0("Sent data\r\n");
                sendp=0;
            }
        }
}
