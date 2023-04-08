void disable_A9_interrupts(void);
void set_A9_IRQ_stack(void);
void config_GIC(void);
void config_PS2(void);
void enable_A9_interrupts(void);
void PS2_ISR(void);
void HEX_PS2(char b1, char b2, char b3);
void changeBoxPosition(char b2, char b3);
void draw_box(int x, int y, int box_color);
void plot_pixel(int x, int y, short int line_color);
void wait_for_vsync();
void clear_screen();

/* This files provides address values that exist in the system */

#define BOARD                 "DE1-SoC"

/* Memory */
#define DDR_BASE              0x00000000
#define DDR_END               0x3FFFFFFF
#define A9_ONCHIP_BASE        0xFFFF0000
#define A9_ONCHIP_END         0xFFFFFFFF
#define SDRAM_BASE            0xC0000000
#define SDRAM_END             0xC3FFFFFF
#define FPGA_ONCHIP_BASE      0xC8000000
#define FPGA_ONCHIP_END       0xC803FFFF
#define FPGA_CHAR_BASE        0xC9000000
#define FPGA_CHAR_END         0xC9001FFF

/* Cyclone V FPGA devices */
#define LEDR_BASE             0xFF200000
#define HEX3_HEX0_BASE        0xFF200020
#define HEX5_HEX4_BASE        0xFF200030
#define SW_BASE               0xFF200040
#define KEY_BASE              0xFF200050
#define JP1_BASE              0xFF200060
#define JP2_BASE              0xFF200070
#define PS2_BASE              0xFF200100
#define PS2_DUAL_BASE         0xFF200108
#define JTAG_UART_BASE        0xFF201000
#define JTAG_UART_2_BASE      0xFF201008
#define IrDA_BASE             0xFF201020
#define TIMER_BASE            0xFF202000
#define AV_CONFIG_BASE        0xFF203000
#define PIXEL_BUF_CTRL_BASE   0xFF203020
#define CHAR_BUF_CTRL_BASE    0xFF203030
#define AUDIO_BASE            0xFF203040
#define VIDEO_IN_BASE         0xFF203060
#define ADC_BASE              0xFF204000

/* Cyclone V HPS devices */
#define HPS_GPIO1_BASE        0xFF709000
#define HPS_TIMER0_BASE       0xFFC08000
#define HPS_TIMER1_BASE       0xFFC09000
#define HPS_TIMER2_BASE       0xFFD00000
#define HPS_TIMER3_BASE       0xFFD01000
#define FPGA_BRIDGE           0xFFD0501C

/* ARM A9 MPCORE devices */
#define   PERIPH_BASE         0xFFFEC000    // base address of peripheral devices
#define   MPCORE_PRIV_TIMER   0xFFFEC600    // PERIPH_BASE + 0x0600

/* Interrupt controller (GIC) CPU interface(s) */
#define MPCORE_GIC_CPUIF      0xFFFEC100    // PERIPH_BASE + 0x100
#define ICCICR                0x00          // offset to CPU interface control reg
#define ICCPMR                0x04          // offset to interrupt priority mask reg
#define ICCIAR                0x0C          // offset to interrupt acknowledge reg
#define ICCEOIR               0x10          // offset to end of interrupt reg
/* Interrupt controller (GIC) distributor interface(s) */
#define MPCORE_GIC_DIST       0xFFFED000    // PERIPH_BASE + 0x1000
#define ICDDCR                0x00          // offset to distributor control reg
#define ICDISER               0x100         // offset to interrupt set-enable regs
#define ICDICER               0x180         // offset to interrupt clear-enable regs
#define ICDIPTR               0x800         // offset to interrupt processor targets regs
#define ICDICFR               0xC00         // offset to interrupt configuration regs

/* VGA colors */
#define WHITE 0xFFFF
#define YELLOW 0xFFE0
#define RED 0xF800
#define GREEN 0x07E0
#define BLUE 0x001F
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define GREY 0xC618
#define PINK 0xFC18
#define ORANGE 0xFC00

#define ABS(x) (((x) > 0) ? (x) : -(x))

/* Screen size. */
#define RESOLUTION_X 320
#define RESOLUTION_Y 240

/* include */
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

/* ********************************************************************************
* basically this is the example code for the ps2 port (hex display make break codes) 
* but now instead of always reading, keyboard input interrupts!!
********************************************************************************/
char byte1 = 0, byte2 = 0, byte3 = 0;
int xRED = 50;
int yRED = 100;
int dxRED = 0;
int dyRED = 0;

bool jumpRED = false;
int countJumpFrameRED = 0;
int prevXPositionsRED[3] = {0};
int prevYPositionsRED[3] = {0};

int xBLUE = 100;
int yBLUE = 150;
int dxBLUE = 0;
int dyBLUE = 0;

bool jumpBLUE = false;
int countJumpFrameBLUE = 0;
int prevXPositionsBLUE[3] = {0};
int prevYPositionsBLUE[3] = {0};

int prev = 0;
volatile int pixel_buffer_start; // global variable

// main program
int main(void) {
    // set up interrupts
    disable_A9_interrupts(); // disable interrupts in the A9 processor
    set_A9_IRQ_stack(); // initialize the stack pointer for IRQ mode
    config_GIC(); // configure the general interrupt controller
    config_PS2(); // configure pushbutton KEYs to generate interrupts
    enable_A9_interrupts(); // enable interrupts in the A9 processor

    //pointer variables
    volatile int * PS2_ptr = (int *)PS2_BASE;
    volatile int * pixel_ctrl_ptr = (int *)0xFF203020;
    
    // reset ps2
    *(PS2_ptr) = 0xFF;

    /* set front pixel buffer to start of FPGA On-chip memory */
	*(pixel_ctrl_ptr + 1) = 0xC8000000; // first store the address in the
										// back buffer

	/* now, swap the front/back buffers, to set the front buffer location */
	wait_for_vsync();
	/* initialize a pointer to the pixel buffer, used by drawing functions */
	pixel_buffer_start = *pixel_ctrl_ptr;
	clear_screen(); // pixel_buffer_start points to the pixel buffer
	/* set back pixel buffer to start of SDRAM memory */
	*(pixel_ctrl_ptr + 1) = 0xC0000000;
	pixel_buffer_start = *(pixel_ctrl_ptr + 1); // we draw on the back buffer
	clear_screen(); // pixel_buffer_start points to the pixel buffer

    
    while (1) // wait for an interrupt
    {
        HEX_PS2(byte1, byte2, byte3); //display make/break codes  
        changeBoxPosition(byte2, byte3); 
        draw_box(50, 108, GREEN);
        wait_for_vsync(); // swap front and back buffers on VGA vertical sync
		pixel_buffer_start = *(pixel_ctrl_ptr + 1); // new back buffer
    }
}
/* setup the PS2 interrupts in the FPGA */
void config_PS2() {
    volatile int * ps2_ptr = (int *) 0xFF200100; // pushbutton KEY base address
    *(ps2_ptr+1) = 0x1;
}
/* This file:
* 1. defines exception vectors for the A9 processor
* 2. provides code that sets the IRQ mode stack, and that dis/enables
* interrupts
* 3. provides code that initializes the generic interrupt controller
*/
void PS2_ISR(void);
void config_interrupt(int, int);
// Define the IRQ exception handler
void __attribute__((interrupt)) __cs3_isr_irq(void) {
    // Read the ICCIAR from the CPU Interface in the GIC
    int interrupt_ID = *((int *)0xFFFEC10C);
    if (interrupt_ID == 79) // check if interrupt is from the KEYs
        PS2_ISR();
    else
        while (1); // if unexpected, then stay here
    // Write to the End of Interrupt Register (ICCEOIR)
    *((int *)0xFFFEC110) = interrupt_ID;
}
// Define the remaining exception handlers
void __attribute__((interrupt)) __cs3_reset(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_undef(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_swi(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_pabort(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_dabort(void) {
    while (1);
}
void __attribute__((interrupt)) __cs3_isr_fiq(void) {
    while (1);
}
/*
* Turn off interrupts in the ARM processor
*/
void disable_A9_interrupts(void) {
    int status = 0b11010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}
/*
* Initialize the banked stack pointer register for IRQ mode
*/
void set_A9_IRQ_stack(void) {
    int stack, mode;
    stack = 0xFFFFFFFF - 7; // top of A9 onchip memory, aligned to 8 bytes
    /* change processor to IRQ mode with interrupts disabled */
    mode = 0b11010010;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
    /* set banked stack pointer */
    asm("mov sp, %[ps]" : : [ps] "r"(stack));
    /* go back to SVC mode before executing subroutine return! */
    mode = 0b11010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(mode));
}
/*
* Turn on interrupts in the ARM processor
*/
void enable_A9_interrupts(void) {
    int status = 0b01010011;
    asm("msr cpsr, %[ps]" : : [ps] "r"(status));
}
/*
* Configure the Generic Interrupt Controller (GIC)
*/
void config_GIC(void) {
    config_interrupt (79, 1); // configure the PS2 interrupt (79)
    // Set Interrupt Priority Mask Register (ICCPMR). Enable interrupts of all
    // priorities
    *((int *) 0xFFFEC104) = 0xFFFF;
    // Set CPU Interface Control Register (ICCICR). Enable signaling of
    // interrupts
    *((int *) 0xFFFEC100) = 1;
    // Configure the Distributor Control Register (ICDDCR) to send pending
    // interrupts to CPUs
    *((int *) 0xFFFED000) = 1;
}
/*
* Configure Set Enable Registers (ICDISERn) and Interrupt Processor Target
* Registers (ICDIPTRn). The default (reset) values are used for other registers
* in the GIC.
*/
void config_interrupt(int N, int CPU_target) {
    int reg_offset, index, value, address;
    /* Configure the Interrupt Set-Enable Registers (ICDISERn).
    * reg_offset = (integer_div(N / 32) * 4
    * value = 1 << (N mod 32) */
    reg_offset = (N >> 3) & 0xFFFFFFFC;
    index = N & 0x1F;
    value = 0x1 << index;
    address = 0xFFFED100 + reg_offset;
    /* Now that we know the register address and value, set the appropriate bit */
    *(int *)address |= value;
    /* Configure the Interrupt Processor Targets Register (ICDIPTRn)
    * reg_offset = integer_div(N / 4) * 4
    * index = N mod 4 */
    reg_offset = (N & 0xFFFFFFFC);
    index = N & 0x3;
    address = 0xFFFED800 + reg_offset + index;
    /* Now that we know the register address and value, write to (only) the
    * appropriate byte */
    *(char *)address = (char)CPU_target;
}
/********************************************************************
* PS2 - Interrupt Service Routine
*
* This routine reads make and break codes when inputted. It then changes 
* global variables byte1-3 for them to be displayed on the hex display
*******************************************************************/
void PS2_ISR(void) {
    volatile int * PS2_ptr = (int *)PS2_BASE;
	
	 int PS2_data, RVALID;
	// char byte1 = 0, byte2 = 0, byte3 = 0;
	// PS/2 mouse needs to be reset (must be already plugged in)
	// *(PS2_ptr) = 0xFF; // reset
	//while (1) {
		PS2_data = *(PS2_ptr); // read the Data register in the PS/2 port
		RVALID = PS2_data & 0x8000; // extract the RVALID field
		if (RVALID) {
			/* shift the next data byte into the display */
			byte1 = byte2;
			byte2 = byte3;
			byte3 = PS2_data & 0xFF;
			//HEX_PS2(byte1, byte2, byte3);
            if ((byte2 == (char)0xAA) && (byte3 == (char)0x00))
			// mouse inserted; initialize sending of data
			*(PS2_ptr) = 0xF4;
		}
	//}
    return;
}

void HEX_PS2(char b1, char b2, char b3) {
	volatile int * HEX3_HEX0_ptr = (int *)HEX3_HEX0_BASE;
	volatile int * HEX5_HEX4_ptr = (int *)HEX5_HEX4_BASE;
	/* SEVEN_SEGMENT_DECODE_TABLE gives the on/off settings for all segments in
	* a single 7-seg display in the DE1-SoC Computer, for the hex digits 0 - F
	*/
	unsigned char seven_seg_decode_table[] = {
	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
	0x7F, 0x67, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71};
	unsigned char hex_segs[] = {0, 0, 0, 0, 0, 0, 0, 0};
	unsigned int shift_buffer, nibble;
	unsigned char code;
	int i;
	shift_buffer = (b1 << 16) | (b2 << 8) | b3;
	for (i = 0; i < 6; ++i) {
		nibble = shift_buffer & 0x0000000F; // character is in rightmost nibble
		code = seven_seg_decode_table[nibble];
		hex_segs[i] = code;
		shift_buffer = shift_buffer >> 4;
	}
	/* drive the hex displays */
	*(HEX3_HEX0_ptr) = *(int *)(hex_segs);
	*(HEX5_HEX4_ptr) = *(int *)(hex_segs + 4);
}

void changeBoxPosition(char b2, char b3) {
    int temp_x = 0;
	int temp_y = 0;
    if (xRED+dxRED < 320 || xRED+dxRED > 0) {
			xRED += dxRED;
    }
	if (yRED+dyRED < 240 || yRED+dyRED > 0) {
			yRED += dyRED;
	}
    if (xBLUE+dxBLUE < 320 || xBLUE+dxBLUE > 0) {
			xBLUE += dxBLUE;
    }
	if (yBLUE+dyBLUE < 240 || yBLUE+dyBLUE > 0) {
			yBLUE += dyBLUE;
	}
    
    prevXPositionsRED[prev] = xRED;
    prevYPositionsRED[prev] = yRED;
    temp_x = prevXPositionsRED[(prev + 1)%3];
    temp_y = prevYPositionsRED[(prev + 1)%3];
    draw_box(temp_x, temp_y, 0x0);

    prevXPositionsBLUE[prev] = xBLUE;
    prevYPositionsBLUE[prev] = yBLUE;
    temp_x = prevXPositionsBLUE[(prev + 1)%3];
    temp_y = prevYPositionsBLUE[(prev + 1)%3];
    draw_box(temp_x, temp_y, 0x0);


    if (prev < 2) {
        prev += 1;
    }
    else {
        prev = 0;
    }
    
    draw_box(xRED, yRED, RED);
    draw_box(xBLUE, yBLUE, CYAN);
    if (jumpRED) {
        if (countJumpFrameRED < 12 && dyRED == -2) {
            countJumpFrameRED += 1;
        }
        else {
            dyRED = 2;
            if (countJumpFrameRED < 0) {
                dyRED = 0;
                jumpRED = false;
            }
            countJumpFrameRED -=1;
        }
    }

    if (jumpBLUE) {
        if (countJumpFrameBLUE < 12 && dyBLUE == -2) {
            countJumpFrameBLUE += 1;
        }
        else {
            dyBLUE = 2;
            if (countJumpFrameBLUE < 0) {
                dyBLUE = 0;
                jumpBLUE = false;
            }
            countJumpFrameBLUE -=1;
        }
    }

    
    if (b2 == 0xF0) {
        if (b3 == 0x23 || b3 == 0x1C) {
            dxRED = 0;
        }
        if (b3 == 0x74 || b3 == 0x6B) {
            dxBLUE = 0;
        }
    }
    else {
        if (b3 == 0x23) {
            dxRED = 1;
        }
        else if (b3 == 0x1C) {
            dxRED = -1;
        }
        else if (b3 == 0x1D) {
            if (!jumpRED) {
                dyRED = -2;
                countJumpFrameRED = 0;
                jumpRED = true;
            }
        }
        else if (b3 == 0x74) {
            dxBLUE = 1;
        }
        else if (b3 == 0x6B) {
            dxBLUE = -1;
        }
        else if (b3 == 0x75) {
            if (!jumpBLUE) {
                dyBLUE = -2;
                countJumpFrameBLUE = 0;
                jumpBLUE = true;
            }
        }
    }
    return;
}

void plot_pixel(int x, int y, short int line_color)
{
	*(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = line_color;
}
	
void draw_box(int x, int y, int box_color)
{
	for(int i = 0; i < 8; i++) {
		for(int j = 0; j < 8; j++)
			plot_pixel(x+i, y+j, box_color);
	}
}

void wait_for_vsync()
{
	volatile int * pixel_ctrl_ptr = (int *)0xFF203020;
	register int status;
	
	*pixel_ctrl_ptr = 1;
	
	status = *(pixel_ctrl_ptr + 3);
	while ((status & 0x01) != 0){
		status = *(pixel_ctrl_ptr + 3);
	}
}

void clear_screen()
{
	for (int x = 0; x < 320; x++){
		for (int y = 0; y < 240; y++){
			*(short int *)(pixel_buffer_start + (y << 10) + (x << 1)) = 0x0;
		}
	}
}