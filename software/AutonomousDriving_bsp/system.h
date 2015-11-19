/*
 * system.h - SOPC Builder system and BSP software package information
 *
 * Machine generated for CPU 'cpu' in SOPC Builder design 'DE0_Nano_SOPC'
 * SOPC Builder design path: C:/Users/Mexx/AutonomousDriving/DE0_Nano_SOPC.sopcinfo
 *
 * Generated: Thu Oct 22 13:18:54 CEST 2015
 */

/*
 * DO NOT MODIFY THIS FILE
 *
 * Changing this file will have subtle consequences
 * which will almost certainly lead to a nonfunctioning
 * system. If you do modify this file, be aware that your
 * changes will be overwritten and lost when this file
 * is generated again.
 *
 * DO NOT MODIFY THIS FILE
 */

/*
 * License Agreement
 *
 * Copyright (c) 2008
 * Altera Corporation, San Jose, California, USA.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * This agreement shall be governed in all respects by the laws of the State
 * of California and by the laws of the United States of America.
 */

#ifndef __SYSTEM_H_
#define __SYSTEM_H_

/* Include definitions from linker script generator */
#include "linker.h"


/*
 * CPU configuration
 *
 */

#define ALT_CPU_ARCHITECTURE "altera_nios2"
#define ALT_CPU_BIG_ENDIAN 0
#define ALT_CPU_BREAK_ADDR 0x4000820
#define ALT_CPU_CPU_FREQ 100000000u
#define ALT_CPU_CPU_ID_SIZE 1
#define ALT_CPU_CPU_ID_VALUE 0x1
#define ALT_CPU_CPU_IMPLEMENTATION "fast"
#define ALT_CPU_DATA_ADDR_WIDTH 0x1b
#define ALT_CPU_DCACHE_LINE_SIZE 32
#define ALT_CPU_DCACHE_LINE_SIZE_LOG2 5
#define ALT_CPU_DCACHE_SIZE 1024
#define ALT_CPU_EXCEPTION_ADDR 0x20
#define ALT_CPU_FLUSHDA_SUPPORTED
#define ALT_CPU_FREQ 100000000
#define ALT_CPU_HARDWARE_DIVIDE_PRESENT 0
#define ALT_CPU_HARDWARE_MULTIPLY_PRESENT 1
#define ALT_CPU_HARDWARE_MULX_PRESENT 0
#define ALT_CPU_HAS_DEBUG_CORE 1
#define ALT_CPU_HAS_DEBUG_STUB
#define ALT_CPU_HAS_JMPI_INSTRUCTION
#define ALT_CPU_ICACHE_LINE_SIZE 32
#define ALT_CPU_ICACHE_LINE_SIZE_LOG2 5
#define ALT_CPU_ICACHE_SIZE 1024
#define ALT_CPU_INITDA_SUPPORTED
#define ALT_CPU_INST_ADDR_WIDTH 0x1b
#define ALT_CPU_NAME "cpu"
#define ALT_CPU_NUM_OF_SHADOW_REG_SETS 0
#define ALT_CPU_RESET_ADDR 0x0


/*
 * CPU configuration (with legacy prefix - don't use these anymore)
 *
 */

#define NIOS2_BIG_ENDIAN 0
#define NIOS2_BREAK_ADDR 0x4000820
#define NIOS2_CPU_FREQ 100000000u
#define NIOS2_CPU_ID_SIZE 1
#define NIOS2_CPU_ID_VALUE 0x1
#define NIOS2_CPU_IMPLEMENTATION "fast"
#define NIOS2_DATA_ADDR_WIDTH 0x1b
#define NIOS2_DCACHE_LINE_SIZE 32
#define NIOS2_DCACHE_LINE_SIZE_LOG2 5
#define NIOS2_DCACHE_SIZE 1024
#define NIOS2_EXCEPTION_ADDR 0x20
#define NIOS2_FLUSHDA_SUPPORTED
#define NIOS2_HARDWARE_DIVIDE_PRESENT 0
#define NIOS2_HARDWARE_MULTIPLY_PRESENT 1
#define NIOS2_HARDWARE_MULX_PRESENT 0
#define NIOS2_HAS_DEBUG_CORE 1
#define NIOS2_HAS_DEBUG_STUB
#define NIOS2_HAS_JMPI_INSTRUCTION
#define NIOS2_ICACHE_LINE_SIZE 32
#define NIOS2_ICACHE_LINE_SIZE_LOG2 5
#define NIOS2_ICACHE_SIZE 1024
#define NIOS2_INITDA_SUPPORTED
#define NIOS2_INST_ADDR_WIDTH 0x1b
#define NIOS2_NUM_OF_SHADOW_REG_SETS 0
#define NIOS2_RESET_ADDR 0x0


/*
 * Define for each module class mastered by the CPU
 *
 */

#define __ALTERA_AVALON_EPCS_FLASH_CONTROLLER
#define __ALTERA_AVALON_JTAG_UART
#define __ALTERA_AVALON_NEW_SDRAM_CONTROLLER
#define __ALTERA_AVALON_ONCHIP_MEMORY2
#define __ALTERA_AVALON_TIMER
#define __ALTERA_NIOS2
#define __ALTPLL
#define __HC_SR04
#define __MOTOR_CONTROL
#define __MP6050


/*
 * System configuration
 *
 */

#define ALT_DEVICE_FAMILY "CYCLONEIVE"
#define ALT_ENHANCED_INTERRUPT_API_PRESENT
#define ALT_IRQ_BASE NULL
#define ALT_LOG_PORT "/dev/null"
#define ALT_LOG_PORT_BASE 0x0
#define ALT_LOG_PORT_DEV null
#define ALT_LOG_PORT_TYPE ""
#define ALT_NUM_EXTERNAL_INTERRUPT_CONTROLLERS 0
#define ALT_NUM_INTERNAL_INTERRUPT_CONTROLLERS 1
#define ALT_NUM_INTERRUPT_CONTROLLERS 1
#define ALT_STDERR "/dev/jtag_uart"
#define ALT_STDERR_BASE 0x40010a0
#define ALT_STDERR_DEV jtag_uart
#define ALT_STDERR_IS_JTAG_UART
#define ALT_STDERR_PRESENT
#define ALT_STDERR_TYPE "altera_avalon_jtag_uart"
#define ALT_STDIN "/dev/jtag_uart"
#define ALT_STDIN_BASE 0x40010a0
#define ALT_STDIN_DEV jtag_uart
#define ALT_STDIN_IS_JTAG_UART
#define ALT_STDIN_PRESENT
#define ALT_STDIN_TYPE "altera_avalon_jtag_uart"
#define ALT_STDOUT "/dev/jtag_uart"
#define ALT_STDOUT_BASE 0x40010a0
#define ALT_STDOUT_DEV jtag_uart
#define ALT_STDOUT_IS_JTAG_UART
#define ALT_STDOUT_PRESENT
#define ALT_STDOUT_TYPE "altera_avalon_jtag_uart"
#define ALT_SYSTEM_NAME "DE0_Nano_SOPC"


/*
 * altpll_sys configuration
 *
 */

#define ALTPLL_SYS_BASE 0x4001090
#define ALTPLL_SYS_IRQ -1
#define ALTPLL_SYS_IRQ_INTERRUPT_CONTROLLER_ID -1
#define ALTPLL_SYS_NAME "/dev/altpll_sys"
#define ALTPLL_SYS_SPAN 16
#define ALTPLL_SYS_TYPE "altpll"
#define ALT_MODULE_CLASS_altpll_sys altpll


/*
 * epcs configuration
 *
 */

#define ALT_MODULE_CLASS_epcs altera_avalon_epcs_flash_controller
#define EPCS_BASE 0x2000000
#define EPCS_IRQ 4
#define EPCS_IRQ_INTERRUPT_CONTROLLER_ID 0
#define EPCS_NAME "/dev/epcs"
#define EPCS_REGISTER_OFFSET 1024
#define EPCS_SPAN 2048
#define EPCS_TYPE "altera_avalon_epcs_flash_controller"


/*
 * hal configuration
 *
 */

#define ALT_MAX_FD 32
#define ALT_SYS_CLK TIMER
#define ALT_TIMESTAMP_CLK none


/*
 * hc_sr04_0 configuration
 *
 */

#define ALT_MODULE_CLASS_hc_sr04_0 hc_sr04
#define HC_SR04_0_BASE 0x4001000
#define HC_SR04_0_IRQ -1
#define HC_SR04_0_IRQ_INTERRUPT_CONTROLLER_ID -1
#define HC_SR04_0_NAME "/dev/hc_sr04_0"
#define HC_SR04_0_SPAN 64
#define HC_SR04_0_TYPE "hc_sr04"


/*
 * jtag_uart configuration
 *
 */

#define ALT_MODULE_CLASS_jtag_uart altera_avalon_jtag_uart
#define JTAG_UART_BASE 0x40010a0
#define JTAG_UART_IRQ 0
#define JTAG_UART_IRQ_INTERRUPT_CONTROLLER_ID 0
#define JTAG_UART_NAME "/dev/jtag_uart"
#define JTAG_UART_READ_DEPTH 64
#define JTAG_UART_READ_THRESHOLD 8
#define JTAG_UART_SPAN 8
#define JTAG_UART_TYPE "altera_avalon_jtag_uart"
#define JTAG_UART_WRITE_DEPTH 64
#define JTAG_UART_WRITE_THRESHOLD 8


/*
 * manuer_queue configuration
 *
 */

#define ALT_MODULE_CLASS_manuer_queue altera_avalon_onchip_memory2
#define MANUER_QUEUE_ALLOW_IN_SYSTEM_MEMORY_CONTENT_EDITOR 0
#define MANUER_QUEUE_ALLOW_MRAM_SIM_CONTENTS_ONLY_FILE 0
#define MANUER_QUEUE_BASE 0x2004000
#define MANUER_QUEUE_CONTENTS_INFO ""
#define MANUER_QUEUE_DUAL_PORT 0
#define MANUER_QUEUE_GUI_RAM_BLOCK_TYPE "Automatic"
#define MANUER_QUEUE_INIT_CONTENTS_FILE "manuer_queue"
#define MANUER_QUEUE_INIT_MEM_CONTENT 0
#define MANUER_QUEUE_INSTANCE_ID "NONE"
#define MANUER_QUEUE_IRQ -1
#define MANUER_QUEUE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MANUER_QUEUE_NAME "/dev/manuer_queue"
#define MANUER_QUEUE_NON_DEFAULT_INIT_FILE_ENABLED 0
#define MANUER_QUEUE_RAM_BLOCK_TYPE "Auto"
#define MANUER_QUEUE_READ_DURING_WRITE_MODE "DONT_CARE"
#define MANUER_QUEUE_SINGLE_CLOCK_OP 0
#define MANUER_QUEUE_SIZE_MULTIPLE 1
#define MANUER_QUEUE_SIZE_VALUE 1024u
#define MANUER_QUEUE_SPAN 1024
#define MANUER_QUEUE_TYPE "altera_avalon_onchip_memory2"
#define MANUER_QUEUE_WRITABLE 1


/*
 * motor_control_0 configuration
 *
 */

#define ALT_MODULE_CLASS_motor_control_0 motor_control
#define MOTOR_CONTROL_0_BASE 0x4001040
#define MOTOR_CONTROL_0_IRQ -1
#define MOTOR_CONTROL_0_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MOTOR_CONTROL_0_NAME "/dev/motor_control_0"
#define MOTOR_CONTROL_0_SPAN 64
#define MOTOR_CONTROL_0_TYPE "motor_control"


/*
 * mp6050_0 configuration
 *
 */

#define ALT_MODULE_CLASS_mp6050_0 mp6050
#define MP6050_0_BASE 0x4001080
#define MP6050_0_IRQ -1
#define MP6050_0_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MP6050_0_NAME "/dev/mp6050_0"
#define MP6050_0_SPAN 16
#define MP6050_0_TYPE "mp6050"


/*
 * sdram configuration
 *
 */

#define ALT_MODULE_CLASS_sdram altera_avalon_new_sdram_controller
#define SDRAM_BASE 0x0
#define SDRAM_CAS_LATENCY 3
#define SDRAM_CONTENTS_INFO ""
#define SDRAM_INIT_NOP_DELAY 0.0
#define SDRAM_INIT_REFRESH_COMMANDS 2
#define SDRAM_IRQ -1
#define SDRAM_IRQ_INTERRUPT_CONTROLLER_ID -1
#define SDRAM_IS_INITIALIZED 1
#define SDRAM_NAME "/dev/sdram"
#define SDRAM_POWERUP_DELAY 100.0
#define SDRAM_REFRESH_PERIOD 15.625
#define SDRAM_REGISTER_DATA_IN 1
#define SDRAM_SDRAM_ADDR_WIDTH 0x18
#define SDRAM_SDRAM_BANK_WIDTH 2
#define SDRAM_SDRAM_COL_WIDTH 9
#define SDRAM_SDRAM_DATA_WIDTH 16
#define SDRAM_SDRAM_NUM_BANKS 4
#define SDRAM_SDRAM_NUM_CHIPSELECTS 1
#define SDRAM_SDRAM_ROW_WIDTH 13
#define SDRAM_SHARED_DATA 0
#define SDRAM_SIM_MODEL_BASE 1
#define SDRAM_SPAN 33554432
#define SDRAM_STARVATION_INDICATOR 0
#define SDRAM_TRISTATE_BRIDGE_SLAVE ""
#define SDRAM_TYPE "altera_avalon_new_sdram_controller"
#define SDRAM_T_AC 5.5
#define SDRAM_T_MRD 3
#define SDRAM_T_RCD 20.0
#define SDRAM_T_RFC 70.0
#define SDRAM_T_RP 20.0
#define SDRAM_T_WR 14.0


/*
 * state_cmd_memory configuration
 *
 */

#define ALT_MODULE_CLASS_state_cmd_memory altera_avalon_onchip_memory2
#define STATE_CMD_MEMORY_ALLOW_IN_SYSTEM_MEMORY_CONTENT_EDITOR 0
#define STATE_CMD_MEMORY_ALLOW_MRAM_SIM_CONTENTS_ONLY_FILE 0
#define STATE_CMD_MEMORY_BASE 0x2005000
#define STATE_CMD_MEMORY_CONTENTS_INFO ""
#define STATE_CMD_MEMORY_DUAL_PORT 0
#define STATE_CMD_MEMORY_GUI_RAM_BLOCK_TYPE "Automatic"
#define STATE_CMD_MEMORY_INIT_CONTENTS_FILE "state_cmd_memory"
#define STATE_CMD_MEMORY_INIT_MEM_CONTENT 0
#define STATE_CMD_MEMORY_INSTANCE_ID "NONE"
#define STATE_CMD_MEMORY_IRQ -1
#define STATE_CMD_MEMORY_IRQ_INTERRUPT_CONTROLLER_ID -1
#define STATE_CMD_MEMORY_NAME "/dev/state_cmd_memory"
#define STATE_CMD_MEMORY_NON_DEFAULT_INIT_FILE_ENABLED 0
#define STATE_CMD_MEMORY_RAM_BLOCK_TYPE "Auto"
#define STATE_CMD_MEMORY_READ_DURING_WRITE_MODE "DONT_CARE"
#define STATE_CMD_MEMORY_SINGLE_CLOCK_OP 0
#define STATE_CMD_MEMORY_SIZE_MULTIPLE 1
#define STATE_CMD_MEMORY_SIZE_VALUE 1024u
#define STATE_CMD_MEMORY_SPAN 1024
#define STATE_CMD_MEMORY_TYPE "altera_avalon_onchip_memory2"
#define STATE_CMD_MEMORY_WRITABLE 1


/*
 * timer configuration
 *
 */

#define ALT_MODULE_CLASS_timer altera_avalon_timer
#define TIMER_ALWAYS_RUN 0
#define TIMER_BASE 0x3000000
#define TIMER_COUNTER_SIZE 32
#define TIMER_FIXED_PERIOD 0
#define TIMER_FREQ 10000000u
#define TIMER_IRQ 5
#define TIMER_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_LOAD_VALUE 9999ull
#define TIMER_MULT 0.0010
#define TIMER_NAME "/dev/timer"
#define TIMER_PERIOD 1
#define TIMER_PERIOD_UNITS "ms"
#define TIMER_RESET_OUTPUT 0
#define TIMER_SNAPSHOT 1
#define TIMER_SPAN 32
#define TIMER_TICKS_PER_SEC 1000u
#define TIMER_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_TYPE "altera_avalon_timer"

#endif /* __SYSTEM_H_ */
