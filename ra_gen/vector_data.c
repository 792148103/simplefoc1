/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = iic_master_rxi_isr, /* IIC0 RXI (Receive data full) */
            [1] = iic_master_txi_isr, /* IIC0 TXI (Transmit data empty) */
            [2] = iic_master_tei_isr, /* IIC0 TEI (Transmit end) */
            [3] = iic_master_eri_isr, /* IIC0 ERI (Transfer error) */
            [4] = can_error_isr, /* CAN0 ERROR (Error interrupt) */
            [5] = can_rx_isr, /* CAN0 MAILBOX RX (Reception complete interrupt) */
            [6] = can_tx_isr, /* CAN0 MAILBOX TX (Transmission complete interrupt) */
            [7] = can_rx_isr, /* CAN0 FIFO RX (Receive FIFO interrupt) */
            [8] = can_tx_isr, /* CAN0 FIFO TX (Transmit FIFO interrupt) */
            [9] = sci_i2c_txi_isr, /* SCI4 TXI (Transmit data empty) */
            [10] = sci_i2c_tei_isr, /* SCI4 TEI (Transmit end) */
            [11] = sci_uart_rxi_isr, /* SCI9 RXI (Receive data full) */
            [12] = sci_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [13] = sci_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [14] = sci_uart_eri_isr, /* SCI9 ERI (Receive error) */
            [15] = r_icu_isr, /* ICU IRQ6 (External pin interrupt 6) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_IIC0_RXI,GROUP0), /* IIC0 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_IIC0_TXI,GROUP1), /* IIC0 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_IIC0_TEI,GROUP2), /* IIC0 TEI (Transmit end) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_IIC0_ERI,GROUP3), /* IIC0 ERI (Transfer error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_CAN0_ERROR,GROUP4), /* CAN0 ERROR (Error interrupt) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_CAN0_MAILBOX_RX,GROUP5), /* CAN0 MAILBOX RX (Reception complete interrupt) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_CAN0_MAILBOX_TX,GROUP6), /* CAN0 MAILBOX TX (Transmission complete interrupt) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_CAN0_FIFO_RX,GROUP7), /* CAN0 FIFO RX (Receive FIFO interrupt) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_CAN0_FIFO_TX,GROUP0), /* CAN0 FIFO TX (Transmit FIFO interrupt) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TXI,GROUP1), /* SCI4 TXI (Transmit data empty) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_SCI4_TEI,GROUP2), /* SCI4 TEI (Transmit end) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_SCI9_RXI,GROUP3), /* SCI9 RXI (Receive data full) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TXI,GROUP4), /* SCI9 TXI (Transmit data empty) */
            [13] = BSP_PRV_VECT_ENUM(EVENT_SCI9_TEI,GROUP5), /* SCI9 TEI (Transmit end) */
            [14] = BSP_PRV_VECT_ENUM(EVENT_SCI9_ERI,GROUP6), /* SCI9 ERI (Receive error) */
            [15] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ6,GROUP7), /* ICU IRQ6 (External pin interrupt 6) */
        };
        #endif
        #endif
