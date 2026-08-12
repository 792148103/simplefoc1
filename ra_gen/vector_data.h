/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (16)
#endif
/* ISR prototypes */
void iic_master_rxi_isr(void);
void iic_master_txi_isr(void);
void iic_master_tei_isr(void);
void iic_master_eri_isr(void);
void can_error_isr(void);
void can_rx_isr(void);
void can_tx_isr(void);
void sci_i2c_txi_isr(void);
void sci_i2c_tei_isr(void);
void sci_uart_rxi_isr(void);
void sci_uart_txi_isr(void);
void sci_uart_tei_isr(void);
void sci_uart_eri_isr(void);
void r_icu_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_IIC0_RXI ((IRQn_Type) 0) /* IIC0 RXI (Receive data full) */
#define IIC0_RXI_IRQn          ((IRQn_Type) 0) /* IIC0 RXI (Receive data full) */
#define VECTOR_NUMBER_IIC0_TXI ((IRQn_Type) 1) /* IIC0 TXI (Transmit data empty) */
#define IIC0_TXI_IRQn          ((IRQn_Type) 1) /* IIC0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_IIC0_TEI ((IRQn_Type) 2) /* IIC0 TEI (Transmit end) */
#define IIC0_TEI_IRQn          ((IRQn_Type) 2) /* IIC0 TEI (Transmit end) */
#define VECTOR_NUMBER_IIC0_ERI ((IRQn_Type) 3) /* IIC0 ERI (Transfer error) */
#define IIC0_ERI_IRQn          ((IRQn_Type) 3) /* IIC0 ERI (Transfer error) */
#define VECTOR_NUMBER_CAN0_ERROR ((IRQn_Type) 4) /* CAN0 ERROR (Error interrupt) */
#define CAN0_ERROR_IRQn          ((IRQn_Type) 4) /* CAN0 ERROR (Error interrupt) */
#define VECTOR_NUMBER_CAN0_MAILBOX_RX ((IRQn_Type) 5) /* CAN0 MAILBOX RX (Reception complete interrupt) */
#define CAN0_MAILBOX_RX_IRQn          ((IRQn_Type) 5) /* CAN0 MAILBOX RX (Reception complete interrupt) */
#define VECTOR_NUMBER_CAN0_MAILBOX_TX ((IRQn_Type) 6) /* CAN0 MAILBOX TX (Transmission complete interrupt) */
#define CAN0_MAILBOX_TX_IRQn          ((IRQn_Type) 6) /* CAN0 MAILBOX TX (Transmission complete interrupt) */
#define VECTOR_NUMBER_CAN0_FIFO_RX ((IRQn_Type) 7) /* CAN0 FIFO RX (Receive FIFO interrupt) */
#define CAN0_FIFO_RX_IRQn          ((IRQn_Type) 7) /* CAN0 FIFO RX (Receive FIFO interrupt) */
#define VECTOR_NUMBER_CAN0_FIFO_TX ((IRQn_Type) 8) /* CAN0 FIFO TX (Transmit FIFO interrupt) */
#define CAN0_FIFO_TX_IRQn          ((IRQn_Type) 8) /* CAN0 FIFO TX (Transmit FIFO interrupt) */
#define VECTOR_NUMBER_SCI4_TXI ((IRQn_Type) 9) /* SCI4 TXI (Transmit data empty) */
#define SCI4_TXI_IRQn          ((IRQn_Type) 9) /* SCI4 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI4_TEI ((IRQn_Type) 10) /* SCI4 TEI (Transmit end) */
#define SCI4_TEI_IRQn          ((IRQn_Type) 10) /* SCI4 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI9_RXI ((IRQn_Type) 11) /* SCI9 RXI (Receive data full) */
#define SCI9_RXI_IRQn          ((IRQn_Type) 11) /* SCI9 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI9_TXI ((IRQn_Type) 12) /* SCI9 TXI (Transmit data empty) */
#define SCI9_TXI_IRQn          ((IRQn_Type) 12) /* SCI9 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI9_TEI ((IRQn_Type) 13) /* SCI9 TEI (Transmit end) */
#define SCI9_TEI_IRQn          ((IRQn_Type) 13) /* SCI9 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI9_ERI ((IRQn_Type) 14) /* SCI9 ERI (Receive error) */
#define SCI9_ERI_IRQn          ((IRQn_Type) 14) /* SCI9 ERI (Receive error) */
#define VECTOR_NUMBER_ICU_IRQ6 ((IRQn_Type) 15) /* ICU IRQ6 (External pin interrupt 6) */
#define ICU_IRQ6_IRQn          ((IRQn_Type) 15) /* ICU IRQ6 (External pin interrupt 6) */
/* The number of entries required for the ICU vector table. */
#define BSP_ICU_VECTOR_NUM_ENTRIES (16)

#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
