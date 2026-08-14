#include "microros_transport.h"

#include "stm32h743_regs.h"
#include "usart.h"
#include "rmw_microros/rmw_microros.h"

#define MICROROS_UART USART1

/* Ring buffer nap boi USART1_IRQHandler() (ngat RXNE, 1 byte/lan) - Transport_Read()
 * chi rut ra, KHONG cho. Port nguyen ven tu OUT_SAVE/testSTM/app/src/microros_transport.c (da
 * xac nhan on dinh tren board that) theo dung sample chinh thuc cua micro-ROS cho STM32
 * (micro_ros_stm32cubemx_utils/extra_sources/microros_transports/it_transport.c) - ban
 * blocking-cho-timeout_ms ban dau la nguyen nhan gay ket noi khong on dinh (xem
 * OUT_SAVE/testSTM/README.md "Trang thai da kiem thu"): read() blocking het nguyen timeout_ms
 * trong 1 lan goi co the chan mat co hoi cho tang XRCE ben tren xen ke doc/ghi kip luc.
 *
 * head chi do task (Transport_Read) ghi, tail chi do ISR ghi -> khong can khoa/mutex,
 * an toan kieu single-producer-single-consumer lock-free (khong FreeRTOS o day, khac
 * ban goc OUT_SAVE/testSTM - nhung ISR + ring buffer khong phu thuoc RTOS ngay tu dau). */
#define RX_RING_SIZE 2048U
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile size_t rx_head = 0U;
static volatile size_t rx_tail = 0U;

/* Ring buffer GUI o chieu GUI (TX) - doi xung voi rx_ring o tren nhung nguoc vai tro:
 * Transport_Write() (task) chi GHI (day), USART1_IRQHandler() (ngat TXE) chi RUT ra va
 * day vao TDR. Truoc day Transport_Write() goi USART_SendByte() (usart.c) - polling
 * cho TXE tung byte, CHAN CPU ~10.8us/byte o 921600 baud (vd tin /joint_fb ~65 byte ->
 * ~700us CHAN CUNG moi lan publish, o 200Hz chiem ~14% chu ky 5ms) - danh doi lay CPU
 * de lam viec khac (CAN, FSM) trong luc phan cung tu dich chuyen bit. TX_RING_SIZE=2048
 * khop RX_RING_SIZE, du du cho ca chuc tin lien tiep (tin lon nhat hien tai ~70-100
 * byte). KHONG doi baud rate/tin hieu tren day - CH340 phia may tinh thay y het,
 * chi khac o cach CPU MCU day byte vao thanh ghi UART. */
#define TX_RING_SIZE 2048U
static volatile uint8_t tx_ring[TX_RING_SIZE];
static volatile size_t tx_head = 0U;   /* ISR (TXE) doc tu day */
static volatile size_t tx_tail = 0U;   /* Transport_Write() ghi vao day */

void USART1_IRQHandler(void)
{
    if ((MICROROS_UART->ISR & USART_ISR_RXNE_Msk) != 0U)
    {
        const uint8_t byte = (uint8_t)MICROROS_UART->RDR;   /* doc RDR tu xoa co RXNE */
        const size_t next_tail = (rx_tail + 1U) % RX_RING_SIZE;
        if (next_tail != rx_head)   /* con cho - bo byte neu day thay vi ghi de */
        {
            rx_ring[rx_tail] = byte;
            rx_tail = next_tail;
        }
    }

    if ((MICROROS_UART->CR1 & USART_CR1_TXEIE_Msk) != 0U &&
        (MICROROS_UART->ISR & USART_ISR_TXE_Msk) != 0U)
    {
        if (tx_head != tx_tail)
        {
            MICROROS_UART->TDR = tx_ring[tx_head];   /* ghi TDR tu xoa co TXE */
            tx_head = (tx_head + 1U) % TX_RING_SIZE;
        }
        else
        {
            /* Het du lieu de gui - TAT TXEIE ngay, neu khong co TXE luon = 1 luc
             * ranh se khien ngat nay chay lien tuc vo ich (busy-loop trong ISR). */
            MICROROS_UART->CR1 &= ~USART_CR1_TXEIE_Msk;
        }
    }
}

static bool Transport_Open(struct uxrCustomTransport *transport)
{
    (void)transport;
    NVIC_IPR[USART1_IRQn] = (uint8_t)(5U << 4);
    NVIC_ISER1 = (1UL << (USART1_IRQn - 32));   /* IRQ 37 nam trong ISER1 (32..63) */
    MICROROS_UART->CR1 |= USART_CR1_RXNEIE_Msk;
    return true;
}

static bool Transport_Close(struct uxrCustomTransport *transport)
{
    (void)transport;
    MICROROS_UART->CR1 &= ~(USART_CR1_RXNEIE_Msk | USART_CR1_TXEIE_Msk);
    return true;
}

static size_t tx_ring_enqueue(const uint8_t *buffer, size_t length)
{
    /* Day byte vao tx_ring - chi CHAN neu ring THAT SU day (khong con cho, truong hop
     * gan nhu khong xay ra voi TX_RING_SIZE=2048 va tin lon nhat ~100 byte), khong con
     * chan CHAC CHAN moi byte nhu USART_SendByte() (polling) truoc day. Vong cho o day
     * (neu co) van an toan vi ISR van dang tu rut ring o nen chay song song, khong bao
     * gio tu treo. */
    for (size_t i = 0U; i < length; i++)
    {
        const size_t next_tail = (tx_tail + 1U) % TX_RING_SIZE;
        while (next_tail == tx_head)
        {
            /* ring day - doi ISR rut bot (hiem khi xay ra, xem comment TX_RING_SIZE) */
        }
        tx_ring[tx_tail] = buffer[i];
        tx_tail = next_tail;
    }
    /* Bat TXEIE de ISR bat dau/tiep tuc rut ring - an toan goi lai nhieu lan (da bat
     * roi thi ghi lai cung gia tri, khong co tac dung phu). */
    MICROROS_UART->CR1 |= USART_CR1_TXEIE_Msk;
    return length;
}

/* TAM THOI test gia thuyet: ring buffer TX (bat dong bo qua ISR) co the "an trom"
 * mot phan thoi gian cho phan hoi cua wait_session_status() (UXR_CONFIG_MIN_
 * SESSION_CONNECTION_INTERVAL=5000ms/lan, session.c) neu ISR bi tre drain vi ly do
 * nao do - thu vien khong doi xac nhan da gui xong (send_message() khong kiem tra
 * gia tri tra ve), chi goi write() roi lap tuc bat dau dem gio doi phan hoi. Quay
 * ve blocking polling (USART_SendByte, dung y het ban TRUOC khi toi uu toc do dem
 * nay) - CHAN CPU that su cho toi khi byte cuoi cung ROI KHOI TDR, dam bao byte
 * that su len day truoc khi ham nay tra ve. XOA macro nay + quay lai
 * tx_ring_enqueue() sau khi xac dinh xong day co phai nguyen nhan hay khong. */
#define TX_BLOCKING_POLLING_TEST 1

static size_t Transport_Write(struct uxrCustomTransport *transport, const uint8_t *buffer,
                               size_t length, uint8_t *error_code)
{
    (void)transport;
    (void)error_code;
#if TX_BLOCKING_POLLING_TEST
    for (size_t i = 0U; i < length; i++)
    {
        USART_SendByte(MICROROS_UART, buffer[i]);
    }
    return length;
#else
    return tx_ring_enqueue(buffer, length);
#endif
}

static size_t Transport_Read(struct uxrCustomTransport *transport, uint8_t *buffer,
                              size_t length, int timeout_ms, uint8_t *error_code)
{
    (void)transport;
    (void)error_code;
    (void)timeout_ms;   /* khong doi - tra ngay nhung gi dang co trong ring buffer, dung
                          * y het contract cua it_transport.c's cubemx_transport_read():
                          * tang goi (uxr_run_session_until_...) tu quan ly vong lap cho/
                          * het gio bang dong ho rieng, khong dua vao read() blocking ho. */
    size_t received = 0U;
    while (received < length && rx_head != rx_tail)
    {
        buffer[received] = rx_ring[rx_head];
        rx_head = (rx_head + 1U) % RX_RING_SIZE;
        received++;
    }
    return received;
}

void MicroRosTransport_DebugRxIsrOpen(void)
{
    (void)Transport_Open(NULL);
}

size_t MicroRosTransport_DebugRxIsrRead(uint8_t *buffer, size_t maxlen)
{
    return Transport_Read(NULL, buffer, maxlen, 0, NULL);
}

void MicroRos_RegisterTransport(void)
{
    (void)rmw_uros_set_custom_transport(
        MICROROS_TRANSPORTS_FRAMING_MODE,
        NULL,
        Transport_Open,
        Transport_Close,
        Transport_Write,
        Transport_Read);
}
