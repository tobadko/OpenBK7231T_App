#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../quicktick.h"
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "errno.h"
#include <lwip/sockets.h>
#include "drv_uart.h"
#include "../hal/hal_pins.h"

#if ENABLE_DRIVER_UART_TCP

#define DEFAULT_BUF_SIZE		512
#define DEFAULT_UART_TCP_PORT	8888
#define INVALID_SOCK			-1
#ifndef UTCP_DEBUG
#define UTCP_DEBUG				0
#endif
#if UTCP_DEBUG
#define STACK_SIZE				8192
#else
#define STACK_SIZE				3584
#endif
extern int Main_HasWiFiConnected();
static uint16_t buf_size = DEFAULT_BUF_SIZE;
static int g_conn_channel = -1;
static int g_baudRate = 115200;
static int listen_sock = INVALID_SOCK;
static int client_sock = INVALID_SOCK;
static xTaskHandle g_start_thread = NULL;
static xTaskHandle g_trx_thread = NULL;
static xTaskHandle g_rx_thread = NULL;
static xTaskHandle g_tx_thread = NULL;
static bool rx_closed, tx_closed;
static byte* g_utcpBuf = 0;

// >>> НАШ КОД: Настройки авто-захвата бутлоадера CB2S / BK7231N <<<
#define BK_CEN_PIN          8   // Номер GPIO программатора, подключенного к CEN (P8)
static int g_bk_synced = 0;    // 0 = ловим бутлоадер, 1 = чип пойман (прозрачный режим)
static uint32_t g_bk_last_reset = 0; // Время последнего сброса (мс)
static int g_magic_match = 0; // Для скользящего окна преамбулы (0x01, 0xE0, 0xFC)
static int g_ack_match = 0;
static int g_last_pulse_ms = 0;
static int g_last_wait_ms = 0;
static int g_reset_attempt = 0; // counter for dynamic sweeping CEN timing   // Для скользящего окна ответа (0x04, 0x0E)

void Start_UART_TCP(void* arg);
void UART_TCP_Deinit();

static void UTCP_TX_Thd(void* param)
{
	int client_fd = *(int*)param;

	while(1)
	{
		int ret = 0;
		int delay = 0;
		memset(g_utcpBuf, 0, buf_size);
		int len = UART_GetDataSize();

		if(client_fd == INVALID_SOCK) goto exit;
		if(g_bk_synced)
		{
			while(len < buf_size && delay < 10)
			{
				rtos_delay_milliseconds(1);
				len = UART_GetDataSize();
				delay++;
			}
		}

		if(len > 0)
		{
			for(int i = 0; i < len; i++)
			{
				g_utcpBuf[i] = UART_GetByte(i);
			}
			UART_ConsumeBytes(len);

			// >>> НАШ КОД: Ловим подтверждение (ACK) от бутлоадера CB2S <<<
			if(!g_bk_synced)
			{
				for(int i = 0; i < len; i++)
				{
					uint8_t b = g_utcpBuf[i];
					if(b == 0x04 && g_ack_match == 0) g_ack_match = 1;
					else if(b == 0x0E && g_ack_match == 1)
					{
						g_ack_match = 0;
						g_bk_synced = 1;
						ADDLOG_INFO(LOG_FEATURE_DRV, "CB2S: Bootloader ACK confirmed (04 0E)! SUCCESS on attempt #%d (pulse=%d ms, wait=%d ms)! Bypass mode ON.", g_reset_attempt, g_last_pulse_ms, g_last_wait_ms);
						break;
					}
					else
					{
						g_ack_match = (b == 0x04) ? 1 : 0;
					}
				}
			}
#if UTCP_DEBUG
			char data[len * 2];
			char* p = data;
			for(int i = 0; i < len; i++)
			{
				sprintf(p, "%02X", g_utcpBuf[i]);
				p += 2;
			}
			ADDLOG_EXTRADEBUG(LOG_FEATURE_DRV, "%d bytes UART RX->TCP TX: %s", len, data);
#endif
			ret = send(client_fd, g_utcpBuf, len, 0);
		}
		else
		{
			if(rx_closed)
			{
				goto exit;
			}
			rtos_delay_milliseconds(2);
			continue;
		}

		if(ret <= 0)
			goto exit;

		rtos_delay_milliseconds(5);
	}

exit:
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "UTCP_TX_Thd closed");
	tx_closed = true;
	rtos_suspend_thread(NULL);
}

static void UTCP_RX_Thd(void* param)
{
	int client_fd = *(int*)param;
	unsigned char buffer[1024];

	while(1)
	{
		int ret = 0;

		if(client_fd == INVALID_SOCK) goto exit;
		ret = recv(client_fd, buffer, sizeof(buffer), 0);
		if(ret > 0)
		{
#if UTCP_DEBUG
			char data[ret * 2];
			char* p = data;
			for(int i = 0; i < ret; i++)
			{
				sprintf(p, "%02X", buffer[i]);
				p += 2;
			}
			ADDLOG_EXTRADEBUG(LOG_FEATURE_DRV, "%d bytes TCP RX->UART TX: %s", ret, data);
#endif
			// >>> НАШ КОД: Проверка пакетов инициализации и сброс CEN <<<
			if(!g_bk_synced)
			{
				for(int i = 0; i < ret; i++)
				{
					uint8_t b = buffer[i];
					if(b == 0x01 && g_magic_match == 0) g_magic_match = 1;
					else if(b == 0xE0 && g_magic_match == 1) g_magic_match = 2;
					else if(b == 0xFC && g_magic_match == 2)
					{
						g_magic_match = 0;
						uint32_t now = (uint32_t)rtos_get_time();
						if((now - g_bk_last_reset) > 300)
						{
							g_bk_last_reset = now;
							g_reset_attempt++;

							// Dynamic sweeping timing:
							// post-reset delay sweeps: 2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35 ms
							g_last_wait_ms = 2 + (g_reset_attempt % 12) * 3;
							int post_delay = g_last_wait_ms;
							g_last_pulse_ms = 20 + ((g_reset_attempt / 2) % 3) * 10;
							int reset_pulse = g_last_pulse_ms;

							ADDLOG_INFO(LOG_FEATURE_DRV, "CB2S: Reset #%d (pulse=%d ms, wait=%d ms)...", g_reset_attempt, reset_pulse, post_delay);

							// 1. Pull CEN low (active reset)
							HAL_PIN_Setup_Output(BK_CEN_PIN);
							HAL_PIN_SetOutputValue(BK_CEN_PIN, 0);
							rtos_delay_milliseconds(reset_pulse);

							// 2. Drive CEN high actively for 2 ms to sharpen rising edge
							HAL_PIN_SetOutputValue(BK_CEN_PIN, 1);
							rtos_delay_milliseconds(2);

							// 3. Release CEN to Hi-Z (input)
							HAL_PIN_Setup_Input(BK_CEN_PIN);

							// 4. Dynamic post-reset delay before bursts
							rtos_delay_milliseconds(post_delay);

							// 5. Send high-density bursts (8 bursts spaced by 5 ms)
							const uint8_t link_pkt[] = { 0x01, 0xE0, 0xFC, 0x01, 0x00 };
							for(int burst = 0; burst < 8; burst++)
							{
								for(int k = 0; k < (int)sizeof(link_pkt); k++)
								{
									UART_SendByte(link_pkt[k]);
								}
								rtos_delay_milliseconds(5);
							}
						}
					}
					else
					{
						g_magic_match = (b == 0x01) ? 1 : 0;
					}
				}
			}

			for(int i = 0; i < ret; i++)
			{
				UART_SendByte(buffer[i]);
			}
		}
		else if(tx_closed)
		{
			goto exit;
		}

		// ret == -1 and socket error == EAGAIN when no data received for nonblocking
		if((ret == -1) && (errno == EAGAIN))
			continue;
		else if(ret <= 0)
		{
			ADDLOG_DEBUG(LOG_FEATURE_DRV, "ret: %i, errno: %i", ret, errno);
			goto exit;
		}

		rtos_delay_milliseconds(5);
	}

exit:
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "UTCP_RX_Thd closed");
	rx_closed = true;
	rtos_suspend_thread(NULL);
}

void UART_TCP_TRX_Thread()
{
	OSStatus err = kNoErr;
	int reuse = 1;
	struct sockaddr_in server_addr =
	{
		.sin_family = AF_INET,
		.sin_addr =
		{
			.s_addr = INADDR_ANY,
		},
		.sin_port = htons(DEFAULT_UART_TCP_PORT),
	};

	if(listen_sock != INVALID_SOCK) close(listen_sock);
	if(client_sock != INVALID_SOCK) close(client_sock);

	listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if(listen_sock < 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "Unable to create socket");
		goto error;
	}
	int flags = fcntl(listen_sock, F_GETFL, 0);
	if(fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK) == -1)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "Unable to set socket non blocking");
		goto error;
	}

	setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

	err = bind(listen_sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
	if(err != 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "Socket unable to bind");
		goto error;
	}
	err = listen(listen_sock, 2);
	if(err != 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_HTTP, "Error occurred during listen");
		goto error;
	}

	while(1)
	{
		struct sockaddr_storage source_addr;
		socklen_t addr_len = sizeof(source_addr);
		client_sock = accept(listen_sock, (struct sockaddr*)&source_addr, &addr_len);
		if(client_sock != INVALID_SOCK)
		{
			// >>> НАШ КОД: Взводим состояние для новой сессии прошивки <<<
			g_bk_synced = 0;
			g_bk_last_reset = 0;
			g_reset_attempt = 0;
			g_reset_attempt = 0;
			g_magic_match = 0;
			g_ack_match = 0;
			ADDLOG_INFO(LOG_FEATURE_DRV, "CB2S: Client connected, ready to listen stream.");

			if(g_conn_channel >= 0) CHANNEL_Set(g_conn_channel, 1, CHANNEL_SET_FLAG_SKIP_MQTT | CHANNEL_SET_FLAG_SILENT);
			rx_closed = true;
			tx_closed = true;

			if(g_tx_thread != NULL)
			{
				rtos_delete_thread(&g_tx_thread);
			}
			err = rtos_create_thread(&g_tx_thread, BEKEN_APPLICATION_PRIORITY - 1,
				"UTCP_TX_Thd",
				(beken_thread_function_t)UTCP_TX_Thd,
				STACK_SIZE,
				(beken_thread_arg_t)&client_sock);
			if(err != kNoErr)
			{
				ADDLOG_ERROR(LOG_FEATURE_DRV, "create \"UTCP_TX_Thd\" thread failed with %i!", err);
			}
			else tx_closed = false;

			//rtos_delay_milliseconds(10);

			if(g_rx_thread != NULL)
			{
				rtos_delete_thread(&g_rx_thread);
			}
			err = rtos_create_thread(&g_rx_thread, BEKEN_APPLICATION_PRIORITY - 1,
				"UTCP_RX_Thd",
				(beken_thread_function_t)UTCP_RX_Thd,
				STACK_SIZE,
				(beken_thread_arg_t)&client_sock);
			if(err != kNoErr)
			{
				ADDLOG_ERROR(LOG_FEATURE_DRV, "create \"UTCP_RX_Thd\" thread failed with %i!", err);
			}
			else rx_closed = false;

			while(1)
			{
				if(tx_closed && rx_closed)
				{
					close(client_sock);
					ADDLOG_DEBUG(LOG_FEATURE_DRV, "UART TCP connection closed", err);
					// >>> НАШ КОД: Сбрасываем флаги при отключении ПК <<<
					g_bk_synced = 0;
					g_reset_attempt = 0;
					g_magic_match = 0;
					g_ack_match = 0;
					ADDLOG_INFO(LOG_FEATURE_DRV, "CB2S: Client disconnected, flags reset.");
					if(g_conn_channel >= 0) CHANNEL_Set(g_conn_channel, 0, CHANNEL_SET_FLAG_SKIP_MQTT | CHANNEL_SET_FLAG_SILENT);
					break;
				}
				else
				{
					if(!Main_HasWiFiConnected()) goto error;
					rtos_delay_milliseconds(10);
				}
			}
		}
		rtos_delay_milliseconds(10);
	}

error:
	ADDLOG_ERROR(LOG_FEATURE_DRV, "UART TCP Error");

	if(g_start_thread != NULL)
	{
		rtos_delete_thread(&g_start_thread);
	}
	err = rtos_create_thread(&g_start_thread, BEKEN_APPLICATION_PRIORITY,
		"UART TCP Restart",
		(beken_thread_function_t)Start_UART_TCP,
		0x800,
		(beken_thread_arg_t)0);
	if(err != kNoErr)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "create \"UART TCP Restart\" thread failed with %i!", err);
	}
}

void Start_UART_TCP(void* arg)
{
	UART_TCP_Deinit();

	g_utcpBuf = (byte*)os_malloc(buf_size);

	OSStatus err = rtos_create_thread(&g_trx_thread, BEKEN_APPLICATION_PRIORITY,
		"UART_TCP_TRX",
		(beken_thread_function_t)UART_TCP_TRX_Thread,
		0x800,
		(beken_thread_arg_t)0);
	if(err != kNoErr)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "create \"UART_TCP_TRX\" thread failed with %i!", err);
	}
	rtos_suspend_thread(NULL);
}

// startDriver UartTCP [baudrate] [buffer size] [connection channel] [hw flow control]
// connection is for led, -1 if not used.
// Sample:
// startDriver UartTCP 115200 8192
// Then connect to 8888

// backlog stopDriver UartTCP; startDriver UartTCP 115200 8192
void UART_TCP_Init()
{
	g_baudRate = Tokenizer_GetArgIntegerDefault(1, g_baudRate);
	uint32_t reqbufsize = Tokenizer_GetArgIntegerDefault(2, DEFAULT_BUF_SIZE);
	buf_size = reqbufsize > 16384 ? 16384 : reqbufsize;
	g_conn_channel = Tokenizer_GetArgIntegerDefault(3, -1);
	int flowcontrol = Tokenizer_GetArgIntegerDefault(4, 0);

	UART_InitUART(g_baudRate, 0, flowcontrol > 0 ? true : false);
	UART_InitReceiveRingBuffer(buf_size * 2);

	// >>> НАШ КОД: Сразу при старте драйвера переводим P8 в Hi-Z (вход) <<<
	HAL_PIN_Setup_Input(BK_CEN_PIN);

	if(g_start_thread != NULL)
	{
		rtos_delete_thread(&g_start_thread);
	}
	OSStatus err = rtos_create_thread(&g_start_thread, BEKEN_APPLICATION_PRIORITY,
		"UART_TCP",
		(beken_thread_function_t)Start_UART_TCP,
		0x800,
		(beken_thread_arg_t)0);
	if(err != kNoErr)
	{
		ADDLOG_ERROR(LOG_FEATURE_DRV, "create \"UART_TCP\" thread failed with %i!", err);
	}
}

void UART_TCP_Deinit()
{
	if(g_trx_thread != NULL)
	{
		rtos_delete_thread(&g_trx_thread);
		g_trx_thread = NULL;
	}
	if(g_rx_thread != NULL)
	{
		rx_closed = true;
		rtos_delete_thread(&g_rx_thread);
		g_rx_thread = NULL;
	}
	if(g_tx_thread != NULL)
	{
		tx_closed = true;
		rtos_delete_thread(&g_tx_thread);
		g_tx_thread = NULL;
	}
	if(g_utcpBuf) free(g_utcpBuf);

	if(listen_sock != INVALID_SOCK) close(listen_sock);
	if(client_sock != INVALID_SOCK) close(client_sock);
}

#endif