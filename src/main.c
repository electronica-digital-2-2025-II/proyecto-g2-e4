#include "xparameters.h"
#include "xil_printf.h"
#include "xstatus.h"
#include "xiicps.h"
#include "sleep.h"
#include "xgpio.h"
#include <string.h>
#include <stdio.h>

// ===================== DEFINES ===================== //

#define IIC_DEVICE_ID   0

#define MAX_ADDR        0x57   // MAX30102
#define MLX_ADDR        0x5A   // MLX90614
#define MLX_REG_TA      0x06   // Temp ambiente
#define MLX_REG_TOBJ1   0x07   // Temp objeto

XIicPs IicInstance;

// ---- BUZZER por AXI GPIO ----
// DeviceId = 0 porque solo hay un XGPIO (ver XPAR_XGPIO_NUM_INSTANCES en xparameters.h)
#define BUZZER_GPIO_DEVICE_ID   0
#define BUZZER_GPIO_CHANNEL     1     // Canal 1 del axi_gpio_0 (GPIO_0)

// *** ACTIVO EN BAJO: 0 = SUENA, 1 = CALLADO ***
#define BUZZER_ON   0
#define BUZZER_OFF  1

XGpio BuzzerGpio;   // instancia del GPIO para el BUZZER

// Frecuencia de muestreo ~50 Hz
#define SAMPLE_PERIOD_US    20000
#define SAMPLE_RATE_HZ      (1000000 / SAMPLE_PERIOD_US)

// UART print
#define PRINT_DECIM         10

// OLED update cada ~0.5 s
#define OLED_UPDATE_DECIM   25

// ===================== OLED SSD1306 ===================== //

#define OLED_ADDR       0x3C
#define OLED_WIDTH      128
#define OLED_HEIGHT     32
#define OLED_PAGES      (OLED_HEIGHT / 8)

static u8 oled_buffer[OLED_WIDTH * OLED_PAGES];

// Prototipos OLED
int  OLED_SendCommand(u8 cmd);
int  OLED_SendData(const u8 *data, u32 len);
void OLED_Init(void);
void OLED_ClearBuffer(void);
void OLED_Update(void);
void OLED_DrawPixel(int x, int y, int on);
void OLED_FillRect(int x0, int y0, int x1, int y1, int on);

// Texto 6x8
void OLED_ClearTopLine(void);
void OLED_DrawChar6x8(int x, int y, char c);
void OLED_DrawString6x8(int x, int y, const char *s);

// Mostrar todo en OLED
void OLED_ShowVitals(float bpm, float Ta, float To, float spo2);

// ===================== I2C / MAX / MLX ===================== //

int IicInit(u16 DeviceId);
int I2C_WriteReg(u8 devAddr, u8 reg, u8 value);
int I2C_ReadReg(u8 devAddr, u8 reg, u8 *value);
int I2C_ReadMulti(u8 devAddr, u8 reg, u8 *buf, u32 len);

int Max_CheckPartID(void);
int Max30102_Reset(void);
int Max30102_Init_Config(void);
int Max30102_ReadLatestRedIR(u32 *red, u32 *ir);

// MLX90614
float MLX90614_ReadTemp(u8 regAddr);

// ===================== HR + SpO2 ===================== //

void HR_Init(void);
void HR_ProcessSample(u32 ir, float *bpm_out);
void SPO2_Update(u32 red_raw, u32 ir_raw, float *spo2_out);

// ---- HR BÁSICO ---- //

static float dc_est        = 0.0f;
static float ac_prev2      = 0.0f;
static float ac_prev1      = 0.0f;
static float ac_curr       = 0.0f;
static float ac_peak_est   = 0.0f;
static int   samples_since_beat = 0;
static int   in_peak       = 0;

#define BPM_HISTORY_LEN 8
static float bpm_hist[BPM_HISTORY_LEN];
static int   bpm_hist_count = 0;
static int   bpm_hist_index = 0;

// Umbral DC para "hay dedo" (IR grande)
#define DC_FINGER_MIN   5000.0f

// ---- SpO2 filtros ---- //

static float spo2_dc_ir  = 0.0f;
static float spo2_dc_red = 0.0f;
static float spo2_ac_ir  = 0.0f;
static float spo2_ac_red = 0.0f;

// Últimas temperaturas válidas
static float g_Ta = -1000.0f;
static float g_To = -1000.0f;

// ===================== MAIN ===================== //

int main(void)
{
    int Status;

    xil_printf("\r\n=== MAX30102 + MLX90614 + OLED (BPM, Ta/To, SpO2 + BUZZER) ===\r\n");

    Status = IicInit(IIC_DEVICE_ID);
    if (Status != XST_SUCCESS) {
        xil_printf("Error inicializando I2C: %d\r\n", Status);
        return XST_FAILURE;
    }
    xil_printf("I2C OK.\r\n");

    // ---- Inicializar GPIO para BUZZER ----
    Status = XGpio_Initialize(&BuzzerGpio, BUZZER_GPIO_DEVICE_ID);
    if (Status != XST_SUCCESS) {
        xil_printf("Error inicializando AXI GPIO del BUZZER\r\n");
        return XST_FAILURE;
    }

    // Canal como salida (bit 0 = BUZZER)
    XGpio_SetDataDirection(&BuzzerGpio, BUZZER_GPIO_CHANNEL, 0x0); // 0 = output

    // Apagar buzzer al inicio (ACTIVO EN BAJO)
    XGpio_DiscreteWrite(&BuzzerGpio, BUZZER_GPIO_CHANNEL, BUZZER_OFF);

    // ==== PRUEBA BRUTA DEL BUZZER (opcional, para verificar hardware) ====
    xil_printf("Probando BUZZER: deberia sonar 2 veces...\r\n");

    for (int i = 0; i < 2; i++) {
        xil_printf("BUZZER ON (iter %d)\r\n", i);
        XGpio_DiscreteWrite(&BuzzerGpio, BUZZER_GPIO_CHANNEL, BUZZER_ON);
        sleep(1);

        xil_printf("BUZZER OFF (iter %d)\r\n", i);
        XGpio_DiscreteWrite(&BuzzerGpio, BUZZER_GPIO_CHANNEL, BUZZER_OFF);
        sleep(1);
    }

    xil_printf("Fin prueba BUZZER, entrando al while() principal\r\n");

    // ==== OLED + Sensores ====
    OLED_Init();
    OLED_ClearBuffer();
    OLED_Update();

    Status = Max_CheckPartID();
    if (Status != XST_SUCCESS) {
        xil_printf("No se detecto MAX30102 en 0x%02X\r\n", MAX_ADDR);
        return XST_FAILURE;
    }

    Status = Max30102_Init_Config();
    if (Status != XST_SUCCESS) {
        xil_printf("Error inicializando MAX30102: %d\r\n", Status);
        return XST_FAILURE;
    }

    HR_Init();

    xil_printf("Sensores listos. Coloca el dedo sobre el MAX y mira OLED.\r\n");

    float bpm  = 0.0f;
    float spo2 = 0.0f;

    int print_counter = 0;
    int oled_counter  = 0;

    while (1) {
        u32 red, ir;

        Status = Max30102_ReadLatestRedIR(&red, &ir);
        if (Status == XST_SUCCESS) {

            // Procesa BPM y SpO2  (NO TOCO ESTO)
            HR_ProcessSample(ir, &bpm);
            SPO2_Update(red, ir, &spo2);

            // --- CONTROL DEL BUZZER SEGÚN BPM ---
            int bpm_int_for_buzzer = (int)(bpm + 0.5f);  // redondear BPM

            // *** SUENA CUANDO BPM >= 105 ***
            if (bpm_int_for_buzzer >= 105) {
                XGpio_DiscreteWrite(&BuzzerGpio, BUZZER_GPIO_CHANNEL, BUZZER_ON);
            } else {
                XGpio_DiscreteWrite(&BuzzerGpio, BUZZER_GPIO_CHANNEL, BUZZER_OFF);
            }

            // UART cada PRINT_DECIM muestras
            print_counter++;
            if (print_counter >= PRINT_DECIM) {
                print_counter = 0;

                int bpm10  = (int)(bpm  * 10.0f);
                int spo210 = (int)(spo2 * 10.0f);

                xil_printf("RED=%lu IR=%lu  BPM=%d.%d  SpO2=%d.%d  Ta=%.2f  To=%.2f\r\n",
                           (unsigned long)red,
                           (unsigned long)ir,
                           bpm10 / 10,  bpm10 % 10,
                           spo210 / 10, spo210 % 10,
                           g_Ta, g_To);
            }

            // OLED cada OLED_UPDATE_DECIM muestras
            oled_counter++;
            if (oled_counter >= OLED_UPDATE_DECIM) {
                oled_counter = 0;

                // Leer temperaturas (una vez por actualización)
                float Ta = MLX90614_ReadTemp(MLX_REG_TA);
                if (Ta != -999.0f) {
                    g_Ta = Ta;
                }

                // pequeño delay antes de leer To
                usleep(10000);

                float To = MLX90614_ReadTemp(MLX_REG_TOBJ1);
                if (To != -999.0f) {
                    g_To = To;
                }

                OLED_ShowVitals(bpm, g_Ta, g_To, spo2);
            }

        } else {
            xil_printf("Error lectura Red/IR: %d\r\n", Status);
        }

        usleep(SAMPLE_PERIOD_US);  // ~50 Hz
    }

    return 0;
}

// ===================== I2C BÁSICO ===================== //

int IicInit(u16 DeviceId)
{
    XIicPs_Config *Config;
    int Status;

    Config = XIicPs_LookupConfig(DeviceId);
    if (Config == NULL) {
        return XST_FAILURE;
    }

    Status = XIicPs_CfgInitialize(&IicInstance, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    XIicPs_Reset(&IicInstance);
    XIicPs_SetSClk(&IicInstance, 100000); // 100 kHz

    return XST_SUCCESS;
}

int I2C_WriteReg(u8 devAddr, u8 reg, u8 value)
{
    u8 buf[2] = { reg, value };
    int Status;

    Status = XIicPs_MasterSendPolled(&IicInstance, buf, 2, devAddr);
    if (Status != XST_SUCCESS) return Status;
    while (XIicPs_BusIsBusy(&IicInstance));

    return XST_SUCCESS;
}

int I2C_ReadReg(u8 devAddr, u8 reg, u8 *value)
{
    int Status;

    Status = XIicPs_MasterSendPolled(&IicInstance, &reg, 1, devAddr);
    if (Status != XST_SUCCESS) return Status;
    while (XIicPs_BusIsBusy(&IicInstance));

    Status = XIicPs_MasterRecvPolled(&IicInstance, value, 1, devAddr);
    if (Status != XST_SUCCESS) return Status;
    while (XIicPs_BusIsBusy(&IicInstance));

    return XST_SUCCESS;
}

int I2C_ReadMulti(u8 devAddr, u8 reg, u8 *buf, u32 len)
{
    int Status;

    Status = XIicPs_MasterSendPolled(&IicInstance, &reg, 1, devAddr);
    if (Status != XST_SUCCESS) return Status;
    while (XIicPs_BusIsBusy(&IicInstance));

    Status = XIicPs_MasterRecvPolled(&IicInstance, buf, len, devAddr);
    if (Status != XST_SUCCESS) return Status;
    while (XIicPs_BusIsBusy(&IicInstance));

    return XST_SUCCESS;
}

// ===================== MAX30102 ===================== //

int Max_CheckPartID(void)
{
    u8 partID;
    int Status;

    Status = I2C_ReadReg(MAX_ADDR, 0xFF, &partID);
    if (Status != XST_SUCCESS) {
        xil_printf("Error leyendo PART ID: %d\r\n", Status);
        return Status;
    }

    xil_printf("PART ID (0xFF) en 0x%02X = 0x%02X\r\n", MAX_ADDR, partID);

    if (partID == 0x15) {
        xil_printf("Detectado MAX30102.\r\n");
        return XST_SUCCESS;
    } else if (partID == 0x11) {
        xil_printf("Detectado MAX30105 (compatible).\r\n");
        return XST_SUCCESS;
    } else {
        xil_printf("PART ID inesperado.\r\n");
        return XST_FAILURE;
    }
}

int Max30102_Reset(void)
{
    int Status = I2C_WriteReg(MAX_ADDR, 0x09, 0x40); // MODE_CONFIG, 0X40 ES BIT DE RESET
    if (Status != XST_SUCCESS) return Status;
    usleep(10000);
    return XST_SUCCESS;
}

int Max30102_Init_Config(void)
{
    int Status;

    Status = Max30102_Reset();
    if (Status != XST_SUCCESS) return Status;

    // Desactivar interrupciones, en 0x00 las deshabilita, no las necesitamos 
    I2C_WriteReg(MAX_ADDR, 0x02, 0x00); // INT_EN1 
    I2C_WriteReg(MAX_ADDR, 0x03, 0x00); // INT_EN2

    // Punteros FIFO, manda a 0, escribee, lee y cuenta desde 0
    I2C_WriteReg(MAX_ADDR, 0x04, 0x00); // FIFO_WR_PTR
    I2C_WriteReg(MAX_ADDR, 0x05, 0x00); // OVF_COUNTER
    I2C_WriteReg(MAX_ADDR, 0x06, 0x00); // FIFO_RD_PTR

    // FIFO_CONFIG (0x08): SMP_AVE=0, rollover habilitado, no se bloquea porque si se llena, sobreescribe
    I2C_WriteReg(MAX_ADDR, 0x08, 0x0F);

    // SPO2_CONFIG (0x0A): rango ADC bajo, 100 Hz, 18 bits, para alta resolución, datasheet 
    I2C_WriteReg(MAX_ADDR, 0x0A, 0x27);

    // MODE: multi-LED (RED + IR) 
    I2C_WriteReg(MAX_ADDR, 0x09, 0x07);

    // Corriente LEDs moderada, evitar ruido y reducir consumo
    I2C_WriteReg(MAX_ADDR, 0x0C, 0x24); // LED1_PA (RED)
    I2C_WriteReg(MAX_ADDR, 0x0D, 0x24); // LED2_PA (IR)

    // Slots: SLOT1 = RED, SLOT2 = IR, se sincroniza para las lecturas FIFO, 
    I2C_WriteReg(MAX_ADDR, 0x11, 0x21); // SLOT1=1, SLOT2=2
    I2C_WriteReg(MAX_ADDR, 0x12, 0x00); // SLOT3/SLOT4 off

    return XST_SUCCESS;
}

int Max30102_ReadLatestRedIR(u32 *red, u32 *ir)
{
    u8 wrPtr, rdPtr; // punteros de lectura de la FIFO 
    int Status;

    //Hallamos muestras nuevas
    Status = I2C_ReadReg(MAX_ADDR, 0x04, &wrPtr); // FIFO_WR_PTR, posicion donde MAX escrbie la muestra
    if (Status != XST_SUCCESS) return Status;

    Status = I2C_ReadReg(MAX_ADDR, 0x06, &rdPtr); // FIFO_RD_PTR, posicion actual
    if (Status != XST_SUCCESS) return Status;

    int numSamples = (wrPtr - rdPtr) & 0x1F;  // FIFO de 32 muestras

    if (numSamples == 0) {
        *red = 0;
        *ir  = 0;
        return XST_SUCCESS; // Si no es 0 aun hay datos por procesar
    }

    // Leer todas las muestras pendientes y quedarnos con la última
    for (int i = 0; i < numSamples; i++) {
        u8 buf[6]; // 3 bytes para IR y 3 para red
        Status = I2C_ReadMulti(MAX_ADDR, 0x07, buf, 6);
        if (Status != XST_SUCCESS) return Status;

        // Junta los 3 bits de cada canal, reconstruye las de 24 bits y las deja como de 18 bits
        u32 red24 = ((u32)buf[0] << 16) | ((u32)buf[1] << 8) | buf[2];
        u32 ir24  = ((u32)buf[3] << 16) | ((u32)buf[4] << 8) | buf[5];

        red24 &= 0x3FFFF; // El sensor usa 18 bits utiles 
        ir24  &= 0x3FFFF;

        *red = red24;
        *ir  = ir24;
    }

    return XST_SUCCESS;
}

// ===================== MLX90614: LECTURA SIMPLE ===================== //
//


float MLX90614_ReadTemp(u8 regAddr)
{
    u8 sendBuf[1]; //direccion
    u8 recvBuf[2]; //medicion
    int Status;

    sendBuf[0] = regAddr; // Ta o To

    // Repeated Start
    XIicPs_SetOptions(&IicInstance, XIICPS_REP_START_OPTION); // patron de lectura

    Status = XIicPs_MasterSendPolled(&IicInstance, sendBuf, 1, MLX_ADDR); 
    if (Status != XST_SUCCESS) {
        XIicPs_ClearOptions(&IicInstance, XIICPS_REP_START_OPTION); 
        return -999.0f; // indica error
    }

    XIicPs_ClearOptions(&IicInstance, XIICPS_REP_START_OPTION); 

    Status = XIicPs_MasterRecvPolled(&IicInstance, recvBuf, 2, MLX_ADDR);
    if (Status != XST_SUCCESS) {
        return -999.0f;
    }

    u16 raw = ((u16)recvBuf[1] << 8) | recvBuf[0]; // se re arma el valor crudo de 16 bits

    // Valor inválido típico
    if (raw == 0xFFFF) {
        return -999.0f;
    }

    // Conversión datasheet: 0.02 K/LSB, offset 273.15 para °C
    return (float)raw * 0.02f - 273.15f;
}

// ===================== OLED IMPLEMENTACIÓN ===================== //

int OLED_SendCommand(u8 cmd)
{
    u8 buf[2];
    buf[0] = 0x00;
    buf[1] = cmd;
    int Status = XIicPs_MasterSendPolled(&IicInstance, buf, 2, OLED_ADDR);
    if (Status != XST_SUCCESS) return Status;
    while (XIicPs_BusIsBusy(&IicInstance));
    return XST_SUCCESS;
}

int OLED_SendData(const u8 *data, u32 len)
{
    u8 buf[1 + 128];
    int Status;

    while (len > 0) {
        u32 chunk = (len > 128) ? 128 : len;  // envia datos de imagen en pedazos de max 128
        buf[0] = 0x40;
        memcpy(&buf[1], data, chunk);

        // Aca se mandan bloques de max 128 bytes 
        Status = XIicPs_MasterSendPolled(&IicInstance, buf, 1 + chunk, OLED_ADDR);
        if (Status != XST_SUCCESS) return Status;
        while (XIicPs_BusIsBusy(&IicInstance));

        data += chunk;
        len  -= chunk;
    }
    return XST_SUCCESS;
}
 //Mnada la secuencia estandar de config del controlador 
void OLED_Init(void)
{
    // Apaga display, configura reloj, limpia buffer, enciende display
    OLED_SendCommand(0xAE);
    OLED_SendCommand(0xD5); OLED_SendCommand(0x80);
    OLED_SendCommand(0xA8); OLED_SendCommand(0x1F);
    OLED_SendCommand(0xD3); OLED_SendCommand(0x00);
    OLED_SendCommand(0x40);
    OLED_SendCommand(0x8D); OLED_SendCommand(0x14);
    OLED_SendCommand(0x20); OLED_SendCommand(0x00);
    OLED_SendCommand(0xA1);
    OLED_SendCommand(0xC8);
    OLED_SendCommand(0xDA); OLED_SendCommand(0x02);
    OLED_SendCommand(0x81); OLED_SendCommand(0x8F);
    OLED_SendCommand(0xD9); OLED_SendCommand(0xF1);
    OLED_SendCommand(0xDB); OLED_SendCommand(0x40);
    OLED_SendCommand(0xA4);
    OLED_SendCommand(0xA6);
    OLED_ClearBuffer();
    OLED_Update();
    OLED_SendCommand(0xAF);
}

void OLED_ClearBuffer(void)
{
    memset(oled_buffer, 0x00, sizeof(oled_buffer));
}

void OLED_Update(void)
{
    OLED_SendCommand(0x21);
    OLED_SendCommand(0);
    OLED_SendCommand(OLED_WIDTH - 1);

    OLED_SendCommand(0x22);
    OLED_SendCommand(0);
    OLED_SendCommand(OLED_PAGES - 1);

    for (int page = 0; page < OLED_PAGES; page++) {
        const u8 *p = &oled_buffer[page * OLED_WIDTH];
        OLED_SendData(p, OLED_WIDTH);
    }
}

void OLED_DrawPixel(int x, int y, int on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;

    int page = y / 8;
    int bit  = y % 8;
    u8 *byte = &oled_buffer[page * OLED_WIDTH + x];

    if (on)
        *byte |= (1 << bit);
    else
        *byte &= ~(1 << bit);
}

void OLED_FillRect(int x0, int y0, int x1, int y1, int on)
{
    if (x0 > x1) { int t = x0; x0 = x1; x1 = t; }
    if (y0 > y1) { int t = y0; y0 = y1; y1 = t; }

    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= OLED_WIDTH)  x1 = OLED_WIDTH - 1;
    if (y1 >= OLED_HEIGHT) y1 = OLED_HEIGHT - 1;

    for (int x = 0; x <= x1; x++) {
        for (int y = 0; y <= y1; y++) {
            OLED_DrawPixel(x, y, on);
        }
    }
}

// ===================== FUENTE 6x8 ===================== //

typedef struct {
    u8 rows[8];
} Glyph6x8;

static const Glyph6x8 GLYPH_SPACE = {{0,0,0,0,0,0,0,0}};
static const Glyph6x8 GLYPH_MINUS = {{0x00,0x00,0x0E,0x0E,0x00,0x00,0x00,0x00}};
static const Glyph6x8 GLYPH_DOT   = {{0x00,0x00,0x00,0x00,0x04,0x04,0x00,0x00}};
static const Glyph6x8 GLYPH_COLON = {{0x04,0x04,0x00,0x00,0x04,0x04,0x00,0x00}};
static const Glyph6x8 GLYPH_EQUAL = {{0x00,0x00,0x0E,0x00,0x0E,0x00,0x00,0x00}};

static const Glyph6x8 GLYPH_0 = {{0x0E,0x11,0x11,0x11,0x11,0x11,0x0E,0x00}};
static const Glyph6x8 GLYPH_1 = {{0x04,0x0C,0x04,0x04,0x04,0x04,0x0E,0x00}};
static const Glyph6x8 GLYPH_2 = {{0x0E,0x11,0x10,0x0E,0x01,0x01,0x1F,0x00}};
static const Glyph6x8 GLYPH_3 = {{0x0E,0x11,0x10,0x0E,0x10,0x11,0x0E,0x00}};
static const Glyph6x8 GLYPH_4 = {{0x11,0x11,0x11,0x1F,0x10,0x10,0x10,0x00}};
static const Glyph6x8 GLYPH_5 = {{0x1F,0x01,0x01,0x0E,0x10,0x11,0x0E,0x00}};
static const Glyph6x8 GLYPH_6 = {{0x0E,0x01,0x01,0x0F,0x11,0x11,0x0E,0x00}};
static const Glyph6x8 GLYPH_7 = {{0x1F,0x10,0x08,0x04,0x02,0x02,0x02,0x00}};
static const Glyph6x8 GLYPH_8 = {{0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E,0x00}};
static const Glyph6x8 GLYPH_9 = {{0x0E,0x11,0x11,0x0F,0x10,0x10,0x0E,0x00}};

static const Glyph6x8 GLYPH_B = {{0x0F,0x11,0x11,0x0F,0x11,0x11,0x0F,0x00}};
static const Glyph6x8 GLYPH_P = {{0x0F,0x11,0x11,0x0F,0x01,0x01,0x01,0x00}};
static const Glyph6x8 GLYPH_M = {{0x11,0x1B,0x15,0x15,0x11,0x11,0x11,0x00}};

static const Glyph6x8 GLYPH_T = {{0x1F,0x04,0x04,0x04,0x04,0x04,0x04,0x00}};
static const Glyph6x8 GLYPH_a = {{0x00,0x0E,0x10,0x1E,0x11,0x1E,0x00,0x00}};
static const Glyph6x8 GLYPH_o = {{0x00,0x0E,0x11,0x11,0x11,0x0E,0x00,0x00}};
static const Glyph6x8 GLYPH_C = {{0x0E,0x11,0x01,0x01,0x01,0x11,0x0E,0x00}};
static const Glyph6x8 GLYPH_S = {{0x0E,0x11,0x03,0x0E,0x18,0x11,0x0E,0x00}};
static const Glyph6x8 GLYPH_p = {{0x00,0x1E,0x11,0x11,0x1E,0x01,0x01,0x00}};
static const Glyph6x8 GLYPH_O = {{0x0E,0x11,0x11,0x11,0x11,0x11,0x0E,0x00}};

static const Glyph6x8* OLED_GetGlyph(char c)
{
    switch (c) {
    case '0': return &GLYPH_0;
    case '1': return &GLYPH_1;
    case '2': return &GLYPH_2;
    case '3': return &GLYPH_3;
    case '4': return &GLYPH_4;
    case '5': return &GLYPH_5;
    case '6': return &GLYPH_6;
    case '7': return &GLYPH_7;
    case '8': return &GLYPH_8;
    case '9': return &GLYPH_9;

    case 'B': return &GLYPH_B;
    case 'P': return &GLYPH_P;
    case 'M': return &GLYPH_M;

    case 'T': return &GLYPH_T;
    case 'a': return &GLYPH_a;
    case 'o': return &GLYPH_o;
    case 'C': return &GLYPH_C;
    case 'S': return &GLYPH_S;
    case 'p': return &GLYPH_p;
    case 'O': return &GLYPH_O;

    case ':': return &GLYPH_COLON;
    case '.': return &GLYPH_DOT;
    case '-': return &GLYPH_MINUS;
    case '=': return &GLYPH_EQUAL;
    case ' ':
    default:
        return &GLYPH_SPACE;
    }
}

void OLED_ClearTopLine(void)
{
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < OLED_WIDTH; x++) {
            OLED_DrawPixel(x, y, 0);
        }
    }
}

void OLED_DrawChar6x8(int x, int y, char c)
{
    const Glyph6x8 *g = OLED_GetGlyph(c);
    for (int row = 0; row < 8; row++) {
        u8 pattern = g->rows[row];
        for (int col = 0; col < 5; col++) {
            int on = (pattern & (1 << col)) ? 1 : 0;
            OLED_DrawPixel(x + col, y + row, on);
        }
        OLED_DrawPixel(x + 5, y + row, 0);
    }
}

void OLED_DrawString6x8(int x, int y, const char *s)
{
    while (*s && x <= (OLED_WIDTH - 6)) {
        OLED_DrawChar6x8(x, y, *s++);
        x += 6;
    }
}

void OLED_ShowVitals(float bpm, float Ta, float To, float spo2)
{
    char line[40];

    OLED_ClearBuffer();

    // Línea 0: BPM
    {
        int bpm10 = (int)(bpm * 10.0f + 0.5f);
        if (bpm10 <= 0) {
            snprintf(line, sizeof(line), "BPM: ---.-");
        } else {
            int entero  = bpm10 / 10;
            int decimal = bpm10 % 10;
            if (entero > 999) entero = 999;
            snprintf(line, sizeof(line), "BPM: %3d.%1d", entero, decimal);
        }
        OLED_DrawString6x8(0, 0, line);
    }

    // Línea 1: Ta y To
    {
        if (Ta < -50.0f || Ta > 100.0f) {
            snprintf(line, sizeof(line), "Ta: --.-C To: --.-C");
        } else {
            int ta10 = (int)(Ta * 10.0f + 0.5f);
            int ta_e = ta10 / 10;
            int ta_d = (ta10 >= 0) ? (ta10 % 10) : -(ta10 % 10);

            if (To < -50.0f || To > 100.0f) {
                snprintf(line, sizeof(line), "Ta:%3d.%1dC To: --.-C",
                         ta_e, ta_d);
            } else {
                int to10 = (int)(To * 10.0f + 0.5f);
                int to_e = to10 / 10;
                int to_d = (to10 >= 0) ? (to10 % 10) : -(to10 % 10);

                snprintf(line, sizeof(line), "Ta:%3d.%1dC To:%3d.%1dC",
                         ta_e, ta_d, to_e, to_d);
            }
        }
        OLED_DrawString6x8(0, 8, line);
    }

    // Línea 2: SpO2
    {
        if (spo2 <= 0.0f) {
            snprintf(line, sizeof(line), "SpO2: ---.-");
        } else {
            int spo210 = (int)(spo2 * 10.0f + 0.5f);
            int se = spo210 / 10;
            int sd = spo210 % 10;
            if (se > 100) se = 100;
            snprintf(line, sizeof(line), "SpO2: %3d.%1d", se, sd);
        }
        OLED_DrawString6x8(0, 16, line);
    }

    OLED_Update();
}

// ===================== HR ===================== //

void HR_Init(void)
{
    dc_est = 0.0f;
    ac_prev2 = ac_prev1 = ac_curr = 0.0f;
    ac_peak_est = 0.0f;
    samples_since_beat = 0;
    in_peak = 0;

    for (int i = 0; i < BPM_HISTORY_LEN; i++) {
        bpm_hist[i] = 0.0f;
    }
    bpm_hist_count = 0;
    bpm_hist_index = 0;

    spo2_dc_ir  = 0.0f;
    spo2_dc_red = 0.0f;
    spo2_ac_ir  = 0.0f;
    spo2_ac_red = 0.0f;

    g_Ta = -1000.0f;
    g_To = -1000.0f;
}

void HR_ProcessSample(u32 ir, float *bpm_out)
{
    samples_since_beat++;

    const float alpha_inv = 16.0f;
    dc_est += ((float)ir - dc_est) / alpha_inv;

    if (dc_est < DC_FINGER_MIN) {
        samples_since_beat = 0;
        in_peak = 0;

        for (int i = 0; i < BPM_HISTORY_LEN; i++) {
            bpm_hist[i] = 0.0f;
        }
        bpm_hist_count = 0;
        bpm_hist_index = 0;

        *bpm_out = 0.0f;
        return;
    }

    ac_prev2 = ac_prev1;
    ac_prev1 = ac_curr;
    ac_curr  = (float)ir - dc_est;

    float ac_abs = (ac_curr > 0) ? ac_curr : -ac_curr;

    const float peak_alpha_inv = 8.0f;
    ac_peak_est += (ac_abs - ac_peak_est) / peak_alpha_inv;

    float dynamic_thresh = ac_peak_est * 0.3f;
    if (dynamic_thresh < 5.0f) {
        dynamic_thresh = 5.0f;
    }

    const float BPM_MIN = 40.0f;
    const float BPM_MAX = 180.0f;

    int min_samples_between_beats = (int)(0.4f * (float)SAMPLE_RATE_HZ);

    float bpm_display = *bpm_out;

    if (!in_peak &&
        (ac_prev1 > ac_prev2) &&
        (ac_prev1 > ac_curr) &&
        (ac_prev1 > dynamic_thresh) &&
        (samples_since_beat > min_samples_between_beats))
    {
        float inst_bpm = 60.0f * (float)SAMPLE_RATE_HZ / (float)samples_since_beat;

        if (inst_bpm >= BPM_MIN && inst_bpm <= BPM_MAX) {
            bpm_hist[bpm_hist_index] = inst_bpm;
            bpm_hist_index = (bpm_hist_index + 1) % BPM_HISTORY_LEN;
            if (bpm_hist_count < BPM_HISTORY_LEN) {
                bpm_hist_count++;
            }

            float sum = 0.0f;
            for (int i = 0; i < bpm_hist_count; i++) {
                sum += bpm_hist[i];
            }
            bpm_display = sum / (float)bpm_hist_count;
        }

        samples_since_beat = 0;
        in_peak = 1;
    }

    if (in_peak && ac_curr < dynamic_thresh * 0.3f) {
        in_peak = 0;
    }

    *bpm_out = bpm_display;
}

// ===================== SpO2 ===================== //

void SPO2_Update(u32 red_raw, u32 ir_raw, float *spo2_out)
{
    float red = (float)red_raw;
    float ir  = (float)ir_raw;

    if (red_raw < 8000 || ir_raw < 8000) {
        *spo2_out = 0.0f;
        return;
    }

    const float dc_alpha_inv = 50.0f;
    spo2_dc_ir  += (ir  - spo2_dc_ir)  / dc_alpha_inv;
    spo2_dc_red += (red - spo2_dc_red) / dc_alpha_inv;

    float ir_ac_sample  = ir  - spo2_dc_ir;
    float red_ac_sample = red - spo2_dc_red;

    float ir_ac_abs  = (ir_ac_sample  > 0) ? ir_ac_sample  : -ir_ac_sample;
    float red_ac_abs = (red_ac_sample > 0) ? red_ac_sample : -red_ac_sample;

    const float ac_alpha_inv = 50.0f;
    spo2_ac_ir  += (ir_ac_abs  - spo2_ac_ir)  / ac_alpha_inv;
    spo2_ac_red += (red_ac_abs - spo2_ac_red) / ac_alpha_inv;

    float spo2_inst;

    if (spo2_dc_ir  > 1000.0f &&
        spo2_dc_red > 1000.0f &&
        spo2_ac_ir  >   10.0f &&
        spo2_ac_red >   10.0f)
    {
        float num = (spo2_ac_red / spo2_dc_red);
        float den = (spo2_ac_ir  / spo2_dc_ir);

        if (den > 1e-6f) {
            float R = num / den;

            spo2_inst = -20.0f * R + 110.0f;

            if (spo2_inst > 100.0f) spo2_inst = 100.0f;
            if (spo2_inst < 80.0f)  spo2_inst = 80.0f;
        } else {
            return;
        }
    } else {
        return;
    }

    const float spo2_smooth_inv = 8.0f;

    if (*spo2_out <= 0.0f) {
        *spo2_out = spo2_inst;
    } else {    
        *spo2_out += (spo2_inst - *spo2_out) / spo2_smooth_inv;
    }
}
