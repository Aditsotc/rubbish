//
// Created by 25664 on 2026/6/2.
//


/**
 ******************************************************************************
 * @file    remote control.c
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2020/12/22
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "remote_control.h"
#include "bsp_CAN.h"
#include "detect_task.h"
#include "bsp_dwt.h"
#include "bsp_usart_idle.h"

RC_Type remote_control = {0};

uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];

uint8_t RC_Data_Buffer[16] = {0};

uint32_t RC_DWT_Count = 0;
float RC_dt = 0;
uint8_t RC_Update = 0;

void Remote_Control_Init(UART_HandleTypeDef *huart)
{
    remote_control.RC_USART = huart;
    // ң����������Ϣ��ʼ�� �����
    // HAL_UART_Receive_DMA(huart, sbus_rx_buf, RC_FRAME_LENGTH);
    USART_IDLE_Init(huart, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

/**
 * @Func		void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
 * @Brief  	DR16���ջ�Э�������� ���DT7��ң�������ջ����ֲ�
 * @Param		RC_Type* rc���洢ң�������ݵĽṹ�塡��uint8_t* buff�����ڽ���Ļ���
 * @Retval		None
 * @Date
 */
/* 低通滤波系数，范围 (0, 1]，越小平滑程度越强，但响应延迟越大 */
#define RC_LOWPASS_ALPHA  0.15f

void Callback_RC_Handle(RC_Type *rc, uint8_t *buff)
{
    /* 静态变量保存滤波后的历史值，以及初始化标志 */
    static float filt_ch1 = 0.0f, filt_ch2 = 0.0f, filt_ch3 = 0.0f, filt_ch4 = 0.0f;
    static float filt_mouse_x = 0.0f, filt_mouse_y = 0.0f, filt_mouse_z = 0.0f;
    static uint8_t filter_active = 0;   /* 标记滤波器是否处于有效状态 */

    if (buff == NULL || rc == NULL)
    {
        return;
    }
    memcpy(RC_Data_Buffer, buff, 16);

    RC_dt = DWT_GetDeltaT(&RC_DWT_Count);

    /* -------- 解析原始数据(未滤波) -------- */
    int16_t raw_ch1 = (buff[0] | (buff[1] << 8)) & 0x07FF;
    raw_ch1 -= RC_CH_VALUE_OFFSET;
    int16_t raw_ch2 = ((buff[1] >> 3) | (buff[2] << 5)) & 0x07FF;
    raw_ch2 -= RC_CH_VALUE_OFFSET;
    int16_t raw_ch3 = ((buff[2] >> 6) | (buff[3] << 2) | (buff[4] << 10)) & 0x07FF;
    raw_ch3 -= RC_CH_VALUE_OFFSET;
    int16_t raw_ch4 = ((buff[4] >> 1) | (buff[5] << 7)) & 0x07FF;
    raw_ch4 -= RC_CH_VALUE_OFFSET;

    uint8_t sw_left  = ((buff[5] >> 4) & 0x000C) >> 2;
    uint8_t sw_right = (buff[5] >> 4) & 0x0003;

    int16_t raw_mouse_x = buff[6]  | (buff[7]  << 8);
    int16_t raw_mouse_y = buff[8]  | (buff[9]  << 8);
    int16_t raw_mouse_z = buff[10] | (buff[11] << 8);

    uint8_t raw_press_left  = buff[12];
    uint8_t raw_press_right = buff[13];
    uint16_t raw_key = buff[14] | (buff[15] << 8);

    /* -------- 有效数据滤波 / 无效数据清零 -------- */
    if (sw_left == 1 || sw_left == 2 || sw_left == 3)
    {
        if (!filter_active)   /* 由无效变有效，或首次进入有效状态 */
        {
            /* 直接用当前原始值初始化滤波器，避免从0缓慢上升 */
            filt_ch1 = (float)raw_ch1;
            filt_ch2 = (float)raw_ch2;
            filt_ch3 = (float)raw_ch3;
            filt_ch4 = (float)raw_ch4;

            filt_mouse_x = (float)raw_mouse_x;
            filt_mouse_y = (float)raw_mouse_y;
            filt_mouse_z = (float)raw_mouse_z;

            filter_active = 1;
        }
        else    /* 连续有效状态，执行低通滤波 */
        {
            /* 一阶低通滤波: Y(n) = Y(n-1) + α * [X(n) - Y(n-1)] */
            filt_ch1 += RC_LOWPASS_ALPHA * ((float)raw_ch1 - filt_ch1);
            filt_ch2 += RC_LOWPASS_ALPHA * ((float)raw_ch2 - filt_ch2);
            filt_ch3 += RC_LOWPASS_ALPHA * ((float)raw_ch3 - filt_ch3);
            filt_ch4 += RC_LOWPASS_ALPHA * ((float)raw_ch4 - filt_ch4);

            filt_mouse_x += RC_LOWPASS_ALPHA * ((float)raw_mouse_x - filt_mouse_x);
            filt_mouse_y += RC_LOWPASS_ALPHA * ((float)raw_mouse_y - filt_mouse_y);
            filt_mouse_z += RC_LOWPASS_ALPHA * ((float)raw_mouse_z - filt_mouse_z);
        }

        /* 将滤波后的值写入遥控器数据结构(舍入取整) */
        rc->ch1 = (int16_t)(filt_ch1 + 0.5f);
        rc->ch2 = (int16_t)(filt_ch2 + 0.5f);
        rc->ch3 = (int16_t)(filt_ch3 + 0.5f);
        rc->ch4 = (int16_t)(filt_ch4 + 0.5f);

        rc->switch_left  = sw_left;
        rc->switch_right = sw_right;

        rc->mouse.x = (int16_t)(filt_mouse_x + 0.5f);
        rc->mouse.y = (int16_t)(filt_mouse_y + 0.5f);
        rc->mouse.z = (int16_t)(filt_mouse_z + 0.5f);

        rc->mouse.press_left  = raw_press_left;
        rc->mouse.press_right = raw_press_right;
        rc->key_code = raw_key;
    }
    else    /* 开关不在有效档位，所有输出清零，并重置滤波器 */
    {
        rc->ch1 = 0;
        rc->ch2 = 0;
        rc->ch3 = 0;
        rc->ch4 = 0;
        rc->switch_left = 0;
        rc->switch_right = 0;

        rc->mouse.x = 0;
        rc->mouse.y = 0;
        rc->mouse.z = 0;

        rc->mouse.press_left = 0;
        rc->mouse.press_right = 0;

        rc->key_code = 0;

        /* 清零滤波器状态，下次有效时重新初始化 */
        filt_ch1 = 0.0f;   filt_ch2 = 0.0f;
        filt_ch3 = 0.0f;   filt_ch4 = 0.0f;
        filt_mouse_x = 0.0f; filt_mouse_y = 0.0f; filt_mouse_z = 0.0f;
        filter_active = 0;
    }

    // if (resetCount < 2000)
    //     Send_RC_Data(&hcan1, buff);
    RC_Update = 1;

    Detect_Hook(RC_TOE);
}
// void Callback_RC_Handle(RC_Type *rc, uint8_t *buff)
// {
//     if (buff == NULL || rc == NULL)
//     {
//         return;
//     }
//     memcpy(RC_Data_Buffer, buff, 16);

//     RC_dt = DWT_GetDeltaT(&RC_DWT_Count);

//     rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
//     rc->ch1 -= RC_CH_VALUE_OFFSET;
//     rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
//     rc->ch2 -= RC_CH_VALUE_OFFSET;
//     rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
//     rc->ch3 -= RC_CH_VALUE_OFFSET;
//     rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
//     rc->ch4 -= RC_CH_VALUE_OFFSET;

//     rc->switch_left = ((buff[5] >> 4) & 0x000C) >> 2;
//     rc->switch_right = (buff[5] >> 4) & 0x0003;

//     rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
//     rc->mouse.y = buff[8] | (buff[9] << 8);
//     rc->mouse.z = buff[10] | (buff[11] << 8);

//     rc->mouse.press_left = buff[12]; // is pressed?
//     rc->mouse.press_right = buff[13];

//     rc->key_code = buff[14] | buff[15] << 8; // key borad code

//     if (rc->switch_left != 1 && rc->switch_left != 2 && rc->switch_left != 3)
//     {
//         rc->ch1 = 0;
//         rc->ch2 = 0;
//         rc->ch3 = 0;
//         rc->ch4 = 0;
//         rc->switch_left = 0;
//         rc->switch_right = 0;

//         rc->mouse.x = 0; // x axis
//         rc->mouse.y = 0;
//         rc->mouse.z = 0;

//         rc->mouse.press_left = 0; // is pressed?
//         rc->mouse.press_right = 0;

//         rc->key_code = 0; // key borad codes
//     }

//     // if (resetCount < 2000)
//     //     Send_RC_Data(&hcan1, buff);
//     RC_Update = 1;

//     Detect_Hook(RC_TOE);
// }

void Solve_RC_Lost(void)
{
    USART_IDLE_Init(remote_control.RC_USART, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

void Solve_RC_Data_Error(void)
{
    USART_IDLE_Init(remote_control.RC_USART, sbus_rx_buf, SBUS_RX_BUF_NUM);
}

uint8_t RC_Data_is_Error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
    if (abs(remote_control.ch1) > 1000)
    {
        goto error;
    }
    if (abs(remote_control.ch2) > 1000)
    {
        goto error;
    }
    if (abs(remote_control.ch3) > 1000)
    {
        goto error;
    }
    if (abs(remote_control.ch4) > 1000)
    {
        goto error;
    }
    if (remote_control.switch_left == 0)
    {
        goto error;
    }
    if (remote_control.switch_right == 0)
    {
        goto error;
    }
    return 0;

error:
    remote_control.ch1 = 0;
    remote_control.ch2 = 0;
    remote_control.ch3 = 0;
    remote_control.ch4 = 0;

    remote_control.mouse.x = 0;
    remote_control.mouse.y = 0;
    remote_control.mouse.z = 0;
    remote_control.switch_left = 0;
    remote_control.switch_right = 0;

    remote_control.mouse.x = 0; // x axis
    remote_control.mouse.y = 0;
    remote_control.mouse.z = 0;

    remote_control.mouse.press_left = 0; // is pressed?
    remote_control.mouse.press_right = 0;

    remote_control.key_code = 0; // key borad code
    return 1;
}
