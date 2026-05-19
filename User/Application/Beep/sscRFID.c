#include "sscRFID.h"
#include "kernel_scheduler.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <string.h>
#include "uart3.h"
#include "sscBEEP.h"
#include "delay.h"
//宏定义数据长度
#define RF_BUFF_LENGTH 50
#define Frame_header 0xBB
#define Frame_header_Second 0x01
#define Frame_read 0x39
#define Frame_write 0x49
#define Frame_read_write_file 0xff
#define Frame_tail 0x7e
//宏定义接口A

#define INTERFACE 2
static uint16_t tool_compare_data[INTERFACE];
typedef enum {
    STATE_SEARCH_HEADER,
    STATE_VERIFY_LENGTH,
    STATE_CHECK_CRC,
    STATE_PROCESS_DATA
} ParserState;

//定义一个结构体，用于储存标签信息（初始速度、最大速度、初始频率、最大频率、初始流量、最大流量、减速比、刀具类型、使用次数）
typedef struct {

    uint8_t speed; //初始速度
    uint8_t maxSpeed; //最大速度
    uint8_t frequency; //初始频率
    uint8_t maxFrequency; //最大频率
    uint8_t flow; //初始流量
    uint8_t maxFlow; //最大流量
    uint8_t reductionRatio; //减速比（用于计算实际速度）
    uint8_t toolType; //刀具类型
    uint8_t useCount; //使用次数
    uint8_t tool_compare_data;//刀具比较值
    uint8_t read_write_bit; //标签ID
}
TagInfo;
//定义一个taginfo数组
static TagInfo tagInfo[2];//A、B两个接口
static uint8_t NO_MASK3_WRITE_EPC[7] = {0XBB, 0X00 ,0X22 ,0X00 ,0X00 ,0X22 ,0X7E };//读取EPC
static uint8_t NO_MASK3_READ_USER[16] = {0xBB, 0x00, 0x39, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x08, 0x4d, 0x7E}; //无掩码读取USER
static unsigned char hop_ch[]={0XBB ,0X00 ,0XAD ,0X00 ,0X01 ,0XFF, 0XAD ,0X7E };//使用跳屏
static unsigned char hop_return[]={0xBB ,0x01 ,0xAD ,0X00 ,0x01 ,0x00 ,0xAF ,0X7E};//跳屏返回码
static unsigned char stop_hop_ch[]={0XBB ,0X00 ,0XAD ,0X00, 0X01, 0x00 ,0xAE ,0X7E};//停止跳屏

static unsigned char  pa_gain15[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X05, 0XDC, 0Xea, 0X7E};//发射功率
static unsigned char  pa_gain10[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X03, 0Xe8, 0Xa3, 0X7E};//发射功率
static unsigned char  pa_gain13[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X05, 0X14, 0XE5, 0X7E}; //发射功率
static unsigned char  pa_gain0[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X00, 0X00, 0XB8, 0X7E}; //发射功率
static unsigned char  pa_gain5[]={0XBB, 0X00, 0XB6, 0X00, 0X02, 0X01, 0XF4, 0XAD, 0X7E}; //
static unsigned char region_set_europe[]={0XBB ,0x00 ,0x07 ,0x00 ,0x01 ,0x03 ,0x0B ,0x7E};//欧洲频段865.1-867.9M
static unsigned char region_set_us[]={0XBB ,0x00 ,0x07 ,0x00 ,0x01 ,0x02 ,0x0A ,0x7E};//美国频段902.25-927.75M
static unsigned char region_set_CHAIN[]={0XBB ,0x00 ,0x07 ,0x00 ,0x01 ,0x01 ,0x09 ,0x7E};//中国1-920.125-924.875M
static unsigned char region_set_CHAINS[]={0XBB ,0x00 ,0x07 ,0x00 ,0x01 ,0x04 ,0x0C ,0x7E};//中国2-840.125-844.875M
static unsigned char region_set_K[]={0XBB ,0x00 ,0x07 ,0x00 ,0x01 ,0x06 ,0x0E ,0x7E};//韩国-917.1-923.3M
static kernel_task_t AUTOMODEGETDATATaskHandle;
//task_t AUTOMODEREADDATATaskHandle;
static kernel_task_t CUTTERSCANTaskHandle;




// 添加RfidHandle函数的声明


//创建手柄识信息队列
static QueueHandle_t RFIDMsgQueue = NULL;
//创建手柄识信息类型
typedef struct {
    uint8_t Channel_Number;         //手柄通道1表示A通道，2表示B通道
    bool Start_flag;             //ture开始识别，false关闭识别
    uint8_t rfid_type;//识别类型，通过判断手柄寄存器的值判断是否是公共接头还是其他（读取user还是epc）
} RFIDMessage_t;

//识别通道类型定义类型定义
#define RFID_MSG_A    1  //A通道
#define RFID_MSG_B  2   //B通道

//发送按键蜂鸣器消息
 void SendKeyRFIDMessageAup(uint8_t rfid_data)
{
    static uint8_t last_rfid_data=0;
    if(RFIDMsgQueue == NULL) return;
    if(last_rfid_data!=rfid_data)
    {
        RFIDMessage_t msg;
        msg.Channel_Number= RFID_MSG_A;
        msg.Start_flag=true;
        msg.rfid_type=rfid_data;//读取user
        (void)Kernel_QueueSend(RFIDMsgQueue, &msg, pdMS_TO_TICKS(0));
        last_rfid_data=rfid_data;
    }

}
 void SendKeyRFIDMessageAdown()
{
    // if(RFIDMsgQueue == NULL) return;
    // RFIDMessage_t msg;
    // msg.Channel_Number= RFID_MSG_A;
    // msg.Start_flag=false;
    // (void)Kernel_QueueSend(RFIDMsgQueue, &msg, pdMS_TO_TICKS(0));
}
 void SendKeyRFIDMessageBup(uint8_t rfid_data)
{
    // if(RFIDMsgQueue == NULL) return;
    // RFIDMessage_t msg;
    // msg.Channel_Number= RFID_MSG_B;
    // msg.Start_flag=true;
    // msg.rfid_type=rfid_data;//读取user还是epc
    // (void)Kernel_QueueSend(RFIDMsgQueue, &msg, pdMS_TO_TICKS(0));
}

 void SendKeyRFIDMessageBdown()
{
    // if(RFIDMsgQueue == NULL) return;
    // RFIDMessage_t msg;
    // msg.Channel_Number= RFID_MSG_B;
    // msg.Start_flag=false;
    // (void)Kernel_QueueSend(RFIDMsgQueue, &msg, pdMS_TO_TICKS(0));
}


void SscRadioFreq_Init(void)
{
  Uart3_SendPacket(pa_gain0, 9);  //功率设置
  Delay_ms(50);

  Uart3_SendPacket(region_set_CHAIN, 8);  //区域设置
  Delay_ms(50);

  Uart3_SendPacket(hop_ch, 8);//跳频hop_ch，取消跳频stop_hop_ch
  Delay_ms(50);
}
//初始化蜂鸣器消息队列
static void RFIDQueue_Init(void)
{
    RFIDMsgQueue = Kernel_QueueCreate(4, sizeof(RFIDMessage_t), "RFIDMsgQueue");
}

//80ms发送一次
static void SplitType_AutoModeGetData_Task(void)         //Check_Connect(void)
{
    uint16_t rlen;
    uint8_t dat[UART3_MAX_PACKET_SIZE] = { 0 };
    static uint8_t rfid_channel = 0;
	static uint8_t rfid_start_flag = 0;
    static uint8_t fail_times = 0;
    static uint8_t rfid_type=0;
    //
    if(RFIDMsgQueue!=NULL)
    {
        RFIDMessage_t msg;
        if(Kernel_QueueReceive(RFIDMsgQueue, &msg, 0) == pdTRUE)
        {
            rfid_channel=msg.Channel_Number;
            rfid_start_flag=msg.Start_flag;
            rfid_type=msg.rfid_type;
        }
    }
    if(rfid_start_flag)
    {
        fail_times++;
        if(fail_times%2)
        {
            // Uart3_DMARecvDataPeek(dat);
            // //判断是否公共接头，后续还需要跳线（相线->识别线），结束以后,默认跳回相线
            // if(rfid_type==0)
            // {
            //  // Uart3_SendPacket(NO_MASK3_READ_USER, 16);//无掩码读取USER区域数据
            //     Uart3_SendPacket(NO_MASK3_WRITE_EPC, 7);//无掩码读取EPC区域数据
            // }
            // else if(rfid_type==1)
            // {
            //   Uart3_SendPacket(NO_MASK3_WRITE_EPC, 7);//无掩码读取EPC区域数据
            // }
       }
        else
        {
            //不发送
            rlen = Uart3_DMARecvDataPeek(dat);
            Uart3_SendPacket(NO_MASK3_WRITE_EPC, 7);//无掩码读取EPC区域数据
                if (rlen < 7)   //不够一个数据包大小
                return ;
            RfidHandle(dat,rfid_channel,true,rfid_type);
             //  SendKeyBeepMessage(1U);
        }
    }
}


static void AUTOMODEGETDATATaskFunc(uint32_t event)
{
  /* USER CODE BEGIN AUTOMODEGETDATATaskFunc */
  /* Infinite loop */
  SplitType_AutoModeGetData_Task();
  /* USER CODE END AUTOMODEGETDATATaskFunc */
}

/**
 * @brief Function implementing the Time thread.
 * @param argument: Not used
 * @retval None 48
 */
//============================================================================
void SscSplitTypeAutoModeGetData_Init(void)
{
    RFIDQueue_Init();
  /* definition and creation of AUTOMODEGETDATATask */
	Kernel_TaskCreate(&AUTOMODEGETDATATaskHandle, AUTOMODEGETDATATaskFunc);
	Kernel_TaskStart(&AUTOMODEGETDATATaskHandle, KERNEL_TASK_ALWAYS, 100);
}


static uint8_t RadioFreq_CRC(uint8_t *data, uint8_t length, uint8_t startposition)
{
  int total = 0;
  uint16_t i = 0;

  for(i = 0; i < length; i++)
  {
	  total += data[startposition + i];
  }

  if (data[length + startposition] == ((uint8_t)(total) & 0xff))
  {
    return 1;
  }

  return 0;
}







/**
 * @brief 处理RFID数据的函数
 * @param uartx_rf_buff 指向UART1接收缓冲区的指针
 * @return 返回处理结果：0表示失败，1表示写操作，2表示读操作，3表示读写文件操作
 * //BB 02 22 00 11 CB 34 00 "E2 00 47 1B B0 00 60 13 04 45 09 D0" 95 82” D4 7E 当读取EPC的时候，需要解析（只有pxbapxbb读取USER，其他读取epc,数据部分12个字节），data[4]=epc长度-固定位17，95-82CRC
 */
//rfid用来解析内容，根据rfid_type判断是读取user还是epc，还是其他操作（后续增加），enable_rfid用来控制是否启用rfid识别功能，interface用来区分A、B通道
void RfidHandle(uint8_t * uartx_rf_buff, uint8_t interface, bool enable_rfid, uint8_t rfid_type)
{
    ParserState state = STATE_SEARCH_HEADER;  // 解析状态机初始状态为搜索帧头
    uint16_t i = 0;  // 缓冲区索引
    uint8_t start_pos = 0;  // 帧起始位置
    uint8_t write_read_data = 0;
    static uint8_t read_fail_times = 0;

    if(!enable_rfid){
        read_fail_times = 0;
        return;
    }

if(uartx_rf_buff[i]==0xbb&&uartx_rf_buff[i+1]==0x02)
{
     SendKeyBeepMessage(1);
}

    // 遍历缓冲区处理数据
    while (i < RF_BUFF_LENGTH) {
        switch (state) {
            case STATE_SEARCH_HEADER:
                // 检查帧头 (0xBB 0x01 0x39/0x3A/0x3B)
                if (uartx_rf_buff[i] == Frame_header &&
                    i + 2 < RF_BUFF_LENGTH &&
                    uartx_rf_buff[i+1] == Frame_header_Second) {
                    start_pos = i;  // 记录帧起始位置
                    state = STATE_VERIFY_LENGTH;  // 转移到验证长度状态
                    i += 2; // 跳过已检查的字节
                } else {
                    i++;
                }
                break;

            case STATE_VERIFY_LENGTH:
                // 检查是否到达帧尾
                if (i < RF_BUFF_LENGTH && uartx_rf_buff[i] == Frame_tail) {
                    // 确保有足够的数据来获取长度信息
                    if (start_pos + 4 >= RF_BUFF_LENGTH) {
                        state = STATE_SEARCH_HEADER;  // 没有足够的数据，重新开始
                        i++;
                        break;
                    }

                    // 验证长度字段
                    uint8_t expected_len = uartx_rf_buff[start_pos + 4];  // 期望的数据长度
                    // 确保计算不会下溢
                    if (i <= start_pos + 6) {
                        state = STATE_SEARCH_HEADER;  // 实际长度小于最小帧长，重新开始
                        i++;
                        break;
                    }
                    uint8_t actual_len = i - start_pos - 6;  // 实际的数据长度

                    // 比较期望长度和实际长度
                    if (expected_len == actual_len) {
                        state = STATE_CHECK_CRC;  // 长度正确，进入CRC校验状态
                    } else {
                        state = STATE_SEARCH_HEADER;  // 长度错误，重新搜索帧头
                        i = start_pos + 1; // 从下一个字节开始搜索
                    }
                } else {
                    i++;
                }
                break;

            case STATE_CHECK_CRC:
                // 确保有足够的数据用于CRC校验
                if (start_pos + uartx_rf_buff[start_pos + 4] + 4 >= RF_BUFF_LENGTH) {
                    state = STATE_SEARCH_HEADER;  // 数据不足，重新开始
                    i = start_pos + 1;
                    break;
                }

                // 校验CRC值
                if (RadioFreq_CRC(uartx_rf_buff,
                               uartx_rf_buff[start_pos + 4] + 4,   // 数据长度+4（包含帧头和长度字段）
                               start_pos + 1)) {  // 从第二个字节开始校验
                    state = STATE_PROCESS_DATA;  // CRC校验成功，进入数据处理状态
                } else {
                    state = STATE_SEARCH_HEADER;  // CRC校验失败，重新搜索帧头
                    i = start_pos + 1;
                }
                break;

            case STATE_PROCESS_DATA:
                // 将数据转换为RfidFrame结构体
                write_read_data = uartx_rf_buff[start_pos+2];

                // 确保数组访问不会越界
                if (start_pos + 15 >= RF_BUFF_LENGTH) {
                    state = STATE_SEARCH_HEADER;
                    break;
                }



                // 根据命令类型处理数据
                switch (write_read_data) {
                    case Frame_write:
                       break;
                    case Frame_read:
                       SendKeyBeepMessage(1U);
                        // 检查数据长度是否足够拷贝TagInfo
                        if (start_pos + sizeof(TagInfo) - 1 < RF_BUFF_LENGTH)
                        {
                            read_fail_times = 0;
                            if(tool_compare_data[interface] == (uartx_rf_buff[start_pos+14]<<8 | uartx_rf_buff[start_pos+15]))
                            {
                                //  SendKeyBeepMessage(100);
                             //什么也不做，或许判断相声
                            }
                            else{
                                // SendKeyBeepMessage(100);
                            tool_compare_data[interface] = uartx_rf_buff[start_pos+14]<<8 | uartx_rf_buff[start_pos+15];
                            memcpy(&tagInfo[interface], &uartx_rf_buff[start_pos], sizeof(TagInfo) - 1);
                            }
                            // 安全拷贝

                            //队列通知更新界面（规格，角度，长度显示）？？？

                        }
                        tagInfo[interface].read_write_bit = 2;
                        break;
                    case Frame_read_write_file:
                         tagInfo[interface].read_write_bit = 3;  // 读写文件失败操作标志
                         read_fail_times++;
                        break;
                    case 0xee:
                        read_fail_times = 0;

                        tagInfo[interface].read_write_bit = 0xcc;//正确读取，但是同样刀具
                        break;
                    default:
                        read_fail_times++;
                        tagInfo[interface].read_write_bit = 0;  // 未知命令
                        break;
                }

                // 处理完成后，跳转到下一帧的可能位置，并重新开始搜索
                i = start_pos + uartx_rf_buff[start_pos + 4] + 6; // 跳到当前帧之后
                state = STATE_SEARCH_HEADER; // 重置状态机
                break;
        }
    }

    if(read_fail_times > 10)
    {
      //通知界面显示刀具图片
      tool_compare_data[interface] = 0;//比较值清零
      read_fail_times = 0;
    }

}
