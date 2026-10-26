#include <sys/ioctl.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include "jSerial.h"
#include "ite/ug.h"
#include "ite/itp.h"
#include "ite/itu.h"

uint8_t  BluetoothLayerFocus = 0;  //P22专有，蓝牙界面按键焦点
uint8_t  MusicLayerFocus = 0;  //P22专有，音乐界面按键焦点
uint8_t  TelePhoneLayerFocus = 0;  //P22专有，电话界面按键焦点
uint8_t  SubsupensionState =0;  //P22专有，减震状态
uint8_t  ModekeyState = 0;  //P22专有，道路模式按键
uint8_t  SelftestMode = 1;   //P22专有，自检模式，1为自检中，0为退出
uint8_t  SelftestSendToMcu = 0;   //P22专有，需要同步发送信息给MCU
uint32_t self_test_tick = 0;  //自检计数器
uint8_t  McuUpgradeProcessShow =0;   //P22专有，MCU的升级进度
int key_send = 0;  //发送给mcu按键值
uint8_t jl_phone_state = 0;
uint8_t NF_PIN12_STATE = 1;
uint8_t EnableFlashDvrPhoto = 0;
uint8_t set_index = 0;

uint32_t SendMCU_Light_Flag = 0;//发送给mcu 灯的状态

uint8_t activation_state = 0;//发送给mcu 激活的状态

int jl_time_year_high = 0;
int jl_time_year_low = 0;
int jl_time_month = 0;
int jl_time_day = 0;
int jl_time_hour = 0;
int jl_time_minute = 0;
int jl_time_sec = 0;
int jl_sys_time_flag = 0;
uint8_t JL_dvr_state = 0;

int os_time_year_high = 0;
int os_time_year_low = 0;
int os_time_month = 0;
int os_time_day = 0;
int os_time_hour = 0;
int os_time_minute = 0;
int os_time_sec = 0;
extern int jce_ui_ver_high;
extern int jce_ui_ver_low;

uint8_t JL_screen_state = 0;
bool bSign_screen_state = 0;
uint8_t JL_DriveTime = 0;
uint8_t FaultCode_7205 = 1;
int bSendDateto7205 = 0;
int bCurrStateUVC = 0;	//7205心跳新增字节，7205对于dvr和投屏的双重保险
int bAppConnected = -1;
int Day[13] = { 0,31,28,31,30,31,30,31,31,30,31,30,31 };
///////////屏外灯///////////////
int coutLeftIcon = 0, coutRightIcon = 0, coutABSIcon = 0, coutBeamIcon = 0, coutAutoSspIcon = 0, coutMotorIcon = 0, coutPLIcon = 0,coutEOIcon=0, coutPSIcon = 0,coutTRUBLEIcon, coutoil_LEDIcon;

tyre_status_t tyre_setStarus[2];
extern uint8_t JL_ver_high;
extern uint8_t JL_ver_low;
tyre_save_value_t tyre_bt_Value[2] = { {{0, 0, 0, 0, 0, 0}, 0, 0, 0, {0, 0}},
                                   {{0, 0, 0, 0, 0, 0}, 0, 0, 0, {0, 0}} };
int bt_start_learn[2] = {0};
int bt_clean_learn;

#if !defined(_WIN32)    // windos 上不能用ITE的串口哦
int   UPDATE_MSG_BUFFER_SIZE = 4096;//定义一个消息的最大长度,目前使用82
int  mculength = 4096;  //默认mcu数据长度128
uint8_t serialport_buffer[4096 + 20] = {0};
mcu_Update_ST_t mcu_UpdateSt;//发送mcu升级文件数据
UPDATE_RECEIVE_BUF Update_pro_receive_buf;
uint8_t mcu_data_update[256 * 1024] = { 0 };  //MCU升级数据, 最多4k 一包
uint8_t length_state = 0;
sem_t UartSem;
void UartCallback(void* arg1, uint32_t arg2)
{
	//don`t add any codes here.
	sem_post(&UartSem);
}

bool checkMcuUpgrade(){
	return isMcuUpgrade;
}
void setMcuUpgradeState(bool bo) {
	isMcuUpgrade = bo;
	if (bo)
		printf(">>>>> McuUpgradeState  \n");
}
void SerialPort_MSG_decode(uint8_t *RecData, int mculen)
{
	/*for(int i =0;i<mculength;i++ ){
		printf("%x;", RecData[i]);
	}
	printf("\r\n");*/
	//printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>SerialPort_MSG_decode len = %d\n",mculen);

	/*把串口数据 传给库处理*/
	jceLoadMcuData(RecData, mculen);

	/*以下的几种方式 测试，取值 是OK的 */

	/** 串口测试OK **/
}
static int prelong_times = 0;
void SendTick() {
	static int send_flag = 0;
	static int jl_flag = 0;
	uint8_t tick[48] = { 0 };
	tick[0] = 0x55;
	tick[1] = 0xae;  //0x00,0x00,0x00,0x02,0x00
	tick[2] = 0x00;
	tick[3] = 0x00;
	tick[4] = 0x00;
	//appendlen
	tick[5] = (sizeof(tick) / sizeof(tick[0])) - 8;  // appendlen
	tick[6] = 0x00;  //headchecksum
	for (int i = 0; i < 4; i++)
	{
		tick[6] = tick[6] + tick[i + 2];
	}
	tick[8] = JL_screen_state;    //投屏状态
	tick[9] = SendMCU_Light_Flag;    //各种灯的显示状态.bit0:左转;bit1:右转;bit2:机油压力;bit3:示廓;bit4:远光;bit5:发动机故障;
	tick[10] = 0;    //当前音乐状态 ，枚举待定
	tick[11] = JL_ver_high;    //当前蓝牙界面焦点， 1为请求焦点到OS，此时MCU不应将界面切走，除非断电， 0为释放焦点此时MCU可按键切走
	tick[12] = JL_ver_low;    //当前电话界面焦点， 1为请求焦点到OS，此时MCU不应将界面切走，除非断电， 0为释放焦点此时MCU可按键切走
	tick[13] = TelePhoneLayerFocus;    //当前音乐界面焦点， 1为请求焦点到OS，此时MCU不应将界面切走，除非断电， 0为释放焦点此时MCU可按键切走
	tick[14] = SubsupensionState;   //P22专有，减震状态
	tick[15] = activation_state; //发送给mcu 激活状态

	tick[16] = 0; //P22专有，自检模式，1为自检中，0为退出
	if (JL_dvr_state) {
		tick[16] |= 0x01;
	}

    tick[17] = jl_sys_time_flag; //mcu收到1再记录时间
  //  printf("#### tick[17]jl_sys_time_flag=%d  \n", tick[17]);

    tick[18] = jl_time_year_high; //年高位
    tick[19] = jl_time_year_low; //年低位
    tick[20] = jl_time_month; //月
    tick[21] = jl_time_day; //日
    tick[22] = jl_time_hour; //时
    tick[23] = jl_time_minute; //分
    tick[24] = jl_time_sec; //秒
    tick[25] = 0;
	/*
 * tick[25] bit0 时间同步触发规则：
 * 1. 系统刚启动时，连续发送5帧 bit0 = 1；
 * 2. 发送完5帧后，bit0恢复为0；
 * 3. 等待1分钟；
 * 4. 再连续发送5帧 bit0 = 1；
 * 5. 重复以上流程。
 */
	{
		/* 上一次完成5次发送的时间，单位：毫秒 */
		static uint32_t last_time_sync_tick = 0;

		/* 当前还需要发送多少次 bit0 = 1；上电默认发送5次 */
		static int time_sync_send_count = 5;

		/* 获取系统当前运行时间，单位：毫秒 */
		uint32_t now_tick = SDL_GetTicks();

		/* 第一次进入时，记录初始时间 */
		if (last_time_sync_tick == 0)
		{
			last_time_sync_tick = now_tick;
		}

		/* 当前处于连续发送5次的阶段 */
		if (time_sync_send_count > 0)
		{
			/* tick[25] 的 bit0 置1，通知MCU接收并记录当前时间 */
			tick[25] |= 0x01;

			/* 已完成一次发送，剩余发送次数减1 */
			time_sync_send_count--;

			/* 第5次发送完成，从当前时间开始计算下一分钟 */
			if (time_sync_send_count == 0)
			{
				last_time_sync_tick = now_tick;
			}
		}
		/* 距离上次发送完成已经达到1分钟 */
		else if ((uint32_t)(now_tick - last_time_sync_tick) >= 60U * 1000U)
		{
			/* 开始新一轮连续发送5次 */
			time_sync_send_count = 5;

			/* 本帧立即发送第1次 */
			tick[25] |= 0x01;
			time_sync_send_count--;

			/* 更新时间基准，防止重复进入 */
			last_time_sync_tick = now_tick;
		}
	}
	if (jl_sys_time_flag > 0)
	{
		tick[25] |= 0x01;
		jl_flag = jl_flag + 1;
		if (jl_flag > 3)
		{
			jl_flag = 0;
			jl_sys_time_flag = 0;
		}
		/*printf("jl_time_year_highss = %d\n", tick[18]);
		printf("jl_time_year_lowss = %d\n", tick[19]);
		printf("jl_time_monthss = %d\n", tick[20]);
		printf("jl_time_dayss = %d\n", tick[21]);
		printf("jl_time_hourss = %d\n", tick[22]);
		printf("jl_time_minutess = %d\n", tick[23]);
		printf("jl_time_secss = %d\n", tick[24]);*/
	}

    tick[9] = 0;			//发给MCU灯的信息

	if (coutLeftIcon > 0) {
		tick[9] |= 0x01;
		printf("**********coutLeftIcon=%d***************\n", tick[9]);
	}
	if (coutRightIcon > 0) {
		tick[9] |= 0x02;
	}

    if (coutPSIcon > 0) {	
        tick[9] |= 0x08;
    }
    if (coutBeamIcon > 0) {
        tick[9] |= 0x10;
    }
    
    if (coutAutoSspIcon > 0) {
        tick[9] |= 0x40;
    }
    if (coutABSIcon == 0) {
        tick[9] |= 0x80;
    }

	//printf("******tick[9]=  0x%02x\n******", tick[9]);
    tick[26] = jl_phone_state; //电话界面
    if(jl_phone_state!=0){
        printf("jl_phone_state=  0x%02x\n******", jl_phone_state);
        
    }

    tick[27] = tyre_bt_Value[0].tyre_address[0];//胎压id 压力
    tick[28] = tyre_bt_Value[0].tyre_address[1];
    tick[29] = tyre_bt_Value[0].tyre_address[2];
    tick[30] = tyre_bt_Value[0].tyre_address[3];
    tick[31] = tyre_bt_Value[0].tyre_address[4];
    tick[32] = tyre_bt_Value[0].tyre_address[5];
    tick[33] = tyre_bt_Value[0].tyre_pressure[0];
    tick[34] = tyre_bt_Value[0].tyre_pressure[1];
    tick[35] = tyre_bt_Value[0].tyre_temperature;
	if (tick[27] || tick[28] || tick[29] || tick[30] || tick[31] || tick[32]) {
		tick[36] = tyre_bt_Value[0].tyre_status;
	}
	else {
		tick[36] = 0;
	}
    
	//printf("\nSend Fornt Id : %02x%02x", tyre_bt_Value[0].tyre_pressure[0], tyre_bt_Value[0].tyre_pressure[1]);
	//printf("%02x%02x", tyre_bt_Value[0].tyre_pressure[2], tyre_bt_Value[0].tyre_pressure[3]);
	//printf("%02x%02x\n", tyre_bt_Value[0].tyre_pressure[4], tyre_bt_Value[0].tyre_pressure[5]);
//	printf("Send Muc Front Tire Status = 0x%02x\n", tyre_bt_Value[0].tyre_status);

    tick[37] = tyre_bt_Value[1].tyre_address[0];
    tick[38] = tyre_bt_Value[1].tyre_address[1];
    tick[39] = tyre_bt_Value[1].tyre_address[2];
    tick[40] = tyre_bt_Value[1].tyre_address[3];
    tick[41] = tyre_bt_Value[1].tyre_address[4];
    tick[42] = tyre_bt_Value[1].tyre_address[5];
    tick[43] = tyre_bt_Value[1].tyre_pressure[0];
    tick[44] = tyre_bt_Value[1].tyre_pressure[1];
    tick[45] = tyre_bt_Value[1].tyre_temperature;
	if (tick[37] || tick[38] || tick[39] || tick[40] || tick[41] || tick[42]) {
		tick[46] = tyre_bt_Value[1].tyre_status;
	}
	else {
		tick[46] = 0;
	}
	tick[47] = set_index;
	tick[7] = 0;  //appendchecksum
	for (int i = 0; i < tick[5]; i++)//获得appendchecksum
	{
		tick[7] = tick[7] + tick[i + 8]; //从第9个开始拼
	}

    // printf("tick[47] = %d  \n", tick[47]);
	//printf("\nSend Rear Id : %02x%02x", tyre_bt_Value[1].tyre_pressure[0], tyre_bt_Value[1].tyre_pressure[1]);
	//printf("%02x%02x", tyre_bt_Value[1].tyre_pressure[2], tyre_bt_Value[1].tyre_pressure[3]);
	//printf("%02x%02x\n", tyre_bt_Value[1].tyre_pressure[4], tyre_bt_Value[1].tyre_pressure[5]);
	//printf("Send Mcu Rear Tire Status = 0x%02x\n", tyre_bt_Value[1].tyre_status);
   // printf("#### tick[37]ID : %02x  \n", tick[37]);


   // printf("000000000key_send = %d  \n", key_send);
    /*
    


    for (int i = 0; i < 26; i++)//获得appendchecksum
    {
        printf("tick[%d] = %d\n",i, tick[i]);
    }
    */
    /*
	static int FirstMustSelfTest = 1;
	if (SelftestSendToMcu ==1) {
		FirstMustSelfTest = 0;
	}
	if (!FirstMustSelfTest) {  //第一次发送必现是自检，否则MCU有异常
		write(MCU_PORT, tick, sizeof(tick) / sizeof(tick[0]));
	}
    */
    write(MCU_PORT, tick, sizeof(tick) / sizeof(tick[0]));
}
void Send_wifiupdate_mcu()
{
	uint8_t in[] = { 0x55,0xAE,0x00,0x0A,0x07,0x00,0x11,0x00 };//wifi 下载完 MCU文件后，进入升级所要发送的报文
	int siz = sizeof(in) / sizeof(in[0]);

    McuUpgradeProcessShow = 1;  //通知UI，要弹窗显示进度条

	write(MCU_PORT, in, siz);
	printf("##########################################################    Send: CMD_REBOOT\n");
	printf("##########################################################    Send: CMD_REBOOT\n");
	printf("##########################################################    Send: CMD_REBOOT\n");
	printf("##########################################################    Send: CMD_REBOOT\n");
}

void SerialPort_sendTranRead(int data, int len, uint8_t* filedata, uint8_t ch1, uint8_t ch2, uint8_t ch3, uint8_t ch4)
{
	// TODO SerialPort_sendTranRead
	//15+len
	memset(serialport_buffer, 0, sizeof(serialport_buffer));
	serialport_buffer[0] = 0x55;
	serialport_buffer[1] = 0xAE;
	serialport_buffer[2] = 0;
	serialport_buffer[3] = 10;
	serialport_buffer[4] = data;
	serialport_buffer[5] = (len + 7);//文件数据长度，加上地址等7个字节
	serialport_buffer[6] = 0;//有效字节累加
	//serialport_buffer[6] = serial + cmd + data + appendLen
	for (int i = 0;i < 4;i++)
	{
		serialport_buffer[6] = serialport_buffer[6] + serialport_buffer[i + 2];
	}

	serialport_buffer[8] = 2;  //状态 2 ok, 1 error
	serialport_buffer[9] = len;    //长度
	serialport_buffer[10] = (len >> 8);  //长度高字节
	serialport_buffer[11] = ch1;  // 起始地址  低字节
	serialport_buffer[12] = ch2;  //
	serialport_buffer[13] = ch3;   //
	serialport_buffer[14] = ch4;   //起始地址  高字节

	for (int i = 0; i < len; i++) {
		serialport_buffer[15 + i] = filedata[i];
	}
	serialport_buffer[7] = 0;
	for (int i = 0; i < len + 7; i++)//获得appendchecksum
	{
		serialport_buffer[7] = serialport_buffer[7] + serialport_buffer[i + 8]; //从第9个开始拼
	}
	/*通过串口发送给MCU, 这里要注意是异步发送，等待线程中发送完成需要时间*/
	write(MCU_PORT, serialport_buffer, len + 15);
}
void SerialPort_sendTransOpen(int data, int filelen)
{
	//serialport_buffer.resize(8+5);  //协议定义了open情况下append_len = 5,加上协议头共13个byte
	memset(serialport_buffer, 0, sizeof(serialport_buffer));
	serialport_buffer[0] = 0x55;
	serialport_buffer[1] = 0xAE;
	serialport_buffer[2] = 0;
	serialport_buffer[3] = 10;      //
	serialport_buffer[4] = data;     //
	serialport_buffer[5] = 5;//数据长度
	serialport_buffer[6] = 0;//有效字节累加
	for (int i = 0;i < 4;i++)
	{
		serialport_buffer[6] = serialport_buffer[6] + serialport_buffer[i + 2];
	}
	serialport_buffer[8] = 4;   //打开成功
	serialport_buffer[9] = (filelen);
	serialport_buffer[10] = (filelen >> 8);
	serialport_buffer[11] = (filelen >> 16);
	serialport_buffer[12] = (filelen >> 24);
	serialport_buffer[7] = 0;//appendchecksum
	for (int i = 0;i < 5;i++)
	{
		serialport_buffer[7] = serialport_buffer[7] + serialport_buffer[i + 8];
	}
	/*通过串口发送给MCU, 这里要注意是异步发送，等待线程中发送完成需要时间*/
	write(MCU_PORT, serialport_buffer, 8 + 5);
}
static FILE *fdMcu;   //mcu文件描述符
uint32_t checkMcuBinState(FILE *fp) {
	if (fp == NULL) {
		printf("[jcelib] Mcu.bin not exists \n");
		return 0;
	}
	else {
		if (fseek(fp, 0, SEEK_END) == 0 && ftell(fp) == 0) {
			printf("[jcelib] Mcu.bin  size 0\n");
			return 0;
		}
		else {
			printf("[jcelib]  Mcu.bin  exists !\n");
			return ftell(fp);
		}
	}
	return 0;
}
void analyMcuUpdate(unsigned char *RecData, int bytes)
{
	uint8_t upgrade_work = RecData[4];
	uint8_t upgrade_data_len = RecData[5];  //长度
	static uint32_t SendDataLen = 0;
	if (bytes < 8 + upgrade_data_len)
	{
		printf(" <<##mcu##>> Error analyMcuUpdate  bytes < 8 + upgrade_data_len \n");
		return;
	}
	//*******************************************************************
	//进入到这里，UI 要直接弹窗显示MCU 的升级进度了。
	//以哈雷为例子，要切换到设置界面 --> 版本号界面，显示弹窗
	//*******************************************************************
	if (upgrade_work == TRANS_OPEN)
	{

		fdMcu = fopen(MCULOCAL_FILE, "r");
		mcu_UpdateSt.state = 2; //进入与MCU的升级通信了，正常UI应该解除工作逻辑不应卡死，不应投屏
	   //******************************************** 9.MCU升级 ********************************
		if (1)    // ----------------------------- wifi 已经下载文件在本地
		{
			printf("*****************\n");
			printf("burn mcu start\n");
			printf("*****************\n");
			McuUpgradeProcessShow = 1;  //通知UI，要弹窗显示进度条,1
			mcu_UpdateSt.len = 0;
			mcu_UpdateSt.address = 0;
			mcu_UpdateSt.progress = 0;
			mcu_UpdateSt.len = checkMcuBinState(fdMcu);
			if (mcu_UpdateSt.len < 50*1024)    //文件不存在或读取不到的情况
			{
				printf("mcu update file:  open error \n");
				mcu_UpdateSt.len = 0;
				fclose(fdMcu);
				fdMcu = fopen(MCULOCAL_BAK_FILE, "r");  //直接从备份中升级
				mcu_UpdateSt.len = checkMcuBinState(fdMcu);
				//return;
			}
			printf("mcu update file:  len =%d  \n", mcu_UpdateSt.len);
			printf("mcu update file:  address =%d  \n", mcu_UpdateSt.address);
			SerialPort_sendTransOpen(TRANS_OPEN, mcu_UpdateSt.len);//回复请求文件的长度
			SendDataLen = 0;
			memset(mcu_data_update, 0xAA, sizeof(mcu_data_update));
			fseek(fdMcu, 0, SEEK_SET);
			fread(mcu_data_update,1, mcu_UpdateSt.len, fdMcu);  //将MCU.bin文件内容整个读到buf中
			fclose(fdMcu);
			printf("$$$$ %x, %x, %x \n", mcu_data_update[0], mcu_data_update[1], mcu_data_update[2]);
		}
	}
	else if (upgrade_work == TRANS_READ)
	{
		if (bytes < 14)
		{
			return;
		}
		if (mcu_UpdateSt.len > 0)
		{
			int Readlen = FILL16(RecData[9], RecData[8]);
			printf("TRANS_READ Readlen  =%d \n", Readlen);
			uint32_t andress = FILL32(RecData[13], RecData[12], RecData[11], RecData[10]);
			printf("mcu offset13= %d \n", RecData[13]);
			printf("mcu offset12= %d \n", RecData[12]);
			printf("mcu offset11= %d \n", RecData[11]);
			printf("mcu offset10= %d \n", RecData[10]);
			printf("mcu andress = %d \n", andress);
			if (mcu_UpdateSt.len < andress + Readlen)
			{
				printf("ERROR  SerialPort::  analyMcuUpdate    mcu_UpdateSt.len < andress + Readlen");
			}
			else
			{
				//读取到MCU 升级 bin文件 并回复给MCU 
				printf("#############andress =%d; %x ; %x ; %x; \n", andress,
					mcu_data_update[andress], mcu_data_update[andress+1], mcu_data_update[andress + 2]);
				printf("mcu_data_update len= %d \n", Readlen);
			}
			SerialPort_sendTranRead(TRANS_READ, Readlen, &mcu_data_update[andress], RecData[10], RecData[11], 
				RecData[12], RecData[13]);
			SendDataLen = andress + Readlen;
			mcu_UpdateSt.progress = SendDataLen * 100 / mcu_UpdateSt.len;
            printf("\n\n********mcu ota progress =%d********** \n\n", mcu_UpdateSt.progress);

			if (mcu_UpdateSt.progress>=97) { //升级接近完成时，删除备份的MCU文件
				DisNorSWProtect();
				int removeret = remove(MCULOCAL_FILE);
				printf("\n\n*********<<<<remove MCU_bak.bin result =%d>>>>************* \n\n", removeret);
               // McuUpgradeProcessShow = 0;  //通知UI，要弹窗显示进度条

				ioctl(ITP_DEVICE_NOR, ITP_IOCTL_FLUSH, NULL);
			}

            if (mcu_UpdateSt.progress >= 100) { //升级接近完成时，删除备份的MCU文件
				printf("************Update 100%*************\n");
                McuUpgradeProcessShow = 0;  //通知UI，要弹窗显示进度条
            }

			if (SendDataLen * 100 / mcu_UpdateSt.len > 70)
			{
				//进度 >70 ,即将升级完成
			}
			printf("SerialPort::analyMcuUpdate    SendDataLen: %d \n", SendDataLen);
		}
	}
	else if (upgrade_work == TRANS_CLOSE)
	{
		printf("SerialPort::analyMcuUpdate  STATE  : TRANS_CLOSE\n");
	}
	else if (upgrade_work >= 8)
	{  //目前mcu 发的是这个，表示要升级系统板子

	}
}
void Send_wifiupdate_rootfs()
{
	char in[] = { 0x55,0xAE,0x00,0x0A,0x08,0x00,0x12,0x00 };//wifi 下载完 rootfs.ext4 文件后，进入升级所要发送的报文
	int siz = sizeof(in) / sizeof(in[0]);
	write(MCU_PORT, in, siz);
	printf("[ldd]  ****************** Send_wifiupdate_rootfs **************************\n");
}
void Send_wifiupdate_tire()
{
	char in[] = { 0x55,0xAE,0x00,0x0A,0x09,0x00,0x13,0x00 };//胎压下载完 tire 文件后，进入升级所要发送的报文
	int siz = sizeof(in) / sizeof(in[0]);
	write(MCU_PORT, in, siz);
	write(MCU_PORT, in, siz);
	printf("[ldd]  ************************* Send_wifiupdate_tire *****************************\n");
}
void* initialUartToMcu(void* arg)
{
	uint8_t rev[UPDATE_MSG_BUFFER_SIZE];
	uint8_t sev[UPDATE_MSG_BUFFER_SIZE];
	uint8_t sendtr1[16] = { 0xFF, 0x10, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x4E };
	int len = 0, position = 0;

	printf(">>>>>>>>>>>>>>>>> Start uart test!\n");
	//sem_init(&UartSem, 0, 0);												//1
	/*串口在系统端已经有初始化，因此使用串口时不用重新初始化*/
	//itpRegisterDevice(MCU_PORT, &itpDeviceUart2);							//1
	//ioctl(MCU_PORT, ITP_IOCTL_INIT, NULL);
#ifdef CFG_UART0_ENABLE
	//ioctl(MCU_PORT, ITP_IOCTL_RESET, NULL);
#endif
	//ioctl(MCU_PORT, ITP_IOCTL_REG_UART_CB, (void*)UartCallback);			//1

	while (1)
	{
		//sem_wait(&UartSem);												//1
		memset(rev, 0, UPDATE_MSG_BUFFER_SIZE);
		memset(sev, 0, UPDATE_MSG_BUFFER_SIZE);
		usleep(50 * 1000);  //10 ms , 根据不同协议确定休眠间隔				//0
		
		len = read(MCU_PORT, rev, UPDATE_MSG_BUFFER_SIZE);
		//printf("############len =%d \n", len);
		if (len > 0)
		{
			position = 0;
			while (len--) {
				//printf("1111111111-%02x ", rev[position]);
				SerialPort_RecvHandle(rev[position]);
				position++;
			}
		}
	}
}

uint8_t Checksum_Calculate(uint8_t *data, uint8_t length)
{
	uint8_t i;
	uint8_t checksum = 0;
	for (i = 0;i < length;i++)
	{
		checksum += data[i];
	}
	return(checksum);
}
void SerialPort_RecvHandle(uint8_t data)
{
	static uint8_t head[2] = { 0x00,0x00 };
	static uint8_t position = 0;
	uint8_t ch;							//	用来保留从缓冲区里读出来的当前字符
	int  length = UPDATE_MSG_BUFFER_SIZE;//,cmd; 默认128
	int checkUpgrade = 0;  // 如果连续几次都是短数据，则应该是进入了升级。  之所以多这个判断，主要曾经遇到过瞎猫，开机刚好乱码进了升级状态
	uint8_t  dlc = 0;
	bool checkSumError;
	uint8_t * p = Update_pro_receive_buf.tempMSGbuffer;
	if (Update_pro_receive_buf.filling == 0)
	{
		//第一个字节
		if (position == 0)
		{
			head[0] = data;
			position = 1;
		}
		else  //第二个字节
		{
			head[1] = data;

			if ((head[0] == PROTOCOLUART_HEADER_1) && (head[1] == PROTOCOLUART_HEADER_2))   //如果 开头符合0x55AE
			{
				Update_pro_receive_buf.filling = 1;          //表示开始抓包
				Update_pro_receive_buf.tempMSGbuffer[0] = head[0];
				Update_pro_receive_buf.tempMSGbuffer[1] = head[1];
				Update_pro_receive_buf.datalenth = 2;           //抓包到的位置，表示从此位置插入buf
				length_state = 0;
				length = UPDATE_MSG_BUFFER_SIZE;
				Update_pro_receive_buf.datalenth_actual = UPDATE_MSG_BUFFER_SIZE - 1;
				//cmd=0;
			}
			else
			{
				head[0] = head[1];
				position = 1; //别漏了这里 ,注意，这里要是position=1;  而不是position=0;
			}
		}
	}

	else if (Update_pro_receive_buf.filling == 1)                                       //开始抓包内容
	{
		Update_pro_receive_buf.tempMSGbuffer[Update_pro_receive_buf.datalenth] = data;

		if ((Update_pro_receive_buf.datalenth >= 7))
		{
			if (!length_state)
			{
				if (Update_pro_receive_buf.tempMSGbuffer[5] >= (UPDATE_MSG_BUFFER_SIZE - 8))
				{
					Update_pro_receive_buf.datalenth_actual = (UPDATE_MSG_BUFFER_SIZE - 1);
				}
				else
				{
					Update_pro_receive_buf.datalenth_actual = Update_pro_receive_buf.tempMSGbuffer[5] + 8;
				}
				length_state = 1;
			}
		}
		Update_pro_receive_buf.datalenth++;
		if (Update_pro_receive_buf.datalenth >= Update_pro_receive_buf.datalenth_actual)
		{
			if (Update_pro_receive_buf.datalenth == COMM_HEAD_LEN)	//8个字节
			{
				ch = p[2] + p[3] + p[4] + p[5];

				if (ch != p[6]) { //aproR.com->headChecksum = p[6]
					printf("\n##Checksum_error \n");
					Update_pro_receive_buf.filling = 0;
					Update_pro_receive_buf.datalenth = 0;
					Update_pro_receive_buf.datalenth_actual = 0;
					return;
				}
			}
       /*
			printf("\nUpdate_pro_receive_buf.datalenth =%d  \n", Update_pro_receive_buf.datalenth);
			//抓打印留
         
			printf("\n");
			for(int i =0;i< Update_pro_receive_buf.datalenth;i++)
			{
				printf("%02x ", Update_pro_receive_buf.tempMSGbuffer[i]);
			}
			printf("\n");
         */   
			if (Update_pro_receive_buf.tempMSGbuffer[3] == 0x0A && Update_pro_receive_buf.datalenth < 50) {  //表示是升级模式 , y
				analyMcuUpdate(Update_pro_receive_buf.tempMSGbuffer, Update_pro_receive_buf.datalenth);  // 分析 MCU 发来的升级指令
				setMcuUpgradeState(true);  // 收到2包MCU 升级数据结构，认为进入了升级状态，则不检测checksum
			}
			else {
				setMcuUpgradeState(false);  // 收到2包MCU 升级数据结构，认为进入了升级状态，则不检测checksum
				if (Update_pro_receive_buf.datalenth == p[5] + COMM_HEAD_LEN) { //p[7] = aproR.comm->appendLen
					int i, j;
					j = p[5];
					ch = 0;
                    //printf("j= %d\n", j);
					if (j)
					{
						p[COMM_HEAD_LEN + 4] = j - 6;
						ch = Checksum_Calculate(&p[COMM_HEAD_LEN + 2], j - 3);
					}
                    //printf("\n111APP_Checksum************ %d, %d\n", p[j + 7], ch);
					if (ch == p[j + 7])
					{
						//第二道校验，防错概率更好
                     
						ch = Checksum_Calculate(&p[8], j);
						if (ch == p[7]) {
                            
							SendTick();  //发送心跳给MCU
							SerialPort_MSG_decode(Update_pro_receive_buf.tempMSGbuffer, Update_pro_receive_buf.datalenth);
                        }
                        else {
                            printf("\nAPP_Checksum_error2222************ %d, %d\n", p[7], ch);
                        }
					}
					else
					{
						printf("\nAPP_Checksum_error************ %d, %d\n", p[j + 7],ch);
					}
				}
			}
			Update_pro_receive_buf.filling = 0;
			Update_pro_receive_buf.datalenth = 0;
			Update_pro_receive_buf.datalenth_actual = 0;
		}

	}
}
#endif




