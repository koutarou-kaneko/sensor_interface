// =============================================================================
//					Filename: node.c
//    This program is for CFS(by Leptrino) 6axis force sensor ros node.
// =============================================================================
//		Ver 1.0.0		2013/11/26
// =============================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>

// For CFS 6axis force sensor communication
#include "cfs_sensor/pCommon.h"
#include "cfs_sensor/rs_comm.h"
#include "cfs_sensor/pComResInternal.h"
#include "cfs_sensor/cfs_sensor_node.h"

// =============================================================================
//	macro definition
// =============================================================================
#define PRG_VER	"Ver 1.0.0"

//ROS RATE
#define S_RATE 100
// ----------------------------------------------------------------------------------
//	main function
// ----------------------------------------------------------------------------------
int main(int argc,char **argv)
{
  ros::init (argc, argv, "cfs_sensor_node");
  ros::NodeHandle n;
  ros::NodeHandle nhp("~");
  cfs_sensor::CFS_Sensor_Node cfs_sensor_node(n, nhp);
  return 0;
}

namespace cfs_sensor
{
  CFS_Sensor_Node::CFS_Sensor_Node (ros::NodeHandle & n, ros::NodeHandle & nhp):n_ (n), nhp_ (nhp)
  {
    int i, l = 0, rt = 0;
    int EndF = 0;
    //----------------------------
    // pComResInternal.h
    //----------------------------
    ST_RES_HEAD *stCmdHead;   //Response header
    ST_R_GET_INF *stGetInfo;  //device info

    ROS_INFO("Starting CFS_Sensor_Node");

    // Get ros param
    nhp_.param("cfs_frame_id", cfs_frame_id, std::string("cfs_frame"));
    nhp_.param("cfs_default_device_name", cfs_default_device_name, std::string("/dev/ttyACM0"));
    nhp_.param("cfs_sensor_pub_name", cfs_sensor_pub_name, std::string("/cfs/data"));
    nhp_.param("cfs_sensor_calib_srv_name", cfs_sensor_calib_srv_name, std::string("cfs_sensor_calib"));
    // CFS034CA301U: 150, 150, 300, 4, 4, 4
    // CFS018CA201U: 100, 100, 200, 1, 1, 1
    nhp_.param("max_fx", cfs_device_rate_val.maxfx, 150);
    nhp_.param("max_fy", cfs_device_rate_val.maxfy, 150);
    nhp_.param("max_fz", cfs_device_rate_val.maxfz, 300);
    nhp_.param("max_mx", cfs_device_rate_val.maxmx, 4);
    nhp_.param("max_my", cfs_device_rate_val.maxmy, 4);
    nhp_.param("max_mz", cfs_device_rate_val.maxmz, 4);

    ros::Rate loop_rate(S_RATE);
    CFS_Sensor_Node::Comm_Init();
    CFS_Sensor_Node::Data_Init();

    if (gSys.com_ok == NG) {
      ROS_INFO("ComPort Open Fail\n");
      exit(0);
    }

    //Generate Force Value message Publisher
    cfs_sensor_Pub_ = n_.advertise<geometry_msgs::WrenchStamped>(cfs_sensor_pub_name,0);
    cfs_sensor_Svs_ = n_.advertiseService(cfs_sensor_calib_srv_name, &CFS_Sensor_Node::start_calibration, this);

    // 製品情報取得
    GetProductInfo();
    while(1) {
      Comm_Rcv();
      if ( Comm_CheckRcv() != 0 ) {		//受信データ有
        CommRcvBuff[0]=0;

        rt = Comm_GetRcvData( CommRcvBuff );
        if ( rt>0 ) {
          stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
          stGetInfo->scFVer[F_VER_SIZE] = 0;
          ROS_INFO("Version:%s", stGetInfo->scFVer);
          stGetInfo->scSerial[SERIAL_SIZE] = 0;
          ROS_INFO("SerialNo:%s", stGetInfo->scSerial);
          stGetInfo->scPName[P_NAME_SIZE] = 0;
          ROS_INFO("Type:%s", stGetInfo->scPName);
          EndF = 1;
        }
      }
      if ( EndF==1 ) break;
    }

    usleep(10000);

    // 連続送信開始 main loop
    CFS_Sensor_Node::SerialStart();
    EndF = 0;
    while(ros::ok()) {
      Comm_Rcv();
      if ( Comm_CheckRcv() != 0 ) {		//check receive
        rt = CFS_Sensor_Node::update_sensor_data();
        if(rt>0){
          CFS_Sensor_Node::convert_sensor_value();
          CFS_Sensor_Node::publish_sensor_msg();
          stCmdHead = (ST_RES_HEAD *)CommRcvBuff;
          if (stCmdHead->ucCmd == CMD_DATA_STOP) {
            printf("Receive Stop Response:");
            l = stCmdHead->ucLen;
            for ( i=0; i<l; i++) {
              printf("%02x ", CommRcvBuff[i]);
            }
            printf("\n");
            EndF = 1;
          }
        }
      }//end of check receive
      if ( EndF==1 ) break;
      ros::spinOnce();
      loop_rate.sleep();
    }//end of main loop
  }

  CFS_Sensor_Node::~CFS_Sensor_Node (void)
  {
    CFS_Sensor_Node::SerialStop();
    CFS_Sensor_Node::Node_Close();
  }

  //initialize sensor data 0.0
  void CFS_Sensor_Node::Data_Init(void)
  {
    cfs_sensor_raw = (CFS_SENSOR_DATA *)malloc(sizeof(CFS_SENSOR_DATA));
    cfs_sensor_offset = (CFS_SENSOR_DATA *)malloc(sizeof(CFS_SENSOR_DATA));
    cfs_sensor_conv = (CFS_SENSOR_DATA *)malloc(sizeof(CFS_SENSOR_DATA));
    CFS_Sensor_Node::cfs_sensor_data_init(cfs_sensor_raw);
    CFS_Sensor_Node::cfs_sensor_data_init(cfs_sensor_offset);
    CFS_Sensor_Node::cfs_sensor_data_init(cfs_sensor_conv);
  }

  void CFS_Sensor_Node::cfs_sensor_data_init(CFS_SENSOR_DATA *cfs_sensor_data)
  {
    cfs_sensor_data->fx = 0.0;
    cfs_sensor_data->fy = 0.0;
    cfs_sensor_data->fz = 0.0;
    cfs_sensor_data->mx = 0.0;
    cfs_sensor_data->my = 0.0;
    cfs_sensor_data->mz = 0.0;
  }

  int CFS_Sensor_Node::update_sensor_data(void)
  {
    int rt;
    ST_R_DATA_GET_F *stForce = (ST_R_DATA_GET_F *)malloc(sizeof(ST_R_DATA_GET_F)); //get data

    memset(CommRcvBuff,0,sizeof(CommRcvBuff));//0 clear CommRcvBuff
    rt = Comm_GetRcvData( CommRcvBuff );      //get rcv data
    if(rt>0){
      stForce = (ST_R_DATA_GET_F *)CommRcvBuff;
#if 0
      ROS_INFO("%5d,%5d,%5d,%5d,%5d,%5d\r\n\f",
               stForce->ssForce[0],stForce->ssForce[1],stForce->ssForce[2],
               stForce->ssForce[3],stForce->ssForce[4],stForce->ssForce[5]);
#endif
      cfs_sensor_raw->fx = stForce->ssForce[0];
      cfs_sensor_raw->fy = stForce->ssForce[1];
      cfs_sensor_raw->fz = stForce->ssForce[2];
      cfs_sensor_raw->mx = stForce->ssForce[3];
      cfs_sensor_raw->my = stForce->ssForce[4];
      cfs_sensor_raw->mz = stForce->ssForce[5];

      update_flag = true;
    }

    return rt;
  }

  void CFS_Sensor_Node::convert_sensor_value(void)
  {
    //raw value is 10000 when the force is at rate value;
    //Fx,y,z = [1e-3 N], Mx,y,z=[1e-6 Nm]
    cfs_sensor_conv->fx = (int)(((cfs_sensor_raw->fx - cfs_sensor_offset->fx) * cfs_device_rate_val.maxfx * 1000)/10000);
    cfs_sensor_conv->fy = (int)(((cfs_sensor_raw->fy - cfs_sensor_offset->fy) * cfs_device_rate_val.maxfy * 1000)/10000);
    cfs_sensor_conv->fz = (int)(((cfs_sensor_raw->fz - cfs_sensor_offset->fz)  * cfs_device_rate_val.maxfz * 1000)/10000);
    cfs_sensor_conv->mx = (int)(((cfs_sensor_raw->mx - cfs_sensor_offset->mx) * cfs_device_rate_val.maxmx * 1000)/10);
    cfs_sensor_conv->my = (int)(((cfs_sensor_raw->my - cfs_sensor_offset->my) * cfs_device_rate_val.maxmy * 1000)/10);
    cfs_sensor_conv->mz = (int)(((cfs_sensor_raw->mz - cfs_sensor_offset->mz) * cfs_device_rate_val.maxmz * 1000)/10);
  }

  void CFS_Sensor_Node::publish_sensor_msg(void)
  {
    geometry_msgs::WrenchStamped cfs_msg;

    cfs_msg.header.stamp = ros::Time::now();
    cfs_msg.header.frame_id = cfs_frame_id;
    cfs_msg.wrench.force.x = cfs_sensor_conv->fx / 1000.0;
    cfs_msg.wrench.force.y = cfs_sensor_conv->fy / 1000.0;
    cfs_msg.wrench.force.z = cfs_sensor_conv->fz / 1000.0;
    cfs_msg.wrench.torque.x = cfs_sensor_conv->mx / 1000000.0;
    cfs_msg.wrench.torque.y = cfs_sensor_conv->my / 1000000.0;
    cfs_msg.wrench.torque.z = cfs_sensor_conv->mz / 1000000.0;

    cfs_sensor_Pub_.publish(cfs_msg);
  }

  bool CFS_Sensor_Node::start_calibration(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    int i;
    CFS_SENSOR_DATA *cfs_data;
    const int buffer_size = 50;

    cfs_data = (CFS_SENSOR_DATA *)malloc(sizeof(CFS_SENSOR_DATA));
    CFS_Sensor_Node::cfs_sensor_data_init(cfs_data);

    ROS_INFO("Starting Force Sensor calibration. Don't touch sensor.");

    for(i=0;i<buffer_size;i++){
      cfs_data->fx += cfs_sensor_raw->fx;
      cfs_data->fy += cfs_sensor_raw->fy;
      cfs_data->fz += cfs_sensor_raw->fz;
      cfs_data->mx += cfs_sensor_raw->mx;
      cfs_data->my += cfs_sensor_raw->my;
      cfs_data->mz += cfs_sensor_raw->mz;

      update_flag = false;
      usleep(20 * 1000);
    }

    cfs_data->fx /= buffer_size;
    cfs_data->fy /= buffer_size;
    cfs_data->fz /= buffer_size;
    cfs_data->mx /= buffer_size;
    cfs_data->my /= buffer_size;
    cfs_data->mz /= buffer_size;

    cfs_sensor_offset->fx = cfs_data->fx;
    cfs_sensor_offset->fy = cfs_data->fy;
    cfs_sensor_offset->fz = cfs_data->fz;
    cfs_sensor_offset->mx = cfs_data->mx;
    cfs_sensor_offset->my = cfs_data->my;
    cfs_sensor_offset->mz = cfs_data->mz;

    ROS_INFO("Set offset: %5d, %5d, %5d, %5d, %5d, %5d",(int)cfs_sensor_offset->fx,(int)cfs_sensor_offset->fy,(int)cfs_sensor_offset->fz,(int)cfs_sensor_offset->mx,(int)cfs_sensor_offset->my,(int)cfs_sensor_offset->mz);
    ROS_INFO("Force Sensor calibration succeeded.");
    return true;
  }

  // ----------------------------------------------------------------------------------
  //	Initialize Comm Port
  // ----------------------------------------------------------------------------------
  void CFS_Sensor_Node::Comm_Init(void)
  {
    int rt;

    if (!n_.getParam("cfs_device_name", cfs_device_name))
      cfs_device_name = cfs_default_device_name;
    ROS_INFO ("\tCFS Device Name: %s", cfs_device_name.c_str());

    //Commポート初期化
    gSys.com_ok = NG;
    rt = Comm_Open(cfs_device_name.c_str());
    if ( rt==OK ) {
      Comm_Setup( 460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
      gSys.com_ok = OK;
    }
  }

  // ----------------------------------------------------------------------------------
  //	Node Close
  // ----------------------------------------------------------------------------------
  void CFS_Sensor_Node::Node_Close(void)
  {
    ROS_INFO("Communication Close\n");

    if ( gSys.com_ok == OK) {
      Comm_Close();
    }
  }

  /*********************************************************************************
   * Function Name  : HST_SendResp
   * Description    : データを整形して送信する
   * Input          : pucInput 送信データ
   *                : 送信データサイズ
   * Output         :
   * Return         :
   *********************************************************************************/
  ULONG CFS_Sensor_Node::SendData(UCHAR *pucInput, USHORT usSize)
  {
    USHORT usCnt;
    UCHAR ucWork;
    UCHAR ucBCC = 0;
    UCHAR *pucWrite = &CommSendBuff[0];
    USHORT usRealSize;

    // データ整形
    *pucWrite = CHR_DLE;					// DLE
    pucWrite++;
    *pucWrite = CHR_STX;					// STX
    pucWrite++;
    usRealSize =2;

    for (usCnt = 0; usCnt < usSize; usCnt++) {
      ucWork = pucInput[usCnt];
      if (ucWork == CHR_DLE) {			// データが0x10ならば0x10を付加
        *pucWrite = CHR_DLE;			// DLE付加
        pucWrite++;						// 書き込み先
        usRealSize++;					// 実サイズ
        // BCCは計算しない!
      }
      *pucWrite = ucWork;					// データ
      ucBCC ^= ucWork;					// BCC
      pucWrite++;							// 書き込み先
      usRealSize++;						// 実サイズ
    }

    *pucWrite = CHR_DLE;					// DLE
    pucWrite++;
    *pucWrite = CHR_ETX;					// ETX
    ucBCC ^= CHR_ETX;						// BCC計算
    pucWrite++;
    *pucWrite = ucBCC;						// BCC付加
    usRealSize += 3;

    Comm_SendData(&CommSendBuff[0], usRealSize);

    return OK;
  }

  void CFS_Sensor_Node::GetProductInfo(void)
  {
    USHORT len;

    ROS_INFO("Get SensorInfo\n");
    len = 0x04;						// データ長
    SendBuff[0] = len;					// レングス
    SendBuff[1] = 0xFF;					// センサNo.
    SendBuff[2] = CMD_GET_INF;				// コマンド種別
    SendBuff[3] = 0;					// 予備

    CFS_Sensor_Node::SendData(SendBuff, len);
  }

  void CFS_Sensor_Node::SerialStart(void)
  {
    USHORT len;

    ROS_INFO("Start");
    len = 0x04;								// データ長
    SendBuff[0] = len;						// レングス
    SendBuff[1] = 0xFF;						// センサNo.
    SendBuff[2] = CMD_DATA_START;			// コマンド種別
    SendBuff[3] = 0;						// 予備

    CFS_Sensor_Node::SendData(SendBuff, len);
  }

  void CFS_Sensor_Node::SerialStop(void)
  {
    USHORT len;

    ROS_INFO("Stop");
    len = 0x04;								// データ長
    SendBuff[0] = len;						// レングス
    SendBuff[1] = 0xFF;						// センサNo.
    SendBuff[2] = CMD_DATA_STOP;			// コマンド種別
    SendBuff[3] = 0;						// 予備

    CFS_Sensor_Node::SendData(SendBuff, len);
  }

}//end of namespace cfs_sensor
