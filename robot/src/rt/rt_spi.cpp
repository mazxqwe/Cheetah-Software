/*!
 * @file rt_spi.h
 * @brief SPI communication to spine board
 */
#ifdef linux

#include <byteswap.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include <linux/spi/spidev.h>
#include "rt/rt_spi.h"
#include <lcm/lcm-cpp.hpp>

unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
unsigned int spi_speed = 6000000;
uint8_t lsb = 0x01;

int spi_1_fd = -1;
int spi_2_fd = -1;

int can_open();

//static spine_cmd_t g_spine_cmd;
//static spine_data_t g_spine_data;

spi_command_t spi_command_drv;
spi_data_t spi_data_drv;
spi_torque_t spi_torque;

pthread_mutex_t spi_mutex;

const float max_torque[3] = {17.f, 17.f, 26.f};  // TODO CHECK WITH BEN
const float wimp_torque[3] = {6.f, 6.f, 6.f};    // TODO CHECK WITH BEN
const float disabled_torque[3] = {0.f, 0.f, 0.f};

// only used for actual robot
const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
const float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
const float knee_side_sign[4] = {-.6429f, .6429f, -.6429f, .6429f};

// only used for actual robot
const float abad_offset[4] = {0.f, 0.f, 0.f, 0.f};
const float hip_offset[4] = {M_PI / 2.f, -M_PI / 2.f, -M_PI / 2.f, M_PI / 2.f};
const float knee_offset[4] = {K_KNEE_OFFSET_POS, -K_KNEE_OFFSET_POS,
                              -K_KNEE_OFFSET_POS, K_KNEE_OFFSET_POS};
                              
/*!
 * Compute SPI message checksum
 * @param data : input
 * @param len : length (in 32-bit words)
 * @return
 */
uint32_t xor_checksum(uint32_t *data, size_t len) {
  uint32_t t = 0;
  for (size_t i = 0; i < len; i++) t = t ^ data[i];
  return t;
}

/*!
 * Emulate the spi board to estimate the torque.
 */
void fake_spine_control(spi_command_t *cmd, spi_data_t *data,
                        spi_torque_t *torque_out, int board_num) {
  torque_out->tau_abad[board_num] =
      cmd->kp_abad[board_num] *
          (cmd->q_des_abad[board_num] - data->q_abad[board_num]) +
      cmd->kd_abad[board_num] *
          (cmd->qd_des_abad[board_num] - data->qd_abad[board_num]) +
      cmd->tau_abad_ff[board_num];

  torque_out->tau_hip[board_num] =
      cmd->kp_hip[board_num] *
          (cmd->q_des_hip[board_num] - data->q_hip[board_num]) +
      cmd->kd_hip[board_num] *
          (cmd->qd_des_hip[board_num] - data->qd_hip[board_num]) +
      cmd->tau_hip_ff[board_num];

  torque_out->tau_knee[board_num] =
      cmd->kp_knee[board_num] *
          (cmd->q_des_knee[board_num] - data->q_knee[board_num]) +
      cmd->kd_knee[board_num] *
          (cmd->qd_des_knee[board_num] - data->qd_knee[board_num]) +
      cmd->tau_knee_ff[board_num];

  const float *torque_limits = disabled_torque;

  if (cmd->flags[board_num] & 0b1) {
    if (cmd->flags[board_num] & 0b10)
      torque_limits = wimp_torque;
    else
      torque_limits = max_torque;
  }

  if (torque_out->tau_abad[board_num] > torque_limits[0])
    torque_out->tau_abad[board_num] = torque_limits[0];
  if (torque_out->tau_abad[board_num] < -torque_limits[0])
    torque_out->tau_abad[board_num] = -torque_limits[0];

  if (torque_out->tau_hip[board_num] > torque_limits[1])
    torque_out->tau_hip[board_num] = torque_limits[1];
  if (torque_out->tau_hip[board_num] < -torque_limits[1])
    torque_out->tau_hip[board_num] = -torque_limits[1];

  if (torque_out->tau_knee[board_num] > torque_limits[2])
    torque_out->tau_knee[board_num] = torque_limits[2];
  if (torque_out->tau_knee[board_num] < -torque_limits[2])
    torque_out->tau_knee[board_num] = -torque_limits[2];
}

/*!
 * Initialize SPI
 */
void init_can() {
  // check sizes:
  size_t command_size = sizeof(spi_command_t);
  size_t data_size = sizeof(spi_data_t);

  memset(&spi_command_drv, 0, sizeof(spi_command_drv));
  memset(&spi_data_drv, 0, sizeof(spi_data_drv));

  if (pthread_mutex_init(&spi_mutex, NULL) != 0)
    printf("[ERROR: RT SPI] Failed to create spi data mutex\n");

  if (command_size != K_EXPECTED_COMMAND_SIZE) {
    printf("[RT SPI] Error command size is %ld, expected %d\n", command_size,
           K_EXPECTED_COMMAND_SIZE);
  } else
    printf("[RT SPI] command size good\n");

  if (data_size != K_EXPECTED_DATA_SIZE) {
    printf("[RT SPI] Error data size is %ld, expected %d\n", data_size,
           K_EXPECTED_DATA_SIZE);
  } else
    printf("[RT SPI] data size good\n");

  printf("This is Dimas from the rt_spi.cpp!\n"); 
  printf("[RT SPI] Open\n");
  can_open();
}

int s = 0; //socket
int s1 = 0; //socket
int s2 = 0; //socket
int s3 = 0; //socket

/*!
 * Open SPI device
 */
int can_open() 
{
  int rv = 0;
  int nonblock = 1;
  std::string can_name = "can0";

  std::cout << "pidor!" << std::endl;
  
  int enable_canfd = 1;
  struct ifreq ifr;
  struct sockaddr_can addr;

  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) { perror("Socket");}

  if( nonblock )
  {
    fcntl(s, F_SETFL, O_NONBLOCK); // non blocking CAN frame receiving => reading from this socket does not block execution.
  }

  strcpy(ifr.ifr_name, can_name.c_str() );
  ioctl(s, SIOCGIFINDEX, &ifr);

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd))) { perror("FD");}

  int optval=7; // valid values are in the range [1,7] ; 1- low priority, 7 - high priority
  if ( setsockopt(s, SOL_SOCKET, SO_PRIORITY, &optval, sizeof(optval))) { perror("Priority");} //makes no difference on receiving
  //if ( setsockopt(*s, SOL_SOCKET, SO_BUSY_POLL, &optval, sizeof(optval))) { perror("Busy poll");} //makes no difference on receiving
  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("Bind");}

  /*********************************************************************************************/

  can_name = "can1";

  std::cout << "snova pidor!" << std::endl;

  if ((s1 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) { perror("Socket");}

  if( nonblock )
  {
    fcntl(s1, F_SETFL, O_NONBLOCK); // non blocking CAN frame receiving => reading from this socket does not block execution.
  }

  strcpy(ifr.ifr_name, can_name.c_str() );
  ioctl(s1, SIOCGIFINDEX, &ifr);

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (setsockopt(s1, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd))) { perror("FD");}

  if ( setsockopt(s1, SOL_SOCKET, SO_PRIORITY, &optval, sizeof(optval))) { perror("Priority");} //makes no difference on receiving
  //if ( setsockopt(*s, SOL_SOCKET, SO_BUSY_POLL, &optval, sizeof(optval))) { perror("Busy poll");} //makes no difference on receiving
  if (bind(s1, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("Bind");}  

  /*********************************************************************************************/

  can_name = "can2";

  std::cout << "i snova pidor!" << std::endl;

  if ((s2 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) { perror("Socket");}

  if( nonblock )
  {
    fcntl(s2, F_SETFL, O_NONBLOCK); // non blocking CAN frame receiving => reading from this socket does not block execution.
  }

  strcpy(ifr.ifr_name, can_name.c_str() );
  ioctl(s2, SIOCGIFINDEX, &ifr);

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (setsockopt(s2, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd))) { perror("FD");}

  if ( setsockopt(s2, SOL_SOCKET, SO_PRIORITY, &optval, sizeof(optval))) { perror("Priority");} //makes no difference on receiving
  //if ( setsockopt(*s, SOL_SOCKET, SO_BUSY_POLL, &optval, sizeof(optval))) { perror("Busy poll");} //makes no difference on receiving
  if (bind(s2, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("Bind");}    

  /*********************************************************************************************/

  can_name = "can3";

  std::cout << "i opyat snova pidor!" << std::endl;

  if ((s3 = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) { perror("Socket");}

  if( nonblock )
  {
    fcntl(s3, F_SETFL, O_NONBLOCK); // non blocking CAN frame receiving => reading from this socket does not block execution.
  }

  strcpy(ifr.ifr_name, can_name.c_str() );
  ioctl(s3, SIOCGIFINDEX, &ifr);

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (setsockopt(s3, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd))) { perror("FD");}

  if ( setsockopt(s3, SOL_SOCKET, SO_PRIORITY, &optval, sizeof(optval))) { perror("Priority");} //makes no difference on receiving
  //if ( setsockopt(*s, SOL_SOCKET, SO_BUSY_POLL, &optval, sizeof(optval))) { perror("Busy poll");} //makes no difference on receiving
  if (bind(s3, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("Bind");}      

  return rv;
}

int spi_driver_iterations = 0;

/*!
 * convert spi command to spine_cmd_t
 */
void spi_to_spine(spi_command_t *cmd, spine_cmd_t *spine_cmd, int leg_0) {
  for (int i = 0; i < 2; i++) {
    // spine_cmd->q_des_abad[i] = (cmd->q_des_abad[i+leg_0] +
    // abad_offset[i+leg_0]) * abad_side_sign[i+leg_0]; spine_cmd->q_des_hip[i]
    // = (cmd->q_des_hip[i+leg_0] + hip_offset[i+leg_0]) *
    // hip_side_sign[i+leg_0]; spine_cmd->q_des_knee[i] =
    // (cmd->q_des_knee[i+leg_0] + knee_offset[i+leg_0]) /
    // knee_side_sign[i+leg_0];
    spine_cmd->q_des_abad[i] =
        (cmd->q_des_abad[i + leg_0] * abad_side_sign[i + leg_0]) + abad_offset[i + leg_0];
    spine_cmd->q_des_hip[i] =
        (cmd->q_des_hip[i + leg_0] * hip_side_sign[i + leg_0]) + hip_offset[i + leg_0];
    spine_cmd->q_des_knee[i] =
        (cmd->q_des_knee[i + leg_0] / knee_side_sign[i + leg_0]) + knee_offset[i + leg_0];

    spine_cmd->qd_des_abad[i] =
        cmd->qd_des_abad[i + leg_0] * abad_side_sign[i + leg_0];
    spine_cmd->qd_des_hip[i] =
        cmd->qd_des_hip[i + leg_0] * hip_side_sign[i + leg_0];
    spine_cmd->qd_des_knee[i] =
        cmd->qd_des_knee[i + leg_0] / knee_side_sign[i + leg_0];

    spine_cmd->kp_abad[i] = cmd->kp_abad[i + leg_0];
    spine_cmd->kp_hip[i] = cmd->kp_hip[i + leg_0];
    spine_cmd->kp_knee[i] = cmd->kp_knee[i + leg_0];

    spine_cmd->kd_abad[i] = cmd->kd_abad[i + leg_0];
    spine_cmd->kd_hip[i] = cmd->kd_hip[i + leg_0];
    spine_cmd->kd_knee[i] = cmd->kd_knee[i + leg_0];

    spine_cmd->tau_abad_ff[i] =
        cmd->tau_abad_ff[i + leg_0] * abad_side_sign[i + leg_0];
    spine_cmd->tau_hip_ff[i] =
        cmd->tau_hip_ff[i + leg_0] * hip_side_sign[i + leg_0];
    spine_cmd->tau_knee_ff[i] =
        cmd->tau_knee_ff[i + leg_0] * knee_side_sign[i + leg_0];

    spine_cmd->flags[i] = cmd->flags[i + leg_0];
  }
  spine_cmd->checksum = xor_checksum((uint32_t *)spine_cmd, 32);
}

/*!
 * convert spine_data_t to spi data
 */
void spine_to_spi(spi_data_t *data, spine_data_t *spine_data, int leg_0) {
  for (int i = 0; i < 2; i++) {
    data->q_abad[i + leg_0] = (spine_data->q_abad[i] - abad_offset[i + leg_0]) *
                              abad_side_sign[i + leg_0];
    data->q_hip[i + leg_0] = (spine_data->q_hip[i] - hip_offset[i + leg_0]) *
                             hip_side_sign[i + leg_0];
    data->q_knee[i + leg_0] = (spine_data->q_knee[i] - knee_offset[i + leg_0]) *
                              knee_side_sign[i + leg_0];

    data->qd_abad[i + leg_0] =
        spine_data->qd_abad[i] * abad_side_sign[i + leg_0];
    data->qd_hip[i + leg_0] = spine_data->qd_hip[i] * hip_side_sign[i + leg_0];
    data->qd_knee[i + leg_0] =
        spine_data->qd_knee[i] * knee_side_sign[i + leg_0];

    data->flags[i + leg_0] = spine_data->flags[i];
  }

  uint32_t calc_checksum = xor_checksum((uint32_t *)spine_data, 14);
  if (calc_checksum != (uint32_t)spine_data->checksum)
    printf("SPI ERROR BAD CHECKSUM GOT 0x%hx EXPECTED 0x%hx\n", calc_checksum,
           spine_data->checksum);
}

static const unsigned char dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7,
					8, 12, 16, 20, 24, 32, 48, 64};

static const unsigned char len2dlc[] = {0, 1, 2, 3, 4, 5, 6, 7, 8,		/* 0 - 8 */
					9, 9, 9, 9,				/* 9 - 12 */
					10, 10, 10, 10,				/* 13 - 16 */
					11, 11, 11, 11,				/* 17 - 20 */
					12, 12, 12, 12,				/* 21 - 24 */
					13, 13, 13, 13, 13, 13, 13, 13,		/* 25 - 32 */
					14, 14, 14, 14, 14, 14, 14, 14,		/* 33 - 40 */
					14, 14, 14, 14, 14, 14, 14, 14,		/* 41 - 48 */
					15, 15, 15, 15, 15, 15, 15, 15,		/* 49 - 56 */
					15, 15, 15, 15, 15, 15, 15, 15};	/* 57 - 64 */

unsigned char can_fd_dlc2len(unsigned char dlc)
{
	return dlc2len[dlc & 0x0F];
}

unsigned char can_fd_len2dlc(unsigned char len)
{
	if (len > 64)
		return 0xF;

	return len2dlc[len];
}

int packFloat(void *buf, float x) {
    unsigned char *b = (unsigned char *)buf;
    unsigned char *p = (unsigned char *) &x;
#if defined MSB
    b[0] = p[3];
    b[1] = p[2];
    b[2] = p[1];
    b[3] = p[0];
#else
    b[0] = p[0];
    b[1] = p[1];
    b[2] = p[2];
    b[3] = p[3];
#endif
    return 4;
}

float unpackFloat(const void *buf) 
{
    float n;
    memcpy(&n, buf, 4);

    return n;
}

/*!
 * send receive data and command from spine
 */
void can_send_receive(spi_command_t *command, spi_data_t *data) 
{
  // update driver status flag
  spi_driver_iterations++;
  data->spi_driver_status = spi_driver_iterations << 16;

  //std::cout << (float)command->qd_des_abad[0] << " " << (float)command->qd_des_knee[0] << " " << (float)command->q_des_hip[0] << std::endl;

  #define leg0 0
  #define sign0 -1.0f

  #define leg1 1
  #define sign1 -1.0f

  #define leg2 2
  #define sign2 -1.0f

  #define leg3 3
  #define sign3 -1.0f  

  float TH_set = 0;
  float VE_set = 0;
  float Kp = 0;
  float Kd = 0;
  float TQ_set = 0;

  canfd_frame frame;

  TH_set = command->q_des_knee[leg0]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_knee[leg0]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_knee[leg0];     // coefficient at angle error, N/rad
  Kd =     command->kd_knee[leg0];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_knee_ff[leg0]; // torque setpoint, N

  frame.can_id = 0x10*10 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign0*TH_set);
  packFloat(&frame.data[4], sign0*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign0*TQ_set);

  if (write(s, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72    

  TH_set = command->q_des_hip[leg0]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_hip[leg0]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_hip[leg0];     // coefficient at angle error, N/rad
  Kd =     command->kd_hip[leg0];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_hip_ff[leg0]; // torque setpoint, N

  frame.can_id = 0x10*11 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign0*TH_set);
  packFloat(&frame.data[4], sign0*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign0*TQ_set);

  if (write(s, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72      

  TH_set = command->q_des_abad[leg0]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_abad[leg0]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_abad[leg0];     // coefficient at angle error, N/rad
  Kd =     command->kd_abad[leg0];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_abad_ff[leg0]; // torque setpoint, N

  frame.can_id = 0x10*12 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign0*TH_set);
  packFloat(&frame.data[4], sign0*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign0*TQ_set);

  if (write(s, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72   

  /********************************************************************************************************/

  TH_set = command->q_des_knee[leg1]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_knee[leg1]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_knee[leg1];     // coefficient at angle error, N/rad
  Kd =     command->kd_knee[leg1];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_knee_ff[leg1]; // torque setpoint, N

  frame.can_id = 0x10*20 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign1*TH_set);
  packFloat(&frame.data[4], sign1*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign1*TQ_set);

  if (write(s1, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72    

  TH_set = command->q_des_hip[leg1]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_hip[leg1]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_hip[leg1];     // coefficient at angle error, N/rad
  Kd =     command->kd_hip[leg1];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_hip_ff[leg1]; // torque setpoint, N

  frame.can_id = 0x10*21 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign1*TH_set);
  packFloat(&frame.data[4], sign1*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign1*TQ_set);

  if (write(s1, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72      

  TH_set = command->q_des_abad[leg1]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_abad[leg1]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_abad[leg1];     // coefficient at angle error, N/rad
  Kd =     command->kd_abad[leg1];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_abad_ff[leg1]; // torque setpoint, N

  frame.can_id = 0x10*22 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign1*TH_set);
  packFloat(&frame.data[4], sign1*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign1*TQ_set);

  if (write(s1, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72   

  /********************************************************************************************************/

  TH_set = command->q_des_knee[leg2]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_knee[leg2]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_knee[leg2];     // coefficient at angle error, N/rad
  Kd =     command->kd_knee[leg2];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_knee_ff[leg2]; // torque setpoint, N

  frame.can_id = 0x10*30 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign2*TH_set);
  packFloat(&frame.data[4], sign2*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign2*TQ_set);

  if (write(s2, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72    

  TH_set = command->q_des_hip[leg2]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_hip[leg2]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_hip[leg2];     // coefficient at angle error, N/rad
  Kd =     command->kd_hip[leg2];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_hip_ff[leg2]; // torque setpoint, N

  frame.can_id = 0x10*31 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign2*TH_set);
  packFloat(&frame.data[4], sign2*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign2*TQ_set);

  if (write(s2, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72      

  TH_set = command->q_des_abad[leg2]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_abad[leg2]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_abad[leg2];     // coefficient at angle error, N/rad
  Kd =     command->kd_abad[leg2];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_abad_ff[leg2]; // torque setpoint, N

  frame.can_id = 0x10*32 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign2*TH_set);
  packFloat(&frame.data[4], sign2*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign2*TQ_set);

  if (write(s2, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72   

  /********************************************************************************************************/

  TH_set = command->q_des_knee[leg3]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_knee[leg3]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_knee[leg3];     // coefficient at angle error, N/rad
  Kd =     command->kd_knee[leg3];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_knee_ff[leg3]; // torque setpoint, N

  frame.can_id = 0x10*40 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign3*TH_set);
  packFloat(&frame.data[4], sign3*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign3*TQ_set);

  if (write(s3, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72    

  TH_set = command->q_des_hip[leg3]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_hip[leg3]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_hip[leg3];     // coefficient at angle error, N/rad
  Kd =     command->kd_hip[leg3];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_hip_ff[leg3]; // torque setpoint, N

  frame.can_id = 0x10*41 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign3*TH_set);
  packFloat(&frame.data[4], sign3*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign3*TQ_set);

  if (write(s3, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72      

  TH_set = command->q_des_abad[leg3]; // output shaft mechanical angle setpoint, rad
  VE_set = command->qd_des_abad[leg3]; // output shaft velocity setpoint, rad/s
  Kp =     command->kp_abad[leg3];     // coefficient at angle error, N/rad
  Kd =     command->kd_abad[leg3];     // coefficient at velocity error, N/(rad/s)
  TQ_set = command->tau_abad_ff[leg3]; // torque setpoint, N

  frame.can_id = 0x10*42 + 0x08;
  frame.len = 20;
  frame.len = can_fd_dlc2len(can_fd_len2dlc(frame.len)); //ensure discrete CAN FD length values

  packFloat(frame.data, sign3*TH_set);
  packFloat(&frame.data[4], sign3*VE_set);
  packFloat(&frame.data[8], Kp);
  packFloat(&frame.data[12], Kd);
  packFloat(&frame.data[16], sign3*TQ_set);

  if (write(s3, &frame, CANFD_MTU) != CANFD_MTU) { perror("Write");} // CANFD_MTU = 72   

  /********************************************************************************************************/  

  /// real-time variables actual values
  float TH_act = 0; // output shaft mechanical angle reading, rad
  float VE_act = 0; // output shaft velocity reading, rad/s
  //float TQ_act = 0; // torque reading, N

  canfd_frame rx_frame;

  for( int i = 0; i < 3; i++)
  {
    int suka = read(s, &rx_frame, CANFD_MTU); // CANFD_MTU = 72
    suka += 1;
    uint8_t device_id = rx_frame.can_id / 0x10; // get responding device Id

    TH_act = unpackFloat(rx_frame.data);
    VE_act = unpackFloat(&rx_frame.data[4]);    

    switch ( device_id )
    {
      case 10:
        data->q_knee[leg0] = sign0*TH_act;
        data->qd_knee[leg0] = sign0*VE_act;
        break;

      case 11:
        data->q_hip[leg0] = sign0*TH_act;
        data->qd_hip[leg0] = sign0*VE_act;   
        break;

      case 12:
        data->q_abad[leg0] = sign0*TH_act;
        data->qd_abad[leg0] = sign0*VE_act;   
        break;      
    }   
  }

  for( int i = 0; i < 3; i++)
  {
    int suka = read(s1, &rx_frame, CANFD_MTU); // CANFD_MTU = 72
    suka += 1;
    uint8_t device_id = rx_frame.can_id / 0x10; // get responding device Id

    TH_act = unpackFloat(rx_frame.data);
    VE_act = unpackFloat(&rx_frame.data[4]);    

    switch ( device_id )
    {
      case 20:
        data->q_knee[leg1] = sign1*TH_act;
        data->qd_knee[leg1] = sign1*VE_act;
        break;

      case 21:
        data->q_hip[leg1] = sign1*TH_act;
        data->qd_hip[leg1] = sign1*VE_act;   
        break;

      case 22:
        data->q_abad[leg1] = sign1*TH_act;
        data->qd_abad[leg1] = sign1*VE_act;   
        break;      
    }   
  }  

  for( int i = 0; i < 3; i++)
  {
    int suka = read(s2, &rx_frame, CANFD_MTU); // CANFD_MTU = 72
    suka += 1;
    uint8_t device_id = rx_frame.can_id / 0x10; // get responding device Id

    TH_act = unpackFloat(rx_frame.data);
    VE_act = unpackFloat(&rx_frame.data[4]);    

    switch ( device_id )
    {
      case 30:
        data->q_knee[leg3] = sign3*TH_act;
        data->qd_knee[leg3] = sign3*VE_act;
        break;

      case 31:
        data->q_hip[leg3] = sign3*TH_act;
        data->qd_hip[leg3] = sign3*VE_act;   
        break;

      case 32:
        data->q_abad[leg3] = sign3*TH_act;
        data->qd_abad[leg3] = sign3*VE_act;   
        break;      
    }   
  }

  for( int i = 0; i < 3; i++)
  {
    int suka = read(s3, &rx_frame, CANFD_MTU); // CANFD_MTU = 72
    suka += 1;
    uint8_t device_id = rx_frame.can_id / 0x10; // get responding device Id

    TH_act = unpackFloat(rx_frame.data);
    VE_act = unpackFloat(&rx_frame.data[4]);    

    switch ( device_id )
    {
      case 40:
        data->q_knee[leg2] = leg2*TH_act;
        data->qd_knee[leg2] = leg2*VE_act;
        break;

      case 41:
        data->q_hip[leg2] = leg2*TH_act;
        data->qd_hip[leg2] = leg2*VE_act;   
        break;

      case 42:
        data->q_abad[leg2] = leg2*TH_act;
        data->qd_abad[leg2] = leg2*VE_act;   
        break;      
    }       
  }    

  data->q_hip[0] = command->q_des_hip[0];
  data->q_hip[1] = command->q_des_hip[1];
  //data->q_hip[2] = command->q_des_hip[2];
  //data->q_hip[3] = command->q_des_hip[3];

  data->qd_hip[0] = command->qd_des_hip[0];
  data->qd_hip[1] = command->qd_des_hip[1];
  //data->qd_hip[2] = command->qd_des_hip[2];
  //data->qd_hip[3] = command->qd_des_hip[3];

  data->q_abad[0] = command->q_des_abad[0];
  data->q_abad[1] = command->q_des_abad[1];
  //data->q_abad[2] = command->q_des_abad[2];
  //data->q_abad[3] = command->q_des_abad[3];

  data->qd_abad[0] = command->qd_des_abad[0];
  data->qd_abad[1] = command->qd_des_abad[1];
  //data->qd_abad[2] = command->qd_des_abad[2];
  //data->qd_abad[3] = command->qd_des_abad[3];

  data->q_knee[0] = command->q_des_knee[0];
  data->q_knee[1] = command->q_des_knee[1];
  //data->q_knee[2] = command->q_des_knee[2];
  //data->q_knee[3] = command->q_des_knee[3];

  data->qd_knee[0] = command->qd_des_knee[0];
  data->qd_knee[1] = command->qd_des_knee[1];
  //data->qd_knee[2] = command->qd_des_knee[2];
  //data->qd_knee[3] = command->qd_des_knee[3];
  
}

/*!
 * Run SPI
 */
void can_driver_run() {
  // do spi board calculations
  for (int i = 0; i < 4; i++) {
    fake_spine_control(&spi_command_drv, &spi_data_drv, &spi_torque, i);
  }

  // in here, the driver is good
  pthread_mutex_lock(&spi_mutex);
  can_send_receive(&spi_command_drv, &spi_data_drv);
  pthread_mutex_unlock(&spi_mutex);
}

/*!
 * Get the spi command
 */
spi_command_t *get_can_command() {
  return &spi_command_drv;
}

/*!
 * Get the spi data
 */
spi_data_t *get_can_data() { return &spi_data_drv; }

#endif
