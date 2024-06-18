#ifndef __GT9147_H__
#define __GT9147_H__

#define GT_CTRL_REG 	        0X8040  /* GT9147控制寄存器         */
#define GT_CFGS_REG				0X8047
#define GT_PID_REG 		        0X8140  /* GT9147产品ID寄存器       */

#define GT_GSTID_REG 	        0X814E  /* GT9147当前检测到的触摸情况 */
#define GT_TP1_REG 		        0X814F  /* 第一个触摸点数据地址 */
#define GT_TP2_REG 		        0X8158	/* 第二个触摸点数据地址 */
#define GT_TP3_REG 		        0X8160  /* 第三个触摸点数据地址 */
#define GT_TP4_REG 		        0X8168  /* 第四个触摸点数据地址  */
#define GT_TP5_REG 		        0X8170	/* 第五个触摸点数据地址   */

#define MAX_SUPORT_FINGER		5
#define GT_TP_LEN				8

#endif