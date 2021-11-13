/**
  ******************************************************************************
  * File Name          :
  * Description        :
  ******************************************************************************
  * @attention
  *
  *
  *
  *
  ******************************************************************************
  */

#include "tool.h"
#include "math.h"

extern long tick;
extern long maxtick;

//void AttitudeAlgorithm(long FREQ) { //姿态解算
//    if(RxFlag1 == 1) { //传感器在线
//        if(MPU.angle[0]) { //滤波
//            ;
//        }

//        double s[3],c[3];

//        for(int i=0; i<3; i++) {
//            s[i] = sin(MPU.angle[i]);
//            c[i] = cos(MPU.angle[i]);
//        }

////    double X[3][3] = {
////      {1, 0,    0},
////      {0, c[0], -s[0]},
////      {0, s[0], c[0]}
////    };
////    double Y[3][3] = {
////      {c[1],  0,  s[1]},
////      {0,     1,  0},
////      {-s[1], 0,  c[1]}
////    };
////    double Z[3][3] = {
////      {c[2],  -s[2],  0},
////      {s[2],  c[2],   0},
////      {0,     0,      1}
////    };

//        double ZYX[3][3] = {
//            {c[0]*c[1], c[0]*s[1]*s[2]-c[2]*s[0], s[0]*s[2]+c[0]*c[2]*s[1]},
//            {c[1]*s[0], c[0]*c[2]+s[0]*s[1]*s[2], c[2]*s[0]*s[1]-c[0]*s[2]},
//            {-s[1],     c[1]*s[2],                c[1]*c[2]}
//        };

//        double ex[3][3] = {//平行于机身轴线，指向飞行器前方
//            {1,0,0},
//            {0,0,0},
//            {0,0,0}
//        };
//        double ey[3][3] = {//垂直于飞行器参考面，指向飞行器右方
//            {0,0,0},
//            {1,0,0},
//            {0,0,0}
//        };
//        double ez[3][3] = {//垂直于XOY平面，指向飞行器下方
//            {0,0,0},
//            {0,0,0},
//            {1,0,0}
//        };

//        double body_dir[3][3],body_a[3],a[3][3];//机体坐标系

//        martixMulti(ZYX,ex,a);

//        for(int i=0; i<3; i++) {
//            body_dir[0][i] = a[i][0];
//            body_a[i] = body_dir[0][i]*MPU.a[i];
//            MPU.sped[i] = body_a[i]/FREQ;
//        }

//        MPU.speed_now = sqrt(pow(MPU.sped[0],2)+pow(MPU.sped[1],2)+pow(MPU.sped[2],2));

//        for(int i=0; i<3; i++) {
//            MPU.now[i] += 0.5*MPU.sped[i]/FREQ;
//        }
//    }
//}

//void martixMulti(double a[3][3],double b[3][3],double c[3][3]) {
//	for(int i=0; i<3; i++) {
//			for(int j=0; j<3; j++) {
//					for(int k=0; k<3; k++) {
//							c[i][j] += a[i][k]*b[k][j];
//					}
//			}
//	}
//}

float pdistance(int x0,int y0,int z0,int x,int y,int z) { //计算三维坐标系两点距离
    float a = sqrt(pow(x0-x,2)+pow(y0-y,2)+pow(z0-z,2));
    return a;
}


float map(float i,float imin,float imax,float rmin,float rmax) {
//    i = limit(i,imin,imax);
    if(fabs(imax-imin)!=0)
        return ((i - imin) * (rmax - rmin)/(imax - imin) + rmin);
    else
        return 0;
}

float limit(float i,float min,float max) {
    if(i<min)i=min;
    if(i>max)i=max;
    return i;
}

int outside(double a,double b,double c) { //如果a越界b，c,则返回1
    return ((a<b) || (a>c));
}

void abs_limit(float *a, float ABS_MAX) {
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

///**
//  ******************************************************************************
//  *作者     ：tou_zi
//  *编写时间 ：2019年4月6日
//  *修改时间 ：2019年4月6日
//  *函数名    ：initPose_Module
//  *函数功能 ：初始化姿态解算模块
//  *输入1    ：*pose 姿态解算结构体指针
//  *返回值    ：无
//  ******************************************************************************
//  */


////#include "pose.h"
////#include "math.h"

//void initPose_Module(Pose_Module *pose)
//{
////标志位初始化
//  pose->flag.run = 1;           //开启计算
//  pose->flag.use_mag = 0;       //使用电子罗盘
////接口初始化
//  pose->interface.data.a_x = 0;
//  pose->interface.data.a_y = 0;
//  pose->interface.data.a_z = 0;
//  pose->interface.data.g_x = 0;
//  pose->interface.data.g_y = 0;
//  pose->interface.data.g_z = 0;
//  pose->interface.data.m_x = 0;
//  pose->interface.data.m_y = 0;
//  pose->interface.data.m_z = 0;
////参数初始化
//  pose->parameter.error_ki = 0;
//  pose->parameter.error_kp = 0.65f;
//  pose->parameter.correct_kp = 0.45f;
////中间变量清空
//  pose->process.error.x = 0;
//  pose->process.error.y = 0;
//  pose->process.error.z = 0;
//  pose->process.error_integral.x = 0;
//  pose->process.error_integral.y = 0;
//  pose->process.error_integral.z = 0;
//
//  pose->process.quaternion[0] = 1;
//  pose->process.quaternion[1] = 0;
//  pose->process.quaternion[2] = 0;
//  pose->process.quaternion[3] = 0;
////数据初始化
//  pose->data.rotate_matrix[0][0] = 0;
//  pose->data.rotate_matrix[0][1] = 0;
//  pose->data.rotate_matrix[0][2] = 0;
//  pose->data.rotate_matrix[1][0] = 0;
//  pose->data.rotate_matrix[1][1] = 0;
//  pose->data.rotate_matrix[1][2] = 0;
//  pose->data.rotate_matrix[2][0] = 0;
//  pose->data.rotate_matrix[2][1] = 0;
//  pose->data.rotate_matrix[2][2] = 0;
//
//  pose->data.mag_world.x = 0;
//  pose->data.mag_world.y = 0;
//  pose->data.mag_world.z = 0;
//
//  pose->data.acc_world.x = 0;
//  pose->data.acc_world.y = 0;
//  pose->data.acc_world.z = 0;
//
//  pose->data.mag_correct.x = 0;
//  pose->data.mag_correct.y = 0;
//  pose->data.mag_correct.z = 0;
//
//  pose->data.acc_correct.x = 0;
//  pose->data.acc_correct.y = 0;
//  pose->data.acc_correct.z = 0;
//
//  pose->data.gyro_correct.x = 0;
//  pose->data.gyro_correct.y = 0;
//  pose->data.gyro_correct.z = 0;
//
//  pose->data.pit = 0;
//  pose->data.rol = 0;
//  pose->data.yaw = 0;
//}

//void simple_3d_trans(XYZ_Data_f *ref, XYZ_Data_f *in, XYZ_Data_f *out) //小范围内正确。
//{
//  static s8 pn;
//  static float h_tmp_x,h_tmp_y;
//
//  h_tmp_x = sqrt(ref->z * ref->z + ref->y * ref->y);
//  h_tmp_y = sqrt(ref->z * ref->z + ref->x * ref->x);
//
//  pn = ref->z < 0 ? -1 : 1;
//
//  out->x = ( h_tmp_x *in->x - pn *ref->x *in->z ) ;
//  out->y = ( pn *h_tmp_y *in->y - ref->y *in->z ) ;
//  out->z = ref->x *in->x + ref->y *in->y + ref->z *in->z ;

//}


///**
//  ******************************************************************************
//  *作者     ：tou_zi
//  *编写时间 ：2019年4月6日
//  *修改时间 ：2019年4月6日
//  *函数名    ：calculatePose_Module
//  *函数功能 ：姿态解算 --  四元数解算
//  *输入1    ：*pose 姿态解算结构体指针
//  *输入2    ：cycle 周期
//  *返回值    ：无
//  ******************************************************************************
//  */

//void calculatePose_Module(Pose_Module *pose, float cycle)
//{
//  float length;
//  XYZ_Data_f acc_tmp;
//  XYZ_Data_f error;
//
//  if (pose->flag.run == 0)
//    return;
//
///////////////////////////////////////////////////////////////////////////////////////////////////
//  //电子罗盘处理
//  if (pose->flag.use_mag == 1)
//  {
//    //利用电子罗盘计算yaw
//    length = sqrt(pose->data.mag_correct.x * pose->data.mag_correct.x + pose->data.mag_correct.y * pose->data.mag_correct.y);
//    if( pose->data.mag_correct.x != 0 && pose->data.mag_correct.y != 0 && pose->data.mag_correct.z != 0 && length != 0)
//    {
//      pose->process.mag_yaw = arctan2(pose->data.mag_correct.y / length, pose->data.mag_correct.x / length);
//    }

//    //计算yaw偏差
//    if(pose->data.rotate_matrix[2][2] > 0.0f )
//    {
//      pose->process.mag_yaw_bias = pose->parameter.correct_kp * translateAngle(pose->data.yaw - pose->process.mag_yaw);
//      //矫正值过大 -- 矫正值错误
//      if(pose->process.mag_yaw_bias > 360 || pose->process.mag_yaw_bias < -360)
//      {
//        pose->process.mag_yaw_bias = 0;
//      }
//    }
//
//    else
//    {
//      pose->process.mag_yaw_bias = 0; //角度过大，停止修正，修正的目标值可能不正确
//    }
//  }

//  else
//  {
//    pose->process.mag_yaw_bias = 0;
//  }
///////////////////////////////////////////////////////////////////////////////////////////////////
//  //加速度计处理
//  length = sqrt(  *(pose->interface.data.a_x) * *(pose->interface.data.a_x) +
//                  *(pose->interface.data.a_y) * *(pose->interface.data.a_y) +
//                  *(pose->interface.data.a_z) * *(pose->interface.data.a_z));
//
//  if( ABS(*(pose->interface.data.a_x)) < 1050.0f &&
//      ABS(*(pose->interface.data.a_y)) < 1050.0f &&
//      ABS(*(pose->interface.data.a_z)) < 1050.0f )
//  {
//    //加速度计归一化
//    acc_tmp.x = *(pose->interface.data.a_x) / length;
//    acc_tmp.y = *(pose->interface.data.a_y) / length;
//    acc_tmp.z = *(pose->interface.data.a_z) / length;
//
//    //叉乘计算偏差  --  在无人机稳定时进行补偿
//    if(800.0f < length && length < 1200.0f)
//    {
//      /* 叉乘得到误差 */
//      error.x = (acc_tmp.y * pose->data.rotate_matrix[2][2] - acc_tmp.z * pose->data.rotate_matrix[1][2]);
//      error.y = (acc_tmp.z * pose->data.rotate_matrix[0][2] - acc_tmp.x * pose->data.rotate_matrix[2][2]);
//      error.z = (acc_tmp.x * pose->data.rotate_matrix[1][2] - acc_tmp.y * pose->data.rotate_matrix[0][2]);
//
//      /* 误差低通 */
//      pose->process.error.x += 1.0f * 3.14f * cycle *(error.x  - pose->process.error.x );
//      pose->process.error.y += 1.0f * 3.14f * cycle *(error.y  - pose->process.error.y );
//      pose->process.error.z += 1.0f * 3.14f * cycle *(error.z  - pose->process.error.z );
//    }
//  }
//  else
//  {
//    pose->process.error.x = 0;
//    pose->process.error.y = 0  ;
//    pose->process.error.z = 0 ;
//  }

//  // 误差积分
//  pose->process.error_integral.x += pose->process.error.x * pose->parameter.error_ki * cycle;
//  pose->process.error_integral.y += pose->process.error.y * pose->parameter.error_ki * cycle;
//  pose->process.error_integral.z += pose->process.error.z * pose->parameter.error_ki * cycle;
//
//  //积分限幅 -- 2°以内
//  pose->process.error_integral.x = LIMIT(pose->process.error_integral.x, - 0.035f ,0.035f );
//  pose->process.error_integral.y = LIMIT(pose->process.error_integral.y, - 0.035f ,0.035f );
//  pose->process.error_integral.z = LIMIT(pose->process.error_integral.z, - 0.035f ,0.035f );
//
///////////////////////////////////////////////////////////////////////////////////////////////////
//  //开始修正陀螺仪值
//  pose->data.gyro_correct.x = (*(pose->interface.data.g_x) - pose->data.rotate_matrix[0][2] * pose->process.mag_yaw_bias) * 0.01745329f +
//            (pose->parameter.error_kp * pose->process.error.x + pose->process.error_integral.x) ;
//  pose->data.gyro_correct.y = (*(pose->interface.data.g_y) - pose->data.rotate_matrix[1][2] * pose->process.mag_yaw_bias) * 0.01745329f +
//            (pose->parameter.error_kp * pose->process.error.y + pose->process.error_integral.y) ;
//  pose->data.gyro_correct.z = (*(pose->interface.data.g_z) - pose->data.rotate_matrix[2][2] * pose->process.mag_yaw_bias) * 0.01745329f +
//            (pose->parameter.error_kp * pose->process.error.z + pose->process.error_integral.z) ;
//
///////////////////////////////////////////////////////////////////////////////////////////////////
//  // 一阶龙格库塔更新四元数值
//  pose->process.quaternion[0] += (-pose->process.quaternion[1] * pose->data.gyro_correct.x - pose->process.quaternion[2] * pose->data.gyro_correct.y - pose->process.quaternion[3] * pose->data.gyro_correct.z) * cycle / 2.0f;
//  pose->process.quaternion[1] +=  (pose->process.quaternion[0] * pose->data.gyro_correct.x + pose->process.quaternion[2] * pose->data.gyro_correct.z - pose->process.quaternion[3] * pose->data.gyro_correct.y) * cycle / 2.0f;
//  pose->process.quaternion[2] +=  (pose->process.quaternion[0] * pose->data.gyro_correct.y - pose->process.quaternion[1] * pose->data.gyro_correct.z + pose->process.quaternion[3] * pose->data.gyro_correct.x) * cycle / 2.0f;
//  pose->process.quaternion[3] +=  (pose->process.quaternion[0] * pose->data.gyro_correct.z + pose->process.quaternion[1] * pose->data.gyro_correct.y - pose->process.quaternion[2] * pose->data.gyro_correct.x) * cycle / 2.0f;

//  //四元数归一化
//  length = sqrt(pose->process.quaternion[0] * pose->process.quaternion[0] +
//                pose->process.quaternion[1] * pose->process.quaternion[1] +
//                pose->process.quaternion[2] * pose->process.quaternion[2] +
//                pose->process.quaternion[3] * pose->process.quaternion[3]);
//
//  if (length != 0)
//  {
//    pose->process.quaternion[0] /= length;
//    pose->process.quaternion[1] /= length;
//    pose->process.quaternion[2] /= length;
//    pose->process.quaternion[3] /= length;
//  }
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
//  //计算旋转矩阵
//  pose->data.rotate_matrix[0][0] =  pose->process.quaternion[0] * pose->process.quaternion[0] + pose->process.quaternion[1] * pose->process.quaternion[1] -
//                                    pose->process.quaternion[2] * pose->process.quaternion[2] - pose->process.quaternion[3] * pose->process.quaternion[3];
//  pose->data.rotate_matrix[0][1] =  2 * (pose->process.quaternion[1] * pose->process.quaternion[2] + pose->process.quaternion[0] * pose->process.quaternion[3]);
//  pose->data.rotate_matrix[0][2] =  2 * (pose->process.quaternion[1] * pose->process.quaternion[3] - pose->process.quaternion[0] * pose->process.quaternion[2]);
//
//  pose->data.rotate_matrix[1][0] =  2 * (pose->process.quaternion[1] * pose->process.quaternion[2] - pose->process.quaternion[0] * pose->process.quaternion[3]);
//  pose->data.rotate_matrix[1][1] =  pose->process.quaternion[0] * pose->process.quaternion[0] - pose->process.quaternion[1] * pose->process.quaternion[1] +
//                                    pose->process.quaternion[2] * pose->process.quaternion[2] - pose->process.quaternion[3] * pose->process.quaternion[3];
//  pose->data.rotate_matrix[1][2] =  2 * (pose->process.quaternion[2] * pose->process.quaternion[3] + pose->process.quaternion[0] * pose->process.quaternion[1]);
//
//  pose->data.rotate_matrix[2][0] =  2 * (pose->process.quaternion[1] * pose->process.quaternion[3] + pose->process.quaternion[0] * pose->process.quaternion[2]);
//  pose->data.rotate_matrix[2][1] =  2 * (pose->process.quaternion[2] * pose->process.quaternion[3] - pose->process.quaternion[0] * pose->process.quaternion[1]);
//  pose->data.rotate_matrix[2][2] =  pose->process.quaternion[0] * pose->process.quaternion[0] - pose->process.quaternion[1] * pose->process.quaternion[1] -
//                                    pose->process.quaternion[2] * pose->process.quaternion[2] + pose->process.quaternion[3] * pose->process.quaternion[3];
//
//  //计算世界坐标系下的磁力计值
//  if (pose->flag.use_mag == 1)
//  {
//    pose->data.mag_world.x =  pose->data.rotate_matrix[0][0] * *(pose->interface.data.m_x) +
//                              pose->data.rotate_matrix[1][0] * *(pose->interface.data.m_y) +
//                              pose->data.rotate_matrix[2][0] * *(pose->interface.data.m_z) ;
//
//    pose->data.mag_world.y =  pose->data.rotate_matrix[0][1] * *(pose->interface.data.m_x) +
//                              pose->data.rotate_matrix[1][1] * *(pose->interface.data.m_y) +
//                              pose->data.rotate_matrix[2][1] * *(pose->interface.data.m_z) ;
//
//    pose->data.mag_world.z =  pose->data.rotate_matrix[0][2] * *(pose->interface.data.m_x) +
//                              pose->data.rotate_matrix[1][2] * *(pose->interface.data.m_y) +
//                              pose->data.rotate_matrix[2][2] * *(pose->interface.data.m_z) ;
//  }
//
//  //计算世界坐标系下的加速度值
//  pose->data.acc_world.x =  pose->data.rotate_matrix[0][0] * *(pose->interface.data.a_x) +
//                            pose->data.rotate_matrix[1][0] * *(pose->interface.data.a_y) +
//                            pose->data.rotate_matrix[2][0] * *(pose->interface.data.a_z) ;
//
//  pose->data.acc_world.y =  pose->data.rotate_matrix[0][1] * *(pose->interface.data.a_x) +
//                            pose->data.rotate_matrix[1][1] * *(pose->interface.data.a_y) +
//                            pose->data.rotate_matrix[2][1] * *(pose->interface.data.a_z) ;
//
//  pose->data.acc_world.z =  pose->data.rotate_matrix[0][2] * *(pose->interface.data.a_x) +
//                            pose->data.rotate_matrix[1][2] * *(pose->interface.data.a_y) +
//                            pose->data.rotate_matrix[2][2] * *(pose->interface.data.a_z) ;
//
//  //求解欧拉角
//  pose->data.rol = arctan2(pose->data.rotate_matrix[2][2], pose->data.rotate_matrix[1][2]);
//  pose->data.pit = -arcsin(pose->data.rotate_matrix[0][2]);
//  pose->data.yaw = arctan2(pose->data.rotate_matrix[0][0], pose->data.rotate_matrix[0][1]);
//
///////////////////////////////////////////////////////////////////////////////////////////////////
//  //计算机体坐标系矫正后的加速度--不受俯仰和翻滚影响
//  pose->data.acc_correct.x =   pose->data.acc_world.x * cos(pose->data.yaw) + pose->data.acc_world.y * sin(pose->data.yaw);
//  pose->data.acc_correct.y =  -pose->data.acc_world.x * sin(pose->data.yaw) + pose->data.acc_world.y * cos(pose->data.yaw);
//  pose->data.acc_correct.z =   pose->data.acc_world.z;
//
//  //计算机体坐标系矫正后的磁场--不受俯仰和翻滚影响
//  if (pose->flag.use_mag == 1)
//  {
//    XYZ_Data_f ref_v = (XYZ_Data_f){pose->data.rotate_matrix[0][2], pose->data.rotate_matrix[1][2], pose->data.rotate_matrix[2][2]};
//    XYZ_Data_f mag_tmp = (XYZ_Data_f){*pose->interface.data.m_x, *pose->interface.data.m_y, *pose->interface.data.m_z};
//
//    length =  sqrt( *(pose->interface.data.m_x) * *(pose->interface.data.m_x) +
//                    *(pose->interface.data.m_y) * *(pose->interface.data.m_y) +
//                    *(pose->interface.data.m_z) * *(pose->interface.data.m_z));
//
////    length *= sqrt( pose->data.rotate_matrix[0][2] * pose->data.rotate_matrix[0][2] +
////                    pose->data.rotate_matrix[1][2] * pose->data.rotate_matrix[1][2] +
////                    pose->data.rotate_matrix[2][2] * pose->data.rotate_matrix[2][2] );
////
//    if (length != 0)
//    {
//      simple_3d_trans(&ref_v, &mag_tmp, &pose->data.mag_correct);
////      pose->data.mag_correct.z = pose->data.mag_world.z;
////      pose->data.mag_correct.x = sqrt(1 - (pose->data.mag_correct.z / length) * (pose->data.mag_correct.z / length)) * *(pose->interface.data.m_x);
////      pose->data.mag_correct.y = sqrt(1 - (pose->data.mag_correct.z / length) * (pose->data.mag_correct.z / length)) * *(pose->interface.data.m_y);
//    }
//  }
//
//}

///*******************************************************************************
//* 函 数 名         : Sqrt
//* 函数功能         : 快速算根值
//* 输    入         : number 根号里边的数
//* 输    出         : 最大的绝对值
//*******************************************************************************/
////float sqrt(float number)
////{
////  long i;
////  float x, y;
////  const float f = 1.5F;
////  x = number * 0.5F;
////  y = number;
////  i = * ( long * ) &y;
////  i = 0x5f3759df - ( i >> 1 );

////  y = * ( float * ) &i;
////  y = y * ( f - ( x * y * y ) );
////  y = y * ( f - ( x * y * y ) );
////  return number * y;
////}
///*******************************************************************************
//* 函 数 名         : Angle_Translate
//* 函数功能         : 将角度转变为-180~180之间
//* 输    入         :
//* 输    出         :
//*******************************************************************************/
//float translateAngle(float angle)
//{
//  while (angle > ANGLE_MAX)         //判断目标角度是否在允许角度范围
//    angle -= 360.0f;
//  while (angle < ANGLE_MIN)
//    angle += 360.0f;
//  return angle;
//}
///*******************************************************************************
//* 函 数 名         : Sin
//* 函数功能         :
//* 输    入         :
//* 输    出         :
//*******************************************************************************/
////float sin(float angle)
////{
////  float sine, rad;
////  angle = translateAngle(angle);
////  rad = angle * PI / 180.0f;
////
////  sine = (rad < 0) ? rad * (1.27323954f + 0.405284735f * rad) : rad * (1.27323954f - 0.405284735f * rad);
////  sine = (sine < 0) ? sine * (-0.225f * (sine + 1) + 1) : sine * (0.225f * (sine - 1) + 1);
////  return sine;
////}
///*******************************************************************************
//* 函 数 名         : Cos
//* 函数功能         :
//* 输    入         :
//* 输    出         :
//*******************************************************************************/
////float cos(float angle)
////{
////  return sin(angle + 90.0f);
////}
///*******************************************************************************
//* 函 数 名         : Tan
//* 函数功能         :
//* 输    入         :
//* 输    出         :
//*******************************************************************************/
////float tan(float angle)
////{
////  return sin(angle) / cos(angle);
////}
///*******************************************************************************
//* 函 数 名         : ATan1
//* 函数功能         : 反正切 （+- 0.09°）
//* 输    入         :
//* 输    出         :
//*******************************************************************************/
//float arctan1(float tan)
//{
//  float angle = (ABS(tan) > 1.0f) ?                                                                                 \
//                90.0f - ABS(1.0f / tan) * (45.0f - (ABS(1.0f / tan) - 1.0f) * (14.0f + 3.83f * ABS(1.0f / tan)))  : \
//                ABS(tan) * (45.0f - (ABS(tan) - 1.0f) * (14.0f + 3.83f * ABS(tan)));
//  return (tan > 0) ? angle : -angle;
//}
///*******************************************************************************
//* 函 数 名         : ATan2
//* 函数功能         : 反正切 （+- 0.09°）
//* 输    入         :
//* 输    出         :
//*******************************************************************************/
//float arctan2(float x, float y)
//{
//  float tan, angle;
//
//  if (x == 0 && y == 0)   //不存在
//    return 0;
//
//  if (x == 0)             //y轴上
//  {
//    if (y > 0)
//      return 90;
//    else
//      return -90;
//  }
//
//  if (y == 0)             //x轴上
//  {
//    if (x > 0)
//      return 0;
//    else
//      return -180.0f;
//  }
//
//  tan = y / x;
//  angle = arctan1(tan);
//  if (x < 0 && angle > 0)
//    angle -= 180.0f;
//  else if (x < 0 && angle < 0)
//    angle += 180.0f;
//  return angle;
//}
///*******************************************************************************
//* 函 数 名         : arcsin1
//* 函数功能         : 反正切 （+- 0.09°）
//* 输    入         :
//* 输    出         :
//*******************************************************************************/
//float arcsin(float i)
//{
//  return arctan1(i / sqrt(1 - i * i));
//}
///*******************************************************************************
//* 函 数 名         : Exp
//* 函数功能         : e指数
//* 输    入         :
//* 输    出         :
//*******************************************************************************/
////float exp(float x)  //计算e^x,实现系统的exp()功能?
////{
////  if (x == 0) return 1;
////  if (x<0) return 1 / exp(-x);

////  double y = x, ex_p1 = 0, ex_p2 = 0, ex_p3 = 0, ex_px = 0, ex_tmp = 1, dex_px = 1, tmp;
////  int l;
////  for (l = 1, tmp = 1; ((ex_px - ex_tmp)>1e-10 || (ex_px - ex_tmp)<-1e-10) && dex_px>1e-10; l++)
////  {
////    ex_tmp = ex_px;
////    tmp *= y;
////    tmp = tmp / l;
////    ex_p1 += tmp;
////    ex_p2 = ex_p1 + tmp*y / (l + 1);
////    ex_p3 = ex_p2 + tmp*y*y / (l + 1) / (l + 2);
////    dex_px = ex_p3 - ex_p2;
////    ex_px = ex_p3 - dex_px*dex_px / (ex_p3 - 2 * ex_p2 + ex_p1);
////  }
////  return ex_px + 1;
////}



