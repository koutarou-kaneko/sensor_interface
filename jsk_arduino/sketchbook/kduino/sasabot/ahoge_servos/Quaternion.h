// Quatanion Rotation Matrix Function
// http://marupeke296.com/DXG_No58_RotQuaternionTrans.html

#ifdef USE_QUATERNION

//http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt( float number )
{
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = number * 0.5F;
  y  = number;
  i  = * ( long * ) &y;                       // evil floating point bit level hacking
  i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
  y  = * ( float * ) &i;
  y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
  //	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

  return y;
}

void outerProduct(
float& ansx, float& ansy, float& ansz,
float& v1x,  float& v1y,  float& v1z,
float& v2x,  float& v2y,  float& v2z
) {
  ansx = v1y*v2z - v1z*v2y;
  ansy = v1z*v2x - v1x*v2z;
  ansz = v1x*v2y - v1y*v2x;
}

inline
void outerProduct(float ans[], float v1[], float v2[])
{
  outerProduct(
  ans[0], ans[1], ans[2],
  v1[0],  v1[1],  v1[2],
  v2[0],  v2[1],  v2[2]
    );
}

float innerProduct(
float v1x, float v1y, float v1z,
float v2x, float v2y, float v2z
)
{
  float ans;
  ans = v1x*v2x+v1y*v2y+v1z*v2z;
  return ans;
}

inline
float innerProduct(float v1[], float v2[])
{
  float ans;
  ans = innerProduct(
  v1[0], v1[1], v1[2],
  v2[0], v2[1], v2[2]
    );
  return ans;
}

void unitVector(
float& ansx, float& ansy, float& ansz,
float  vx,   float  vy,   float vz
)
{
  float tmp;
  tmp = invSqrt(vx*vx+vy*vy+vz*vz);
  ansx = tmp*vx;
  ansy = tmp*vy;
  ansz = tmp*vz;
}

inline
void unitVector(float ans[], float v[])
{
  unitVector(
  ans[0], ans[1], ans[2],
  v[0],   v[1],   v[2]
    );
}

///////////////////////////////////////////////
// クォータニオン→回転行列変換
//
// m11-m33 : 回転行列成分（出力）
// qx, qy, qz, qw : クォータニオン成分
//
// ※注意：
// 行列成分はDirectX形式（行方向が軸の向き）です
// OpenGL形式（列方向が軸の向き）の場合は
// 転置した値を格納するようにして下さい。

void transformQuaternionToRotMat(
float &m11, float &m12, float &m13,
float &m21, float &m22, float &m23,
float &m31, float &m32, float &m33,
float qx, float qy, float qz, float qw
) {
  m11 = 1.0f - 2.0f * qy * qy - 2.0f * qz * qz;
  m12 = 2.0f * qx * qy + 2.0f * qw * qz;
  m13 = 2.0f * qx * qz - 2.0f * qw * qy;

  m21 = 2.0f * qx * qy - 2.0f * qw * qz;
  m22 = 1.0f - 2.0f * qx * qx - 2.0f * qz * qz;
  m23 = 2.0f * qy * qz + 2.0f * qw * qx;

  m31 = 2.0f * qx * qz + 2.0f * qw * qy;
  m32 = 2.0f * qy * qz - 2.0f * qw * qx;
  m33 = 1.0f - 2.0f * qx * qx - 2.0f * qy * qy;
}


///////////////////////////////////////////////
// クォータニオン→回転行列変換
//
// m1-m3 : 回転行列成分（出力）
// q : クォータニオン成分(x,y,z,w)
//
// ※注意：
// 行列成分はDirectX形式（行方向が軸の向き）です
// OpenGL形式（列方向が軸の向き）の場合は
// 転置した値を格納するようにして下さい。

void transformQuaternionToRotMat(
float m1[3],
float m2[3],
float m3[3],
const float q[4]
) {
  transformQuaternionToRotMat(
  m1[0], m1[1], m1[2],
  m2[0], m2[1], m2[2],
  m3[0], m3[1], m3[2],
  q[0], q[1], q[2], q[3]
    );
}


///////////////////////////////////////////////
// クォータニオン→回転行列変換
//
// m : 回転行列成分（出力）
// q : クォータニオン成分(x, y, z, w)
//
// ※注意：
// 行列成分はDirectX形式（行方向が軸の向き）です
// OpenGL形式（列方向が軸の向き）の場合は
// 転置した値を格納するようにして下さい。

void transformQuaternionToRotMat(
float m[16],
const float q[4]
) {
  memset( m, 0, sizeof(float) * 16 );
  transformQuaternionToRotMat(
  m[0], m[1], m[2],
  m[4], m[5], m[6],
  m[8], m[9], m[10],
  q[0], q[1], q[2], q[3]
    );
  m[15] = 1.0f;
}


///////////////////////////////////////////////
// 回転行列→クォータニオン変換
//
// qx, qy, qz, qw : クォータニオン成分（出力）
// m11-m33 : 回転行列成分
//
// ※注意：
// 行列成分はDirectX形式（行方向が軸の向き）です
// OpenGL形式（列方向が軸の向き）の場合は
// 転置した値を入れて下さい。

bool transformRotMatToQuaternion(
float &qx, float &qy, float &qz, float &qw,
float m11, float m12, float m13,
float m21, float m22, float m23,
float m31, float m32, float m33
) {
  // 最大成分を検索
  float elem[ 4 ]; // 0:x, 1:y, 2:z, 3:w
  elem[ 0 ] = m11 - m22 - m33 + 1.0f;
  elem[ 1 ] = -m11 + m22 - m33 + 1.0f;
  elem[ 2 ] = -m11 - m22 + m33 + 1.0f;
  elem[ 3 ] = m11 + m22 + m33 + 1.0f;

  unsigned biggestIndex = 0;
  for ( int i = 1; i < 4; i++ ) {
    if ( elem[i] > elem[biggestIndex] )
      biggestIndex = i;
  }

  if ( elem[biggestIndex] < 0.0f )
    return false; // 引数の行列に間違いあり！

  // 最大要素の値を算出
  float *q[4] = {
    &qx, &qy, &qz, &qw  };
  float v = sqrt( elem[biggestIndex] ) * 0.5f;
  *q[biggestIndex] = v;
  float mult = 0.25f / v;

  switch ( biggestIndex ) {
  case 0: // x
    *q[1] = (m12 + m21) * mult;
    *q[2] = (m31 + m13) * mult;
    *q[3] = (m23 - m32) * mult;
    break;
  case 1: // y
    *q[0] = (m12 + m21) * mult;
    *q[2] = (m23 + m32) * mult;
    *q[3] = (m31 - m13) * mult;
    break;
  case 2: // z
    *q[0] = (m31 + m13) * mult;
    *q[1] = (m23 + m32) * mult;
    *q[3] = (m12 - m21) * mult;
    break;
  case 3: // w
    *q[0] = (m23 - m32) * mult;
    *q[1] = (m31 - m13) * mult;
    *q[2] = (m12 - m21) * mult;
    break;
  }

  return true;
}

///////////////////////////////////////////////
// 回転行列→クォータニオン変換
//
// qx, qy, qz, qw : クォータニオン成分（出力）
// m1[3] : 回転行列成分 m11 - m13
// m2[3] : 回転行列成分 m21 - m23
// m3[3] : 回転行列成分 m31 - m33
//
// ※注意：
// 行列成分はDirectX形式（行方向が軸の向き）です
// OpenGL形式（列方向が軸の向き）の場合は
// 転置した値を入れて下さい。

bool transformRotMatToQuaternion(
float q[4],
const float m1[3],
const float m2[3],
const float m3[3]
) {
  return transformRotMatToQuaternion(
  q[0], q[1], q[2], q[3],
  m1[0], m1[1], m1[2],
  m2[0], m2[1], m2[2],
  m3[0], m3[1], m3[2]
    );
}

////////////////////////////////////////////
// クォータニオン球面線形補間
//

bool slerpQuaternion(
float out[4],
const float q1[4],
const float q2[4],
float t
) {
  // 角度算出
  const float len1 = sqrt( q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2] + q1[3] * q1[3] );
  const float len2 = sqrt( q2[0] * q2[0] + q2[1] * q2[1] + q2[2] * q2[2] + q2[3] * q2[3] );

  if ( len1 == 0.0f || len2 == 0.0f )
    return false; // 不正なクォータニオン

  const float cos_val = (q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3]) / (len1 * len2);
  const float w = acos( cos_val );

  // 球面線形補間
  const float sin_w = sin( w );
  const float sin_t_w = sin( t * w );
  const float sin_inv_t_w = sin( (1.0f - t) * w );
  const float mult_q1 = sin_inv_t_w / sin_w;
  const float mult_q2 = sin_t_w / sin_w;

  for ( int i = 0; i < 4; i++ ) {
    out[i] = mult_q1 * q1[i] + mult_q2 * q2[i];
  }
  return true;
}


////////////////////////////////////////////
// 回転行列による補間(汎用)
//
// out : 補間回転行列（出力）
// rm1 : 始点回転行列
// rm2 : 終点回転行列
// t : 補間値（0.0f～1.0f）

void slerpRotMat(
float out[16],
const float rm1[16],
const float rm2[16],
float t
) {
  memset( out, 0, sizeof(float) * 16 );

  // 始点・終点のクォータニオンを算出
  float Q[4], Q1[4], Q2[4];
  const float *x1 = rm1 + 0;
  const float *y1 = rm1 + 4;
  const float *z1 = rm1 + 8;
  const float *x2 = rm2 + 0;
  const float *y2 = rm2 + 4;
  const float *z2 = rm2 + 8;
  transformRotMatToQuaternion( Q1, x1, y1, z1 );
  transformRotMatToQuaternion( Q2, x2, y2, z2 );

  // クォータニオン間の球面線形補間
  slerpQuaternion( Q, Q1, Q2, t );

  // 回転行列に戻す
  transformQuaternionToRotMat( out, Q );
}

#endif// USE_QUATERNION

