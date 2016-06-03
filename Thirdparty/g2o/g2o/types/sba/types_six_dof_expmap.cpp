// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "types_six_dof_expmap.h"

#include "../../core/factory.h"
#include "../../stuff/macros.h"

namespace g2o {

using namespace std;

G2O_REGISTER_TYPE_GROUP(expmap);
G2O_REGISTER_TYPE(VERTEX_SE3:EXPMAP, VertexSE3Expmap);
G2O_REGISTER_TYPE(EDGE_SE3:EXPMAP, EdgeSE3Expmap);
G2O_REGISTER_TYPE(EDGE_PROJECT_XYZ2UV:EXPMAP, EdgeProjectXYZ2UV);
G2O_REGISTER_TYPE(EDGE_PROJECT_XYZ2UVU:EXPMAP, EdgeProjectXYZ2UVU);
G2O_REGISTER_TYPE(PARAMS_CAMERAPARAMETERS, CameraParameters);
//G2O_REGISTER_TYPE(EDGE_PROJECT_INVERSE_DEPTH2SE3:EXPMAP, EdgeProjectInverseDepth2SE3);

CameraParameters
::CameraParameters()
  : focal_length(1.),
    principle_point(Vector2d(0., 0.)),
    baseline(0.5)  {
}

Vector2d project2d(const Vector3d& v)  {
  Vector2d res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

Vector3d unproject2d(const Vector2d& v)  {
  Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

inline Vector3d invert_depth(const Vector3d & x){
  return unproject2d(x.head<2>())/x[2];
}

Vector2d  CameraParameters::cam_map(const Vector3d & trans_xyz) const {
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*focal_length + principle_point[0];
  res[1] = proj[1]*focal_length + principle_point[1];
  return res;
}

Vector3d CameraParameters::stereocam_uvu_map(const Vector3d & trans_xyz) const {
  Vector2d uv_left = cam_map(trans_xyz);
  double proj_x_right = (trans_xyz[0]-baseline)/trans_xyz[2];
  double u_right = proj_x_right*focal_length + principle_point[0];
  return Vector3d(uv_left[0],uv_left[1],u_right);
}


VertexSE3Expmap::VertexSE3Expmap() : BaseVertex<6, SE3Quat>() {
}

bool VertexSE3Expmap::read(std::istream& is) {
  Vector7d est;
  for (int i=0; i<7; i++)
    is  >> est[i];
  SE3Quat cam2world;
  cam2world.fromVector(est);
  setEstimate(cam2world.inverse());
  return true;
}

bool VertexSE3Expmap::write(std::ostream& os) const {
  SE3Quat cam2world(estimate().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  return os.good();
}

void VertexSE3Expmap::oplusImpl(const double *update_)
{
    Eigen::Map<const Vector6d> update(update_);
    setEstimate(SE3Quat::exp(update)*estimate());
}

EdgeSE3Expmap::EdgeSE3Expmap() :
  BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>() {
}

bool EdgeSE3Expmap::read(std::istream& is)  {
  Vector7d meas;
  for (int i=0; i<7; i++)
    is >> meas[i];
  SE3Quat cam2world;
  cam2world.fromVector(meas);
  setMeasurement(cam2world.inverse());
  //TODO: Convert information matrix!!
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3Expmap::write(std::ostream& os) const {
  SE3Quat cam2world(measurement().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

EdgeProjectXYZ2UV::EdgeProjectXYZ2UV() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>() {
  _cam = 0;
  resizeParameters(1);
  installParameter(_cam, 0);
}

bool EdgeProjectPSI2UV::write(std::ostream& os) const  {
  os << _cam->id() << " ";
  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

bool EdgeProjectPSI2UV::read(std::istream& is) {
  int paramId;
  is >> paramId;
  setParameterId(0, paramId);

  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

void EdgeProjectPSI2UV::computeError(){
  const VertexSBAPointXYZ * psi = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
  const VertexSE3Expmap * T_p_from_world = static_cast<const VertexSE3Expmap*>(_vertices[1]);
  const VertexSE3Expmap * T_anchor_from_world = static_cast<const VertexSE3Expmap*>(_vertices[2]);
  const CameraParameters * cam = static_cast<const CameraParameters *>(parameter(0));

  Vector2d obs(_measurement);
  _error = obs - cam->cam_map(T_p_from_world->estimate()
        *T_anchor_from_world->estimate().inverse()
        *invert_depth(psi->estimate()));
}

inline Matrix<double,2,3> d_proj_d_y(const double & f, const Vector3d & xyz){
  double z_sq = xyz[2]*xyz[2];
  Matrix<double,2,3> J;
  J << f/xyz[2], 0,           -(f*xyz[0])/z_sq,
      0,           f/xyz[2], -(f*xyz[1])/z_sq;
  return J;
}

inline Matrix<double,3,6> d_expy_d_y(const Vector3d & y){
  Matrix<double,3,6> J;
  J.topLeftCorner<3,3>() = -skew(y);
  J.bottomRightCorner<3,3>().setIdentity();

  return J;
}

inline Matrix3d d_Tinvpsi_d_psi(const SE3Quat & T, const Vector3d & psi){
  Matrix3d R = T.rotation().toRotationMatrix();
  Vector3d x = invert_depth(psi);
  Vector3d r1 = R.col(0);
  Vector3d r2 = R.col(1);
  Matrix3d J;
  J.col(0) = r1;
  J.col(1) = r2;
  J.col(2) = -R*x;
  J*=1./psi.z();
  return J;
}

void EdgeProjectPSI2UV::linearizeOplus(){
  VertexSBAPointXYZ* vpoint = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3d psi_a = vpoint->estimate();
  VertexSE3Expmap * vpose = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T_cw = vpose->estimate();
  VertexSE3Expmap * vanchor = static_cast<VertexSE3Expmap *>(_vertices[2]);
  const CameraParameters * cam
      = static_cast<const CameraParameters *>(parameter(0));

  SE3Quat A_aw = vanchor->estimate();
  SE3Quat T_ca = T_cw*A_aw.inverse();
  Vector3d x_a = invert_depth(psi_a);
  Vector3d y = T_ca*x_a;
  Matrix<double,2,3> Jcam
      = d_proj_d_y(cam->focal_length,
                   y);
  _jacobianOplus[0] = -Jcam*d_Tinvpsi_d_psi(T_ca, psi_a);
  _jacobianOplus[1] = -Jcam*d_expy_d_y(y);
  _jacobianOplus[2] = Jcam*T_ca.rotation().toRotationMatrix()*d_expy_d_y(x_a);
}



EdgeProjectXYZ2UVU::EdgeProjectXYZ2UVU() : BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>()
{
}

bool EdgeProjectXYZ2UV::read(std::istream& is){
  int paramId;
  is >> paramId;
  setParameterId(0, paramId);

  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeProjectXYZ2UV::write(std::ostream& os) const {
  os << _cam->id() << " ";
  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeSE3Expmap::linearizeOplus() {
  VertexSE3Expmap * vi = static_cast<VertexSE3Expmap *>(_vertices[0]);
  SE3Quat Ti(vi->estimate());

  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat Tj(vj->estimate());

  const SE3Quat & Tij = _measurement;
  SE3Quat invTij = Tij.inverse();

  SE3Quat invTj_Tij = Tj.inverse()*Tij;
  SE3Quat infTi_invTij = Ti.inverse()*invTij;

  _jacobianOplusXi = invTj_Tij.adj();
  _jacobianOplusXj = -infTi_invTij.adj();
}

void EdgeProjectXYZ2UV::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3d xyz = vi->estimate();
  Vector3d xyz_trans = T.map(xyz);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;

  const CameraParameters * cam = static_cast<const CameraParameters *>(parameter(0));

  Matrix<double,2,3> tmp;
  tmp(0,0) = cam->focal_length;
  tmp(0,1) = 0;
  tmp(0,2) = -x/z*cam->focal_length;

  tmp(1,0) = 0;
  tmp(1,1) = cam->focal_length;
  tmp(1,2) = -y/z*cam->focal_length;

  _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

  _jacobianOplusXj(0,0) =  x*y/z_2 *cam->focal_length;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *cam->focal_length;
  _jacobianOplusXj(0,2) = y/z *cam->focal_length;
  _jacobianOplusXj(0,3) = -1./z *cam->focal_length;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *cam->focal_length;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *cam->focal_length;
  _jacobianOplusXj(1,1) = -x*y/z_2 *cam->focal_length;
  _jacobianOplusXj(1,2) = -x/z *cam->focal_length;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *cam->focal_length;
  _jacobianOplusXj(1,5) = y/z_2 *cam->focal_length;
}

bool EdgeProjectXYZ2UVU::read(std::istream& is){
  for (int i=0; i<3; i++){
    is  >> _measurement[i];
  }
  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeProjectXYZ2UVU::write(std::ostream& os) const {
  for (int i=0; i<3; i++){
    os  << measurement()[i] << " ";
  }

  for (int i=0; i<3; i++)
    for (int j=i; j<3; j++){
      os << " " << information()(i,j);
    }
  return os.good();
}


EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZ::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


void EdgeSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3d xyz = vi->estimate();
  Vector3d xyz_trans = T.map(xyz);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;

  Matrix<double,2,3> tmp;
  tmp(0,0) = fx;
  tmp(0,1) = 0;
  tmp(0,2) = -x/z*fx;

  tmp(1,0) = 0;
  tmp(1,1) = fy;
  tmp(1,2) = -y/z*fy;

  _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;
}

Vector2d EdgeSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

EdgeProjectInverseDepth2SE3::EdgeProjectInverseDepth2SE3(): BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>() {
    bRefinePose = false;
}

bool EdgeProjectInverseDepth2SE3::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeProjectInverseDepth2SE3::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}

void EdgeProjectInverseDepth2SE3::computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    // convert inverse depth parameterization for 3D point location to Euclidean XYZ representation
    Vector3d v2_inverseDepthParam = v2->estimate();
    //in cam frame
    Vector3d v2_XYZ(v2_inverseDepthParam[0]/v2_inverseDepthParam[2], v2_inverseDepthParam[1]/v2_inverseDepthParam[2], 1.0/v2_inverseDepthParam[2]);

    Vector2d obs(_measurement); // in camera frame
    _error = (obs-cam_project(v1->estimate().map(v2_XYZ)));
//    std::cout << "[g2o:471] DEBUG obs " << obs << endl;
//    std::cout << "[g2o:472] DEBUG v1->estimate.map(v2_XYZ) " << v1->estimate().map(v2_XYZ) << endl;
//    std::cout << "[g2o:473] DEBUG v2_XYZ " << v2_XYZ << endl;
//    SE3Quat T(v1->estimate());
//    std::cout << "[g2o:474] DEBUG v1->rotation " << endl << T.rotation().toRotationMatrix() << endl << T.translation() << endl;
//    std::cout << "[g2o:473] Edge " << this->id() << " compute Error: " << sqrt(_error[0]*_error[0] + _error[1]*_error[1]) << " chi2 " << this->chi2() << std::endl;
//    std::cout << v2_inverseDepthParam << endl<< endl;
}


void EdgeProjectInverseDepth2SE3::linearizeOplus() {
    VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
    SE3Quat T(vj->estimate());
    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Vector3d xyRou = vi->estimate(); //[x y rou]

    Matrix<double,3,3> R1 = T.rotation().toRotationMatrix();
    Vector3d t1 = T.translation();
    double r1, r2, r3, r4, r5, r6, r7, r8, r9;
    r1 = R1(0,0); r2 = R1(0,1); r3 = R1(0,2);
    r4 = R1(1,0); r5 = R1(1,1); r6 = R1(1,2);
    r7 = R1(2,0); r8 = R1(2,1); r9 = R1(2,2);

    double a1, b1, c1;
    a1 = t1[0]; b1 = t1[1]; c1 = t1[2];

    double x, y, Rou;
    x = xyRou[0]; y = xyRou[1]; Rou = xyRou[2];

    // assin value to functions
    double g1 = fx*(r1*x + r2*y + r3 + a1*Rou);
    double g2 = fy*(r4*x + r5*y + r6 + b1*Rou);
    double g3 = r7*x + r8*y + r9 + c1*Rou;

    double dg1dx = r1*fx;
    double dg1dy = r2*fx;
    double dg1dRou = a1*fx;

    double dg2dx = r4*fy;
    double dg2dy = r5*fy;
    double dg2dRou = b1*fy;

    double dg3dx = r7;
    double dg3dy = r8;
    double dg3dRou = c1;

    double g32 = g3*g3;
    // assigne value to jacobian matrix for VertexSBAPointXYZ only, since VertexSE3Expmap is fixed
    _jacobianOplusXi(0,0) = (-1.0)*(g3*dg1dx - g1*dg3dx)/g32;
    _jacobianOplusXi(1,0) = (-1.0)*(g3*dg2dx - g2*dg3dx)/g32;

    _jacobianOplusXi(0,1) = (-1.0)*(g3*dg1dy - g1*dg3dy)/g32;
    _jacobianOplusXi(1,1) = (-1.0)*(g3*dg2dy - g2*dg3dy)/g32;

    _jacobianOplusXi(0,2) = (-1.0)*(g3*dg1dRou - g1*dg3dRou)/g32;
    _jacobianOplusXi(1,2) = (-1.0)*(g3*dg2dRou - g2*dg3dRou)/g32;

    // jacobian of SE3 pose
    double w1, w2, w3, t_1, t2, t3; w1 = w2 = w3 = t_1 = t2 = t3 = 0;
    double P11 = r1 - w3*r4 + w2*r7;
    double P12 = r2 - w3*r5 + w2*r8;
    double P13 = r3 - w3*r6 + w2*r9;
    double P14 = a1 - w3*b1 + w2*c1 + t_1;

    double P21 = w3*r1 + r4 - w1*r7;
    double P22 = w3*r2 + r5 - w1*r8;
    double P23 = w3*r3 + r6 - w1*r9;
    double P24 = w3*a1 + b1 - w1*c1 + t2;

    double P31 = -w2*r1 + w1*r4 + r7;
    double P32 = -w2*r2 + w1*r5 + r8;
    double P33 = -w2*r3 + w1*r6 + r9;
    double P34 = -w2*a1 + w1*b1 + c1 + t3;

    double g_1 = P11*x + P12*y + P13 + Rou*P14;
    double g_2 = P21*x + P22*y + P23 + Rou*P24;
    double g_3 = P31*x + P32*y + P33 + Rou*P34;

    double g_33 = g_3*g_3;

    double dg1dw1 = 0;
    double dg1dw2 = r7*x + r8*y + r9 + c1*Rou;
    double dg1dw3 = -r4*x - r5*y - r6 - b1*Rou;
    double dg1dt1 = Rou;
    double dg1dt2 = 0;
    double dg1dt3 = 0;

    double dg2dw1 = -r7*x - r8*y - r9 - c1*Rou;
    double dg2dw2 = 0;
    double dg2dw3 = r1*x + r2*y + r3 + a1*Rou;
    double dg2dt1 = 0;
    double dg2dt2 = Rou;
    double dg2dt3 = 0;

    double dg3dw1 = r4*x + r5*y + r6 + b1*Rou;
    double dg3dw2 = -r1*x - r2*y - r3 - a1*Rou;
    double dg3dw3 = 0;
    double dg3dt1 = 0;
    double dg3dt2 = 0;
    double dg3dt3 = Rou;

    _jacobianOplusXj(0,0) = (-1.0)*fx*((g_3*dg1dw1 - g_1*dg3dw1)/g_33);
    _jacobianOplusXj(0,1) = (-1.0)*fx*((g_3*dg1dw2 - g_1*dg3dw2)/g_33);
    _jacobianOplusXj(0,2) = (-1.0)*fx*((g_3*dg1dw3 - g_1*dg3dw3)/g_33);
    _jacobianOplusXj(0,3) = (-1.0)*fx*((g_3*dg1dt1 - g_1*dg3dt1)/g_33);
    _jacobianOplusXj(0,4) = (-1.0)*fx*((g_3*dg1dt2 - g_1*dg3dt2)/g_33);
    _jacobianOplusXj(0,5) = (-1.0)*fx*((g_3*dg1dt3 - g_1*dg3dt3)/g_33);

    _jacobianOplusXj(1,0) = (-1.0)*fy*((g_3*dg2dw1 - g_2*dg3dw1)/g_33);
    _jacobianOplusXj(1,1) = (-1.0)*fy*((g_3*dg2dw2 - g_2*dg3dw2)/g_33);
    _jacobianOplusXj(1,2) = (-1.0)*fy*((g_3*dg2dw3 - g_2*dg3dw3)/g_33);
    _jacobianOplusXj(1,3) = (-1.0)*fy*((g_3*dg2dt1 - g_2*dg3dt1)/g_33);
    _jacobianOplusXj(1,4) = (-1.0)*fy*((g_3*dg2dt2 - g_2*dg3dt2)/g_33);
    _jacobianOplusXj(1,5) = (-1.0)*fy*((g_3*dg2dt3 - g_2*dg3dt3)/g_33);
//    std::cout << "DEBUG _jacobians " << std::endl;
//    std::cout << _jacobianOplusXi << std::endl;
//    std::cout << _jacobianOplusXj << std::endl;
}

Vector2d EdgeProjectInverseDepth2SE3::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}

EdgeSE3ToSE3::EdgeSE3ToSE3(): BaseBinaryEdge<3, Vector3d, VertexSE3Expmap, VertexSE3Expmap>(){

}

bool EdgeSE3ToSE3::read(istream &is)
{
    for (int i=0; i<3; i++){
      is >> _measurement[i];
    }
    for (int i=0; i<3; i++)
      for (int j=i; j<3; j++) {
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    return true;
}

bool EdgeSE3ToSE3::write(ostream &os) const
{
    for (int i=0; i<3; i++){
      os << measurement()[i] << " ";
    }

    for (int i=0; i<3; i++)
      for (int j=i; j<3; j++){
        os << " " <<  information()(i,j);
      }
    return os.good();
}

void EdgeSE3ToSE3::computeError()
{
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

    // KF1 - KF0
    SE3Quat T1(v1->estimate());
    SE3Quat T2(v2->estimate());

    // get camera center
    Matrix<double,3,3> R1 = T1.rotation().toRotationMatrix();
    Vector3d t1 = T1.translation();
    Vector3d camCenter1 = (-1.0)*R1.transpose()*t1;

    Matrix<double,3,3> R2 = T2.rotation().toRotationMatrix();
    Vector3d t2 = T2.translation();
    Vector3d camCenter2 = (-1.0)*R2.transpose()*t2;

    Vector3d camCenterDiff2m1 = camCenter2 - camCenter1;

    // compute error
    Vector3d obs(_measurement);
    _error = (obs-camCenterDiff2m1);
}

void EdgeSE3ToSE3::linearizeOplus()
{
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

    // KF2 - KF1
    SE3Quat T1(v1->estimate());
    SE3Quat T2(v2->estimate());

    // compute jacobian of SE3 cam1 pose
    Matrix<double,3,3> R1 = T1.rotation().toRotationMatrix();
    Vector3d translation1 = T1.translation();

    double r1, r2, r3, r4, r5, r6, r7, r8, r9, a1, b1, c1;
    r1 = R1(0,0); r2 = R1(0,1); r3 = R1(0,2);
    r4 = R1(1,0); r5 = R1(1,1); r6 = R1(1,2);
    r7 = R1(2,0); r8 = R1(2,1); r9 = R1(2,2);
    a1 = translation1(0); b1 = translation1(1); c1 = translation1(2);

    double w1, w2, w3, t1, t2, t3; w1 = w2 = w3 = t1 = t2 = t3 = 0;
    double P11 = r1 - w3*r4 + w2*r7;
    double P12 = r2 - w3*r5 + w2*r8;
    double P13 = r3 - w3*r6 + w2*r9;
    double P14 = a1 - w3*b1 + w2*c1 + t1;

    double P21 = w3*r1 + r4 - w1*r7;
    double P22 = w3*r2 + r5 - w1*r8;
    double P23 = w3*r3 + r6 - w1*r9;
    double P24 = w3*a1 + b1 - w1*c1 + t2;

    double P31 = -w2*r1 + w1*r4 + r7;
    double P32 = -w2*r2 + w1*r5 + r8;
    double P33 = -w2*r3 + w1*r6 + r9;
    double P34 = -w2*a1 + w1*b1 + c1 + t3;

    _jacobianOplusXi(0,0) = (-1.0)*(-r7*P24 -c1*P21 + r4*P34 + P31*b1);
    _jacobianOplusXi(0,1) = (-1.0)*(r7*P14 + P11*c1 -r1*P34 - a1*P31);
    _jacobianOplusXi(0,2) = (-1.0)*(-r4*P14 - b1*P11 + r1*P24 + P21*a1);
    _jacobianOplusXi(0,3) = -P11;
    _jacobianOplusXi(0,4) = -P21;
    _jacobianOplusXi(0,5) = -P31;

    _jacobianOplusXi(1,0) = (-1.0)*(-r8*P24 - c1*P22 + r5*P34 + P32*b1);
    _jacobianOplusXi(1,1) = (-1.0)*(r8*P14 + P12*c1 -r2*P34 - a1*P32);
    _jacobianOplusXi(1,2) = (-1.0)*(-r5*P14 - P12*b1 + r2*P24 + P22*a1);
    _jacobianOplusXi(1,3) = -P12;
    _jacobianOplusXi(1,4) = -P22;
    _jacobianOplusXi(1,5) = -P32;

    _jacobianOplusXi(2,0) = (-1.0)*(-r9*P24 - P23*c1 + r6*P34 + P33*b1);
    _jacobianOplusXi(2,1) = (-1.0)*(r9*P14 + P13*c1 - r3*P34 - a1*P33);
    _jacobianOplusXi(2,2) = (-1.0)*(-r6*P14 - b1*P13 + r3*P24 + P23*a1);
    _jacobianOplusXi(2,3) = -P13;
    _jacobianOplusXi(2,4) = -P23;
    _jacobianOplusXi(2,5) = -P33;

    // compute jacobian of camera 2 pose
    R1 = T2.rotation().toRotationMatrix();
    translation1 = T2.translation();

    r1 = R1(0,0); r2 = R1(0,1); r3 = R1(0,2);
    r4 = R1(1,0); r5 = R1(1,1); r6 = R1(1,2);
    r7 = R1(2,0); r8 = R1(2,1); r9 = R1(2,2);
    a1 = translation1(0); b1 = translation1(1); c1 = translation1(2);

    w1 = w2 = w3 = t1 = t2 = t3 = 0;
    P11 = r1 - w3*r4 + w2*r7;
    P12 = r2 - w3*r5 + w2*r8;
    P13 = r3 - w3*r6 + w2*r9;
    P14 = a1 - w3*b1 + w2*c1 + t1;

    P21 = w3*r1 + r4 - w1*r7;
    P22 = w3*r2 + r5 - w1*r8;
    P23 = w3*r3 + r6 - w1*r9;
    P24 = w3*a1 + b1 - w1*c1 + t2;

    P31 = -w2*r1 + w1*r4 + r7;
    P32 = -w2*r2 + w1*r5 + r8;
    P33 = -w2*r3 + w1*r6 + r9;
    P34 = -w2*a1 + w1*b1 + c1 + t3;

    _jacobianOplusXj(0,0) = (-r7*P24 -c1*P21 + r4*P34 + P31*b1);
    _jacobianOplusXj(0,1) = (r7*P14 + P11*c1 -r1*P34 - a1*P31);
    _jacobianOplusXj(0,2) = (-r4*P14 - b1*P11 + r1*P24 + P21*a1);
    _jacobianOplusXj(0,3) = P11;
    _jacobianOplusXj(0,4) = P21;
    _jacobianOplusXj(0,5) = P31;

    _jacobianOplusXj(1,0) = (-r8*P24 - c1*P22 + r5*P34 + P32*b1);
    _jacobianOplusXj(1,1) = (r8*P14 + P12*c1 -r2*P34 - a1*P32);
    _jacobianOplusXj(1,2) = (-r5*P14 - P12*b1 + r2*P24 + P22*a1);
    _jacobianOplusXj(1,3) = P12;
    _jacobianOplusXj(1,4) = P22;
    _jacobianOplusXj(1,5) = P32;

    _jacobianOplusXj(2,0) = (-r9*P24 - P23*c1 + r6*P34 + P33*b1);
    _jacobianOplusXj(2,1) = (r9*P14 + P13*c1 - r3*P34 - a1*P33);
    _jacobianOplusXj(2,2) = (-r6*P14 - b1*P13 + r3*P24 + P23*a1);
    _jacobianOplusXj(2,3) = P13;
    _jacobianOplusXj(2,4) = P23;
    _jacobianOplusXj(2,5) = P33;
}



} // end namespace
