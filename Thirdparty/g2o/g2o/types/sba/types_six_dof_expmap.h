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

// Modified by Raúl Mur Artal (2014)
// Added EdgeSE3ProjectXYZ (project using focal_length in x,y directions)

#ifndef G2O_SIX_DOF_TYPES_EXPMAP
#define G2O_SIX_DOF_TYPES_EXPMAP

#include "../../core/base_vertex.h"
#include "../../core/base_binary_edge.h"
#include "../../types/slam3d/se3_ops.h"
#include "types_sba.h"
#include <Eigen/Geometry>

namespace g2o {
namespace types_six_dof_expmap {
void init();
}

using namespace Eigen;

typedef Matrix<double, 6, 6> Matrix6d;

class G2O_TYPES_SBA_API CameraParameters : public g2o::Parameter
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  CameraParameters();

  CameraParameters(double focal_length,
                   const Vector2d & principle_point,
                   double baseline)
    : focal_length(focal_length),
      principle_point(principle_point),
      baseline(baseline){}

  Vector2d cam_map (const Vector3d & trans_xyz) const;

  Vector3d stereocam_uvu_map (const Vector3d & trans_xyz) const;

  virtual bool read (std::istream& is){
    is >> focal_length;
    is >> principle_point[0];
    is >> principle_point[1];
    is >> baseline;
    return true;
  }

  virtual bool write (std::ostream& os) const {
    os << focal_length << " ";
    os << principle_point.x() << " ";
    os << principle_point.y() << " ";
    os << baseline << " ";
    return true;
  }

  double focal_length;
  Vector2d principle_point;
  double baseline;
};

/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
class G2O_TYPES_SBA_API VertexSE3Expmap : public BaseVertex<6, SE3Quat>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexSE3Expmap();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  virtual void setToOriginImpl() {
    _estimate = SE3Quat();
  }

  virtual void oplusImpl(const double* update_)  {
    Eigen::Map<const Vector6d> update(update_);
    setEstimate(SE3Quat::exp(update)*estimate());
  }
};


/**
 * \brief 6D edge between two Vertex6
 */
class G2O_TYPES_SBA_API EdgeSE3Expmap : public BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3Expmap();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
    const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

    SE3Quat C(_measurement);
    SE3Quat error_= v2->estimate().inverse()*C*v1->estimate();
    _error = error_.log();
  }

  virtual void linearizeOplus();
};


class G2O_TYPES_SBA_API EdgeProjectXYZ2UV : public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeProjectXYZ2UV();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    const CameraParameters * cam
        = static_cast<const CameraParameters *>(parameter(0));
    Vector2d obs(_measurement);
    _error = obs-cam->cam_map(v1->estimate().map(v2->estimate()));
  }
  
  

  virtual void linearizeOplus();

  CameraParameters * _cam;
};

class G2O_TYPES_SBA_API EdgeSE3ProjectXYZ: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3ProjectXYZ();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector2d obs(_measurement);
    _error = obs-cam_project(v1->estimate().map(v2->estimate()));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }
    

  virtual void linearizeOplus();

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
};

class G2O_TYPES_SBA_API EdgeSE3GProjectXYZ: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeSE3GProjectXYZ();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector2d obs(_measurement);
    g2o::SE3Quat Tc_test = Tcb*v1->estimate(); // Tcb*Tbw
    g2o::SE3Quat Tc = v1->estimate()*Tcb;// Tbw*Tcb
    std::cout << "Tcb -- g2o:" << Tcb << std::endl;
    std::cout << "Tbw -- v1-est-g2o: " << v1->estimate() << std::endl;
    std::cout << "Tcw -- g2o: R: " << Tc.rotation().toRotationMatrix() << std::endl << "\tT: " << Tc.translation() << std::endl;
    std::cout << "MPw -- v2->est-g2o: " << v2->estimate() << std::endl;
    Tc = Tc.inverse();
    _error = obs-cam_project(Tc.map(v2->estimate()));// obs-cam_project(Tc.map(map_point))
//    Eigen::Matrix<double,2,1> error = obs-cam_project(Tc_test.map(v2->estimate()));
    std::cout << "_error: " << _error(0) << " | " << _error(1) << std::endl;
//    std::cout << "error_test: " << error(0) << " | " << error(1) << std::endl;
    std::cout << "obs: " << obs << std::endl;
    std::cout << "cam_project: " << cam_project(Tc.map(v2->estimate())) << std::endl;
    std::cout << "fx: " << fx << " | fy: " << fy << " | cx: " << cx << " | cy: " << cy << std::endl << std::endl;
    std::cout << "######################## NEXT EDGE ########################## " << std::endl << std::endl;
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }

//  virtual void linearizeOplus();

  void setT(g2o::SE3Quat T) {
    Tcb = T;
  }

  Vector2d cam_project(const Vector3d & trans_xyz) const;

  double fx, fy, cx, cy;
  g2o::SE3Quat Tcb; // R and t matrices for transformation from camera frame to base frame
};

// TODO: SE3rayXYZ can be deleted after BA/ Optimization completely works
class G2O_TYPES_SBA_API SE3rayXYZ: public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SE3rayXYZ();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError()  {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector3d obs(_measurement);
    Vector3d estimated_unit_feature_ray = v1->estimate().map(v2->estimate());
    estimated_unit_feature_ray.normalize();
    // _error is Vector3d even though we only need scalar
    _error << 1-obs.dot(estimated_unit_feature_ray), 0, 0;
//    _error = obs-cam_project(v1->estimate().map(v2->estimate()));
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    return (v1->estimate().map(v2->estimate()))(2)>0.0;
  }

  double fx, fy, cx, cy;
};

class G2O_TYPES_SBA_API EdgeProjectPSI2UV : public  g2o::BaseMultiEdge<2, Vector2d>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeProjectPSI2UV()  {
    resizeParameters(1);
    installParameter(_cam, 0);
  }

  virtual bool read  (std::istream& is);
  virtual bool write (std::ostream& os) const;
  void computeError  ();
  virtual void linearizeOplus ();
  CameraParameters * _cam;
};



//Stereo Observations
// U: left u
// V: left v
// U: right u
class G2O_TYPES_SBA_API EdgeProjectXYZ2UVU : public  BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeProjectXYZ2UVU();

  bool read(std::istream& is);

  bool write(std::ostream& os) const;

  void computeError(){
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    const CameraParameters * cam
        = static_cast<const CameraParameters *>(parameter(0));
    Vector3d obs(_measurement);
    _error = obs-cam->stereocam_uvu_map(v1->estimate().map(v2->estimate()));
  }
  //  virtual void linearizeOplus();
};

} // end namespace

#endif
