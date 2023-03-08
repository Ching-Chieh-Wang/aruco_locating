#ifndef BAFRAME_H
#define BAFRAME_H


#include "g2otypes_marker.h"
namespace markerslam {

struct BAMarker
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    int id;

    BAMarker(float markersize):isFirstMarker_(false)
    {
      float s = markersize / 2.;
      pxy_w<<  -s, s, s, -s,
                    -s, -s, s, s,
                    0., 0.,0., 0.,
                     1., 1., 1., 1.;
      pxy_m<<  -s, s, s, -s,
                    -s, -s, s, s,
                    0., 0.,0., 0.,
                     1., 1., 1., 1.;
      cxy_w << 0,0,0,1;
    }


    Eigen::Matrix4d pxy_w;    //  every column save a coner's coordinate in world frame
    Eigen::Vector4d cxy_w;    //  marker center coordinate in world frame

    Eigen::Matrix4d pxy_m;    //  every column save a coner's coordinate in marker frame

    Eigen::Matrix4d T_w_m;   //  marker to world
    Eigen::Matrix4d T_m_w;  //  world to marker

    Eigen::Matrix<double,2,4> puv_;    // pixel coordinate of four conners
    Eigen::Vector2d cuv_;                 // pixel coordinate of center


     g2o::VertexSE3Expmap * v_m_;

    bool isFirstMarker_;     //

    int   last_projected_kf_id_;    //!< Flag for the reprojection: don't reproject a marker twice.
    //Eigen::Matrix4d T_c_m;    // marker to frame;

    inline Eigen::Vector3d TmwPosition() const
    {
      Eigen::Matrix4d T = T_w_m.inverse();
      return Eigen::Vector3d(T(0,3),T(1,3),T(2,3));
    }
    inline Eigen::Matrix3d TmwRotation()const
    {
      Eigen::Matrix4d T = T_w_m.inverse();
      return T.block(0, 0, 3, 3);
    }
};

typedef Eigen::Matrix<double , 2, 4> Matrix24d;
struct ObsTag
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Matrix24d puv_;    // pixel coordinate of four conners
    BAMarker* marker_;    // correspond marker in map
    int tagid_;

    std::vector< std::pair<Eigen::Vector2d, Eigen::Vector4d> > point;    // points inner tag, used to improve the solvepnp

    ObsTag(Matrix24d px, BAMarker* m,int id):
    puv_(px),
    marker_(m),
    tagid_(id)
    {}

};

class BAFrame
{
private:
    int refmarker_id_;     // every frame at least  have a marker which is in world frame,
                                          // we use this marker as media to transfrom the new mark to world frame
                                          //  refmarker_id_ is  the marker index in frame_markers_ .

public:

    BAFrame(int id ):frame_id_(id){}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int frame_id_;
    Eigen::Matrix4d T_w_c_;                                   // camera coordinate in world frame

    std::vector< BAMarker* > frame_markers_;   // marker in  this frame
    std::list< ObsTag* > obs_tags_;

    bool is_keyframe_;                                             //!< Was this frames selected as keyframe?

    g2o::VertexSE3Expmap * v_kf_;

    /// Was this frame selected as keyframe?
    inline bool isKeyframe() const { return is_keyframe_; }

    void setKeyframe();
    void addMarker(BAMarker* mark);
    void addObsTag(ObsTag* tag);
    void setRefmarker(int  id ){refmarker_id_ = id;}

    bool EstimatePose(const std::vector<BAMarker *> oldtags, double fx, double fy, double px, double py);
    bool EstimatePose(const std::list<ObsTag*> oldtags, double fx,double fy, double px,double py);
    bool EstimatePose(double fx, double fy, double cx, double cy);

    /// Check if a point in (w)orld coordinate frame is visible in the image.
    bool isVisible(const int markerID, int& index) const;

    /// Return the pose of the frame in the (w)orld coordinate frame.
    inline Eigen::Vector3d pos() const
    {
      return Eigen::Vector3d(T_w_c_(0,3),T_w_c_(1,3),T_w_c_(2,3));
    }
    inline Eigen::Matrix3d rotation()const
    {
      return T_w_c_.block(0, 0, 3, 3);
    }

    inline Eigen::Vector3d TcwPosition() const
    {
      Eigen::Matrix4d T = T_w_c_.inverse();
      return Eigen::Vector3d(T(0,3),T(1,3),T(2,3));
    }
    inline Eigen::Matrix3d TcwRotation()const
    {
      Eigen::Matrix4d T = T_w_c_.inverse();
      return T.block(0, 0, 3, 3);
    }


};

typedef std::shared_ptr< BAFrame > FramePtr;
}
#endif // FRAME_H
