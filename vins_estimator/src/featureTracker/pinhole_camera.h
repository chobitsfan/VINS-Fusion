// Minimal pinhole camera model extracted from camodocal.
// Contains only what feature_tracker uses:
//   CameraFactory::instance()->generateCameraFromYamlFile()
//   liftProjective()   pixel -> normalized projective ray (undistortion)
//   spaceToPlane()     3D point -> pixel (projection)
#ifndef PINHOLE_CAMERA_H
#define PINHOLE_CAMERA_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>

namespace camodocal
{

class PinholeCamera
{
public:
    class Parameters
    {
    public:
        Parameters();

        bool readFromYamlFile(const std::string& filename);

        std::string cameraName;
        int imageWidth;
        int imageHeight;
        double k1, k2, p1, p2;   // distortion
        double fx, fy, cx, cy;   // projection
    };

    PinholeCamera();
    explicit PinholeCamera(const Parameters& params);

    const Parameters& getParameters(void) const;
    void setParameters(const Parameters& parameters);

    // Lift a point from the image plane to its projective ray.
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;

    // Project a 3D point to the image plane.
    void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;

private:
    // Distortion of a point on the normalised plane: p_d = p_u + d_u.
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;

    Parameters mParameters;

    double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
    bool m_noDistortion;
};

typedef boost::shared_ptr<PinholeCamera> PinholeCameraPtr;
// feature_tracker stores cameras as CameraPtr; keep the alias so its code is unchanged.
typedef boost::shared_ptr<PinholeCamera> CameraPtr;

// Kept as a singleton with the same call shape used by feature_tracker:
//   CameraFactory::instance()->generateCameraFromYamlFile(file)
class CameraFactory
{
public:
    static boost::shared_ptr<CameraFactory> instance(void);

    CameraPtr generateCameraFromYamlFile(const std::string& filename);

private:
    static boost::shared_ptr<CameraFactory> m_instance;
};

}

#endif
