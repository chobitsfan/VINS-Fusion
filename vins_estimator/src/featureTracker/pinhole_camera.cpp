// Minimal pinhole camera model extracted from camodocal.
#include "pinhole_camera.h"

namespace camodocal
{

// ---------------------------------------------------------------------------
// PinholeCamera::Parameters
// ---------------------------------------------------------------------------

PinholeCamera::Parameters::Parameters()
 : imageWidth(0)
 , imageHeight(0)
 , k1(0.0)
 , k2(0.0)
 , p1(0.0)
 , p2(0.0)
 , fx(0.0)
 , fy(0.0)
 , cx(0.0)
 , cy(0.0)
{
}

bool
PinholeCamera::Parameters::readFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        return false;
    }

    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (sModelType.compare("PINHOLE") != 0)
        {
            return false;
        }
    }

    fs["camera_name"] >> cameraName;
    imageWidth = static_cast<int>(fs["image_width"]);
    imageHeight = static_cast<int>(fs["image_height"]);

    cv::FileNode n = fs["distortion_parameters"];
    k1 = static_cast<double>(n["k1"]);
    k2 = static_cast<double>(n["k2"]);
    p1 = static_cast<double>(n["p1"]);
    p2 = static_cast<double>(n["p2"]);

    n = fs["projection_parameters"];
    fx = static_cast<double>(n["fx"]);
    fy = static_cast<double>(n["fy"]);
    cx = static_cast<double>(n["cx"]);
    cy = static_cast<double>(n["cy"]);

    return true;
}

// ---------------------------------------------------------------------------
// PinholeCamera
// ---------------------------------------------------------------------------

PinholeCamera::PinholeCamera()
 : m_inv_K11(1.0)
 , m_inv_K13(0.0)
 , m_inv_K22(1.0)
 , m_inv_K23(0.0)
 , m_noDistortion(true)
{
}

PinholeCamera::PinholeCamera(const Parameters& params)
{
    setParameters(params);
}

const PinholeCamera::Parameters&
PinholeCamera::getParameters(void) const
{
    return mParameters;
}

void
PinholeCamera::setParameters(const PinholeCamera::Parameters& parameters)
{
    mParameters = parameters;

    m_noDistortion = (mParameters.k1 == 0.0) &&
                     (mParameters.k2 == 0.0) &&
                     (mParameters.p1 == 0.0) &&
                     (mParameters.p2 == 0.0);

    // Inverse camera projection matrix parameters
    m_inv_K11 = 1.0 / mParameters.fx;
    m_inv_K13 = -mParameters.cx / mParameters.fx;
    m_inv_K22 = 1.0 / mParameters.fy;
    m_inv_K23 = -mParameters.cy / mParameters.fy;
}

// Lifts a point from the image plane to its projective ray.
void
PinholeCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
    double mx_d, my_d, mx_u, my_u;

    // Lift points to normalised plane
    mx_d = m_inv_K11 * p(0) + m_inv_K13;
    my_d = m_inv_K22 * p(1) + m_inv_K23;

    if (m_noDistortion)
    {
        mx_u = mx_d;
        my_u = my_d;
    }
    else
    {
        // Recursive distortion model
        int n = 8;
        Eigen::Vector2d d_u;
        distortion(Eigen::Vector2d(mx_d, my_d), d_u);
        // Approximate value
        mx_u = mx_d - d_u(0);
        my_u = my_d - d_u(1);

        for (int i = 1; i < n; ++i)
        {
            distortion(Eigen::Vector2d(mx_u, my_u), d_u);
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);
        }
    }

    // Obtain a projective ray
    P << mx_u, my_u, 1.0;
}

// Projects a 3D point to the image plane.
void
PinholeCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
{
    Eigen::Vector2d p_u, p_d;

    // Project points to the normalised plane
    p_u << P(0) / P(2), P(1) / P(2);

    if (m_noDistortion)
    {
        p_d = p_u;
    }
    else
    {
        // Apply distortion
        Eigen::Vector2d d_u;
        distortion(p_u, d_u);
        p_d = p_u + d_u;
    }

    // Apply generalised projection matrix
    p << mParameters.fx * p_d(0) + mParameters.cx,
         mParameters.fy * p_d(1) + mParameters.cy;
}

// Apply distortion to a point on the normalised plane: p_d = p_u + d_u.
void
PinholeCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
{
    double k1 = mParameters.k1;
    double k2 = mParameters.k2;
    double p1 = mParameters.p1;
    double p2 = mParameters.p2;

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

// ---------------------------------------------------------------------------
// CameraFactory
// ---------------------------------------------------------------------------

boost::shared_ptr<CameraFactory> CameraFactory::m_instance;

boost::shared_ptr<CameraFactory>
CameraFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CameraFactory);
    }
    return m_instance;
}

CameraPtr
CameraFactory::generateCameraFromYamlFile(const std::string& filename)
{
    PinholeCamera::Parameters params;
    if (!params.readFromYamlFile(filename))
    {
        return CameraPtr();
    }

    PinholeCameraPtr camera(new PinholeCamera);
    camera->setParameters(params);
    return camera;
}

}
