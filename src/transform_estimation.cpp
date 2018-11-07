////////////////////////////////////////////////////////////////////////////////
//
//  @file calibrate.cpp
//
//  @brief Finds transformation between camera and world coordinates
//
//  @copy 2018 Elementary Robotics. All rights reserved.
//
////////////////////////////////////////////////////////////////////////////////
#include <atom.h>
#include <element.h>
#include <redis.h>
#include <hiredis/hiredis.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation_svd.h>

static const std::string REALSENSE_ELEMENT = "realsense";
static const std::string COLOR_STREAM = "color";
static const std::string PC_STREAM = "pointcloud";
static const std::string FILE_PATH = "data/transform.csv";
static const std::string FILE_HEADER = "t_x,t_y,t_z,q_x,q_y,q_z,q_w";
#define CHECKERBOARD_CORNERS_HORIZONTAL 9
#define CHECKERBOARD_CORNERS_VERTICAL 6
// Distance from top edge of sheet to first corner in meters
#define CALIBRATION_SHEET_TOP_EDGE_OFFSET 0.043
// Size of checkerboard square in meters
#define SQUARE_SIZE 0.022

struct frame_data {
    uint8_t *data;
    size_t len;
};

////////////////////////////////////////////////////////////////////////////////
//
//  @brief Find transformation between imgpoints and objpoints
//
////////////////////////////////////////////////////////////////////////////////
std::vector<float> estimate_transformation(std::vector<cv::Point3f> imgpoints,
                                           std::vector<cv::Point3f> objpoints)
{
    if (imgpoints.size() != objpoints.size()) {
        std::string error =
            "imgpoints has a different number of points than objpoints!";
        throw error;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_img, cloud_obj;
    for (unsigned i = 0; i < imgpoints.size(); ++i) {
        cloud_img.push_back(pcl::PointXYZ(imgpoints.at(i).x,
                                          imgpoints.at(i).y,
                                          imgpoints.at(i).z));
        cloud_obj.push_back(pcl::PointXYZ(objpoints.at(i).x,
                                          objpoints.at(i).y,
                                          objpoints.at(i).z));
    }

    Eigen::Matrix4f img2obj;
    pcl::registration::TransformationEstimationSVD
        <pcl::PointXYZ,pcl::PointXYZ> TESVD;

    // Calling the PCL function to obtain transformation
    TESVD.estimateRigidTransformation(cloud_img, cloud_obj, img2obj);

    // Make the transformation matrix
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix << img2obj(0,0), img2obj(0,1), img2obj(0,2),
                       img2obj(1,0), img2obj(1,1), img2obj(1,2),
                       img2obj(2,0), img2obj(2,1), img2obj(2,2);
    // Get the quaternion from the transformation matrix
    Eigen::Quaternionf q(rotation_matrix);

    // Add the offset so the origin is at the edge of the calibration sheet.
    img2obj(0, 3) += CALIBRATION_SHEET_TOP_EDGE_OFFSET;

    std::vector<float> transformation{
        img2obj(0, 3), img2obj(1, 3), img2obj(2, 3),
        q.x(), q.y(), q.z(), q.w()};

    return transformation;
}


////////////////////////////////////////////////////////////////////////////////
//
//  @brief Uses OpenCV's findChessboardCorners to find imgpoints
//
////////////////////////////////////////////////////////////////////////////////
std::vector<cv::Point3f> get_imgpoints(cv::Size board_shape,
                                       cv::Mat img, cv::Mat pc)
{
    std::vector<cv::Point2f> imgpoints2d;
    std::vector<cv::Point3f> imgpoints3d;

    // findChessboardCorners has an easier time with larger images
    bool success = cv::findChessboardCorners(img, board_shape, imgpoints2d);

    if (!success) {
        std::string error = "Could not find checkerboard corners! "
                            "Is the calibration sheet in place?";
        throw error;
    }

    for (cv::Point2f& point : imgpoints2d) {
        // X and Y axes are opposite between ROS and OpenCV
        cv::Point3f point3d = pc.at<cv::Point3f>((int)round(point.y),
                                                 (int)round(point.x));
        imgpoints3d.push_back(point3d);
        // Check for 0 or NaN values
        if ((imgpoints3d.back().x == 0.0 && imgpoints3d.back().y == 0.0 &&
            imgpoints3d.back().z == 0.0) || (imgpoints3d.back().x*0 != 0.0 &&
            imgpoints3d.back().y*0 != 0.0 && imgpoints3d.back().z*0 != 0.0)) {
            std::stringstream error_ss;
            error_ss << "Could not find point cloud coordinates for "
                  << (int)round(point.y) << ", " << (int)round(point.x)
                  << "\nTry running again, fixing exposure, "
                  << "or adjusting calibration sheet.";
            throw error_ss.str();
        }
    }

    return imgpoints3d;
}


////////////////////////////////////////////////////////////////////////////////
//
//  @brief Gets objpoints based on board_shape (# corners) and square_size (m)
//
////////////////////////////////////////////////////////////////////////////////
std::vector<cv::Point3f> get_objpoints(cv::Size board_shape, double square_size)
{
    std::vector<cv::Point3f> objpoints3d;

    for (int i = 0; i < board_shape.height; ++i) {
        for (int j = 0; j < board_shape.width; ++j) {
            objpoints3d.push_back(cv::Point3f(i*square_size, j*square_size, 0));
        }
    }

    return objpoints3d;
}


////////////////////////////////////////////////////////////////////////////////
//
//  @brief Finds distortion vector for use of dewarping image.
//
////////////////////////////////////////////////////////////////////////////////
cv::Mat get_distortion(std::vector<cv::Point3f> objpoints,
                       std::vector<cv::Point3f> imgpoints,
                       cv::Size img_size, cv::Mat intrinsic)
{
    cv::Mat dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    // Convert imgpoints to 2d
    std::vector<cv::Point2f> imgpoints2d;
    for (cv::Point3f& point : imgpoints) {
        imgpoints2d.push_back(cv::Point2f(point.x, point.y));
    }

    std::vector<std::vector<cv::Point2f>> imgpoints_vector;
    imgpoints_vector.push_back(imgpoints2d);
    std::vector<std::vector<cv::Point3f>> objpoints_vector;
    objpoints_vector.push_back(objpoints);

    cv::calibrateCamera(objpoints_vector, imgpoints_vector, img_size, intrinsic,
                    dist_coeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS);

    return dist_coeffs;
}


////////////////////////////////////////////////////////////////////////////////
//
//  @brief Puts data from entries into user_data during call to 
//         element_entry_read_n()
//
////////////////////////////////////////////////////////////////////////////////
bool realsense_data_cb(
    const struct redis_xread_kv_item *kv_items,
    int n_kv_items,
    void *user_data)
{
    struct frame_data *data = (struct frame_data *)user_data;
    data->data = (uint8_t*)malloc(kv_items[0].reply->len);
    data->len = kv_items[0].reply->len;
    if (data->data == NULL) {
        data->len = 0;
        fprintf(stderr, "Failed to allocate data pointer\n");
        return false;
    } else {
        memcpy(data->data, kv_items[0].reply->str, data->len);
    }
    return true;
}


////////////////////////////////////////////////////////////////////////////////
//
//  @brief Gets most recent entry from specified stream from realsense element
//         Returns the data as a vector of binary tiff data
//
////////////////////////////////////////////////////////////////////////////////
std::vector<uint8_t> get_realsense_data(
    redisContext *ctx,
    struct element *element,
    std::string stream)
{
    struct element_entry_read_info info;
    struct redis_xread_kv_item entry_items[1];
    struct frame_data frame;
    info.element = REALSENSE_ELEMENT.c_str();
    info.stream = stream.c_str();
    info.kv_items = entry_items;
    info.n_kv_items = sizeof(entry_items)/sizeof(struct redis_xread_kv_item);
    info.response_cb = realsense_data_cb;
    info.user_data = &frame;

    entry_items[0].key = "data";
    entry_items[0].key_len = strlen(entry_items[0].key);

    int err_code = element_entry_read_n(ctx, element, &info, 1);
    if (err_code != 0) {
        std::string error = "Failed to retrieve data from realsense skill\n";
        throw error;
    }

    // Put data into vector
    std::vector<uint8_t> rs_data;
    for (int i = 0; i < (int)frame.len; ++i) {
        rs_data.push_back(frame.data[i]);
    }

    // Free the memory allocated in the callback
    if (frame.data != NULL) {
        free(frame.data);
    }

    return rs_data;
}

////////////////////////////////////////////////////////////////////////////////
//
//  @brief Grabs frames from camera, gets image and object points and calculates
//         transformation, writing it to a file.
//
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    redisContext *ctx = redis_context_init();
    struct element *element = element_init(ctx, "transform_estimation");
    if (element == NULL) {
        fprintf(stderr, "Failed to create element!\n");
        return 1;
    }

    // Get color and pointcloud data
    std::vector<uint8_t> color_vec;
    std::vector<uint8_t> pc_vec;
    try {
        color_vec = get_realsense_data(ctx, element, COLOR_STREAM);
        pc_vec = get_realsense_data(ctx, element, PC_STREAM);
    } catch (std::string msg) {
        fprintf(stderr, "%s\n", msg.c_str());
        return 1;
    }

    // Convert data from tiff to OpenCV matrix
    cv::Mat color_img = cv::imdecode(color_vec, cv::IMREAD_COLOR);
    cv::Mat pc_flat = cv::imdecode(pc_vec, cv::IMREAD_UNCHANGED);

    // Unflatten pointcloud
    cv::Mat pc = cv::Mat(color_img.rows, color_img.cols, CV_32FC3);
    for (int y = 0; y < pc.rows; ++y) {
        for (int x = 0; x < pc.cols; ++x) {
            pc.at<cv::Vec3f>(y, x)[0] = pc_flat.at<cv::Vec3f>(y*pc.cols+x)[0];
            pc.at<cv::Vec3f>(y, x)[1] = pc_flat.at<cv::Vec3f>(y*pc.cols+x)[1];
            pc.at<cv::Vec3f>(y, x)[2] = pc_flat.at<cv::Vec3f>(y*pc.cols+x)[2];
        }
    }
    
    /* TODO Implement getting intrinsic matrix for distortion
    // Get intrinsic matrix from ros info
    cv::Mat intrinsic = cv::Mat::zeros(3, 3, CV_64F);
    for (unsigned i = 0; i < 3; ++i) {
        for (unsigned j = 0; j < 3; ++j) {
            intrinsic.at<double>(i, j) = info_msg.K[i*3+j];
        }
    }
    */

    cv::Size board_shape = cv::Size(CHECKERBOARD_CORNERS_HORIZONTAL,
                                    CHECKERBOARD_CORNERS_VERTICAL);

    /* TODO Implement getting intrinsic matrix for distortion
    cv::Size img_size = cv::Size(color_img.cols, color_img.rows);
    cv::Mat distortion;
    */

    std::vector<float> transformation;
    try {
        std::vector<cv::Point3f> imgpoints = get_imgpoints(board_shape,
                                                           color_img, pc);
        std::vector<cv::Point3f> objpoints = get_objpoints(board_shape,
                                                           SQUARE_SIZE);
        transformation = estimate_transformation(imgpoints, objpoints);

        /* TODO Implement getting intrinsic matrix for distortion
        distortion = get_distortion(objpoints, imgpoints, img_size, intrinsic);
        cout << "Undistortion is not yet implemented! "
             << "If these values are high, the calibration will not be good.\n"
             << distortion << endl;
        */
    } catch (std::string msg) {
        fprintf(stderr, "%s\n", msg.c_str());
        return 1;
    }

    // Write transform to file
    std::ofstream transform_file(FILE_PATH);
    if (transform_file.is_open()) {
        transform_file << FILE_HEADER << std::endl;
        std::string separator = "";
        for (auto v : transformation) {
            transform_file << separator << v;
            transform_file.flush();
            separator = ",";
        }
        transform_file << std::endl;
        transform_file.close();
    } else {
        fprintf(stderr, "Failed to open file at path %s\n", FILE_PATH.c_str());
        return 1;
    }

    element_cleanup(ctx, element);
    fprintf(stderr, "Transform estimation successful!\n");
    return 0;
}
