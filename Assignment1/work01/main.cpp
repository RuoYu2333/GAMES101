#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    Eigen::Matrix4f rotate;
    rotate << cos(rotation_angle * MY_PI / 180), -sin(rotation_angle * MY_PI / 180), 0, 0,
              sin(rotation_angle * MY_PI / 180), cos(rotation_angle * MY_PI / 180), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    model = rotate * model;

    return model;
}

Eigen::Vector3f trans_to_identity(const Eigen::Vector3f& axis)
{
    float length = axis.norm(); // 计算模长：sqrt(x² + y² + z²)
    const float epsilon = 1e-8f; // 浮点精度阈值

    if (length < epsilon) {
        throw std::runtime_error("Cannot normalize a zero vector.");
        // 或返回零向量：return Eigen::Vector3f::Zero();
    }
    return axis / length; // 归一化
}

Eigen::Matrix4f convert3x3To4x4(const Eigen::Matrix3f& mat3x3) {
    Eigen::Matrix4f mat4x4 = Eigen::Matrix4f::Identity(); // 初始化为单位矩阵
    mat4x4.block<3, 3>(0, 0) = mat3x3; // 将3x3矩阵复制到左上角
    return mat4x4;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle, Vector3f axis)
{
    trans_to_identity(axis);
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    //绕任意轴旋转
    const Eigen::Matrix3f matrix1_=cos(rotation_angle * MY_PI / 180)*Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f matrix2_=(1-cos(rotation_angle * MY_PI / 180))*axis*axis.transpose();
    Eigen::Matrix3f matrix3_;
    matrix3_<<0,-axis[2],axis[1],
              axis[2],0,-axis[0],
              -axis[1],axis[0],0;
    Eigen::Matrix3f rotation_matrix=matrix1_+matrix2_+sin(rotation_angle)*matrix3_;
    convert3x3To4x4(rotation_matrix);
    model=convert3x3To4x4(rotation_matrix)*model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f proj, ortho;

    proj << zNear, 0, 0, 0,
            0, zNear, 0, 0,
            0, 0, zNear + zFar, -zNear * zFar,
            0, 0, 1, 0;

    double w, h, z;
    h = -zNear * tan(eye_fov*MY_PI / 360); ;
    w = h * aspect_ratio;
    z = zNear -zFar ;

    ortho << 1 / w, 0, 0, 0,
             0, 1 / h, 0, 0,
             0, 0, 2 / z, -(zFar + zNear) / z,
             0, 0, 0, 1;
    projection = ortho * proj * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
     float x, y, z;

    // 提示用户输入
    std::cout << "请输入旋转轴的三个分量（x y z，用空格分隔）: ";
    
    // 读取输入
    if (!(std::cin >> x >> y >> z)) {
        std::cerr << "输入格式错误，请确保输入三个数字！" << std::endl;
        return 1; // 非正常退出
    }

    // 构造Eigen向量
    Eigen::Vector3f axis;
    axis << x, y, z;
    
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle,axis));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle,axis));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
