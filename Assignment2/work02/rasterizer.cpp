// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // 1. Check if the point is inside the triangle using barycentric coordinates
    // 2. If the point is inside, return true; otherwise, return false
     int flag = -1;

    for(int i = 0; i < 3; i++) {
        // the current point
        Eigen::Vector3f p0 = {x, y, 0};
        // the 1st vertex
        Eigen::Vector3f p1 = _v[i];
        // the 2nd vertex
        Eigen::Vector3f p2 = _v[(i+1)%3];//next point
        
        // the 1st vector (p1-p0)
        Eigen::Vector3f v1 = p1-p0;
        // the 2nd vector (p1-p2)
        Eigen::Vector3f v2 = p1-p2;

        // get the cross product
        float cp = v1.cross(v2).z();
        if(cp == 0) continue;

        int sign = cp < 0 ? 0: 1;
        if(flag == -1) flag = sign;
        if(flag != sign) return false;
    }

    return true;
    

}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
        resolve_to_framebuffer();
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // 计算三角形包围盒
    float min_x = std::min({v[0].x(), v[1].x(), v[2].x()});
    float max_x = std::max({v[0].x(), v[1].x(), v[2].x()});
    float min_y = std::min({v[0].y(), v[1].y(), v[2].y()});
    float max_y = std::max({v[0].y(), v[1].y(), v[2].y()});

    // 遍历包围盒内每个像素
    for (int y = min_y; y <= max_y; ++y) {
        for (int x = min_x; x <= max_x; ++x) {
            // 遍历子样本（2x2）
            int num_covered = 0;
            Vector3f blended_color(0, 0, 0);
            
            for (int dy = 0; dy < ssaa_factor; ++dy) {
                for (int dx = 0; dx < ssaa_factor; ++dx) {
                    // 计算子样本坐标（偏移0.25实现2x2网格）
                    float sample_x = x + (dx + 0.5) / ssaa_factor;
                    float sample_y = y + (dy + 0.5) / ssaa_factor;

                    // 检查子样本是否在三角形内
                    if (insideTriangle(sample_x, sample_y, t.v)) {
                        // 计算重心坐标
                        auto [alpha, beta, gamma] = computeBarycentric2D(sample_x, sample_y, t.v);
                        
                        // 插值深度
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z()/v[0].w() + beta * v[1].z()/v[1].w() + gamma * v[2].z()/v[2].w();
                        z_interpolated *= w_reciprocal;

                        // 子样本索引
                        int sample_index = dy * ssaa_factor + dx;
                        int pixel_index = get_index(x, y);
                        
                        // 深度测试
                        if (z_interpolated < ssaa_depth_buf[pixel_index][sample_index]) {
                            ssaa_depth_buf[pixel_index][sample_index] = z_interpolated;
                            ssaa_frame_buf[pixel_index][sample_index] = t.getColor();
                        }
                    }
                }
            }
        }
    }
}
// 解决SSAA到主缓冲区
void rst::rasterizer::resolve_to_framebuffer() {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Vector3f avg_color(0, 0, 0);
            int pixel_idx = get_index(x, y);
            
            // 混合所有子样本颜色
            for (const auto& sample : ssaa_frame_buf[pixel_idx]) {
                avg_color += sample;
            }
            avg_color /= (ssaa_factor * ssaa_factor);
            
            // 写入主缓冲区
            frame_buf[pixel_idx] = avg_color;
        }
    }
}
void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    // 初始化SSAA缓冲区（每个像素 ssaa_factor^2 个子样本）
    int samples = ssaa_factor * ssaa_factor;
    ssaa_frame_buf.resize(w * h, std::vector<Vector3f>(samples, Vector3f(0,0,0)));
    ssaa_depth_buf.resize(w * h, std::vector<float>(samples, std::numeric_limits<float>::infinity()));
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}


// clang-format on