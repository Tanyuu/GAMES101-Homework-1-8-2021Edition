// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

extern bool MSAA;

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


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f bas = {x,y,0};
    Vector3f edg0 = _v[0];
    Vector3f edg1 = _v[1];
    Vector3f edg2 = _v[2];
    float ans0=(bas-edg0).cross(edg1-edg0)[2];
    float ans1=(bas-edg1).cross(edg2-edg1)[2];
    float ans2=(bas-edg2).cross(edg0-edg2)[2];
    if((ans0>=0&&ans1>=0&&ans2>=0)||(ans0<=0&&ans1<=0&&ans2<=0)) return 1;
    else return 0;
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
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    float xminf=INFINITY,xmaxf=-INFINITY,yminf=INFINITY,ymaxf=-INFINITY;
    for(int i=0;i<3;i++)
    {
        xminf=std::min(xminf,v[i][0]);
        xmaxf=std::max(xmaxf,v[i][0]);
        yminf=std::min(yminf,v[i][1]);
        ymaxf=std::max(ymaxf,v[i][1]);

    }
    int xmini=floor(xminf),xmaxi=ceil(xmaxf),ymini=floor(yminf),ymaxi=ceil(ymaxf);
    for(int i=xmini;i<=xmaxi;i++)
    {
        for(int j=ymini;j<=ymaxi;j++)
        {
            if(!MSAA)
            {
                float x=i+0.5,y=j+0.5;  
                if(insideTriangle(x,y,t.v))
                {
                    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if(z_interpolated < depth_buf[get_index(x,y)])
                    {
                        depth_buf[get_index(x,y)]=z_interpolated;
                        set_pixel({1.0f*(int)x,1.0f*(int)y,z_interpolated},t.getColor());
                    }
                }
            }
            else
            {
                float x,y;
                //int hitCount=0;
                static float dx[]={0.25, 0.75, 0.25, 0.75},dy[]={0.25, 0.25, 0.75, 0.75};
                //Vector3f colorSum = Vector3f::Zero();
                for(int k=0;k<4;k++)
                {
                    x=i+dx[k];
                    y=j+dy[k];
                    if(insideTriangle(x,y,t.v))
                    {
                        auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        if(z_interpolated<depth_buf[get_index(x*2,y*2)])
                        {
                            depth_buf[get_index(x*2,y*2)]=z_interpolated;
                            mix_pixel({1.0f*(int)(x),1.0f*(int)(y),z_interpolated},get_msaa_buf({1.0f*(int)(x*2),1.0f*(int)(y*2),z_interpolated})/(-4.0));
                            //mix_pixel({1.0f*(int)(x),1.0f*(int)(y),z_interpolated},{-0,-0,-0});
                            set_msaa_buf({1.0f*(int)(x*2),1.0f*(int)(y*2),z_interpolated},t.getColor());
                            mix_pixel({1.0f*(int)(x),1.0f*(int)(y),z_interpolated},t.getColor()/(4.0));

                        }
                    }
                }
            }
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
    if ((buff & rst::Buffers::MSAA) == rst::Buffers::MSAA)
    {
        std::fill(MSAA_frame_buf.begin(), MSAA_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    if(MSAA)
    {
        depth_buf.resize(w * h * 4);
        MSAA_frame_buf.resize(w * h * 4);
    }
    else
    {
        depth_buf.resize(w * h);
    }
}

int rst::rasterizer::get_index(int x, int y)
{
    if(MSAA)
        return (height*2-1-y)*width*2 + x;
    else
        return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind;
    ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::mix_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind;
    ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] += color;
}

void rst::rasterizer::set_msaa_buf(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind;
    ind = (height*2-1-point.y())*width*2 + point.x();
    MSAA_frame_buf[ind] = color;
}

Eigen::Vector3f rst::rasterizer::get_msaa_buf(const Eigen::Vector3f& point)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind;
    ind = (height*2-1-point.y())*width*2 + point.x();
    return MSAA_frame_buf[ind];
}
// clang-format on