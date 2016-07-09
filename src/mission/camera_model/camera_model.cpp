#include "camera_model.h"

#include <cmath>
#include <iostream>

Point2d CameraModel::calc_undistort(Point2d distort_pixel) const
{
    Point2d undistort_pixel;

    double x = (distort_pixel.x - center_x_.get()) / focal_x_.get();
    double y = (distort_pixel.y - center_y_.get()) / focal_y_.get();
    double x2 = x * x;
    double y2 = y * y;
    double r = std::sqrt(x2 + y2);

    double r2 = r * r;
    double r4 = r2 * r2;
    double r6 = r4 * r2;

    undistort_pixel.x = x * (1 + k1_.get() * r2 + k2_.get() * r4 + k3_.get() * r6)
        + 2 * t1_.get() * y * x + t2_.get() * (r2 + 2 * x2);
    undistort_pixel.y = y * (1 + k1_.get() * r2 + k2_.get() * r4 + k3_.get() * r6)
        + t1_.get() * (r2 + 2 * y2) + 2 * t2_.get() * x * y;

    return undistort_pixel;
}

double CameraModel::calc_fov() const
{
    return atan2(w_.get() * 0.5, focal_x_.get()) / (0.5 * M_PI / 180.0);
}

Point2d CameraModel::frame_coord(Point2d pixel) const
{
    Point2d new_coord = calc_undistort(pixel);

    new_coord.y = -new_coord.y;

    return new_coord;
}

double CameraModel::heading_to_pixel(Point2d pixel) const
{
    Point2d coord = frame_coord(pixel);

    return std::atan(coord.x);
}

double CameraModel::heading_to_point(Point2d point) const
{
    return std::atan(point.x);
}

double CameraModel::calc_dist_to_object(double real_size, Point2d start_point, Point2d end_point) const
{
    Point2d pixel_center(center_x_.get(), center_y_.get());

    start_point.x *= focal_x_.get();
    start_point.y *= focal_y_.get();
    start_point += pixel_center;

    end_point.x *= focal_x_.get();
    end_point.y *= focal_y_.get();
    end_point += pixel_center;

    double pixel_size = norm(end_point - start_point);

    return calc_dist_to_object(real_size, pixel_size);
}

double CameraModel::calc_dist_to_object(double real_size, int pixel_size) const
{
    double d = focal_x_.get() * real_size / pixel_size;

    return d;
}

double CameraModel::calc_object_width(double dist_to_object, Point2d start_pixel, Point2d end_pixel) const
{
    double a = start_pixel.x * fov_ / w_.get();
    double b = end_pixel.x * fov_ / w_.get();
    a *= M_PI / 180.0;
    b *= M_PI / 180.0;
    a *= dist_to_object;
    b *= dist_to_object;

    return fabs(a - b);
}

double CameraModel::calc_object_height(double dist_to_object, Point2d start_pixel, Point2d end_pixel) const
{
    double a = start_pixel.y * fov_ / h_.get();
    double b = end_pixel.y * fov_ / h_.get();
    a *= M_PI / 180.0;
    b *= M_PI / 180.0;
    a *= dist_to_object;
    b *= dist_to_object;

    return fabs(a - b);
}

double CameraModel::get_w() const
{
    return w_.get();
}

double CameraModel::get_h() const
{
    return h_.get();
}

Point2d CameraModel::get_size() const
{
    return Point2d(w_.get(), h_.get());
}