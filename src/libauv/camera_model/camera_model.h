#pragma once

#include <config_reader/yaml_reader.h>
#include <point/point.h>

class CameraModel
{
public:
    static CameraModel create_front_camera()
    {
        return CameraModel(YamlReader("front_camera.yml", "libauv"));
    }

    static CameraModel create_bottom_camera()
    {
        return CameraModel(YamlReader("bottom_camera.yml", "libauv"));
    }

    CameraModel(const YamlReader& cfg): cfg_(cfg) { fov_ = calc_fov(); }


    //преобразование координат пикселя в координаты, связанные с системой координат кадра
    //предварительно вычисляются координаты пикселя без дисторсии
    libauv::Point2d frame_coord(libauv::Point2i pixel) const;

    //принимает координаты пикселя, приводит их в систему координат кадра, убирает дисторсию и возвращает
    //относительный курс на этот пиксель
    //Курс возрастает вправо, измеряется в радианах
    double heading_to_pixel(libauv::Point2i pixel) const;

    double heading_to_point(libauv::Point2d point) const;

    //принимает реальный размер объекта в метрах и начальную и конечную точку объекта в кадре в пикселях.
    //Возвращает расстояние до объекта. В метрах
    double calc_dist_to_object(double real_size, libauv::Point2i start_pixel, libauv::Point2i end_pixel) const;
    double calc_dist_to_object(double real_size, int pixel_size) const;

    //возвращает размер объекта в метра исходя из его размеров в кадре и расстояния до объекта. Координаты
    //объекта должны быть в координатах кадра
    double calc_object_width(double dist_to_object, libauv::Point2i start_pixel, libauv::Point2i end_pixel) const;
    double calc_object_height(double dist_to_object, libauv::Point2i start_pixel, libauv::Point2i end_pixel) const;

    double get_w() const;
    double get_h() const;
    libauv::Point2i get_size() const; //округляет до целых


    libauv::Point2d calc_undistort(libauv::Point2i distort_pixel) const;
private:
    double calc_fov() const;

    YamlReader cfg_;

    AUTOPARAM(double, focal_x_);
    AUTOPARAM(double, focal_y_);
    AUTOPARAM(double, w_);
    AUTOPARAM(double, h_);
    AUTOPARAM(double, center_x_);
    AUTOPARAM(double, center_y_);
    AUTOPARAM(double, k1_);
    AUTOPARAM(double, k2_);
    AUTOPARAM(double, k3_);
    AUTOPARAM(double, t1_);
    AUTOPARAM(double, t2_);
    double fov_;
};