#pragma once

#include <config_reader/yaml_reader.h>
#include <point/point.h>

#include <navig/MsgLocalPosition.h>

class CameraModel
{
public:
    CameraModel(const YamlReader& cfg): cfg_(cfg) { fov_ = calc_fov(); }

    //преобразование координат пикселя в координаты, связанные с системой координат кадра
    //предварительно вычисляются координаты пикселя без дисторсии
    Point2d frame_coord(Point2d pixel) const;
    Point2d undistort_pixel(Point2d point) const;

    //принимает координаты пикселя, приводит их в систему координат кадра, убирает дисторсию и возвращает
    //относительный курс на этот пиксель
    //Курс возрастает вправо, измеряется в градусах
    double heading_to_pixel(Point2d pixel) const;

    virtual double bearing_to_point(Point2d point) const = 0;

    //принимает реальный размер объекта в метрах и начальную и конечную точку объекта в кадре в координатах кадра.
    //Возвращает расстояние до объекта. В метрах
    double calc_dist_to_object(double real_size, Point2d start_point, Point2d end_point) const;
    double calc_dist_to_object(double real_size, int pixel_size) const;

    virtual navig::MsgLocalPosition navig_offset_to_object(
        double heading, double real_size, int pixel_size, Point2d pixel) const = 0;
    virtual navig::MsgLocalPosition navig_offset_to_object(
        double heading, double real_size, Point2d start, Point2d end, Point2d center) const = 0;

    virtual double calc_depth_to_object(double real_size, int pixel_size, Point2d pixel) const = 0;
    virtual double calc_depth_to_object(double real_size, Point2d start, Point2d end, Point2d center) const = 0;

    //возвращает размер объекта в метра исходя из его размеров в кадре и расстояния до объекта. Координаты
    //объекта должны быть в координатах кадра
    double calc_object_width(double dist_to_object, Point2d start_pixel, Point2d end_pixel) const;
    double calc_object_height(double dist_to_object, Point2d start_pixel, Point2d end_pixel) const;

    double get_w() const;
    double get_h() const;
    Point2d get_size() const; //округляет до целых


    Point2d calc_undistort(Point2d distort_pixel) const;
protected:
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

class BottomCamera: public CameraModel
{
public:
    BottomCamera(const YamlReader& cfg = YamlReader("bottom_camera.yml", "mission")): CameraModel(cfg) {}

    double bearing_to_point(Point2d point) const override;

    double calc_depth_to_object(double real_size, int pixel_size, Point2d pixel) const override;
    double calc_depth_to_object(double real_size, Point2d start, Point2d end, Point2d center) const override;

    navig::MsgLocalPosition navig_offset_to_object(
        double heading, double real_size, int pixel_size, Point2d pixel) const override;
    navig::MsgLocalPosition navig_offset_to_object(
        double heading, double real_size, Point2d start, Point2d end, Point2d center) const override;

};

class FrontCamera: public CameraModel
{
public:
    FrontCamera(const YamlReader& cfg = YamlReader("front_camera.yml", "mission")): CameraModel(cfg) {}

    double bearing_to_point(Point2d point) const override;

    double calc_depth_to_object(double real_size, int pixel_size, Point2d pixel) const override;
    double calc_depth_to_object(double real_size, Point2d start, Point2d end, Point2d center) const override;

    navig::MsgLocalPosition navig_offset_to_object(
        double heading, double real_size, int pixel_size, Point2d pixel) const override;
    navig::MsgLocalPosition navig_offset_to_object(
        double heading, double real_size, Point2d start, Point2d end, Point2d center) const override;

};

