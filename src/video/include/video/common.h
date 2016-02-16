#pragma once

#include <map>
#include <opencv2/opencv.hpp>

enum class Mode
{
    Onboard,
    Debug,
};


enum class Camera
{
    Front,
    Bottom,
    None,
};

const std::map<Camera, std::string> camera_typename = {
    {Camera::Front, "front"},
    {Camera::Bottom, "bottom"},
    {Camera::None, "none"},
};

enum class Ipc
{
    On,
    Off,
};

enum class Color
{
    Green,
    Yellow,
    Red,
    Orange,
    Blue,
    Black,
    White,
};

const std::map<Color, std::string> name_by_color = {
    {Color::Green, "green"},
    {Color::Yellow, "yellow"},
    {Color::Red, "red"},
    {Color::Orange, "orange"},
    {Color::Blue, "blue"},
    {Color::Black, "black"},
    {Color::White, "white"},
};

const std::map<std::string, Color> color_by_name = {
    {"green", Color::Green},
    {"yellow", Color::Yellow},
    {"red", Color::Red},
    {"orange", Color::Orange},
    {"blue", Color::Blue},
    {"black", Color::Black},
    {"white", Color::White},
};

const std::map<Color, cv::Scalar> scalar_by_color = {
    {Color::Orange, cv::Scalar(30, 75, 250)},
    {Color::Red, cv::Scalar(0, 0, 255)},
    {Color::Green, cv::Scalar(0, 255, 0)}
};
