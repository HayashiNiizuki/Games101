#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) {
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) {
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 + 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, double t) {
    // TODO: Implement de Casteljau's algorithm
    auto &p0 = control_points[0];
    auto &p1 = control_points[1];
    auto &p2 = control_points[2];
    auto &p3 = control_points[3];

    auto p10 = t * p0 + (1 - t) * p1;
    auto p11 = t * p1 + (1 - t) * p2;
    auto p12 = t * p2 + (1 - t) * p3;

    auto p20 = t * p10 + (1 - t) * p11;
    auto p21 = t * p11 + (1 - t) * p12;

    return t * p20 + (1 - t) * p21;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) {
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        cv::Point2f point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        printf("%f\t%f\t%f\t\n", t, point.x, point.y);

        float pos_x = (point.x - floor(point.x)) > 0.5 ? 1 : -1;
        float pos_y = (point.y - floor(point.y)) > 0.5 ? 1 : -1;

        std::vector<cv::Point2f> vec;
        vec.push_back(cv::Point2f(floor(point.x + pos_x), floor(point.y)));
        vec.push_back(cv::Point2f(floor(point.x), floor(point.y + pos_y)));
        vec.push_back(cv::Point2f(floor(point.x + pos_x), floor(point.y + pos_y)));

        auto d = cv::Point2f(point.x - floor(point.x) - 0.5, point.y - floor(point.y) - 0.5);
        float dis = sqrt(d.x * d.x + d.y * d.y);

        for (const auto &p : vec) {
            float cx = p.x + 0.5;
            float cy = p.y + 0.5;

            auto d1 = cv::Point2f(cx - floor(point.x) - 0.5, cy - floor(point.y) - 0.5);
            float l = sqrt(d1.x * d1.x + d1.y * d1.y);
            
            auto color = window.at<cv::Vec3b>(cy, cx)[1];
            window.at<cv::Vec3b>(cy, cx)[1] = std::max((int)color, (int)(255 * (dis / l)));
        }
    }
}

int main() {
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) {
        for (auto &point : control_points) {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) {
            if (false)
                naive_bezier(control_points, window);
            else
                   bezier(control_points, window);
            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
