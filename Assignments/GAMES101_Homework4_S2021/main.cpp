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
    // Implement de Casteljau's algorithm
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

cv::Vec3b maxColor(const cv::Vec3b &a, const cv::Vec3b &b) {
    cv::Vec3b c(std::max(a[0], b[0]), std::max(a[1], b[1]), std::max(a[2], b[2]));
    return c;
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window, bool ifAntiAlias) {
    // Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.0005) {
        cv::Point2f center = recursive_bezier(control_points, t);
        cv::Vec3b color(126, 36, 131);
        // bool ifAntiAlias = true;
        if (ifAntiAlias) {

            float pos_x = (center.x - floor(center.x)) > 0.5 ? 1 : -1;
            float pos_y = (center.y - floor(center.y)) > 0.5 ? 1 : -1;

            cv::Point2f xBias(center.x + pos_x, center.y), yBias(center.x, center.y + pos_y), xyBias(center.x + pos_x, center.y + pos_y);

            auto biasX = fabs(center.x - floor(center.x) - 0.5);
            auto biasY = fabs(center.y - floor(center.y) - 0.5);

            float centerPercent = sqrt((1 - biasX) * (1 - biasX) + (1 - biasY) * (1 - biasY));
            float xBiasPercent = sqrt(biasX * biasX + (1 - biasY) * (1 - biasY));
            float yBiasPercent = sqrt((1 - biasX) * (1 - biasX) + biasY * biasY);
            float xyBiasPercent = sqrt(biasX * biasX + biasY * biasY);

            window.at<cv::Vec3b>(center.y, center.x) = maxColor((centerPercent * color), window.at<cv::Vec3b>(center.y, center.x));
            window.at<cv::Vec3b>(xBias.y, xBias.x) = maxColor((xBiasPercent * color), window.at<cv::Vec3b>(xBias.y, xBias.x));
            window.at<cv::Vec3b>(yBias.y, yBias.x) = maxColor((yBiasPercent * color), window.at<cv::Vec3b>(yBias.y, yBias.x));
            window.at<cv::Vec3b>(xyBias.y, xyBias.x) = maxColor((xyBiasPercent * color), window.at<cv::Vec3b>(xyBias.y, xyBias.x));
        } else {
            window.at<cv::Vec3b>(center.y, center.x)[1] = 255;
        }
        // printf("%f\t%f\t%f\t\n", t, point.x, point.y);
    }
}

int main(int argc, char *argv[]) {
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    bool ifAntiAlias;
    if (argc == 1) {
        ifAntiAlias = (argv[0] == "true");
    } else {
        ifAntiAlias = true;
    }
    while (key != 27) {
        for (auto &point : control_points) {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) {
            if (false)
                naive_bezier(control_points, window);
            else
                bezier(control_points, window, ifAntiAlias);
            cv::imshow("Bezier Curve", window);
            if (ifAntiAlias)
                cv::imwrite("my_bezier_curve_alias.png", window);
            else
                cv::imwrite("my_bezier_curve_normal.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
