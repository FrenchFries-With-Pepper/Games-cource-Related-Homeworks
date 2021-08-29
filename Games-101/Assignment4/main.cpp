#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

template <class T>
T max(T a, T b)
{
    return a > b ? a : b;
}

template <class T>
T min(T a, T b)
{
    return a < b ? a : b;
}

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

cv::Point2f lerp(float t, const cv::Point2f &p0, const cv::Point2f &p1)
{
    return (1 - t) * p0 + t * p1;
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> points = control_points;
    int degree = control_points.size() - 1;
    int offset = 0;
    for (int i = 0; i < degree; i++)
    {
        for (int j = 0; j < degree - i; j++)
        {
            points.push_back(lerp(t, points[offset + j], points[offset + j + 1]));
        }
        offset += control_points.size() - i;
    }
    return points[points.size() - 1];
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    for (int i = 0; i < 1000; i++)
    {
        float t = i / 1000.f;
        auto point = recursive_bezier(control_points, t);
        int x = point.x;
        int xmin = x - 1;
        int xmax = x + 1;
        int y = point.y;
        int ymin = y - 1;
        int ymax = y + 1;
        float xminBlend = 1 - (point.x - x);
        float xmaxBlend = point.x - x;
        float yminBlend = 1 - (point.y - y);
        float ymaxBlend = point.y - y;
        window.at<cv::Vec3b>(ymin, xmin)[1] = min(255, (int)(window.at<cv::Vec3b>(ymin, xmin)[1] + 255 * xminBlend * yminBlend));
        window.at<cv::Vec3b>(ymin, x)[1] = min(255, (int)(window.at<cv::Vec3b>(ymin, x)[1] + 255 * yminBlend));
        window.at<cv::Vec3b>(ymin, xmax)[1] = min(255, (int)(window.at<cv::Vec3b>(ymin, xmax)[1] + 255 * xmaxBlend * yminBlend));
        window.at<cv::Vec3b>(y, xmin)[1] = min(255, (int)(window.at<cv::Vec3b>(y, xmin)[1] + 255 * xminBlend));
        window.at<cv::Vec3b>(y, x)[1] = 255;
        window.at<cv::Vec3b>(y, xmax)[1] = min(255, (int)(window.at<cv::Vec3b>(y, xmax)[1] + 255 * xmaxBlend));
        window.at<cv::Vec3b>(ymax, xmin)[1] = min(255, (int)(window.at<cv::Vec3b>(ymax, xmin)[1] + 255 * xminBlend * ymaxBlend));
        window.at<cv::Vec3b>(ymax, x)[1] = min(255, (int)(window.at<cv::Vec3b>(ymax, x)[1] + 255 * ymaxBlend));
        window.at<cv::Vec3b>(ymax, xmax)[1] = min(255, (int)(window.at<cv::Vec3b>(ymax, xmax)[1] + 255 * xmaxBlend * ymaxBlend));

        // window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27)
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4)
        {
            // naive_bezier(control_points, window);
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
