#include "opencv2/opencv.hpp"
#include <iostream>


class Grid {
public:
    cv::Mat map, mapObstical, tempMap;
    std::vector<std::vector<int>> grid;
    int  start[3];
    float startTheat;
    int goal[3];
    float pixel2meter;
    bool LeftMouseDown;
    bool RightMouseDown;
    int ImageHieght;

    void DrawDirection() {
        int x2 = 10 * std::cos(start[2]);
        int y2 = 10 * std::sin(start[2]);
        cv::circle(map, cv::Point(start[0], start[1]), 4, cv::Scalar(0, 0, 255));
        cv::line(map, cv::Point(start[0], start[1]), cv::Point(start[0] + x2, start[1] - y2), cv::Scalar(0, 0, 255));
    }


    Grid(std::string mapPath, float pixelToMeter = 1.0, float raduis = 0.1, int costSize = 5) {

        goal[0] = -1;
        start[0] = -1;
        // Read map
        map = cv::imread(mapPath);
        ImageHieght = map.rows;
        pixel2meter = pixelToMeter;
        std::cout << "Map Size: " << map.cols * pixel2meter << " mtr," << map.rows * pixel2meter << " mtr\n";
        if (map.empty())
        {
            std::cout << "[ERROR] the was no map loaded...";
            exit;
        }
        cv::namedWindow("map", cv::WINDOW_NORMAL);
        cv::resizeWindow("map", 840, 620);
        //set the callback function for any mouse event
        cv::setMouseCallback("map", mouseHandler, this);

        // extand obstical according to the raduis
        cv::cvtColor(map, mapObstical, cv::COLOR_BGR2GRAY);
        int dilation_type = 0;
        int dilation_size = raduis / pixel2meter;
        cv::Mat element = cv::getStructuringElement(dilation_type,
            cv::Size(dilation_size, dilation_size),
            cv::Point(-1, -1));
        cv::erode(mapObstical, mapObstical, element);

        cv::cvtColor(mapObstical, mapObstical, cv::COLOR_GRAY2BGR);
        grid = ConvertMatToVector(mapObstical);

        cv::Mat _map;

        BlurPenaltyMap(costSize, grid);

        while (true)
        {
            _map = ComputeCostMap(grid);

            if (start[0] != -1)
            {
                int x2 = 10 * std::cos(start[2]);
                int y2 = 10 * std::sin(start[2]);
                cv::circle(_map, cv::Point(start[1], start[0]), 3, cv::Scalar(0, 0, 255));
                cv::line(_map, cv::Point(start[1], start[0]), cv::Point(start[1] + x2, start[0] - y2), cv::Scalar(0, 0, 255));
            }
            if (goal[0] != -1)
            {
                int x2 = 10 * std::cos(goal[2]);
                int y2 = 10 * std::sin(goal[2]);
                cv::circle(_map, cv::Point(goal[1], goal[0]), 3, cv::Scalar(255, 255, 0));
                cv::line(_map, cv::Point(goal[1], goal[0]), cv::Point(goal[1] + x2, goal[0] - y2), cv::Scalar(255, 255, 0));
            }
            //PrintGridCost(grid);
            double alpha = 0.4;
            double beta = (1.0 - alpha);
            addWeighted(mapObstical, alpha, _map, beta, 0.0, _map);
            cv::imshow("map", _map);
            // Wait until user press some key
            char ikey = cv::waitKey(1);
            if (ikey == 'q')
            {
                start[2] = start[2] + M_PI / 2;
                goal[2] = goal[2] + M_PI / 2;
                break;
            }

        }

    }

    Grid() {

    }
    cv::Mat ConvertToMat(std::vector<std::vector<Node>> grid) {
        int x = grid[0].size();
        int y = grid.size();
        cv::Mat map(cv::Size(x, y), CV_8UC3, cv::Scalar(255, 255, 255));
        for (int j = 0; j < y; j++)
        {
            for (int i = 0; i < x; i++)
            {
                if (grid[j][i].state == State::kEmpty)
                {
                    map.at<cv::Vec3b>(j, i) = cv::Vec3b(255, 255, 255);
                }
                else
                {
                    map.at<cv::Vec3b>(j, i) = (0, 0, 0);
                }
            }
        }
        return map;
    }

    std::vector<std::vector < int >> ConvertMatToVector(cv::Mat map) {
        std::vector<std::vector < int >> board;
        for (int y = 0; y < map.rows; y++)
        {
            std::vector<int> row;
            for (int x = 0; x < map.cols; x++)
            {
                if (map.at<cv::Vec3b>(y, x) == cv::Vec3b(255, 255, 255))
                {
                    row.push_back(255);
                }
                else
                {
                    row.push_back(0);
                }

            }
            board.push_back(row);
        }

        return board;
    }

    void DrawPath(std::vector<Node> path) {
        std::vector<float> pathX;
        std::vector<float> pathY;
        for (int i = 0; i < path.size(); i++)
        {
            map.at<cv::Vec3b>(path[i].x, path[i].y) = cv::Vec3b(0, 255, 0);
            pathX.push_back(path[i].x);
            pathY.push_back(path[i].y);
        }
        map.at<cv::Vec3b>(start[0], start[1]) = cv::Vec3b(255, 0, 0);
        map.at<cv::Vec3b>(goal[0], goal[1]) = cv::Vec3b(0, 0, 255);
        cout << "Start: " << start[0] * pixel2meter << "," << start[1] * pixel2meter << "\n";
        cout << "Goal: " << goal[0] * pixel2meter << "," << goal[1] * pixel2meter << "\n";
        DrawDirection();
        /*
        Spline2D csp_obj(pathX, pathY);

        std::vector<float> r_x, r_y, ryaw, rcurvature, rs;
        std::vector<cv::Point> curvePoints;
        for (float i = 0; i < csp_obj.s.back(); i += 25) {
            std::array<float, 2> point_ = csp_obj.calc_postion(i);
            //r_x.push_back(point_[0]);
            //r_y.push_back(point_[1]);
            curvePoints.push_back(cv::Point(point_[1], point_[0]));
            //ryaw.push_back(csp_obj.calc_yaw(i));
            //rcurvature.push_back(csp_obj.calc_curvature(i));
            //rs.push_back(i);
        }
        cv::polylines(map, curvePoints, false, cv::Scalar(0, 0, 255), 1,150,0);
        */
    }

    void DrawDotPath(std::vector<std::vector<float>> path) {
        std::vector<cv::Point> curvePoints;
        for (auto point : path)
        {
            curvePoints.push_back(cv::Point2f(point[1], point[0]));
        }

        cv::polylines(map, curvePoints, false, cv::Scalar(0, 0, 255), 1, 150, 0);
        cv::imshow("map", map);
        // Wait until user press some key
        char ikey = cv::waitKey(0);
    }

    static void mouseHandler(int event, int x, int y, int flags, void* param) {
        Grid* p = (Grid*)param;
        if (event == cv::EVENT_LBUTTONDOWN) {
            p->start[0] = y;
            p->start[1] = x;
            p->LeftMouseDown = true;
            //cout << "Start: " << x  << "," << y << "\n";
            p->map.at<cv::Vec3b>(p->start[0], p->start[1]) = cv::Vec3b(255, 0, 0);
        }
        else if (event == cv::EVENT_LBUTTONUP)
        {
            p->LeftMouseDown = false;
        }
        else if (event == cv::EVENT_MOUSEMOVE)
        {
            if (p->LeftMouseDown)
            {
                p->start[2] = std::atan2f(((p->ImageHieght - y) - (p->ImageHieght - p->start[0])), (x - p->start[1]));
                //std::cout << p->start[1] << " , " << (p->ImageHieght - p->start[0]) << " , " << 10 * std::cos(p->startTheat) << " , " << 10 * std::sin(p->startTheat) << " , " << p->startTheat << "\n";
            }
            else if (p->RightMouseDown)
            {
                p->goal[2] = std::atan2f(((p->ImageHieght - y) - (p->ImageHieght - p->goal[0])), (x - p->goal[1]));
            }
        }
        else if (event == cv::EVENT_RBUTTONDOWN)
        {
            p->goal[0] = y;
            p->goal[1] = x;
            p->RightMouseDown = true;
            //cout << "Goal: " << x << "," << y << "\n";
        }
        else if (event == cv::EVENT_RBUTTONUP)
        {
            p->RightMouseDown = false;
        }
    }

private:

    void BlurPenaltyMap(int blurSize, std::vector<std::vector<Node>>& grid) {
        int kernelSize = blurSize * 2 + 1;
        int kernelExtents = (kernelSize - 1) / 2;

        int gridSizeY = grid.size();
        int gridSizeX = grid[1].size();
        std::vector<std::vector<float>> penaltiesHorizontalPass;
        std::vector<std::vector<float>> penaltiesVerticalPass;

        for (int y = 0; y < gridSizeY; y++)
        {
            std::vector<float> _penaltiesHorizontalPass;
            std::vector<float> _penaltiesVerticalPass;
            for (int x = 0; x < gridSizeX; x++)
            {
                _penaltiesHorizontalPass.push_back(0);
                _penaltiesVerticalPass.push_back(0);
            }
            penaltiesHorizontalPass.push_back(_penaltiesHorizontalPass);
            penaltiesVerticalPass.push_back(_penaltiesVerticalPass);

        }


        for (int y = 0; y < gridSizeY; y++) {
            for (int x = -kernelExtents; x <= kernelExtents; x++) {
                int sampleX = clamp(x, 0, kernelExtents);
                penaltiesHorizontalPass[0][y] += grid[sampleX][y].cost;
            }

            for (int x = 1; x < gridSizeX; x++) {
                int removeIndex = clamp(x - kernelExtents - 1, 0, gridSizeX);
                int addIndex = clamp(x + kernelExtents, 0, gridSizeX - 1);

                penaltiesHorizontalPass[x][y] = penaltiesHorizontalPass[x - 1][y] - grid[removeIndex][y].cost + grid[addIndex][y].cost;
            }
        }

        for (int x = 0; x < gridSizeX; x++) {
            for (int y = -kernelExtents; y <= kernelExtents; y++) {
                int sampleY = clamp(y, 0, kernelExtents);
                penaltiesVerticalPass[x][0] += penaltiesHorizontalPass[x][sampleY];
            }

            float blurredPenalty = penaltiesVerticalPass[x][0] / (kernelSize * kernelSize);
            grid[x][0].cost = blurredPenalty;

            for (int y = 1; y < gridSizeY; y++) {
                int removeIndex = clamp(y - kernelExtents - 1, 0, gridSizeY);
                int addIndex = clamp(y + kernelExtents, 0, gridSizeY - 1);

                penaltiesVerticalPass[x][y] = penaltiesVerticalPass[x][y - 1] - penaltiesHorizontalPass[x][removeIndex] + penaltiesHorizontalPass[x][addIndex];
                blurredPenalty = (float)penaltiesVerticalPass[x][y] / (kernelSize * kernelSize);
                grid[x][y].cost = blurredPenalty;
            }
        }
    }

    cv::Mat ComputeCostMap(std::vector<std::vector<Node>> grid) {

        int x = grid[0].size();
        int y = grid.size();
        cv::Mat map(cv::Size(x, y), CV_8UC3, cv::Scalar(255, 255, 255));
        for (int j = 0; j < y; j++)
        {
            for (int i = 0; i < x; i++)
            {
                if (grid[j][i].cost > 10)
                {
                    map.at<cv::Vec3b>(j, i) = cv::Vec3b(grid[j][i].cost, 0, 0);
                }
                else
                {
                    map.at<cv::Vec3b>(j, i) = cv::Vec3b(255, 255, 255);
                }
            }
        }
        return map;

    }

};
