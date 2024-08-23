#pragma once
#include<opencv2/opencv.hpp>


class Plot
{
public:
	int hight_image;
	int width_image;
	int x_ratio;
	int y_ratio;
	int x_offsit;
	int y_offsit;
	cv::Mat plot;
	int cellSize;

	Plot(int image_height, int image_width, int map_size)
	{
		x_offsit = 50;
		y_offsit = 50;
		hight_image = image_height;
		width_image = image_width;
		y_ratio = hight_image / map_size;
		x_ratio = width_image / map_size;
		cellSize = map_size;

		plot = cv::Mat((hight_image + y_offsit*2), (width_image + x_offsit*2), CV_32FC3);
		plot = cv::Scalar(255, 255, 255);
		string text = "Plot Title";
		// get boundary of this text
		cv::Size textsize = cv::getTextSize(text, 0, 1, 2,0);
		cv::Point textOrg((plot.cols - textsize.width) / 2, 30);
		cv::putText(plot, text, textOrg, 0, 1, cv::Scalar(0, 0, 0),2);
	}

	std::tuple<float, float> computeArray(float x, float y, float theta, float length = 20)
	{
		std::vector<float> xv;
		std::vector<float> yv;

		float xg = x + length * cos(theta);
		float yg = y - length * sin(theta);

		return std::make_tuple(xg, yg);
	}

	void PlotLine(cv::Mat& img, std::vector<double> x, std::vector<double>y)
	{
		std::vector<cv::Point> pointsL;
		for (int i = 0; i < x.size(); i++)
		{
			pointsL.push_back(cv::Point(x[i] * x_ratio + x_offsit, hight_image - (y[i] * y_ratio) + y_offsit));
		}
		cv::polylines(img, pointsL, false, cv::Scalar(0, 0, 255), 2, 150, 0);
	}

	void plotObstical(cv::Mat& plot, std::vector<std::vector<int>> map)
	{
		for (int y = 0; y < map.size(); y++)
		{
			for (int x = 0; x < map[y].size(); x++)
			{
				if (map[y][x] == 1)
				{
					cv::circle(plot, cv::Point2d(x * x_ratio + x_offsit, hight_image - (y * y_ratio) + y_offsit), 6, cv::Scalar(0, 0, 0), -1);
				}
			}
		}
	}

	void process(std::vector<double> x, std::vector<double> y, std::vector<double> theta, std::vector<std::vector<int>> map, bool save_image = false)
	{
		cv::Mat plot_temp;
		DrawGrid(plot);
		plotObstical(plot, map);

		for (int i = 0; i < x.size(); i++)
		{
			plot.copyTo(plot_temp);
			float point_x = x[i] * x_ratio + x_offsit;
			float point_y = hight_image - (y[i] * y_ratio)+ y_offsit;
			float xg, yg;
			std::tie(xg, yg) = computeArray(point_x, point_y, theta[i]);
			DrawRectangle(plot_temp, x[i], y[i], theta[i]);
			//cv::circle(plot_temp, cv::Point(point_x, point_y), 5, cv::Scalar(255, 0, 0), -1);
			cv::line(plot_temp, cv::Point(point_x, point_y), cv::Point(xg, yg), cv::Scalar(255, 0, 0), 2);
			cv::imshow("img", plot_temp);
			cv::waitKey(90);
			if (save_image != 0)
			{
				std::string savingName = std::to_string(i) + ".jpg";
				//std::string savingName = "datas.jpg";
				cv::imwrite(savingName, plot_temp);
			}
		}
		PlotLine(plot_temp, x, y);
		if (save_image != 0)
		{
			std::string savingName =  "final.jpg";
			//std::string savingName = "datas.jpg";
			cv::imwrite(savingName, plot_temp);
		}
		cv::imshow("img", plot_temp);
		cv::waitKey();
	}

	void DrawGrid(cv::Mat& img)
	{
		float _x_ration = x_ratio * 2;
		float _y_ration = y_ratio * 2;
		int i = 0;
		for (int x = 0; x <= width_image+ y_offsit; x += _x_ration)
		{
			cv::line(img, cv::Point(x+ x_offsit, y_offsit), cv::Point(x + x_offsit, hight_image+ y_offsit), cv::Scalar(0, 0, 0), 1);
			cv::putText(img, to_string(int(i)), cv::Point(x - 8 + x_offsit, hight_image + y_offsit + 20), 1, 1, cv::Scalar(0, 0, 0));
			i+= cellSize/10;
		}

		for (int y = 0; y <= hight_image + x_ratio; y += _y_ration)
		{
			i -= cellSize / 10;
			cv::line(img, cv::Point(0 + x_offsit, y+ y_offsit), cv::Point(width_image + x_offsit, y+ y_offsit), cv::Scalar(0, 0, 0), 1);
			cv::putText(img, to_string(int(i)), cv::Point(10 , y + y_offsit+5), 1, 1, cv::Scalar(0, 0, 0));
		}
	}

	void DrawRectangle(cv::Mat& map, float x, float y, float theta)
	{
		float LENGTH = 2;
		float WIDTH = 1;
		float RARE_TO_CG = 0.5;

		double x1 = (x - (LENGTH / 2) + RARE_TO_CG);
		double y1 = y - (WIDTH - ((WIDTH / 2)));
		double x2 = (x + (LENGTH / 2) + RARE_TO_CG);
		double y2 = y - (WIDTH - ((WIDTH / 2)));
		double x3 = (x - (LENGTH / 2) + RARE_TO_CG);
		double y3 = y + (WIDTH - ((WIDTH / 2)));
		double x4 = (x + (LENGTH / 2) + RARE_TO_CG);
		double y4 = y + (WIDTH - ((WIDTH / 2)));

		double xw1 = x - 0.15 ;
		double yw1 = y + (WIDTH - ((WIDTH / 2))) - WIDTH*0.2;
		double xw2 = x + 0.15 ;
		double yw2 = y + (WIDTH - ((WIDTH / 2))) - WIDTH * 0.2;

		double xw3 = x - 0.15;
		double yw3 = y - (WIDTH - ((WIDTH / 2))) + WIDTH * 0.2;
		double xw4 = x + 0.15;
		double yw4 = y - (WIDTH - ((WIDTH / 2))) + WIDTH * 0.2;

		auto p1 = rotatePoints(x1, y1, x, y, theta);
		auto p2 = rotatePoints(x2, y2, x, y, theta);
		auto p3 = rotatePoints(x3, y3, x, y, theta);
		auto p4 = rotatePoints(x4, y4, x, y, theta);

		auto pw1 = rotatePoints(xw1, yw1, x, y, theta);
		auto pw2 = rotatePoints(xw2, yw2, x, y, theta);
		auto pw3 = rotatePoints(xw3, yw3, x, y, theta);
		auto pw4 = rotatePoints(xw4, yw4, x, y, theta);

		//cout << x1 << ", " << x2 << ", " << x3 << ", " << x4 << endl;
		//cout << y1 << ", " << y2 << ", " << y3 << ", " << y4 << endl;

		cv::line(map, p1, p2, cv::Scalar(0, 0, 0), 1.2);
		cv::line(map, p1, p3, cv::Scalar(0, 0, 0), 1.2);
		cv::line(map, p3, p4, cv::Scalar(0, 0, 0), 1.2);
		cv::line(map, p2, p4, cv::Scalar(0, 0, 0), 1.2);
		cv::line(map, pw1, pw2, cv::Scalar(0, 0, 0), 2);
		cv::line(map, pw3, pw4, cv::Scalar(0, 0, 0), 2);
	}

	cv::Point rotatePoints(
		double _x,     //X coords to rotate - replaced on return
		double _y,     //Y coords to rotate - replaced on return
		double cx,      //X coordinate of center of rotation
		double cy,      //Y coordinate of center of rotation
		double angle)   //Angle of rotation (radians, counterclockwise)
	{

		double temp;
		double x = ((_x - cx) * cos(angle) - (_y - cy) * sin(angle)) + cx;
		double y = ((_x - cx) * sin(angle) + (_y - cy) * cos(angle)) + cy;

		x = x * x_ratio + x_offsit;
		y = hight_image - (y * y_ratio) + y_offsit;

		return cv::Point(x, y);
	}
};











