#pragma once
#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <tuple>
#define M_PI 3.141592653589793
#define radian(x) ((x * M_PI) / 180)
#define degree(x) ((x * 180) / M_PI)

/// [m] --- The number of discretizations in heading
static const int headings = 72;
/// [c*M_PI] --- The discretization value of heading (goal condition)
static const float deltaHeadingRad = 2 * M_PI / (float)headings;

/*!
   \fn float normalizeHeadingRad(float t)
   \brief Normalizes a heading given in rad to (0,2PI]
   \param t heading in rad
*/
static inline float normalizeHeadingRad(float t) {
	if (t < 0) {
		t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
		return 2.f * M_PI + t;
	}

	return t - 2.f * M_PI * (int)(t / (2.f * M_PI));

}
// R = 6, 6.75 DEG
const float dy[3] = { 0,        -0.0415893,  0.0415893 };
const float dx[3] = { 0.7068582,   0.705224,   0.705224 };
const float dt[3] = { 0,         0.1178097,   -0.1178097 };


using namespace std;
class State {
public:
	State() {}
	State(float x_, float y_, float theta_) {
		x = x_;
		y = y_;
		theta = theta_;
		cost = 0.0001;
		total_cost = 0;
		move_idx = -1;
	}

	float distance(State s) {
		return sqrt((x - s.x) * (x - s.x) + (y - s.y) * (y - s.y));
	}

	bool state_close(State s) {
		float d_angle = abs(fmod(theta - s.theta + M_PI, 2 * M_PI) - M_PI);
		if (d_angle > radian(15))
			return false;
		if (distance(s) > 2)
			return false;
		return true;
	}

	State end_pose(float curvature, float length, bool parent=false) {
		float xx, yy, tt;
		if (parent)
		{
			xx = previous->x;
			yy = previous->y;
			tt = previous->theta;
		}
		else
		{
			xx = x;
			yy = y;
			tt = theta;
		}
		if (curvature == 0) {
			float x_ = xx + length * cos(tt);
			float y_ = yy + length * sin(tt);
			return State(x_, y_, theta);
		}
		else {
			float tx = cos(tt);
			float ty = sin(tt);
			float radius = 1 / curvature;
			float xc = xx - radius * ty;
			float yc = yy + radius * tx;
			float angle = length / radius;
			float cosa = cos(angle);
			float sina = sin(angle);
			float nx = xc + radius * (cosa * ty + sina * tx);
			float ny = yc + radius * (sina * ty - cosa * tx);
			float ntheta = fmod(tt + angle + M_PI, (2 * M_PI)) - M_PI;
			return State(nx, ny, ntheta);
		}
	}

	// This is a new function use to compute the new state
	State end_pos_new(const int i)
	{
		float xx, yy, tt;
		// calculate the forward position
		if (i < 3)
		{
			xx = x + dx[i] * cos(theta) - dy[i] * sin(theta);
			yy = y + dx[i] * sin(theta) - dy[i] * cos(theta);
			tt = normalizeHeadingRad(theta + dt[i]);
		}
		else
		{
			xx = x - dx[i - 3] * cos(theta) - dy[i - 3] * sin(theta);
			yy = y - dx[i - 3] * sin(theta) - dy[i - 3] * cos(theta);
			tt = normalizeHeadingRad(theta - dt[i - 3]);
		}
		return State(xx, yy, tt);
	}

	tuple<int, int, int> pose_index()
	{
		float pos_raster = 1.0;
		float heading_raster = radian(10.0);
		int xi = int(floor(x / pos_raster));
		int yi = int(floor(y / pos_raster));
		int thetai = int(floor(abs(theta) / heading_raster));
		return make_tuple(xi, yi, thetai);

	}

	vector<State> segment_points(float curvature, float length, float delta_length)
	{
		//"""Return points of segment, at delta_length intervals."""
		float l = 0.0;
		delta_length = copysign(delta_length, length);
		vector< State> points;
		while (abs(l) < abs(length))
		{
			points.push_back(end_pose(curvature, l, true));
			l += delta_length;
		}
		return points;
	}

	void print() {
		if (previous == nullptr)
		{
			std::cout << total_cost << " , " << cost << " , " << x << " , " << y << "\n";
		}
		else
		{
			std::cout << total_cost << " , " << cost << " , " << x << " , " << y << " ==> "
				<< previous->x << " , " << previous->y << " , " << previous->theta << " , " << previous->move_idx << "\n";
		}
	}

	bool operator==(const State& s) const {
		if (abs(x - s.x) > 0.01)
			return false;
		if (abs(y - s.y) > 0.01)
			return false;
		if (abs(theta - s.theta) > 0.01)
			return false;
		return true;
	}

	bool operator<(const State& s) const {
		if (total_cost > s.total_cost)
			return true;
		return false;
	}

	int setIdx(int width, int height)
	{
		this->idx = (int)(theta / deltaHeadingRad) * (int)width * (int)height + (int)y * (int)width + (int)x;
		return this->idx;
	}
	//vector<State> getNextStates();
	float x, y, theta, cost, total_cost;
	int move_idx, idx;
	State* previous;
};
