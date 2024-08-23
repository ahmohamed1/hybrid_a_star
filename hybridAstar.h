#include "stdio.h"
#include "State.h"
#include <queue>
#include <iostream>
#include <chrono>
// #include<boost/heap/priority_queue.hpp>
#include"plot.h"

// #define CVPLOT_HEADER_ONLY
// #include <CvPlot/cvplot.h>

using namespace std::chrono;


template<typename T>                                                      //default argument
std::vector<std::vector<std::vector<T>>> make_3d_vector(int z, int y, int x, T value = T{})
{
	return std::vector<std::vector<std::vector<T>>>(z, std::vector<std::vector<T>>(y, std::vector<T>(x, value)));
}
template<typename T>
std::vector<std::vector<T>> make_2d_vector(int y, int x, T value = T{})
{
	return std::vector<std::vector<std::vector<T>>>(y, std::vector<T>(x, value));
}

class Hybrid
{
public:
	Hybrid(int map_size, bool optimize = false)
	{
		OPTIMIZE = optimize;
		MAP_SIZE = map_size;
		world_extents[0] = { map_size };
		world_extents[1] = { map_size };

		if (OPTIMIZE)
		{
			visited = make_3d_vector(map_size, map_size, 36, -1);
		}
		//visited2d = make_2d_vector(map_size, map_size, -1);
	}

	vector<State> search(State start_pose, State goal_pose, std::vector<std::vector < int >>  obstacles)
	{
		vector<State> path;
		// boost::heap::priority_queue<State> pq;
		std::priority_queue<State> pq;
		start_pose.total_cost = start_pose.distance(goal_pose);
		start_pose.setIdx(MAP_SIZE, MAP_SIZE);
		pq.push(start_pose);
		int z = 0;
		while (!pq.empty())
		{
			if (pq.size() > 5000000)
			{
				cout << " pq size: " << pq.size() << "\n";
				cout << "Timeout." << endl;
				break;
			}
			State current = pq.top();
			pq.pop();

			if (OPTIMIZE)
			{
				int xi, yi, thetai;
				tie(xi, yi, thetai) = current.pose_index();
				if (visited[xi][yi][thetai] == 1)
				{
					//std::cout << "visited\n";
					continue;
				}
				/// This use to optimise the search and limited 
				visited[yi][xi][thetai] = 1;
			}
			


			//current.print();
			if (current.state_close(goal_pose))
			{
				State* s = &current;
				//path.push_back(goal_pose);
				while (!(*s == start_pose))
				{
					int move_index = s->move_idx;
					float x = s->previous->x;
					float y = s->previous->y;
					float theta = s->previous->theta;
					auto _temp_path = s->segment_points(movements[move_index][0], movements[move_index][1], 1);
					reverse(_temp_path.begin(), _temp_path.end());
					path.insert(path.end(), _temp_path.begin(), _temp_path.end());
					//path.push_back(*s);
					s = s->previous;
				}
				path.push_back(*s);
				reverse(path.begin(), path.end());
				cout << "Number of visited node: " << z << "\n";
				return path;
			}

			for (int i = 0; i < max_movement_id; i++)
			{
				float curvature = movements[i][0];
				float length = movements[i][1];
				State new_pose = current.end_pose(curvature, length);
				//State new_pose = current.end_pos_new(i);
				new_pose.setIdx(MAP_SIZE, MAP_SIZE);
				if (new_pose.x < 0 || new_pose.x >= world_extents[1])
					continue;
				if (new_pose.y < 0 || new_pose.y >= world_extents[0])
					continue;
				if (obstacles[(world_extents[0]-1) - int(new_pose.y)][int(new_pose.x)] != 1)
				{
					new_pose.cost = current.cost + abs(length);
					new_pose.total_cost = new_pose.cost + new_pose.distance(goal_pose);// +movements_cost[i];
					new_pose.previous = new State(current);
					new_pose.move_idx = i;
					//cout << i << " ===";
					//new_pose.print();
					pq.push(new_pose);
					//visited2d[19 - int(new_pose.y)][int(new_pose.x)] = 1;
				}
				//else {
				//	cout << int(new_pose.x) <<" , " << 19 - int(new_pose.y) << "====>";
				//	cout << "obstical\n";
				//}
			}
			z++;
		}
		cout << "Number of visited node: " << z << "\n";
		cout << " Failed to find path...\n";
		return path;
	}

	void SetLength(float length)
	{
		LENGTH = length;
	}

	void SetMovements(float _length, float angle, float steps)
	{
		vector<float> length{ -_length , _length };
		for (float l:length)
		{
			for (float a = -angle; a < angle+0.01; a += steps)
			{
				std::vector<float> direction{float(radian(a)), l};
				MOVEMENTS.push_back(direction);
			}
		}
	}
	void PrintMovements()
	{
		for (int i = 0; i< MOVEMENTS.size(); i ++)
		{
			cout << i << "--> angle: " << MOVEMENTS[i][0] << " , length: " << MOVEMENTS[i][1] << endl;
		}
	}

private:
	float LENGTH = 2;
	float movements[8][2] = { {(2.0 / 10), LENGTH}, {(1.0 / 10), LENGTH}, {0.0, LENGTH}, {(-1.0 / 10), LENGTH},{(-2.0 / 10), LENGTH}, {(1.0 / 10), -LENGTH}, {0.0, -LENGTH}, {(-1.0 / 10), -LENGTH} };
	float movements_cost[6] = { 0.05, 0.0, 0.05, 0.2, 0.1, 0.2 };
	int max_movement_id = 8;
	int world_extents[2];
	bool OPTIMIZE;
	int MAP_SIZE;

	std::vector<std::vector<float>> MOVEMENTS;
	std::vector<std::vector<std::vector<int>>> visited;
	//std::vector<std::vector<int>> visited2d;
	//int visited[20][20][36];

	
};
