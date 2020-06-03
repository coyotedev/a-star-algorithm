#include <iostream>

#include "AStarAlgorithm.h"
#include <algorithm>

constexpr auto ROW = 9;
constexpr auto COL = 11;

int main() {

	AStarAlgorithm::MATRIX grid =
			{
					{ 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1
					, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1
					, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1
					, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1
					, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1
					, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1
					, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1
					, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1
					, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1 }
					, {ROW, COL}
			};

	AStarAlgorithm::AStarAlgorithm path(AStarAlgorithm::METRIC::Manhattan);

	auto vec = path.getPath(grid, {8, 0}, {0, 10});

	for (const auto& it : vec)
	{
		const auto start = std::distance(vec.begin(), std::find(vec.begin(), vec.end(), it)) == 0 ? "" : " -> ";
		std::cout << start << "[" << it.first << ", " << it.second << "]";
	}
	std::cout << std::endl;

	for(int i = 0; i < ROW; ++i)
	{
		for(int j = 0; j < COL; ++j)
		{
			const auto it = std::find(vec.begin(), vec.end(), std::pair<size_t, size_t>{i, j});
			std::cout << "|";
			if (it != vec.end())
			{
				if (it == vec.begin())
				{
					std::cout << "\033[1;32m*\033[0m";
				}
				else if (it == vec.end() - 1)
				{
					std::cout << "\033[1;34m*\033[0m";
				}
				else
				{
					std::cout << "\033[1;31m*\033[0m";
				}
			}
			else
			{
				if (not grid.first[i*COL + j])
				{
					std::cout << "\033[1;46m \033[0m";
				}
				else
				{
					std::cout << " ";
				}
			}
		}
		std::cout << "|" << std::endl;
	}

	return 0;
}
