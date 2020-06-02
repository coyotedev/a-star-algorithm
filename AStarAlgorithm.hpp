#ifndef A_STAR_ASTARALGORITHM_HPP
#define A_STAR_ASTARALGORITHM_HPP

#include <array>
#include <vector>
#include <set>
#include <cfloat>
#include <cmath>

constexpr auto DISTANCE_UNIT_STRAIGHT = 1.f;
constexpr auto DISTANCE_UNIT_DIAGONAL = 1.414213f;

namespace AStarAlgorithm
{
	template<size_t ROW, size_t COL>
	using MATRIX = std::array<std::array<bool, COL>, ROW>;

	enum METRIC
	{
		Manhattan
		, Euclidean
	};

	struct Cell
	{
		std::pair<int, int> posParent = {-1, -1};
		float f = FLT_MAX;
		float g = FLT_MAX;
		float h = FLT_MAX;
	};

	template<typename F, typename S>
	std::pair<F, S> operator+(const std::pair<F, S>& l, const std::pair<F, S>& r)
	{
		return {l.first + r.first, l.second + r.second};
	}

	template<size_t ROW, size_t COL>
	class AStarAlgorithm
	{
	public:
		AStarAlgorithm() = delete;

		AStarAlgorithm(MATRIX<ROW, COL>& grid, METRIC metric)
			: m_grid(grid)
			, m_metric(metric)
		{
		}

		void setCellBlocked(std::pair<size_t, size_t> pos, bool isBlocked)
		{
			m_grid.at(pos.first).at(pos.second) = isBlocked;
		}

		bool isCellBlocked(std::pair<size_t, size_t> pos)
		{
			return not m_grid.at(pos.first).at(pos.second);
		}

		std::vector<std::pair<size_t, size_t>> getPath(std::pair<size_t, size_t> start, std::pair<size_t, size_t> finish);

	private:
		MATRIX<ROW, COL> m_grid;
		METRIC m_metric;
	};

	template<size_t ROW, size_t COL>
	std::vector<std::pair<size_t, size_t>>
	AStarAlgorithm<ROW, COL>::getPath(std::pair<size_t, size_t> start, std::pair<size_t, size_t> finish)
	{
		auto isValid = [](const std::pair<int, int>& cell)
		{
			return (cell.first >= 0 && cell.first < ROW) && (cell.second >= 0 && cell.second < COL);
		};

		auto isDestination = [finish](const std::pair<size_t, size_t>& cell)
		{
			return cell == finish;
		};

		auto calculateManhattan = [](const std::pair<size_t, size_t>& current, const std::pair<size_t, size_t>& dest) -> float
		{
			return abs(int(current.first) - int(dest.first)) + abs(int(current.second) - int(dest.second));
		};
		auto calculateEuclidean = [](const std::pair<size_t, size_t>& current, const std::pair<size_t, size_t>& dest) -> float
		{
			return sqrt(pow(int(current.first) - int(dest.first), 2) + pow(int(current.second) - int(dest.second), 2));
		};
		auto calculate = (m_metric == METRIC::Manhattan) ? calculateManhattan : calculateEuclidean;

		auto processPath = [](const std::array<std::array<Cell, COL>, ROW>& cellMatrix, const std::pair<size_t, size_t>& dest)
		{
			std::vector<std::pair<size_t, size_t>> ret;
			size_t row = dest.first;
			size_t col = dest.second;

			while (not (cellMatrix[row][col].posParent.first == row && cellMatrix[row][col].posParent.second == col))
			{
				ret.emplace(ret.begin(), row, col);
				std::pair<size_t, size_t> tmp = {cellMatrix[row][col].posParent.first, cellMatrix[row][col].posParent.second};
				row = tmp.first;
				col = tmp.second;
			}
			ret.emplace(ret.begin(), row, col); // add start node

			return ret;
		};

		std::vector<std::pair<std::pair<int, int>, float>> directionOffsets =
				{
						  {{-1, 0}, DISTANCE_UNIT_STRAIGHT}	///< NORTH
						, {{1, 0}, DISTANCE_UNIT_STRAIGHT}	///< SOUTH
						, {{0, 1}, DISTANCE_UNIT_STRAIGHT}	///< EAST
						, {{0, -1}, DISTANCE_UNIT_STRAIGHT}	///< WEST
				};
		if (m_metric == METRIC::Euclidean)
		{
			directionOffsets.emplace_back(std::pair<int, int>{-1, 1}, DISTANCE_UNIT_DIAGONAL);	///< NORTH-EAST
			directionOffsets.emplace_back(std::pair<int, int>{-1, -1}, DISTANCE_UNIT_DIAGONAL);	///< NORTH-WEST
			directionOffsets.emplace_back(std::pair<int, int>{1, 1}, DISTANCE_UNIT_DIAGONAL);	///< SOUTH-EAST
			directionOffsets.emplace_back(std::pair<int, int>{1, -1}, DISTANCE_UNIT_DIAGONAL);	///< SOUTH-WEST
		}

		// check if input cells is invalid, abort if so
		if (not (isValid(start) and isValid(finish)))
		{
			return {};
		}

		// check if input cells is blocked, abort if so
		if (isCellBlocked(start) or isCellBlocked(finish))
		{
			return {};
		}

		// check if input cell is identical, it means we have one-cell path
		if (isDestination(start))
		{
			return {start};
		}

		std::array<std::array<Cell, COL>, ROW> cellMatrix = {};
		std::array<std::array<bool, COL>, ROW> closedList = {};
		std::set<std::pair<float, std::pair<int, int>>> openList = {};

		// initialization
		cellMatrix[start.first][start.second] = {{start.first, start.second}, .0f, .0f, .0f};
		openList.emplace(.0f, start);

		while (not openList.empty())
		{
			auto current = *openList.begin();
			openList.erase(openList.begin());
			closedList[current.second.first][current.second.second] = true;

			for (const auto& it : directionOffsets)
			{
				const auto pairPos = current.second + it.first;
				if (isValid(pairPos))
				{
					const auto row = pairPos.first;
					const auto col = pairPos.second;

					if (isDestination(pairPos))
					{
						cellMatrix[row][col].posParent = current.second;
						return processPath(cellMatrix, finish);
					}

					if ((not closedList[row][col]) and (not isCellBlocked(pairPos)))
					{
						const auto l_g = cellMatrix[current.second.first][current.second.second].g + it.second;
						const auto l_h = calculate(pairPos, finish);
						const auto l_f = l_g + l_h;

						if (cellMatrix[row][col].f > l_f)
						{
							openList.emplace(l_f, pairPos);
							cellMatrix[row][col].g = l_g;
							cellMatrix[row][col].h = l_h;
							cellMatrix[row][col].f = l_f;
							cellMatrix[row][col].posParent = current.second;
						}
					}
				}
			}
		}
		return {};
	}
}

#endif //A_STAR_ASTARALGORITHM_HPP
