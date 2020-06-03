#include "AStarAlgorithm.h"

template<typename F, typename S>
std::pair<F, S> operator+(const std::pair<F, S>& l, const std::pair<F, S>& r)
{
	return {l.first + r.first, l.second + r.second};
}

AStarAlgorithm::AStarAlgorithm::AStarAlgorithm(METRIC metric, bool allowDiagonal)
		: m_metric(metric)
		, m_allowDiagonal(allowDiagonal)
{
}

void AStarAlgorithm::AStarAlgorithm::setAllowDiagonal(bool allowDiagonal)
{
	m_allowDiagonal = allowDiagonal;
}

bool AStarAlgorithm::AStarAlgorithm::isAllowDiagonal()
{
	return m_allowDiagonal;
}

std::vector<std::pair<size_t, size_t>>
AStarAlgorithm::AStarAlgorithm::getPath(const MATRIX& grid, const std::pair<size_t, size_t>& start, const std::pair<size_t, size_t>& finish)
{
	const auto& [gridArray, size] = grid;
	const auto [ROW, COL] = size;

	auto isValid = [ROW, COL](const std::pair<int, int>& cell)
	{
		return (cell.first >= 0 && cell.first < ROW) && (cell.second >= 0 && cell.second < COL);
	};

	auto isCellBlocked = [gridArray, COL](const std::pair<int, int>& cell)
	{
		const auto& [row, col] = cell;
		return not gridArray.at(row * COL + col);
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

	auto processPath = [COL](const std::vector<Cell>& cellMatrix, const std::pair<size_t, size_t>& dest)
	{
		std::vector<std::pair<size_t, size_t>> ret;
		auto [row, col] = dest;
		auto pos = row * COL + col;

		while (not (cellMatrix[pos].posParent.first == row && cellMatrix[pos].posParent.second == col))
		{
			ret.emplace(ret.begin(), row, col);
			std::pair<size_t, size_t> tmp = {cellMatrix[pos].posParent.first, cellMatrix[pos].posParent.second};
			row = tmp.first;
			col = tmp.second;
			pos = row * COL + col;
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
	if (isAllowDiagonal())
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

	std::vector<Cell> cellMatrix = {};
	std::vector<bool> closedList = {};
	std::set<std::pair<float, std::pair<int, int>>> openList = {};

	// initialization
	cellMatrix.resize(ROW * COL);
	closedList.resize(ROW * COL);
	const auto& [startRow, startCol] = start;
	cellMatrix[startRow * COL + startCol] = {start, .0f, .0f, .0f};
	openList.emplace(.0f, start);

	while (not openList.empty())
	{
		auto current = *openList.begin();
		const auto& [currentRow, currentCol] = current.second;
		openList.erase(openList.begin());
		closedList[currentRow * COL + currentCol] = true;

		for (const auto& it : directionOffsets)
		{
			const auto pairPos = current.second + it.first;
			if (isValid(pairPos))
			{
				const auto& [pairPosRow, pairPosCol] = pairPos;

				if (isDestination(pairPos))
				{
					cellMatrix[pairPosRow * COL + pairPosCol].posParent = current.second;
					return processPath(cellMatrix, finish);
				}

				if ((not closedList[pairPosRow * COL + pairPosCol]) and (not isCellBlocked(pairPos)))
				{
					const auto l_g = cellMatrix[currentRow * COL + currentCol].g + it.second;
					const auto l_h = calculate(pairPos, finish);
					const auto l_f = l_g + l_h;

					if (cellMatrix[pairPosRow * COL + pairPosCol].f > l_f)
					{
						openList.emplace(l_f, pairPos);
						cellMatrix[pairPosRow * COL + pairPosCol].g = l_g;
						cellMatrix[pairPosRow * COL + pairPosCol].h = l_h;
						cellMatrix[pairPosRow * COL + pairPosCol].f = l_f;
						cellMatrix[pairPosRow * COL + pairPosCol].posParent = current.second;
					}
				}
			}
		}
	}
	return {};
}