#ifndef A_STAR_ASTARALGORITHM_H
#define A_STAR_ASTARALGORITHM_H

#include <array>
#include <vector>
#include <set>
#include <cfloat>
#include <cmath>

constexpr auto DISTANCE_UNIT_STRAIGHT = 1.f;
constexpr auto DISTANCE_UNIT_DIAGONAL = 1.414213f;

namespace AStarAlgorithm
{
	using MATRIX = std::pair<std::vector<size_t>, std::pair<size_t, size_t>>;

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

	class AStarAlgorithm
	{
	public:
		AStarAlgorithm() = delete;
		AStarAlgorithm(METRIC metric, bool allowDiagonal = true);

		void setAllowDiagonal(bool allowDiagonal);

		bool isAllowDiagonal();

		std::vector<std::pair<size_t, size_t>> getPath(const MATRIX& grid, const std::pair<size_t, size_t>& start, const std::pair<size_t, size_t>& finish);

	private:
		METRIC m_metric;
		bool m_allowDiagonal;
	};
}

#endif //A_STAR_ASTARALGORITHM_H
