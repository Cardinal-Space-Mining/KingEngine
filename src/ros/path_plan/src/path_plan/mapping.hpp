#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>
#include <queue>
#include <functional>

#include <Eigen/Core>


// copied from Sick-Perception (grid.hpp)
/** Generic grid helpers */
namespace GridUtils {

	/** Align a point to a box grid of the given resolution and offset origin. Result may be negative if lower than current offset. */
	template<typename IntT = int, typename FloatT = float>
	inline static Eigen::Vector2<IntT> gridAlign(FloatT x, FloatT y, const Eigen::Vector2<FloatT>& off, FloatT res) {
		return Eigen::Vector2<IntT>{
			static_cast<IntT>( std::floor((x - off.x()) / res) ),	// always floor since grid cells are indexed by their "bottom left" corner's raw position
			static_cast<IntT>( std::floor((y - off.y()) / res) )
		};
	}
	template<typename IntT = int, typename FloatT = float>
	inline static Eigen::Vector2<IntT> gridAlign(const Eigen::Vector4<FloatT>& pt, const Eigen::Vector2<FloatT>& off, FloatT res) {
		return gridAlign<IntT, FloatT>(pt.x(), pt.y(), off, res);
	}

	/** Get a raw buffer idx from a 2d index and buffer size (templated on major-order) */
	template<bool X_Major = false, typename IntT = int>
	inline static int64_t gridIdx(const IntT x, const IntT y, const Eigen::Vector2<IntT>& size) {
		if constexpr(X_Major) {
			return static_cast<int64_t>(x) * size.y() + y;	// x-major = "contiguous blocks along [parallel to] y-axis" --> idx = (x * ymax) + y
		} else {
			return static_cast<int64_t>(y) * size.x() + x;	// y-major = "contiguous blocks along [parallel to] x-axis" --> idx = (y * xmax) + x
		}
	}
	template<bool X_Major = false, typename IntT = int>
	inline static int64_t gridIdx(const Eigen::Vector2<IntT>& loc, const Eigen::Vector2<IntT>& size) {
		return gridIdx<X_Major, IntT>(loc.x(), loc.y(), size);
	}

	/** Get the 2d location corresponding to a raw buffer idx for the provded grid size (templated on major-order) */
	template<bool X_Major = false, typename IntT = int>
	inline static Eigen::Vector2<IntT> gridLoc(const size_t idx, const Eigen::Vector2<IntT>& size) {
		if constexpr(X_Major) {
			return Eigen::Vector2<IntT>{	// x-major = "contiguous blocks along [parallel to] y-axis" --> x = idx / ymax, y = idx % ymax
				static_cast<IntT>(idx / size.y()),
				static_cast<IntT>(idx % size.y())
			};
		} else {
			return Eigen::Vector2<IntT>{	// y-major = "contiguous blocks along [parallel to] x-axis" --> x = idx % xmax, y = idx / xmax
				static_cast<IntT>(idx % size.x()),
				static_cast<IntT>(idx / size.x())
			};
		}
	}

	/** Copy a 2d windows of elemens - expects element types to be POD (templated on major-order) */
	template<typename T, bool X_Major = false, typename IntT = int, size_t T_Bytes = sizeof(T)>
	static void memcpyWindow(
		T* dest, const T* src,
		const Eigen::Vector2<IntT>& dest_size, const Eigen::Vector2<IntT>& src_size,
		const Eigen::Vector2<IntT>& diff
	) {
		if constexpr(X_Major) {
			for(int64_t _x = 0; _x < src_size.x(); _x++) {	// iterate through source "rows" of contiguous memory (along y -- for each x)
				memcpy(
					dest + ((_x + diff.x()) * dest_size.y() + diff.y()),
					src + (_x * src_size.y()),
					src_size.y() * T_Bytes
				);
			}
		} else {
			for(int64_t _y = 0; _y < src_size.y(); _y++) {	// iterate through source "rows" of contiguous memory (along x -- for each y)
				memcpy(
					dest + ((_y + diff.y()) * dest_size.x() + diff.x()),
					src + (_y * src_size.x()),
					src_size.x() * T_Bytes
				);
			}
		}
	}

};


namespace v2 {

/** A container for nodes. Size, resolution, origin, etc. are stored externally and passed in to all helper functions (below) */
template<
	typename mapsize_t = size_t,
	typename fweight_t = float>
struct NodeGraph {
	static_assert(std::is_floating_point<fweight_t>::value, "");

	using MSize_T = mapsize_t;
	using FWeight_T = fweight_t;
	using This_T = NodeGraph<mapsize_t, fweight_t>;

	struct Node {
		mapsize_t	// changed to 1D indexes
			self_idx,
			parent_idx;
		fweight_t
			g, h;
	};

	struct NodeCmp{
		bool operator()(const Node& a, const Node& b) { return a.g + a.h > b.g + b.h; }
		bool operator()(const Node* a, const Node* b) { return a->g + a->h > b->g + b->h; }
	};

	static constexpr double
		SQRT_2 = 1.41421356;
	static constexpr std::tuple<int8_t, int8_t, fweight_t> MOVES[] = {
		{0 , 1 , 1.f},
		{-1, 0 , 1.f},
		{0 , -1, 1.f},
		{1 , 0 , 1.f},
		{1 , 1 , SQRT_2},
		{-1, 1 , SQRT_2},
		{-1, -1, SQRT_2},
		{1 , -1, SQRT_2}
	};
	static constexpr size_t
		NUM_MOVES = sizeof(MOVES) / sizeof(MOVES[0]);

	std::vector<Node> map;


// static workers

	static inline void clearNodes(This_T& graph) {
		memset(graph.map.data(), 0x00, graph.map.size() * sizeof(typename decltype(graph)::Node));
	}

	template<typename IntT = int, bool X_Major = false>
	fweight_t get_linear_cost(
		const uint8_t* weights,
		const Eigen::Vector2<IntT>& wsize,
		const Eigen::Vector2<IntT>& a,
		const Eigen::Vector2<IntT>& b
	) {
		fweight_t sum = static_cast<fweight_t>(0);

		const int64_t
			a_idx = GridUtils::gridIdx<X_Major, IntT>(a, wsize),
			b_idx = GridUtils::gridIdx<X_Major, IntT>(b, wsize),
			area = wsize.x() * wsize.y();

		if(area == 0 || a_idx >= area || b_idx >= area || a_idx == b_idx) return sum;

		if(a.x() == b.x()) {
			sum += (static_cast<fweight_t>(weights[a_idx]) + weights[b_idx]) / 2;
			for(mapsize_t y = std::min(a.y(), b.y()) + 1; y < std::max(a.y(), b.y()); y++) {
				sum += static_cast<fweight_t>( weights[GridUtils::gridIdx<X_Major, IntT>(a.x(), y, wsize)] );
			}
			return sum;
		}

		if(a.y() == b.y()) {
			sum += (static_cast<fweight_t>(weights[a_idx]) + weights[b_idx]) / 2;
			for(mapsize_t x = std::min(a.x(), b.x()) + 1; x < std::max(a.x(), b.x()); x++) {
				sum += static_cast<fweight_t>( weights[GridUtils::gridIdx<X_Major, IntT>(x, a.y(), wsize)] );
			}
			return sum;
		}

		const IntT
			diff_x = b.x() - a.x(),
			diff_y = b.y() - a.y();
		const int64_t
			sgn = diff_x * diff_y > 0 ? 1 : -1;

		if(std::abs(diff_x) == std::abs(diff_y)) {
			sum += (static_cast<fweight_t>(weights[a_idx]) + weights[b_idx]) / 2;
			const Eigen::Vector2<IntT>& _start = a.x() < b.x() ? a : b;
			for(mapsize_t i = 1; i < std::abs(diff_x); i++) {
				sum += static_cast<fweight_t>( weights[GridUtils::gridIdx<X_Major, IntT>(_start.x() + i, _start.y() + sgn * i, wsize)] );
			}
			return sum * SQRT_2;
		}

		const float _m = static_cast<float>(diff_y) / diff_x;

		if(_m < 1.0) {

		}

	}

	template<typename IntT = int, bool X_Major = false>
	std::vector<mapsize_t> navigate(
		This_T& graph,
		const uint8_t* weights,
		const Eigen::Vector2<IntT>& wsize,

	) {

	}

};


};



// template<bool X_Major = false>
// class NavMap {
// public:
// 	using mapsize_t = uint16_t;
// 	using fast_mapsize_t = uint_fast16_t;
// 	using weight_t = uint8_t;
// 	using fweight_t = float;

// 	using int_t = int;
// 	using float_t = float;

// 	using MapSize = Eigen::Vector2<mapsize_t>;
// 	using Vec2i = Eigen::Vector2<int_t>;
// 	using Vec2f = Eigen::Vector2<float_t>;

// 	struct Node {
// 		mapsize_t
// 			self_x,
// 			self_y,
// 			parent_x,
// 			parent_y;
// 		fweight_t
// 			g, h;
// 	};
	
// 	struct NodeCmp{
// 		bool operator()(const Node& a, const Node& b) {}	// TODO
// 		bool operator()(const Node* a, const Node* b) {}
// 	};

// 	struct NeighborsMove {
// 		int8_t dx, dy;
// 		fweight_t weight_scale;
// 	};

// 	using point_t = MapSize;
// 	using path_t = std::vector<point_t>;
// 	using queue_t = std::priority_queue<Node*, std::vector<Node*>, NodeCmp>;

// 	static constexpr weight_t
// 		DEFAULT_MAX_WEIGHT = 255,
// 		DEFAULT_MIN_WEIGHT = 1;
// 	static constexpr fweight_t
// 		SQRT_2 = 1.41421356;
// 	static constexpr NeighborsMove MOVES[] = {
// 		{0 , 1 , 1.f},
// 		{-1, 0 , 1.f},
// 		{0 , -1, 1.f},
// 		{1 , 0 , 1.f},
// 		{1 , 1 , SQRT_2},
// 		{-1, 1 , SQRT_2},
// 		{-1, -1, SQRT_2},
// 		{1 , -1, SQRT_2}
// 	};
// 	static constexpr size_t
// 		NUM_MOVES = sizeof(MOVES) / sizeof(MOVES[0]);

// public:
// 	NavMap() = default;


// 	void updateAndSpread(
// 		const uint8_t* data,
// 		mapsize_t x_dim,
// 		mapsize_t y_dim,
// 		float_t origin_x,
// 		float_t origin_y,
// 		float_t cell_res,
// 		mapsize_t spread_radius);

// 	path_t makePath(
// 		mapsize_t src_x,
// 		mapsize_t src_y,
// 		mapsize_t dest_x,
// 		mapsize_t dest_y);


// protected:
// 	Vec2f map_origin{};
// 	MapSize map_size{};
// 	float_t
// 		cell_res{ 1. },
// 		turn_cost{ 1. };

// 	Node* nodes{ nullptr };


// };
