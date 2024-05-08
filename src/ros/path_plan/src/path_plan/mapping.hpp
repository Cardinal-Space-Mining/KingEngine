#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>
#include <queue>
#include <functional>
#include <algorithm>
// #include <iostream>

#include <Eigen/Core>

// copied from Sick-Perception (grid.hpp)
/** Generic grid helpers */
namespace GridUtils
{

	/** Align a point to a box grid of the given resolution and offset origin. Result may be negative if lower than current offset. */
	template <typename IntT = int, typename FloatT = float>
	inline static Eigen::Vector2<IntT> gridAlign(FloatT x, FloatT y, const Eigen::Vector2<FloatT> &off, FloatT res)
	{
		return Eigen::Vector2<IntT>{
			static_cast<IntT>(std::floor((x - off.x()) / res)), // always floor since grid cells are indexed by their "bottom left" corner's raw position
			static_cast<IntT>(std::floor((y - off.y()) / res))};
	}
	template <typename IntT = int, typename FloatT = float>
	inline static Eigen::Vector2<IntT> gridAlign(const Eigen::Vector4<FloatT> &pt, const Eigen::Vector2<FloatT> &off, FloatT res)
	{
		return gridAlign<IntT, FloatT>(pt.x(), pt.y(), off, res);
	}

	/** Get a raw buffer idx from a 2d index and buffer size (templated on major-order) */
	template <bool X_Major = false, typename IntT = int>
	inline static int64_t gridIdx(const IntT x, const IntT y, const Eigen::Vector2<IntT> &size)
	{
		if constexpr (X_Major)
		{
			return static_cast<int64_t>(x) * size.y() + y; // x-major = "contiguous blocks along [parallel to] y-axis" --> idx = (x * ymax) + y
		}
		else
		{
			return static_cast<int64_t>(y) * size.x() + x; // y-major = "contiguous blocks along [parallel to] x-axis" --> idx = (y * xmax) + x
		}
	}
	template <bool X_Major = false, typename IntT = int>
	inline static int64_t gridIdx(const Eigen::Vector2<IntT> &loc, const Eigen::Vector2<IntT> &size)
	{
		return gridIdx<X_Major, IntT>(loc.x(), loc.y(), size);
	}

	/** Get the 2d location corresponding to a raw buffer idx for the provded grid size (templated on major-order) */
	template <bool X_Major = false, typename IntT = int>
	inline static Eigen::Vector2<IntT> gridLoc(const size_t idx, const Eigen::Vector2<IntT> &size)
	{
		if constexpr (X_Major)
		{
			return Eigen::Vector2<IntT>{// x-major = "contiguous blocks along [parallel to] y-axis" --> x = idx / ymax, y = idx % ymax
										static_cast<IntT>(idx / size.y()),
										static_cast<IntT>(idx % size.y())};
		}
		else
		{
			return Eigen::Vector2<IntT>{// y-major = "contiguous blocks along [parallel to] x-axis" --> x = idx % xmax, y = idx / xmax
										static_cast<IntT>(idx % size.x()),
										static_cast<IntT>(idx / size.x())};
		}
	}

	/** Check if v is GEQ than min and LESS than max */
	template <typename IntT = int>
	inline static bool inRange(const Eigen::Vector2<IntT> &v, const Eigen::Vector2<IntT> &min, const Eigen::Vector2<IntT> &max)
	{
		return (
			min.x() <= v.x() && v.x() < max.x() &&
			min.y() <= v.y() && v.y() < max.y());
	}

};

/** constexpr conditional value (v1 = true val, v2 = false val) */
template <bool _test, typename T, T tV, T fV>
struct conditional_literal
{
	static constexpr T value = tV;
};
template <typename T, T tV, T fV>
struct conditional_literal<false, T, tV, fV>
{
	static constexpr T value = fV;
};

/** A container for nodes. Size, resolution, origin, etc. are stored externally and passed in to all helper functions (below) */
template <
	typename mapsize_t = int64_t,
	typename fweight_t = float>
class NavMap
{
public:
	static_assert(std::is_integral<mapsize_t>::value, "");
	static_assert(std::is_floating_point<fweight_t>::value, "");

	using MSize_T = mapsize_t;
	using FWeight_T = fweight_t;
	using This_T = NavMap<mapsize_t, fweight_t>;

	using Vec2m = Eigen::Vector2<mapsize_t>; // represents a location on the map
	using NodeMove = std::tuple<Vec2m, fweight_t>;
	// using Path = Eigen::Matrix2X<mapsize_t>;
	using Path = std::vector<Vec2m>;

	struct Node
	{
		mapsize_t // changed to 1D indexes to be more compact
			self_idx,
			parent_idx;
		fweight_t
			g,
			h;
	};

	struct NodeCmp
	{
		bool operator()(const Node &a, const Node &b) { return a.g + a.h > b.g + b.h; }
		bool operator()(const Node *a, const Node *b) { return a->g + a->h > b->g + b->h; }
	};

	static constexpr double
		SQRT_2 = 1.41421356;
	inline static const NodeMove MOVES[] = {
		{Vec2m{0, 1}, 1.f},
		{Vec2m{-1, 0}, 1.f},
		{Vec2m{0, -1}, 1.f},
		{Vec2m{1, 0}, 1.f},
		{Vec2m{1, 1}, SQRT_2},
		{Vec2m{-1, 1}, SQRT_2},
		{Vec2m{-1, -1}, SQRT_2},
		{Vec2m{1, -1}, SQRT_2}};
	static constexpr size_t
		NUM_MOVES = sizeof(MOVES) / sizeof(MOVES[0]);
	static constexpr mapsize_t
		INVALID_IDX = conditional_literal<
			std::is_signed<mapsize_t>::value,
			mapsize_t,
			static_cast<mapsize_t>(-1),
			std::numeric_limits<mapsize_t>::max()>::value; // use -1 for signed types (more robust), max value for unsigned

public:
	/** Call resize() to setup the nodes */
	NavMap() = default;
	~NavMap() = default;

	/** Resizes the vector of nodes and assigns member indices for each newly added instance */
	This_T &resize(size_t len)
	{ // enforces nodes always having their index self-assigned since this is the only way the internal array can be resized
		size_t i = this->size();
		this->map.resize(len);

		for (; i < len; i++)
			this->map[i].self_idx = static_cast<mapsize_t>(i);

		return *this;
	}

	/** Erases all data. */
	inline This_T &clear()
	{
		this->map.clear();
		return *this;
	}

	/** Get the number of nodes currently stored. */
	inline size_t size() const
	{
		return this->map.size();
	}

	template <typename WeightT = uint8_t, bool X_Major = false>
	This_T::Path navigate(
		const WeightT *weights,
		mapsize_t wsize_x, mapsize_t wsize_y,
		double worigin_x, double worigin_y,
		double cell_resolution,
		double start_x, double start_y,
		double end_x, double end_y,
		WeightT turn_cost = static_cast<WeightT>(1))
	{
		const Eigen::Vector2d
			origin{worigin_x, worigin_y};
		const This_T::Vec2m
			wsize{wsize_x, wsize_y},
			start = GridUtils::gridAlign<mapsize_t, double>(start_x, start_y, origin, cell_resolution),
			end = GridUtils::gridAlign<mapsize_t, double>(end_x, end_y, origin, cell_resolution);

		return this->navigate<WeightT, X_Major, mapsize_t>(
			weights,
			wsize,
			start,
			end,
			turn_cost);
	}

	/** Navigate a provided weightmap, starting from and ending at the given cell locations */
	template <typename WeightT = uint8_t, bool X_Major = false, typename IntT = mapsize_t>
	This_T::Path navigate(
		const WeightT *weights,
		const Eigen::Vector2<IntT> &wsize,
		const Eigen::Vector2<IntT> &start,
		const Eigen::Vector2<IntT> &end,
		const WeightT turn_cost = static_cast<WeightT>(1))
	{
		const int64_t
			_area = static_cast<int64_t>(wsize.x()) * wsize.y(),
			_start_idx = GridUtils::gridIdx<X_Major, IntT>(start, wsize),
			_end_idx = GridUtils::gridIdx<X_Major, IntT>(end, wsize);
		static const Eigen::Vector2<IntT>
			_zero = Eigen::Vector2<IntT>::Zero();

		// std::cout << "nav init >> area: " << _area << ", start idx: " << _start_idx << ", end idx: " << _end_idx << std::endl;

		if (
			!GridUtils::inRange<IntT>(start, _zero, wsize) ||
			!GridUtils::inRange<IntT>(end, _zero, wsize) ||
			(_start_idx == _end_idx))
		{
			return This_T::Path{}; // empty path
		}

		this->resize(_area);
		for (size_t i = 0; i < this->map.size(); i++)
		{
			this->map[i].h = static_cast<fweight_t>(
				(end - GridUtils::gridLoc<X_Major, IntT>(i, wsize)).norm() // "length" of the difference
			);
			this->map[i].g = std::numeric_limits<fweight_t>::infinity();
			this->map[i].parent_idx = This_T::INVALID_IDX;
		}

		This_T::Node &_start_node = this->map[_start_idx];
		_start_node.g = static_cast<fweight_t>(0);

		std::vector<This_T::Node *> queue_backer;
		queue_backer.reserve(static_cast<size_t>(_area));

		std::priority_queue<This_T::Node *, std::vector<This_T::Node *>, This_T::NodeCmp> queue{This_T::NodeCmp{}, std::move(queue_backer)};
		queue.push(&_start_node);

		while (!queue.empty())
		{
			This_T::Node &_node = *queue.top();
			queue.pop();

			if (_node.self_idx == _end_idx)
			{
				// backtrace path
				This_T::Path path{};
				const This_T::Node *_itr_node = &_node;
				while (_itr_node->parent_idx != This_T::INVALID_IDX)
				{
					path.emplace_back(GridUtils::gridLoc<X_Major, IntT>(_itr_node->self_idx, wsize).template cast<mapsize_t>());
					_itr_node = &this->map[_itr_node->parent_idx];
				}
				// assert size
				path.emplace_back(GridUtils::gridLoc<X_Major, IntT>(_itr_node->self_idx, wsize).template cast<mapsize_t>());
				std::reverse(path.begin(), path.end());
				return path;
			}
			else
			{
				// make moves -- I didn't split out this method because we only have a single navigation method currently
				const Eigen::Vector2<IntT>
					_loc = GridUtils::gridLoc<X_Major, IntT>(_node.self_idx, wsize);

				for (size_t i = 0; i < This_T::NUM_MOVES; i++)
				{

					const Eigen::Vector2<IntT> _neighbor_loc = _loc + (std::get<0>(This_T::MOVES[i]).template cast<IntT>());
					if (!GridUtils::inRange<IntT>(_neighbor_loc, _zero, wsize))
						continue;

					const int64_t _idx = GridUtils::gridIdx<X_Major, IntT>(_neighbor_loc, wsize);
					This_T::Node &_neighbor = this->map[_idx];

					const fweight_t pivot_cost = turn_cost + _node.g + std::get<1>(This_T::MOVES[i]) * (weights[_node.self_idx] + weights[_neighbor.self_idx]) * 0.5;

					if (pivot_cost < _neighbor.g)
					{
						if (_node.parent_idx != This_T::INVALID_IDX)
						{
							This_T::Node &_parent = this->map[_node.parent_idx];
							const fweight_t direct_cost =
								_parent.g +
								this->get_linear_cost<WeightT, X_Major, IntT>(
									weights,
									wsize,
									_parent.self_idx,
									_neighbor.self_idx);
							if (direct_cost <= pivot_cost)
							{
								_neighbor.parent_idx = _parent.self_idx;
								_neighbor.g = direct_cost;
								queue.push(&_neighbor);
								continue;
							}
						}
						_neighbor.parent_idx = _node.self_idx;
						_neighbor.g = pivot_cost;
						queue.push(&_neighbor);
					}
				}

			}
		}

		return This_T::Path{}; // failure, but handle gracefully by returning an empty path
	}

protected:
	template <typename WeightT = uint8_t, bool X_Major = false, typename IntT = mapsize_t>
	fweight_t get_linear_cost(
		const WeightT *weights,
		const Eigen::Vector2<IntT> &wsize,
		const mapsize_t a_idx,
		const mapsize_t b_idx)
	{
		fweight_t sum = static_cast<fweight_t>(0);

		const int64_t _area = wsize.x() * wsize.y();
		const Eigen::Vector2<IntT>
			_a = GridUtils::gridLoc<X_Major, IntT>(a_idx, wsize),
			_b = GridUtils::gridLoc<X_Major, IntT>(b_idx, wsize),
			_diff = _b - _a,
			_min = _b.cwiseMin(_a),
			_max = _b.cwiseMax(_a);

		if (_area == 0 || a_idx >= _area || b_idx >= _area || a_idx == b_idx)
		{
			return sum;
		}

		if (_a.x() == _b.x())
		{
			sum += (static_cast<fweight_t>(weights[a_idx]) + weights[b_idx]) / 2;
			for (IntT y = _min.y() + 1; y < _max.y(); y++)
			{
				sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_a.x(), y, wsize)]);
			}
			return sum;
		}

		if (_a.y() == _b.y())
		{
			sum += (static_cast<fweight_t>(weights[a_idx]) + weights[b_idx]) / 2;
			for (IntT x = _min.x() + 1; x < _max.x(); x++)
			{
				sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(x, _a.y(), wsize)]);
			}
			return sum;
		}

		const int64_t sgn = _diff.x() * _diff.y() > 0 ? 1 : -1;
		const Eigen::Vector2<IntT> &x_start = _a.x() < _b.x() ? _a : _b;

		if (std::abs(_diff.x()) == std::abs(_diff.y()))
		{
			sum += (static_cast<fweight_t>(weights[a_idx]) + weights[b_idx]) / 2;
			for (mapsize_t i = 1; i < std::abs(_diff.x()); i++)
			{
				sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(x_start.x() + i, x_start.y() + sgn * i, wsize)]);
			}
			return sum * This_T::SQRT_2;
		}

		float _m = std::abs(static_cast<float>(_diff.y()) / _diff.x());

		if (_m < 1.0)
		{

			const Eigen::Vector2<IntT> &x_end = _a.x() > _b.x() ? _a : _b;
			Eigen::Vector2<IntT> _p = x_start;

			sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_p, wsize)]) * 0.5;

			float err = _m / 2.0;
			for (_p.x() += 1; _p.x() < x_end.x(); _p.x()++)
			{
				if (err + _m >= 0.5f)
				{
					const float t = (0.5f - err) / _m;
					sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_p, wsize)]) * t;
					_p.y() += sgn;
					err -= 1.f;
					sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_p, wsize)]) * (1.f - t);
				}
				else
				{
					sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_p, wsize)]);
				}
				err += _m;
			}

			_p.x() = x_end.x();
			sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_p, wsize)]) * 0.5;
		}
		else
		{

			_m = 1.f / _m;

			const Eigen::Vector2<IntT>
				&y_start = _a.y() < _b.y() ? _a : _b,
				&y_end = _a.y() > _b.y() ? _a : _b;
			Eigen::Vector2<IntT> _p = y_start;

			sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_p, wsize)]) * 0.5;

			float err = _m / 2.0;
			for (_p.y() += 1; _p.y() < y_end.y(); _p.y()++)
			{
				if (err + _m >= 0.5f)
				{
					const float t = (0.5f - err) / _m;
					sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_p, wsize)]) * t;
					_p.x() += sgn;
					err -= 1.f;
					sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_p, wsize)]) * (1.f - t);
				}
				else
				{
					sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_p, wsize)]);
				}
				err += _m;
			}

			_p.y() = y_end.y();
			sum += static_cast<fweight_t>(weights[GridUtils::gridIdx<X_Major, IntT>(_p, wsize)]) * 0.5;
		}

		return sum * std::sqrt(1 + _m * _m);
	}

protected:
	std::vector<Node> map;


};
