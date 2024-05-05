#pragma once

#include <cstddef>	 //std::size_t
#include <stdexcept> // std::out_of_range
#include <cstring>	 //memcpy

#define UNUSED(x) (void)x;

template <typename T>
class ArrayView
{
public:
	ArrayView(T *arr, size_t len) : arr(arr) 
#ifdef _DEBUG
, length(len)
#endif
 	{
#ifndef _DEBUG
	UNUSED(len);
#endif


	}

	T &operator[](size_t index)
	{
#ifdef _DEBUG
		if (index >= length)
		{
			throw std::out_of_range("Array View Accessed Out Of Bounds");
		}
#endif
		return arr[index];
	}

	const T &operator[](size_t index) const
	{
#ifdef _DEBUG
		if (index >= length)
		{
			throw std::out_of_range("Array View Accessed Out Of Bounds");
		}
#endif
		return arr[index];
	}

private:
	T *arr;
#ifdef _DEBUG
	const size_t length;
#endif

};

template <typename T>
class BlockArray2DRT
{
public:
	BlockArray2DRT(size_t dim_1_in, size_t dim_2_in) : arr(new T[dim_1_in * dim_2_in]), dim_1(dim_1_in), dim_2(dim_2_in){};

	BlockArray2DRT(BlockArray2DRT &other) : arr(new T[other.dim_1, other.dim_2]), dim_1(other.dim_1), dim_2(other.dim_2)
	{
		for (size_t i = 0; i < dim_1 * dim_2; i++)
		{
			arr[i] = std::copy(other.arr[i]);
		}
	};

	// Copy assignment
	// Do deep copy of a.m_ptr to m_ptr
	BlockArray2DRT& operator=(const BlockArray2DRT& other)
	{
		// Self-assignment detection
		if (&other == this)
			return *this;

		// Release any resource we're holding
		delete arr;

		// Copy the resource
		arr = new T[other.dim_1, other.dim_2];
		dim_1 = other.dim_1;
		dim_2 = other.dim_2;

		for (size_t i = 0; i < dim_1 * dim_2; i++)
		{
			arr[i] = std::copy(other.arr[i]);
		}

		return *this;
	}

	// Move Constructor
	// Transfer ownership of a.m_ptr to m_ptr
	BlockArray2DRT(BlockArray2DRT &&other) noexcept: arr(other.arr), dim_1(other.dim_1), dim_2(other.dim_2) { 
		// Remove ownership from other
		other.arr = nullptr;
		other.dim_1 = 0;
		other.dim_2 = 0;
	};

	// Move assignment
	// Transfer ownership of a.m_ptr to m_ptr
	BlockArray2DRT& operator=(BlockArray2DRT&& other) noexcept { 
		// Self-assignment detection
		if (&other == this)
			return *this;

		// Release any resource we're holding
		delete arr;

		// Transfer ownership of other.arr to self
		arr = other.arr;
		dim_1 = other.dim_1;
		dim_2 = other.dim_2;

		// Remove ownership from other
		other.arr = nullptr; 
		other.dim_1 = 0;
		other.dim_2 = 0;

		return *this;
	}

	ArrayView<T> operator[](size_t index)
	{
#ifdef _DEBUG
		if (index >= dim_1)
		{
			throw std::out_of_range("Array View Accessed Out Of Bounds");
		}
#endif
		return ArrayView<T>(&arr[index * dim_2], dim_2);
	}

	const ArrayView<T> operator[](size_t index) const
	{
#ifdef _DEBUG
		if (index >= dim_1)
		{
			throw std::out_of_range("Array View Accessed Out Of Bounds");
		}
#endif
		return ArrayView<T>(&arr[index * dim_2], dim_2);
	}

	~BlockArray2DRT()
	{
		delete[] arr;
	}

private:
	T * arr;
	size_t dim_1, dim_2;
};

