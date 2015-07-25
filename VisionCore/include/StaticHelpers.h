#ifndef __STATICHELPERS_H__
#define __STATICHELPERS_H__


/*///////////////////////////////////////////////////////////////////////////////////////
//                                StaticHelpers                                        //
//-------------------------------------------------------------------------------------//
// Этот класс предоставляет ряд статичных функций, которые облегачают определенные     //
// операции с OpenCV и Vision3D                                                        //
/*///////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <opencv2/opencv.hpp>

class StaticHelpers
{
public:

	//Печать матрицы в стандартный поток вывода
	//Вроде шаблоны должны быть только в .h
	template <typename T>
	static void printMatrix(cv::Mat matrix)
	{
		printMatrixStream<T>(matrix, cout);
	}

	//Печатать матрицы в заданный поток вывода
	template <typename T>
	static void printMatrixStream(cv::Mat matrix, std::ostream& stream)
	{
		for (int i = 0; i < matrix.rows; i++)
		{
			for (int j = 0; j < matrix.cols; j++)
				stream << matrix.at<T>(i, j) << "\t";
			stream << "\n";
		}
	}

	static cv::Mat average_disparity(std::vector<cv::Mat>& disparities)
	{
		cv::Mat example_mat = disparities[0];
		cv::Mat average_disparity(example_mat);			//Среднее арифметическое всех элементов disparities

		int disparities_count = disparities.size(); //Число всех элементов disparities

		//Размеры каждой из матриц disparities
		int cols = example_mat.cols;
		int rows = example_mat.rows;

		//Среднее значение элемента матрицы с координатами x, y
		float average_element_value = 0.0;

		//Усреднение по каждой точке каждого элемента disparity
		for (int x = 0; x < cols; x++)
		{
			for (int y = 0; y < rows; y++)
			{
				//Среднее арифметическое для всех элементов с координатами x, y
				for (int i = 0; i < disparities_count; i++)
					average_element_value += disparities[i].at<float>(x, y);

				average_disparity.at<float>(x, y) = average_element_value / disparities_count;
			}
		}

		return average_disparity;
	}
};

#endif