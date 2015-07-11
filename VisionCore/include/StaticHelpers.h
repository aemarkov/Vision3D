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
};

#endif