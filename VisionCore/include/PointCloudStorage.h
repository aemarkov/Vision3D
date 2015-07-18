#ifndef __POINTCLOUDSTORAGE_H__
#define __POINTCLOUDSTORAGE_H__

/*///////////////////////////////////////////////////////////////////////////////////////
//                               PointCloudStorage                                     //
//-------------------------------------------------------------------------------------//
// Этот класс служит для хранения облака точек.                                        //
// Он сохраняет трехмерные информации о точках, а так же информацию о цвете точки      //
// из исходных изображений.                                                            //
//                                                                                     //
// Данный класс имеет методы для сохранения облака точек в файлы, которые распознаются //
// внешними программа отображения 3d моделей.                                          //
// Список поддерживаемых форматов:                                                     //
//	1. obj                                                                             //
/*///////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>

//Хранит информацию о положении и цвете точки
struct ColoredPoint3d
{
	float X;
	float Y;
	float Z;
	cv::Scalar Color;

	ColoredPoint3d(float x, float y, float z, cv::Scalar color)
	{
		X = x;
		Y = y;
		Z = z;
		Color = color;
	}
};

class PointCloudStorage
{
public:
	PointCloudStorage(){}

	/* Создает облако точек на основе матриц c 3хмерными
	 * координатами точек и данными о цвете
	 * param[in] cloud - облако точек, результат reprojectImageTo3d
	 * param[in] colors - информация о цвете точек
	 */
	PointCloudStorage(cv::Mat cloud, cv::Mat colors);

	/* Создает облако точек на основе матриц c 3хмерными
	* координатами точек
	* param[in] cloud - облако точек, результат reprojectImageTo3d
	* param[in] colors - информация о цвете точек
	*/
	PointCloudStorage(cv::Mat cloud);

	//Сохраняет все в файл obj
	void SaveToObj(const char* filename) const;

private:
	std::vector<ColoredPoint3d> _points;

	//Создает обоако точек на основе координат и цвета
	//Поддерживает отсутсвтие цвета
	//Конструкторы - обертка над этим методом для большей красоты
	void _getPointsFromMat(cv::Mat cloud, cv::Mat colors=cv::Mat(0,0,CV_8UC3));
	
};

#endif