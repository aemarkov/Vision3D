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

#include "BaseObject3D.h"
#include "Object3D.h"

class PointCloudStorage : BaseObject3D
{
public:

	PointCloudStorage();

	/* Создает облако точек на основе матриц c 3хмерными
	* координатами точек
	* param[in] cloud - облако точек, результат reprojectImageTo3d
	*/
	PointCloudStorage(cv::Mat cloud);
	~PointCloudStorage();

	//Сохраняет все в файл obj
	void SaveToObj(const char* filename) const;

	//Возвращает тип
	Object3DType GetType();

	void SeparateObjects(float maxDist);

private:
	
	//Массив потомков
	struct Children
	{
		BaseObject3D*** childrenArray;
		int width, height;
	} _children;
};

#endif