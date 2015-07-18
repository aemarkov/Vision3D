#include "PointCloudStorage.h"

/* Создает облако точек на основе матриц c 3хмерными
* координатами точек и данными о цвете
* param[in] cloud - облако точек, результат reprojectImageTo3d
* param[in] colors - информация о цвете точек
*/
PointCloudStorage::PointCloudStorage(cv::Mat cloud, cv::Mat colors)
{
	_getPointsFromMat(cloud, colors);
}

/* Создает облако точек на основе матриц c 3хмерными
* координатами точек
* param[in] cloud - облако точек, результат reprojectImageTo3d
* param[in] colors - информация о цвете точек
*/
PointCloudStorage::PointCloudStorage(cv::Mat cloud)
{
	_getPointsFromMat(cloud);
}

//Создает обоако точек на основе координат и цвета
//Поддерживает отсутсвтие цвета
void PointCloudStorage::_getPointsFromMat(cv::Mat cloud, cv::Mat colors)
{
	bool isColorExists = (cloud.rows == colors.rows) && (cloud.cols == colors.cols);
	cv::Scalar color(0, 0, 0);

	for (int x = 0; x < cloud.cols; x++)
	{
		for (int y = 0; y < cloud.rows; y++)
		{
			if (isColorExists)
				color = colors.at<cv::Scalar>(y, x);

			auto point = cloud.at<cv::Vec3f>(y, x);
			if (isinf(point[0]) || isinf(point[1]) || isinf(point[2]))continue;

			_points.push_back(ColoredPoint3d(point[0], point[1], point[2], color));
		}
	}
}

//Сохраняет все в файл obj
void PointCloudStorage::SaveToObj(const char* filename) const
{
	std::ofstream stream(filename);
	for (int i = 0; i < _points.size(); i++)
	{
		auto point = _points[i];
		stream << "v " << point.X <<" "<< point.Y << " " << point.Z << "\n";
	}
	stream.close();
}