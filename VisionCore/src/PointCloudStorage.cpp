#include "PointCloudStorage.h"

PointCloudStorage::PointCloudStorage()
{
	_children.childrenArray = NULL;
}

/* Создает облако точек на основе матриц c 3хмерными
* координатами точек
* param[in] cloud - облако точек, результат reprojectImageTo3d
*/
PointCloudStorage::PointCloudStorage(cv::Mat cloud)
{
	_children.childrenArray = new BaseObject3D**[cloud.rows];
	_children.width = cloud.cols;
	_children.height = cloud.rows;

	for (int i = 0; i < cloud.rows; i++)
	{
		_children.childrenArray[i] = new BaseObject3D*[cloud.cols];
		for (int j = 0; j < cloud.cols; j++)
		{
			cv::Vec3f point = cloud.at<cv::Vec3f>(i, j);
			if (!isinf(point[0]) && !isinf(point[1]) && !isinf(point[2]))
				_children.childrenArray[i][j] = new BaseObject3D(cloud.at<cv::Vec3f>(i, j), this);
			else
				_children.childrenArray[i][j] = NULL;
		}
	}

}

//Удаляет объект
PointCloudStorage::~PointCloudStorage()
{
	/*if (_children.childrenArray == NULL)
		return;

	for (int i = 0; i < _children.width; i++)
	{
		for (int j = 0; j < _children.height; j++)
		{
			delete _children.childrenArray[i][j];
		}
		delete[] _children.childrenArray[i];
	}
	delete[] _children.childrenArray;*/
}

bool isDistLess(BaseObject3D& o1, BaseObject3D& o2, float maxDist)
{
	float dist = sqrt(pow(o1.GetCoord()[0] - o2.GetCoord()[0], 2) + pow(o1.GetCoord()[1] - o2.GetCoord()[1], 2) + pow(o1.GetCoord()[2] - o2.GetCoord()[2], 2));
	return dist <= maxDist;
}

//Разбиение точек на объекты
void PointCloudStorage::SeparateObjects(float maxDist)
{
	cv::Mat img = cv::Mat_<cv::Scalar>(_children.height, _children.width);
	cv::Scalar lastColor(0, 0, 0);

	//cv::Scalar colors = { cv::Scalar(1, 0, 0), cv::Scalar() }

	//1. Выделяем новый массив под указатели на потомков
	BaseObject3D*** tempArr = new BaseObject3D**[_children.height];
	for (int i = 0; i < _children.height; i++)
		tempArr[i] = new BaseObject3D*[_children.width];

	for (int i = 0; i < _children.height; i++)
	{
		for (int j = 0; j < _children.width; j++)
		{
			BaseObject3D* child = _children.childrenArray[i][j];
			BaseObject3D* childLeft = NULL;
			BaseObject3D* childUp = NULL;
			if (child != NULL)
			{
				if (j > 0)
					childLeft = _children.childrenArray[i][j - 1];
				if (i > 0)
					childUp = _children.childrenArray[i - 1][j];

				bool isLeft = (childLeft != NULL) && isDistLess(*childLeft, *child, maxDist);
				bool isUp = (childUp != NULL) && isDistLess(*childUp, *child, maxDist);

				if (isLeft && !isUp)
				{
					child->Parent = childLeft->Parent;
					tempArr[i][j] = child->Parent;
					//img.at < cv::Scalar>(i, j) = lastColor;
				}
				else if (isUp && !isLeft)
				{
					child->Parent = childUp->Parent;
					tempArr[i][j] = child->Parent;
					//img.at < cv::Scalar>(i, j) = lastColor;
				}
				else if (isLeft && isUp)
				{
					//Переназначаем все объекты левой группы верхней
					img.at < cv::Scalar>(i, j) = cv::Scalar(1,1,1);
				}
				else
				{
					Object3D* obj = new Object3D(this);
					obj->AddChild(child);
					tempArr[i][j] = obj;

					lastColor[0] += 0.5;
					if (lastColor[0] > 1)
					{
						lastColor[1] += 0.5;
						lastColor[0] = 0;
					}
					if (lastColor[1] > 1)
					{
						lastColor[2] += 0.5;
						lastColor[1] = 0;
					}
					if (lastColor[2] > 1)
						lastColor[2] = 0;
					
					//img.at < cv::Scalar>(i, j) = lastColor;
					//std::cout << lastColor[0] << " " << lastColor[1] << " " << lastColor[2] << "\n";
				}

				

			}
		}
	

	}

	cv::imshow("wtf", img);
	cv::waitKey(1);
	//cv::destroyWindow("wtf");
}

//Сохраняет все в файл obj
void PointCloudStorage::SaveToObj(const char* filename) const
{
	std::ofstream stream(filename);
	for (int i = 0; i < _children.width; i++)
	{
		for (int j = 0; j < _children.height; j++)
		{
			BaseObject3D* obj = _children.childrenArray[i][j];
			if (obj->GetType() == Object3DType::TYPE_POINT)
			{
				cv::Vec3f point = obj->GetCoord();
				stream << "v " << point[0] << point[1] << point[2];
			}
		}
	}
	stream.close();
}


//Возвращает тип
PointCloudStorage::Object3DType PointCloudStorage::GetType()
{
	return Object3DType::TYPE_SCENE;
}