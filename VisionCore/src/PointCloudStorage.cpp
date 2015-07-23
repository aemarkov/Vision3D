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
	for (int i = 0; i < _children.height; i++)
	{
		for (int j = 0; j < _children.width; j++)
		{
			delete _children.childrenArray[i][j];
		}
		delete[] _children.childrenArray[i];
	}
	delete[] _children.childrenArray;
}

bool isDistLess(BaseObject3D& o1, BaseObject3D& o2, float maxDist)
{
	float dist = sqrt(pow(o1.GetCoord()[0] - o2.GetCoord()[0], 2) + pow(o1.GetCoord()[1] - o2.GetCoord()[1], 2) + pow(o1.GetCoord()[2] - o2.GetCoord()[2], 2));
	return dist <= maxDist;
}

/*
Object3D* obj = new Object3D(this);
obj->AddChild(child);
tempArr[i][j] = obj;

*/

//Разбиение точек на объекты
void PointCloudStorage::SeparateObjects(float maxDist)
{
	cv::Mat img = cv::Mat_<cv::Scalar>(_children.height, _children.width);
	cv::Scalar lastColor(0, 0, 0);

	//1. Выделяем новый массив под указатели на потомков
	BaseObject3D*** tempArr = new BaseObject3D**[_children.height];
	for (int i = 0; i < _children.height; i++)
	{
		tempArr[i] = new BaseObject3D*[_children.width];
		for (int j = 0; j < _children.width; j++)
			tempArr[i][j] = NULL;
	}

	//2. Создаем матрицу размером с childrenArray, показывающую, посетили ли
	//   мы уже ячейку
	bool** visited = new bool*[_children.height];
	for (int i = 0; i < _children.height; i++)
	{
		visited[i] = new bool[_children.width];
		for (int j = 0; j < _children.width; j++)
		{
			visited[i][j] = false;
		}
	}

	//3. Выполняем разбивку
	for (int i = 0; i < _children.height; i++)
	{
		for (int j = 0; j < _children.width; j++)
		{
			//Берем очередного потомка
			BaseObject3D* child = _children.childrenArray[i][j];
			if ((child != NULL) && !visited[i][j])
			{
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

				//Если он существует, то 
				Object3D* newParent = new Object3D(this);
				_childrenList.push_back(newParent);
				_findNearbyChildren(i, j,_children, tempArr, visited, maxDist, newParent, img, lastColor);
			}
		}
	

	}

	//Удаляем стврый childrenArray
	/*for (int i = 0; i < _children.height; i++)
		delete[] _children.childrenArray[i];
	delete[] _children.childrenArray;

	//Удаляем visited
	for (int i = 0; i < _children.height; i++)
		delete[] visited[i];
	delete[] visited;*/

	_children.childrenArray = tempArr;
	cv::imshow("wtf", img);
}

void PointCloudStorage::_findNearbyChildren(int row, int col, Children children, BaseObject3D*** tempArr, bool** visited, float MaxDist, Object3D* newParent, cv::Mat img, cv::Scalar color)
{
	BaseObject3D* current = children.childrenArray[row][col];
	BaseObject3D* up = NULL;
	BaseObject3D* down = NULL;
	BaseObject3D* left = NULL;
	BaseObject3D* right = NULL;
	bool vUp = false;
	bool vDown = false;
	bool vLeft = false;
	bool vRight = false;

	//Добавляем этот объект в потомки нового объекта
	tempArr[row][col] = newParent;
	newParent->AddChild(current);
	visited[row][col] = true;

	img.at<cv::Scalar>(row, col) = color;
	//cv::imshow("wtf", img);
	//cv::waitKey(1);

	//Определяем объекты с 4х сторон от текущего
	if (row > 0)
	{
		up = children.childrenArray[row - 1][col];
		vUp = visited[row - 1][col];
	}
	if (row < children.height - 1)
	{
		down = children.childrenArray[row + 1][col];
		vDown = visited[row + 1][col];
	}
	if (col > 0)
	{
		left = children.childrenArray[row][col - 1];
		vLeft = visited[row][col - 1];
	}
	if (col < children.width - 1)
	{
		right = children.childrenArray[row][col + 1];
		vRight = visited[row][col + 1];
	}

	if (up && !vUp && isDistLess(*up, *current, MaxDist))
		_findNearbyChildren(row - 1, col, children, tempArr, visited, MaxDist, newParent, img, color);

	if (down && !vDown && isDistLess(*down, *current, MaxDist))
		_findNearbyChildren(row + 1, col, children, tempArr, visited, MaxDist, newParent, img, color);

	if (left && !vLeft && isDistLess(*left, *current, MaxDist))
		_findNearbyChildren(row, col - 1, children, tempArr, visited, MaxDist, newParent, img, color);

	if (right && !vRight && isDistLess(*right, *current, MaxDist))
		_findNearbyChildren(row, col + 1, children, tempArr, visited, MaxDist, newParent, img, color);

}

void _saveFromObject(Object3D* object, std::ofstream& stream)
{
	for (int i = 0; i < object->ChildrenCount(); i++)
	{
		BaseObject3D* curObject = object->GetChild(i);
		if (curObject != NULL)
		{
			if (curObject->GetType() == BaseObject3D::TYPE_OBJECT)
				_saveFromObject(dynamic_cast<Object3D*>(curObject), stream);
			else
			{
				cv::Vec3f point = curObject->GetCoord();
				stream << "v " << point[0] << point[1] << point[2];
			}
		}
	}
}

//Сохраняет все в файл obj
void PointCloudStorage::SaveToObj(const char* filename, int minCount) const
{
	std::ofstream stream(filename);
	for (int i = 0; i < _childrenList.size(); i++)
	{
		BaseObject3D* obj = _childrenList[i];
		if (obj != NULL)
		{
			if (obj->GetType() == Object3DType::TYPE_POINT)
			{
				cv::Vec3f point = obj->GetCoord();
				stream << "v " << point[0] << point[1] << point[2];
			}
			else if (dynamic_cast<Object3D*>(obj)->ChildrenCount()>minCount)
				_saveFromObject((Object3D*)obj, stream);
		}
	}
	stream.close();
}


//Возвращает тип
PointCloudStorage::Object3DType PointCloudStorage::GetType()
{
	return Object3DType::TYPE_SCENE;
}