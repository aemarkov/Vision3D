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
			{
				_children.childrenArray[i][j] = new BaseObject3D(cloud.at<cv::Vec3f>(i, j), this);
				_childrenList.push_back(_children.childrenArray[i][j]);
			}
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
	_childrenList.clear();

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

	//Очередь индексов точек
	std::stack<Point> points;

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

				//Если он существует, то находим все близкие объекты и добавляем их
				//В новый объект-группу
				Object3D* newParent = new Object3D(this);
				_childrenList.push_back(newParent);
				points.push(Point(j, i));

				while (points.size() > 0)
				{
					//Берем последнюю точку
					Point currentPoint = points.top();
					points.pop();
					
					//Добавляем текущий объект в потомки новому объекту-группе
					BaseObject3D* currentObject = _children.childrenArray[currentPoint.Y][currentPoint.X];

					newParent->AddChild(currentObject);
					visited[currentPoint.Y][currentPoint.X] = true;
					
					//Добавляем в очередь всех близких соседей этого объекта
					for (int y = currentPoint.Y - 1; y <= currentPoint.Y + 1; y++)
					{
						for (int x = currentPoint.X - 1; x <= currentPoint.X + 1; x++)
						{
							if ((y>0) && (x > 0) && (y < _children.height - 1) && (x < _children.width - 1) && !visited[y][x])
							{
								//std::cout << y << " "<<x << " yes\n";
								BaseObject3D* object = _children.childrenArray[y][x];
								if ((object != NULL) && isDistLess(*object, *currentObject, maxDist))
								{
									points.push(Point(x, y));
								}
							}
						}
					}

					img.at<cv::Scalar>(currentPoint.Y, currentPoint.X) = lastColor;
				}
			}
		}
	

	}

	//Удаляем стврый childrenArray
	for (int i = 0; i < _children.height; i++)
		delete[] _children.childrenArray[i];
	delete[] _children.childrenArray;

	//Удаляем visited
	for (int i = 0; i < _children.height; i++)
		delete[] visited[i];
	delete[] visited;

	_children.childrenArray = tempArr;
	cv::imshow("wtf", img);

	DeleteNoise(0);
}

//Удаляет объекты с числом потомков меньше заданного
void PointCloudStorage::DeleteNoise(int minCount)
{
	_filteredChildrenList.clear();
	for (std::vector<BaseObject3D*>::iterator it = _childrenList.begin(); it != _childrenList.end(); ++it)
	{
		BaseObject3D* child = *it;
		if ((child != NULL) && ((child->GetType() == Object3DType::TYPE_OBJECT) && (dynamic_cast<Object3D*>(child)->ChildrenCount() > minCount)))
		{
			_filteredChildrenList.push_back(child);
		}
	}
}


//Рекурсивно сохраняет объект
void PointCloudStorage::_saveFromObject(Object3D* object, std::ofstream& stream) const
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
				stream << "v " << point[0] << " " << point[1] << " " << point[2] << "\n";
			}
		}
	}
}

//Сохраняет все в файл obj
void PointCloudStorage::SaveToObj(const char* filename) const
{
	std::ofstream stream(filename);
	for (int i = 0; i < _childrenList.size(); i++)
	{
		BaseObject3D* child = _childrenList[i];
		if (child != NULL)
		{
			if (child->GetType() == Object3DType::TYPE_POINT)
			{
				cv::Vec3f point = child->GetCoord();
				stream << "v " << point[0] << " " << point[1] << " " << point[2] << "\n";
			}
			else if (child->GetType()==Object3DType::TYPE_OBJECT)
				_saveFromObject((Object3D*)child, stream);
		}
	}
	stream.close();
}


//Возвращает тип
PointCloudStorage::Object3DType PointCloudStorage::GetType() const
{
	return Object3DType::TYPE_SCENE;
}

//Возращает число потомков
int PointCloudStorage::ChildrenCount() const
{
	return _filteredChildrenList.size();
}

//Возвращает потомка
BaseObject3D* PointCloudStorage::GetChild(int index) const
{
	return _filteredChildrenList[index];
}