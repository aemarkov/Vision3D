#include "BaseObject3D.h"

//Создает объект с заданными коордиантами
BaseObject3D::BaseObject3D()
{
	Parent = NULL;
	_coord = cv::Vec3f(0, 0, 0);
}

BaseObject3D::BaseObject3D(cv::Vec3f coord, BaseObject3D* parent)
{
	_coord = coord;
	Parent = parent;
}

BaseObject3D::BaseObject3D(float x, float y, float z, BaseObject3D* parent)
{
	_coord = cv::Vec3f(x, y, z);
	Parent = parent;
}

//Установка и получение коордианты
void BaseObject3D::SetCoord(cv::Vec3f coord)
{
	_coord = coord;
}

void BaseObject3D::SetCoord(float x, float y, float z)
{
	_coord = cv::Vec3f(x, y, z);
}

cv::Vec3f BaseObject3D::GetCoord()
{
	return _coord;
}

//Возращает тип
BaseObject3D::Object3DType BaseObject3D::GetType()
{
	return Object3DType::TYPE_POINT;
}