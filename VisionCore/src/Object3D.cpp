#include "Object3D.h"

//Создает новый объект
Object3D::Object3D()
{
	_coordsSum = cv::Vec3f(0, 0, 0);
}

Object3D::Object3D(BaseObject3D* parent)
{
	Parent = parent;
}

/* Добавляет потомка и пересчитывает координаты центра
param[in] - ссылка на нового потомка
*/
void Object3D::AddChild(BaseObject3D* child)
{
	//Добавляем указаталь на потомка в список потомков
	_children.push_back(child);
	child->Parent = this;
	
	//Пересчитываем координаты центра
	cv::Vec3f childCoord = child->GetCoord();
	_coordsSum += childCoord;
	_coord[0] = _coordsSum[0] / _children.size();
	_coord[1] = _coordsSum[1] / _children.size();
	_coord[2] = _coordsSum[2] / _children.size();
}

//Возвращает тип объекта
Object3D::Object3DType Object3D::GetType()
{
	return Object3DType::TYPE_OBJECT;
}