#ifndef __OBJECT3D_H__
#define __OBJECT3D_H__

/*///////////////////////////////////////////////////////////////////////////////////////
//                               Object3D                                              //
//-------------------------------------------------------------------------------------//
// Данный класс представляет собой фрагмент облака точек, представляющий союой         //
// объект. Объекты на сцене (PointCloudStorage) объеденины в иеархическую структуру    //
// Данный класс унаследован от базвого  класса объекта облака точек и                  //
// Представляет собой промежуточную часть иеархии.                                     //
//                                                                                     //
// PointCloudStorage:BaseObject3D                                                      //
//               |                                                                     //
//     Object3D:BaseObject3D                                                           //
//               |                                                                     //
//              ...                                                                    //
//               |                                                                     //
//    Object3D:BaseObject3D                                                            //
//               |                                                                     //
//          BaseObject3D                                                               //
/*///////////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <opencv2/opencv.hpp>

#include "BaseObject3D.h"

class Object3D: public BaseObject3D
{
public:

	//Создает новый объект
	Object3D();
	Object3D(BaseObject3D* parent);
	
	/* Добавляет потомка и пересчитывает координаты центра
	   param[in] - ссылка на нового потомка
	 */
	void AddChild(BaseObject3D* child);

	//Возращает число потомков
	int ChildrenCount() const;

	//Возвращает потомка
	BaseObject3D* GetChild(int index) const;

	//Возвращает тип объекта
	Object3DType GetType() const;

protected:

	//Список потомков
	std::vector<BaseObject3D*> _children;

	//Сумма координат всех потомков
	cv::Vec3f _coordsSum;

};

#endif