#ifndef __BASEOBJECT3D_H__
#define __BASEOBJECT3D_H__


/*///////////////////////////////////////////////////////////////////////////////////////
//                               Object3D                                              //
//-------------------------------------------------------------------------------------//
// Базовый класс, представляющий объект облака точек.                                  //
// Он имеет координаты и указатель на родителя, фактически, предсатавляет собой точку  //
// Другие объекты наследуются от него, чтобы можно было использовать указатели на      //
// потомков и родителей.                                                               //
// Иеархия:                                                                            //
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

#include <opencv2/opencv.hpp>

class BaseObject3D
{
public:

	//Варианты типа объекта
	static enum Object3DType {TYPE_SCENE, TYPE_OBJECT, TYPE_POINT};

	BaseObject3D* Parent;		//Указатель на родителя

	//Создает объект с заданными коордиантами
	BaseObject3D();
	BaseObject3D(cv::Vec3f coord, BaseObject3D* parent);
	BaseObject3D(float x, float y, float z, BaseObject3D* parent);

	//Установка и получение коордианты
	void SetCoord(cv::Vec3f coord);
	void SetCoord(float x, float y, float z);	
	cv::Vec3f GetCoord();
	
	//Возращает тип
	virtual Object3DType GetType() const;

protected:
	cv::Vec3f _coord;
};

#endif