#ifndef __GLUTVIEWER_H__
#define __GLUTVIEWER_H__

#include "GL/freeglut.h"
#include <thread>
#include <mutex>

#include "PointCloudStorage.h"

/*///////////////////////////////////////////////////////////////////////////////////////
//                               Object3D                                              //
//-------------------------------------------------------------------------------------//
// Создает окно с просмоторщиком облака точек из PointCLoudStorage                     //
// Разные объекты подсвечиваются разным цветом                                         //
//                                                                                     //
// ВНИМАНИЕ                                                                            //
// Из-за особенностей GLUT работа нескольких экземпляров под вопросом                  //
/*///////////////////////////////////////////////////////////////////////////////////////

class GlutViewer
{
public:
	GlutViewer(int argc, char** argv, PointCloudStorage* cloud);
	~GlutViewer();
	void UpdateGeometry(PointCloudStorage* cloud);

private:

	//Структура цвета
	//Чтобы не тянуть в этот класс зависимость OpenCV из-за одного Scalar
	struct Color
	{
		float R, G, B;
		Color(float r, float b, float g)
		{
			R = r; B = b; G = g;
		}
	};

	PointCloudStorage* cloud;

	//Вращение и масштаб
	int rotX, rotY, rotZ;
	float posX, posY, posZ;
	float scale;
	
	//Поток и окно
	std::thread renderThread;	//Поток, в котором выполняется код glut
	int windowId;				//id окна
	bool closed;				//Завершен ли поток
	std::mutex closedMutex;		//Мьютекс для разделения доступа к closed.

	//Функция потока
	static void threadFunctionWrapper(int argc, char** argv, GlutViewer* instance);
	void threadFunction(int argc, char** argv);

	static void idleWrapper();
	void idle();

	//Функция отрисовки
	static void renderSceneWrapper(void);
	void renderScene();

	//Рекурсивный рендер объекта
	void renderObject(Object3D* object, Color color);

	//Обработчик специальных клавиш
	static void keyPressedWrapper(int key, int x, int y);
	void keyPressed(int key, int x, int y);

	//Обработчик обычных клавиш
	static void generalKeyPressedWrapper(unsigned char key, int x, int y);
	void generalKeyPressed(int key, int x, int y);

};

#endif