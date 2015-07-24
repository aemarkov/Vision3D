#ifndef __GLUTVIEWER_H__
#define __GLUTVIEWER_H__

#include "GL/freeglut.h"
#include <thread>

#include "PointCloudStorage.h"

class GlutViewer
{
public:
	GlutViewer(int argc, char** argv, PointCloudStorage* cloud);
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
	
	std::thread renderThread;

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