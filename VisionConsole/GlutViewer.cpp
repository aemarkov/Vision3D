#include "GlutViewer.h"

//Конструктор
GlutViewer::GlutViewer(int argc, char** argv, PointCloudStorage* cloud)
{
	rotX = 0;
	rotY = 0;
	rotZ = 0;
	posX = 0;
	posY = 0;
	posZ = 0;
	scale = 1;

	this->cloud = cloud;

	//Запуск потока
	renderThread = std::thread(threadFunctionWrapper, argc, argv, this);
	renderThread.detach();
}

void GlutViewer::UpdateGeometry(PointCloudStorage* cloud)
{
	if (cloud != NULL)
		this->cloud = cloud;

	renderScene();
}

//Статическая функция-обертка над потоком
void GlutViewer::threadFunctionWrapper(int argc, char** argv, GlutViewer* instance)
{
	instance->threadFunction(argc, argv);
}

void GlutViewer::threadFunction(int argc, char** argv)
{
	//Инициализация
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

	//Создание окна
	glutInitWindowSize(800, 800);
	int windowId = glutCreateWindow("3D View");

	//Передаем в статическую функцию-обработчик указатель на класс
	glutSetWindowData(this);

	//Назначаем обработчики и ф-ию отрисовки
	glutDisplayFunc(renderSceneWrapper);
	glutSpecialFunc(keyPressedWrapper);
	glutKeyboardFunc(generalKeyPressedWrapper);
	glutIdleFunc(idleWrapper);
	glutMainLoop();
}


void GlutViewer::idleWrapper()
{
	GlutViewer* instance = (GlutViewer*)glutGetWindowData();
	instance->idle();
}

void GlutViewer::idle()
{
	renderScene();
}

//Статическая функция обертка над функцией рендера
void GlutViewer::renderSceneWrapper(void)
{
	GlutViewer* instance = (GlutViewer*)glutGetWindowData();
	instance->renderScene();
}

//Функция рендера
void GlutViewer::renderScene() {
	
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glPushMatrix();

	if (cloud == NULL)return;

	//Трансформации сцены
	glRotated(rotX, 1, 0, 0);
	glRotated(rotY, 0, 1, 0);
	glRotated(rotZ, 0, 0, 1);
	glTranslated(posX, posY, posZ);
	glScaled(scale, scale, scale);

	glBegin(GL_POINTS);

	//Отрисовываем модель
	Color color(0, 0, 0);
	for (int i = 0; i < cloud->ChildrenCount(); i++)
	{
		BaseObject3D* object = cloud->GetChild(i);
		if (object != NULL)
		{
			if (object->GetType() == BaseObject3D::TYPE_OBJECT)
			{
				color.R += 0.5;
				if (color.R > 1)
				{
					color.R = 0;
					color.B += 0.5;
				}
				if (color.G > 1)
				{
					color.G = 0;
					color.B += 0.5;
				}
				if (color.B > 1)
					color.B = 0;

				renderObject(dynamic_cast<Object3D*>(object), color);
			}
			else if (object->GetType() == BaseObject3D::TYPE_POINT)
			{
				cv::Vec3f coord = object->GetCoord();
				glVertex3f(coord[0]/5, coord[1]/5, coord[2]/5);
			}
		}
	}

	glEnd();

	glutSwapBuffers();
}

//Рекурсивный рендер объекта
void GlutViewer::renderObject(Object3D* object, Color color)
{
	for (int i = 0; i < object->ChildrenCount(); i++)
	{
		BaseObject3D* curObject = object->GetChild(i);
		if (curObject != NULL)
		{
			if (curObject->GetType() == BaseObject3D::TYPE_OBJECT)
				renderObject(dynamic_cast<Object3D*>(curObject), color);
			else
			{
				cv::Vec3f coord = curObject->GetCoord();
				glColor3f(color.R, color.G, color.B);
				glVertex3f(coord[0]/5, coord[1]/5, coord[2]/5);
			}
		}
	}
}

void GlutViewer::keyPressedWrapper(int key, int x, int y)
{
	GlutViewer* instance = (GlutViewer*)glutGetWindowData();
	instance->keyPressed(key, x, y);
}

void GlutViewer::keyPressed(int key, int x, int y)
{
	if (key == GLUT_KEY_LEFT)
		rotY--;
	else if (key == GLUT_KEY_RIGHT)
		rotY++;
	else if (key == GLUT_KEY_UP)
		rotX--;
	else if (key == GLUT_KEY_DOWN)
		rotX++;

	renderScene();
}


void GlutViewer::generalKeyPressedWrapper(unsigned char key, int x, int y)
{
	GlutViewer* instance = (GlutViewer*)glutGetWindowData();
	instance->generalKeyPressed(key, x, y);
}

void GlutViewer::generalKeyPressed(int key, int x, int y)
{
	//this is '+' without shift
	if (key == '=')
		scale += 0.01;
	else if (key == '-')
		scale -= 0.01;
	else if (key == 'w')
		posZ -= 0.1;
	else if (key == 's')
		posZ += 0.1;
	else if (key == 'a')
		posX -= 0.1;
	else if (key == 'd')
		posX += 0.1;

	renderScene();
}