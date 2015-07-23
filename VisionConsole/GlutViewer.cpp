#include "GlutViewer.h"

//Конструктор
GlutViewer::GlutViewer(int argc, char** argv)
{
	rotX = 0;
	rotY = 0;
	rotZ = 0;
	scale = 1;

	//Запуск потока
	renderThread = std::thread(threadFunctionWrapper, argc, argv, this);
	renderThread.detach();
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
	glutInitWindowSize(400, 400);
	int windowId = glutCreateWindow("3D View");

	//Передаем в статическую функцию-обработчик указатель на класс
	glutSetWindowData(this);

	//Назначаем обработчики и ф-ию отрисовки
	glutDisplayFunc(renderSceneWrapper);
	glutSpecialFunc(keyPressedWrapper);
	glutKeyboardFunc(generalKeyPressedWrapper);
	glutMainLoop();
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

		//Вращаем сцену
		glRotated(rotX, 1, 0, 0);
		glRotated(rotY, 0, 1, 0);
		glRotated(rotZ, 0, 0, 1);

		glScaled(scale, scale, scale);

		glBegin(GL_POINTS);

		for (float i = -2; i <= 2; i += 0.5)
			for (float j = -2; j <= 2; j += 0.5)
				for (float k = -2; k <= 2; k += 0.5)
					glVertex3f(i, j, k);
		glEnd();

		glutSwapBuffers();
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
		scale += 0.1;
	else if (key == '-')
		scale -= 0.1;

	renderScene();
}