#ifndef __GLUTVIEWER_H__
#define __GLUTVIEWER_H__

#include "GL/freeglut.h"
#include <thread>

class GlutViewer
{
public:
	GlutViewer(int argc, char** argv);

private:

	//Вращение и масштаб
	int rotX, rotY, rotZ;
	float scale;
	
	std::thread renderThread;

	static void threadFunctionWrapper(int argc, char** argv, GlutViewer* instance);
	void threadFunction(int argc, char** argv);

	static void renderSceneWrapper(void);
	void renderScene();

	static void keyPressedWrapper(int key, int x, int y);
	void keyPressed(int key, int x, int y);

	static void generalKeyPressedWrapper(unsigned char key, int x, int y);
	void generalKeyPressed(int key, int x, int y);

};

#endif