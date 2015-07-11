#include "StereoVision.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////                        КОНСТРУКТОРЫ                           ///////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

//Создает новый объект
StereoVision::StereoVision()
{
	
}

//Загружает параметры калибровки из файла
StereoVision::StereoVision(const char *name)
{
	calibData = StereoCalibData(name);
	_createUndistortRectifyMaps(calibData);
}

//Получает параметры калибровки
StereoVision::StereoVision(StereoCalibData calibData)
{
	this->calibData = calibData;
	_createUndistortRectifyMaps(calibData);
}


///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////                        ИНТЕРФЕЙС                              ///////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


/* Калибровка стерео-камеры (пары камер)
* param[in] left - изображение с левой камеры (CV_8UC1 - серое)
* param[in] right - изображение с правой камеры
* param[in] patternSize - число углов шахматной доски
(число квадратов на стороне - 1)
* result - необходимый набор параметров для построения облака точек
*/
StereoCalibData StereoVision::Calibrate(const cv::Mat& left, const cv::Mat& right, cv::Size patternSize)
{
	/* Алгоритм стерео-калибровки:
	1. Находим на них шахматную доску и определяем углы
	2. Создаем реальные координаты углов шахматной доски:
	считаем, что начальный угол имеет координаты (0,0,0), а остальные
	смещаются от него на заданное расстояние - размер клетки
	3. Выполняем stereoCalibrate, чтобы получитьт
	матрицы камер, дисторсию и матрицы перехода между камерами (R, T)
	4. Выполняем stereoRectify
	5. Находим матрицы устранения искажений
	*/

	const int size = 1;

	// Точки на изображении
	std::vector<cv::Point2f> imagePointsLeftSingle, imagePointsRightSingle;			//Вектор найденных на изображении точек
	std::vector<std::vector<cv::Point2f>> imagePointsLeft, imagePointsRight;		//Вектор векторов точек - по вектору на изображение (у нас по одному на камеру)

	// Реальные точки
	std::vector<cv::Point3f> objectPointsSingle;									//Реальные координаты точек
	std::vector<std::vector<cv::Point3f>> objectPoints;								//Вектор векторов координат - по вектору на изображение

	//Выходные параметры stereoCalibrate
	cv::Mat CM1(3, 3, CV_64FC1), CM2(3, 3, CV_64FC1);								//CameraMatrix указывает как перейти от 3D точек в мировой СК вк 2D точкам в изображении
	cv::Mat D1, D2;		//Коэффициенты дисторсии
	cv::Mat R,			//Матрица поворота между СК первой и второй камер
		T,				//Вектор перехода между СК первой и второй камер
		E,				//Существенная матрица (Устанавливает соотношения между точками изображения?????) (тут что-то связано с эпиполярными линиями)
		F;				//Фундаментальная матрица

	//Выходные параметры для stereoRectify
	cv::Mat R1, R2,		//Матрицы поворота для исправления (3х3)
		P1, P2;			//Проективная матрица в исправленной системе координат (3х4)
						//Первые 3 столбца этих матриц - новые матрицы камер
	cv::Mat Q;			//Матрица перевода смещения в глубину

	//1. --------------------------- Ищем шахматные доски на изображении --------------------------------------------------------

	//Документация: http://goo.gl/kV8Ms1 (Используется минифицированные ссылки, потому что исходная содержит пробелы)
	bool isFoundLeft = cv::findChessboardCorners(left, patternSize, imagePointsLeftSingle, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	bool isFoundRight = cv::findChessboardCorners(right, patternSize, imagePointsRightSingle, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

	if (!isFoundLeft || !isFoundRight)
	{
		//Углы не найдены - кидаем исключение?
	}

	//Уточняем углы (назначение параметров не известно)
	//Документация: http://goo.gl/7BjZKd
	cornerSubPix(left, imagePointsLeftSingle, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
	cornerSubPix(right, imagePointsRightSingle, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

	//Добавляем в вектор векторов
	imagePointsLeft.push_back(imagePointsLeftSingle);
	imagePointsRight.push_back(imagePointsRightSingle);

	//2. --------------------------- Строим реальные кооринаты точек ------------------------------------------------------------

	/* Здесь мы задаем "реальные" координаты углов шахматной доски. Т.к реальные координаты мы не знаем, то
	   просто принимаем их такими:
	   (0,0) (0,1) ... 
	   (1,0) (1,1) ...
	*/

	for (int j = 0; j<patternSize.height*patternSize.width; j++)
		objectPointsSingle.push_back(cv::Point3f(j / patternSize.width, j%patternSize.width, 0.0f));
	objectPoints.push_back(objectPointsSingle);

	//3. --------------------------- Стерео калибровка --------------------------------------------------------------------------
	//Документация: http://goo.gl/mKCH63
	stereoCalibrate(objectPoints, imagePointsLeft, imagePointsRight,
		CM1, D1, CM2, D2, left.size(), R, T, E, F ,CV_CALIB_FIX_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
		cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));

	cv::stereoRectify(CM1, D1, CM2, D2, left.size(), R, T, R1, R2, P1, P2, Q);

	// -------------------------- Сохранение результатов ------------------------------------------------------------------------

	D1 = (cv::Mat_<double>(1, 4) << 0, 0, 0, 0);
	D2 = D1;
	R1 = (cv::Mat_<double>(3, 3) <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1);
	R2 = R1;

	calibData.ImageSize = left.size();
	calibData.LeftCameraMatrix = CM1.clone();
	calibData.RightCameraMatrix = CM2.clone();
	calibData.LeftCameraDistortions = D1.clone();
	calibData.RightCameraDistortions = D2.clone();
	calibData.LeftCameraRectifiedProjection = P1.clone();
	calibData.RightCameraRectifiedProjection = P2.clone();
	calibData.LeftCameraRot = R1.clone();
	calibData.RightCameraRot = R2.clone();
	calibData.Q = Q;

	_createUndistortRectifyMaps(calibData);

	return calibData;
}


/* Построение облака точек по двум изображениям
* param[in] left - левое изображение с откалиброванной камеры
* param[in] right - правое изображение с откалиброванной камеры
* result - облако точек
*/
IPointCloudStorage* StereoVision::CalculatePointCloud(const cv::Mat& left, const cv::Mat& right) const
{
	cv::Mat leftRemaped, rightRemaped;

	cv::remap(left, leftRemaped, calibData.LeftMapX, calibData.LeftMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
	cv::remap(right, rightRemaped, calibData.RightMapX, calibData.RightMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

	cv::imwrite("images\\left_remap.png", leftRemaped);
	cv::imwrite("images\\right_remap.png", rightRemaped);

	cv::imshow("w1", leftRemaped);
	cv::imshow("w2", rightRemaped);
	cv::waitKey(0);
	cv::destroyWindow("w1");
	cv::destroyWindow("w2");

	return NULL;
}

StereoCalibData StereoVision::GetCalibData()
{
	return StereoCalibData();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////                      ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ                  ///////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

//Создает матрицы исправленяи искажений
void StereoVision::_createUndistortRectifyMaps(StereoCalibData& data)
{
	cv::initUndistortRectifyMap(data.LeftCameraMatrix, data.LeftCameraDistortions, data.LeftCameraRot, data.LeftCameraRectifiedProjection, data.ImageSize, CV_32FC1, data.LeftMapX, data.LeftMapY);
	cv::initUndistortRectifyMap(data.RightCameraMatrix, data.RightCameraDistortions, data.RightCameraRot, data.RightCameraRectifiedProjection, data.ImageSize, CV_32FC1, data.RightMapX, data.RightMapY);
}