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
StereoVision::StereoVision(StereoCalibData calibData, cv::Ptr<cv::StereoMatcher> stereoMatcher)
{
	this->calibData = calibData;
	_createUndistortRectifyMaps(calibData);

	this->stereoMatcher = stereoMatcher;
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
StereoCalibData StereoVision::Calibrate(const std::vector<cv::Mat>& left, const std::vector<cv::Mat>& right, cv::Size patternSize)
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

	cv::Size imSize;

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

	//2. --------------------------- Строим реальные кооринаты точек ------------------------------------------------------------

	/* Здесь мы задаем "реальные" координаты углов шахматной доски. Т.к реальные координаты мы не знаем, то
	просто принимаем их такими:
	(0,0) (0,1) ...
	(1,0) (1,1) ...
	*/

	for (int j = 0; j<patternSize.height*patternSize.width; j++)
		objectPointsSingle.push_back(cv::Point3f(j / patternSize.width, j%patternSize.width, 0.0f));


	//1. --------------------------- Ищем шахматные доски на изображении --------------------------------------------------------

	for (int i = 0; i < left.size(); i++)
	{
		//Документация: http://goo.gl/kV8Ms1 (Используется минифицированные ссылки, потому что исходная содержит пробелы)
		bool isFoundLeft = cv::findChessboardCorners(left[i], patternSize, imagePointsLeftSingle, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		bool isFoundRight = cv::findChessboardCorners(right[i], patternSize, imagePointsRightSingle, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		imSize = left[i].size();

		if (isFoundLeft && isFoundRight)
		{
			std::cout << i<<": success\n";


			//Уточняем углы (назначение параметров не известно)
			//Документация: http://goo.gl/7BjZKd
			cornerSubPix(left[i], imagePointsLeftSingle, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			cornerSubPix(right[i], imagePointsRightSingle, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			//Добавляем в вектор векторов
			imagePointsLeft.push_back(imagePointsLeftSingle);
			imagePointsRight.push_back(imagePointsRightSingle);
			objectPoints.push_back(objectPointsSingle);

			cv::Mat l = cv::Mat(left[i].rows, left[i].cols, CV_8UC3, cv::Scalar(0, 255, 0));
			cv::Mat r = cv::Mat(left[i].rows, left[i].cols, CV_8UC3, cv::Scalar(0, 255, 0));

			cv::cvtColor(left[i], l, CV_GRAY2BGR);
			cv::cvtColor(right[i], r, CV_GRAY2BGR);

			cv::drawChessboardCorners(l, patternSize, std::vector<cv::Point2f>(imagePointsLeftSingle), true);
			cv::drawChessboardCorners(r, patternSize, std::vector<cv::Point2f>(imagePointsRightSingle), true);

			cv::imshow("left", l);
			cv::imshow("right", r);
			cv::waitKey(0);
			cv::destroyWindow("left");
			cv::destroyWindow("right");
		}
		else
		{
			std::cout << i << ": fail\n";
		}
	}

	//3. --------------------------- Стерео калибровка --------------------------------------------------------------------------
	//Документация: http://goo.gl/mKCH63
	stereoCalibrate(objectPoints, imagePointsLeft, imagePointsRight,
		CM1, D1, CM2, D2, imSize, R, T, E, F, CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
		cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));

	cv::stereoRectify(CM1, D1, CM2, D2, imSize, R, T, R1, R2, P1, P2, Q);

	// -------------------------- Сохранение результатов ------------------------------------------------------------------------


	calibData.ImageSize = imSize;
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
* param[out] disparity - карта смещения для визуализации
* param[in] disparityOnly - строить карту глубины без облака точек
* result - облако точек
*/
PointCloudStorage StereoVision::CalculatePointCloud(const cv::Mat& left, const cv::Mat& right, cv::Mat& disparity, bool disparityOnly) const
{
	return _calculatePointCloud(left, right, false, disparity, disparityOnly);
}

/* Построение облака точек по двум изображениям
* param[in] left - левое изображение с откалиброванной камеры
* param[in] right - правое изображение с откалиброванной камеры
* param[in] disparityOnly - строить карту глубины без облака точек
* result - облако точек
*/
PointCloudStorage StereoVision::CalculatePointCloud(const cv::Mat& left, const cv::Mat& right, bool disparityOnly) const
{
	cv::Mat mat;
	return _calculatePointCloud(left, right, true, mat, disparityOnly);
}

//Строит облако точек
//Соответствующие публичные методы - обертка вокруг него, для красоты
PointCloudStorage StereoVision::_calculatePointCloud(const cv::Mat& left, const cv::Mat& right, bool noDisparityOut, cv::Mat& disparityResult, bool disparityOnly) const
{
	cv::Mat leftRemaped, rightRemaped;
	cv::Mat leftGrey, rightGrey;
	cv::Mat disparity, normalDisparity;
	cv::Mat cloud;


	//Цветное изображение обесцвечиваем
	if (left.channels() == 3)
		cv::cvtColor(left, leftGrey, CV_RGB2GRAY);
	else
		leftGrey = left;

	if (right.channels() == 3)
		cv::cvtColor(right, rightGrey, CV_RGB2GRAY);
	else
		rightGrey = right;

	//Выпрямляем
	cv::remap(leftGrey, leftRemaped, calibData.LeftMapX, calibData.LeftMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
	cv::remap(rightGrey, rightRemaped, calibData.RightMapX, calibData.RightMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

	//Строим карту различий
	stereoMatcher->compute(rightRemaped, leftRemaped, disparity);
	cv::normalize(disparity, normalDisparity, 0, 255, CV_MINMAX, CV_8U);
	
	if (!noDisparityOut)
	{
		disparityResult = normalDisparity.clone();
	}

	if (!disparityOnly)
	{
		//Создаем облако точек
		cv::reprojectImageTo3D(normalDisparity, cloud, calibData.Q, true);
		return PointCloudStorage(cloud.clone());
	}
	else
	{
		//Не создаем облако точек
		return PointCloudStorage();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////                         ГЕТТЕРЫ И СЕТТЕРЫ                  //////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

//Возвращает данные о калибровке
StereoCalibData StereoVision::GetCalibData()
{
	return calibData;
}

//Задат параметры калибровки
void StereoVision::SetCalibData(StereoCalibData calibData)
{
	this->calibData = calibData;
}

//Возвращает объект StereoMatcher
cv::Ptr<cv::StereoMatcher> StereoVision::GetStereoMatcher()
{
	return stereoMatcher;
}

//Задает объект StereoBM
void StereoVision::SetStereoMatcher(cv::Ptr<cv::StereoMatcher> stereoMatcher)
{
	this->stereoMatcher = stereoMatcher;
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