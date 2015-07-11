#include "StereoVision.h"

StereoVision::StereoVision(std::ostream* stream)
{
	out = stream;
}

/*StereoVision::StereoVision(const char *name)
{

}

StereoVision::StereoVision(StereoCalibData calibData)
{

}*/

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
		5. Получаем матрицы для remap
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
			T,			//Вектор перехода между СК первой и второй камер
			E,			//Существенная матрица (Устанавливает соотношения между точками изображения?????) (тут что-то связано с эпиполярными линиями)
			F;			//Фундаментальная матрица

	//Выходные параметры для stereoRectify
	cv::Mat R1, R2,		//Матрицы поворота для исправления (3х3)
			P1, P2;		//Проективная матрица в исправленной системе координат (3х4)
						//Первые 3 столбца этих матриц - новые матрицы камер
	cv::Mat Q;			//Матрица перевода смещения в глубину

	//Выходные данные для построение карт выравнивания
	cv::Mat mapLeftX, mapLeftY, mapRightX, mapRightY;

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


	//Отладочный вывод - эти найденные углы
	cv::drawChessboardCorners(left, patternSize, imagePointsLeftSingle, true);
	cv::drawChessboardCorners(right, patternSize, imagePointsRightSingle, true);

	cv::imwrite("images\\corners_left.png", left);
	cv::imwrite("images\\corners_right.png", right);

	//2. --------------------------- Строим реальные кооринаты точек ------------------------------------------------------------
	/*for (int i = 0; i < patternSize.height; i++)
	{
		for (int j = 0; j < patternSize.width; j++)
		{
			objectPointsSingle.push_back(cv::Point3f(i*size, j*size, 0));
		}
	}*/

	for (int j = 0; j<patternSize.height*patternSize.width; j++)
		objectPointsSingle.push_back(cv::Point3f(j / patternSize.width, j%patternSize.width, 0.0f));
	objectPoints.push_back(objectPointsSingle);

	//3. --------------------------- Стерео калибровка --------------------------------------------------------------------------
	//Документация: http://goo.gl/mKCH63
	stereoCalibrate(objectPoints, imagePointsLeft, imagePointsRight,
					CM1, D1, CM2, D2, left.size(), R, T, E, F, CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST,
					cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));

	//4. Получение данных для выравнивания
	cv::stereoRectify(CM1, D1, CM2, D2, left.size(), R, T, R1, R2, P1, P2, Q);

	//5. Построение карт выравнивания
	cv::initUndistortRectifyMap(CM1, D1, R1, P1, left.size(), CV_32FC1, mapLeftX, mapLeftY);
	cv::initUndistortRectifyMap(CM2, D2, R2, P2, left.size(), CV_32FC1, mapRightX, mapRightY);

	// -------------------------- Сохранение результатов ------------------------------------------------------------------------

	calibData.LeftCameraMatrix = CM1.clone();
	calibData.RightCameraMatrix = CM2.clone();
	calibData.LeftCameraDistortions = D1.clone();
	calibData.RightCameraDistortions = D2.clone();
	calibData.MapLeftX = mapLeftX.clone();
	calibData.MapLeftY = mapLeftY.clone();
	calibData.MapRightX = mapRightX.clone();
	calibData.MapRightY = mapRightY.clone();
	calibData.Q = Q;

	/*(*out) << "CM1:\n";
	StaticHelpers::printMatrixStream<double>(CM1, *out);

	(*out) << "\nCM2:\n";
	StaticHelpers::printMatrixStream<double>(CM2, *out);

	(*out) << "\nD1:\n";
	StaticHelpers::printMatrixStream<double>(D1, *out);

	(*out) << "\nD2:\n";
	StaticHelpers::printMatrixStream<double>(D2, *out);

	(*out) << "\nR:\n";
	StaticHelpers::printMatrixStream<double>(R, *out);

	(*out) << "\nT:\n";
	StaticHelpers::printMatrixStream<double>(T, *out);

	(*out) << "\nE:\n";
	StaticHelpers::printMatrixStream<double>(E, *out);

	(*out) << "\nF:\n";
	StaticHelpers::printMatrixStream<double>(F, *out);

	(*out) << "\nP1:\n";
	StaticHelpers::printMatrixStream<double>(P1, *out);

	(*out) << "\nR1:\n";
	StaticHelpers::printMatrixStream<double>(R2, *out);

	(*out) << "\nR2:\n";
	StaticHelpers::printMatrixStream<double>(R2, *out);

	(*out) << "\nP1:\n";
	StaticHelpers::printMatrixStream<double>(P1, *out);

	(*out) << "\nP2:\n";
	StaticHelpers::printMatrixStream<double>(P2, *out);

	(*out) << "\nQ:\n";
	StaticHelpers::printMatrixStream<double>(Q, *out);*/

	this->calibData = calibData;
	return StereoCalibData();
}


/* Построение облака точек по двум изображениям
 * param[in] left - левое изображение с откалиброванной камеры
 * param[in] right - правое изображение с откалиброванной камеры
 * result - облако точек
 */
IPointCloudStorage* StereoVision::CalculatePointCloud(const cv::Mat& left, const cv::Mat& right) const
{
	cv::Mat leftRemaped, rightRemaped;
	cv::remap(left, leftRemaped, calibData.MapLeftX, calibData.MapLeftY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
	cv::remap(right, rightRemaped, calibData.MapRightX, calibData.MapRightY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

	cv::imwrite("images\\left_remap.png", leftRemaped);
	cv::imwrite("images\\right_remap.png", rightRemaped);


	return NULL;
}

StereoCalibData StereoVision::GetCalibData()
{
	return StereoCalibData();
}