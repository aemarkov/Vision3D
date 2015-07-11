#ifndef __STEREOVISION_H__
#define __STEREOVISION_H__

/*///////////////////////////////////////////////////////////////////////////////////////
//                                 StereoVision                                        //
//-------------------------------------------------------------------------------------//
// Этот класс предоставляет функционал по созданию облака точек по ДВУМ изображения    //
// (в.т.ч., полученным с ДВУХ камер).                                                  //
// Основные функции данного класса:                                                    //
//  - осуществление калибровки камеры\камер                                            //
//  - загрузка и сохранение результатов калибровки (с использованием класса            //
//    StereoCameraCalibData)                                                           //
//  - обработка пар снимков, сделанных при помощи откалиюрованных камер и построение   //
//    карты глубины и облака точек (за сохранение облака точек отвечает интерфейс      //
//    IPointCloudStorage)                                                              //
/*///////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>

#include <vector>
#include <ostream>

#include "StereoCalibData.h"
#include "IPointCloudStorage.h"
#include "StaticHelpers.h"

class StereoVision
{
public:

	StereoVision(std::ostream* strean);
	//StereoVision(const char *name);
	//StereoVision(StereoCalibData calibData);

	StereoCalibData StereoVision::Calibrate(const cv::Mat& left, const cv::Mat& right, cv::Size patternSize);

	IPointCloudStorage* CalculatePointCloud(const cv::Mat& left, const cv::Mat& right) const;

	StereoCalibData GetCalibData();

private:
	StereoCalibData calibData;			//Калибровочные данные
	std::ostream* out;					//Поток отладочного вывода
};

#endif