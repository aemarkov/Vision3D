#include "StereoCalibData.h"

//Конструктор по-умолчанию
StereoCalibData::StereoCalibData()
{

}

//Создание объекта из файла
StereoCalibData::StereoCalibData(const char* filename)
{	
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["IMS"] >> ImageSize;
	fs["CML"] >> LeftCameraMatrix;
	fs["CMR"] >> RightCameraMatrix;
	fs["CDL"] >> LeftCameraDistortions;
	fs["CDR"] >> RightCameraDistortions;
	fs["PL"] >> LeftCameraRectifiedProjection;
	fs["PR"] >> RightCameraRectifiedProjection;
	fs["RL"] >> LeftCameraRot;
	fs["RR"] >> RightCameraRot;
	fs["Q"] >> Q;
	fs.release();
}

//Конструктор копирования (создание ссылки)
StereoCalibData::StereoCalibData(const StereoCalibData& other)
{
	this->ImageSize = other.ImageSize;
	this->LeftCameraMatrix = other.LeftCameraMatrix.clone();
	this->RightCameraMatrix = other.RightCameraMatrix.clone();
	this->LeftCameraDistortions = other.LeftCameraDistortions.clone();
	this->RightCameraDistortions = other.RightCameraDistortions.clone();
	this->LeftCameraRectifiedProjection = other.LeftCameraRectifiedProjection;
	this->RightCameraRectifiedProjection = other.RightCameraRectifiedProjection;
	this->LeftCameraRot = other.LeftCameraRot;
	this->RightCameraRot = other.RightCameraRot;
	this->Q = other.Q.clone();
}

//Оператор присвоения (создание ссылки)
StereoCalibData& StereoCalibData::operator=(const StereoCalibData& other)
{
	if (&other != this)
	{
		this->ImageSize = other.ImageSize;
		this->LeftCameraMatrix = other.LeftCameraMatrix;
		this->RightCameraMatrix = other.RightCameraMatrix;
		this->LeftCameraDistortions = other.LeftCameraDistortions;
		this->RightCameraDistortions = other.RightCameraDistortions;
		this->LeftCameraRectifiedProjection = other.LeftCameraRectifiedProjection;
		this->RightCameraRectifiedProjection = other.RightCameraRectifiedProjection;
		this->LeftCameraRot = other.LeftCameraRot;
		this->RightCameraRot = other.RightCameraRot;
		this->Q = other.Q;
	}

	return *this;
}

//Создание полной копии
StereoCalibData& StereoCalibData::Clone()
{
	StereoCalibData data;

	data.ImageSize = this->ImageSize;
	data.LeftCameraMatrix = this->LeftCameraMatrix.clone();
	data.RightCameraMatrix = this->RightCameraMatrix.clone();
	data.LeftCameraDistortions = this->LeftCameraDistortions.clone();
	data.RightCameraDistortions = this->RightCameraDistortions.clone();
	data.LeftCameraRectifiedProjection = this->LeftCameraRectifiedProjection;
	data.RightCameraRectifiedProjection = this->RightCameraRectifiedProjection;
	data.LeftCameraRot = this->LeftCameraRot;
	data.RightCameraRot = this->RightCameraRot;
	data.Q = this->Q.clone();

	return data;
}


//Сохраняет данные
void StereoCalibData::Save(const char* filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	fs << "IMS" << ImageSize;
	fs << "CML" << LeftCameraMatrix;
	fs << "CMR" << RightCameraMatrix;
	fs << "CDL" << LeftCameraDistortions;
	fs << "CDR" << RightCameraDistortions;
	fs << "PL" << LeftCameraRectifiedProjection;
	fs << "PR" << RightCameraRectifiedProjection;
	fs << "RL" << LeftCameraRot;
	fs << "RR" << RightCameraRot;
	fs << "Q" << Q;
	fs.release();

}