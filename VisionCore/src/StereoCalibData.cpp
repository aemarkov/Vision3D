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
	fs["T"] >> CameraTransform;
	fs.release();
}

//Конструктор копирования (создание ссылки)
StereoCalibData::StereoCalibData(const StereoCalibData& other)
{
	_copyToThis(other);
}

//Оператор присвоения (создание ссылки)
StereoCalibData& StereoCalibData::operator=(const StereoCalibData& other)
{
	if (&other != this)
	{
		_copyToThis(other);
	}

	return *this;
}

//Общий код копирования в этот объект
void StereoCalibData::_copyToThis(const StereoCalibData& other)
{
	this->ImageSize = other.ImageSize;
	this->LeftCameraMatrix = other.LeftCameraMatrix.clone();
	this->RightCameraMatrix = other.RightCameraMatrix.clone();
	this->LeftCameraDistortions = other.LeftCameraDistortions.clone();
	this->RightCameraDistortions = other.RightCameraDistortions.clone();
	this->LeftCameraRectifiedProjection = other.LeftCameraRectifiedProjection.clone();
	this->RightCameraRectifiedProjection = other.RightCameraRectifiedProjection.clone();
	this->LeftCameraRot = other.LeftCameraRot.clone();
	this->RightCameraRot = other.RightCameraRot.clone();
	this->Q = other.Q.clone();
	this->CameraTransform = other.CameraTransform.clone();

	this->LeftMapX = other.LeftMapX.clone();
	this->LeftMapY = other.LeftMapY.clone();
	this->RightMapX = other.RightMapX.clone();
	this->RightMapY = other.RightMapY.clone();
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
	data.LeftCameraRectifiedProjection = this->LeftCameraRectifiedProjection.clone();
	data.RightCameraRectifiedProjection = this->RightCameraRectifiedProjection.clone();
	data.LeftCameraRot = this->LeftCameraRot;
	data.RightCameraRot = this->RightCameraRot;
	data.Q = this->Q.clone();
	data.CameraTransform = this->CameraTransform.clone();;


	data.LeftMapX = this->LeftMapX.clone();
	data.LeftMapY = this->LeftMapY.clone();
	data.RightMapX = this->RightMapX.clone();
	data.RightMapY = this->RightMapY.clone();


	return data;
}


//Сохраняет данные
void StereoCalibData::Save(const char* filename) const
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
	fs << "T" << CameraTransform;
	fs.release();

}