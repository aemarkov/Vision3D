#include "StereoCalibData.h"

//Конструктор по-умолчанию
StereoCalibData::StereoCalibData()
{

}

//Конструктор копирования (создание ссылки)
StereoCalibData::StereoCalibData(const StereoCalibData& other)
{
	this->LeftCameraMatrix = other.LeftCameraMatrix;
	this->RightCameraMatrix = other.LeftCameraMatrix;
	this->LeftCameraDistortions = other.LeftCameraDistortions;
	this->RightCameraDistortions = other.RightCameraDistortions;
	this->MapLeftX = other.MapLeftX;
	this->MapLeftY = other.MapLeftY;
	this->MapRightX = other.MapRightY;
	this->MapRightY = other.MapRightY;
	this->Q = other.Q;
}

//Оператор присвоения (создание ссылки)
StereoCalibData& StereoCalibData::operator=(const StereoCalibData& right)
{
	return StereoCalibData(right);
}

//Создание полной копии
StereoCalibData& StereoCalibData::Clone()
{
	StereoCalibData data;

	data.LeftCameraMatrix = this->LeftCameraMatrix.clone();
	data.RightCameraMatrix = this->RightCameraMatrix.clone();
	data.LeftCameraDistortions = this->LeftCameraDistortions.clone();
	data.RightCameraDistortions = this->RightCameraDistortions.clone();
	data.MapLeftX = this->MapLeftX.clone();
	data.MapLeftY = this->MapLeftY.clone();
	data.MapRightX = this->MapRightX.clone();
	data.MapRightY = this->MapRightY.clone();
	data.Q = this->Q.clone();

	return data;
}