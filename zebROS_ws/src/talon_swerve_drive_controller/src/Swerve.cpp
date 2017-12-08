#include <talon_swerve_drive_controller/Swerve.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <iomanip>	
#include <functional>
#include <cmath>
using namespace std;
using namespace Eigen;

swerve::swerve(array<Vector2d, WHEELCOUNT> _wheelCoordinates, string _fileAddress, bool _wheelAngleInvert, std::function<double(int, int)> _encoderPosition, std::function<double(int, int)> _encoderVelocity, ratios _ratio, encoderUnits _units, driveModel _drive)
{
	wheelCoordinates = _wheelCoordinates;

	swerveMath = swerveDriveMath(wheelCoordinates);

	encoderPosition = _encoderPosition;
	encoderVelocity = _encoderVelocity;
	ratio = _ratio;
	units = _units;
	drive = _drive;
	fileAddress = _fileAddress;
	for(int i =0; i < WHEELCOUNT; i++)
	{
		getWheelAngle(i);
	}
	wheelAngleInvert = _wheelAngleInvert ? -1 : 1;
	ifstream offsetRead;
	offsetRead.open(fileAddress);
	if(!offsetRead)
	{
		cout<< "No Offset File!!!" << endl;
		array<double, WHEELCOUNT> darray; 
                for(int i = 0; i < WHEELCOUNT; i++) 
                { 
                        darray[i] = 0; 
                }
		saveNewOffsets(false, darray);
	}
	else
	{
		double offset;
		int i = 0;
		while(offsetRead >> offset)
		{
			offsets[i] = offset;
			i++;
		}
	}

};
array<Vector2d, WHEELCOUNT> swerve::motorOutputs(Vector2d velocityVector, double rotation, double angle, bool forceRead, array<bool, WHEELCOUNT> &reverses, bool park, int rotationCenterID, bool overrideID, Vector2d centerOfRotation)
{
	array<Vector2d, WHEELCOUNT> speedsAndAngles;
	if(!park)
	{
		velocityVector/=drive.maxSpeed;

		if(overrideID)
		{
			multiplierSet newSet;
			newSet.multipliers = swerveMath.wheelMultipliersXY(centerOfRotation);
			newSet.maxRotRate = furthestWheel(centerOfRotation)/drive.maxSpeed;	
			multiplierSets[rotationCenterID] = newSet;
		}
		rotation/=multiplierSets[rotationCenterID].maxRotRate;
		speedsAndAngles = swerveMath.wheelSpeedsAngles(multiplierSets[rotationCenterID].multipliers, velocityVector, rotation, angle);
		for(int i = 0; i < WHEELCOUNT; i++)
		{
			double nearestAngle;
			bool reverse;
			if(forceRead)
			{
				getWheelAngle(i);
				nearestAngle = leastDistantAngleWithinHalfPi(savedEncoderVals[i], speedsAndAngles[i][1], reverse);
			}
			else
			{
				nearestAngle = leastDistantAngleWithinHalfPi(savedEncoderVals[i], speedsAndAngles[i][1], reverse);
				if(abs(savedEncoderVals[i] - nearestAngle) > M_PI/4)
				{
					getWheelAngle(i);
					nearestAngle = leastDistantAngleWithinHalfPi(savedEncoderVals[i], speedsAndAngles[i][1], reverse);
				}
			}
			reverses[i] = reverse;
			speedsAndAngles[i][0]*=((drive.maxSpeed/(drive.wheelRadius*2*M_PI))/ratio.encodertoRotations)*units.rotationSetV* (reverse ? -1 : 1);
			speedsAndAngles[i][1] = (nearestAngle/(2*M_PI))*units.steeringSet - offsets[i];
		}
	}
	else
	{	
		for(int i = 0; i < WHEELCOUNT; i++)
		{
			speedsAndAngles[i][1] = swerveMath.parkingAngle[i];
			speedsAndAngles[i][0] = 0;
		}

	}
	return speedsAndAngles;
}
void swerve::saveNewOffsets(bool useVals, array<double, WHEELCOUNT> newOffsets)
{
	if(!useVals)
	{
		for(int i = 0; i < WHEELCOUNT; i++)
		{
			newOffsets[i] = encoderPosition(i, 1);
		}
	}
	offsets = newOffsets;
	ofstream offsetFile;
	offsetFile.open(fileAddress);
	if(offsetFile)
	{
		offsetFile.close();
		offsetFile.open(fileAddress, ios::out | ios::trunc);

	}
	for(int i = 0; i < WHEELCOUNT; i++)
	{
		offsetFile << offsets[i] << endl;
	}
	offsetFile.close();
}
/*
Vector2d calculateOdom()
{

//Steal code from steered wheel base

}
*/

double swerve::getWheelAngle(int index)
{
	savedEncoderVals[index] = (encoderPosition(index, 1)+offsets[index])*units.steeringGet*2*M_PI*wheelAngleInvert;
	return savedEncoderVals[index];
}

double swerve::furthestWheel(Vector2d centerOfRotation)
{
	double maxD = 0;
	for(int i = 0; i < WHEELCOUNT; i++)
	{
		double dist = sqrt(pow(wheelCoordinates[i][0] - centerOfRotation[0], 2) + pow(wheelCoordinates[i][1] - centerOfRotation[1], 2));
		if(dist>maxD){maxD=dist;}
	}
	return maxD;
}
