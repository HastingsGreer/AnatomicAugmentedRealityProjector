#include "CameraInput.hpp"

#include "FlyCapture2.h"

#include <QTime>

#include <iomanip>
#include <iostream>
#include <stdio.h>
//#include <ostream>

using namespace FlyCapture2;

cv::Mat rotation_180( cv::Mat mat )
{
  cv::transpose( mat, mat );
  cv::flip( mat, mat, 0 );
  cv::transpose( mat, mat );
  cv::flip( mat, mat, 0 );
  return mat;
}

CameraInput::CameraInput() : NbImages( 1 ), Camera()
{
}

CameraInput::~CameraInput()
{
  Error error;
  // Stop capturing images
  error = this->Camera.StopCapture();
  if( error != PGRERROR_OK )
    {
    error.PrintErrorTrace();
    }

  // Disconnect the camera
  error = this->Camera.Disconnect();
  if( error != PGRERROR_OK )
    {
    error.PrintErrorTrace();
    }
}

bool CameraInput::Run()
{
  Error error;
  BusManager busMgr;
  //sleep(5);
  cv::waitKey( 2 );
  PGRGuid guid;
  unsigned int numCameras;

  error = busMgr.GetNumOfCameras( &numCameras );
  if( error != PGRERROR_OK )
    {
    error.PrintErrorTrace();
    return false;
    }
  if( numCameras < 1 )
    {
    std::cout << "No camera detected." << std::endl;
    return false;
    }
  else
    {
    std::cout << "Number of cameras detected: " << numCameras << std::endl;
    }

  error = busMgr.GetCameraFromIndex( 0, &guid );
  if( error != PGRERROR_OK )
    {
    error.PrintErrorTrace();
    return false;
    }

  error = Camera.Connect( &guid );
  if( error != PGRERROR_OK )
    {
    error.PrintErrorTrace();
    return false;
    }

  error = Camera.StartCapture();
  if( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
    std::cout << "Bandwidth exceeded" << std::endl;
    return false;
    }
  else if( error != PGRERROR_OK )
    {
    std::cout << "Failed to start image capture" << std::endl;
    return false;
    }
  return true;
}

void CameraInput::SetCameraFrameRate( double frameRate )
{
  Error error;

  // Check if the camera supports the FRAME_RATE property
  std::cout << "Detecting frame rate from camera... " << std::endl;
  PropertyInfo propInfo;
  propInfo.type = FRAME_RATE;
  error = this->Camera.GetPropertyInfo( &propInfo );
  if( error != PGRERROR_OK )
    {
    error.PrintErrorTrace();
    return;
    }
  if( propInfo.present == true )
    {
    // Get the frame rate
    Property prop;
    prop.type = FRAME_RATE;
    error = this->Camera.GetProperty( &prop );
    if( error != PGRERROR_OK )
      {
      error.PrintErrorTrace();
      }
    else
      {
      prop.autoManualMode = false;
      // Set the frame rate.
      // Note that the actual recording frame rate may be slower,
      // depending on the bus speed and disk writing speed.
      prop.absValue = frameRate;
      error = this->Camera.SetProperty( &prop );
      if( error != PGRERROR_OK )
        {
        error.PrintErrorTrace();
        return;
        }
      }
    }
  std::cout << "Asking frame rate of " << std::fixed << std::setprecision( 1 ) << frameRate << std::endl;
}

double CameraInput::GetCameraFrameRate()
{
  Error error;

  // Check if the camera supports the FRAME_RATE property
  PropertyInfo propInfo;
  propInfo.type = FRAME_RATE;
  error = this->Camera.GetPropertyInfo( &propInfo );
  if( error != PGRERROR_OK )
    {
    error.PrintErrorTrace();
    return 0;
    }
  if( propInfo.present == true )
    {
    // Get the frame rate
    Property prop;
    prop.type = FRAME_RATE;
    error = this->Camera.GetProperty( &prop );
    if( error != PGRERROR_OK )
      {
      error.PrintErrorTrace();
      }
    else
      {
      // Set the frame rate.
      // Note that the actual recording frame rate may be slower,
      // depending on the bus speed and disk writing speed.
      std::cout << "Using frame rate of " << std::fixed << std::setprecision( 1 ) << prop.absValue << std::endl;
      return prop.absValue;
      }
    }
  return 0;
}
// Note : Check the returned value when calling the function 

bool CameraInput::RecordImages()
{
  std::cout << "Grabbing " << this->NbImages << " images" << std::endl;

  Error error;
  Image rawImage;
  for( int imageCount = 0; imageCount < this->NbImages; imageCount++ )
    {
    // Retrieve an image
    error = this->Camera.RetrieveBuffer( &rawImage );
    if( error != PGRERROR_OK )
      {
      error.PrintErrorTrace();
      continue;
      }

    std::cout << ".";

    // Get the raw image dimensions
    PixelFormat pixFormat;
    unsigned int rows, cols, stride;
    rawImage.GetDimensions( &rows, &cols, &stride, &pixFormat );

    // Create a converted image
    Image convertedImage;

    // Convert the raw image
    error = rawImage.Convert( PIXEL_FORMAT_BGRU, &convertedImage );
    if( error != PGRERROR_OK )
      {
      error.PrintErrorTrace();
      return false;
      }

    // Get the camera information
    CameraInfo camInfo;
    error = this->Camera.GetCameraInfo( &camInfo );
    if( error != PGRERROR_OK )
      {
      error.PrintErrorTrace();
      return false;
      }
    // Create a unique filename
    std::ostringstream filename;
    QString date = QDateTime::currentDateTime().toString( "yyyy-MM-dd_hh.mm.ss.zzz" );
    filename << "Results\\" << camInfo.serialNumber << "-" << imageCount << "-" << date.toStdString() << ".bmp";

    // Save the image. If a file format is not passed in, then the file
    // extension is parsed to attempt to determine the file format.
    error = convertedImage.Save( filename.str().c_str() );
    if( error != PGRERROR_OK )
      {
      error.PrintErrorTrace();
      return false;
      }
    }
  std::cout << std::endl;
  std::cout << "Finished grabbing images" << std::endl;
  return true;
}

cv::Mat CameraInput::DisplayImages()
{
  FlyCapture2::Error error;
  static bool flag = false;
  FlyCapture2::Image rawImage;
  error = this->Camera.RetrieveBuffer( &rawImage );
  if( error != FlyCapture2::PGRERROR_OK )
    {
    error.PrintErrorTrace();
    }
  FlyCapture2::Image rgbImage;
  rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

  // convert to OpenCV Mat
  unsigned int rowBytes = ( double )rgbImage.GetReceivedDataSize() / ( double )rgbImage.GetRows();
  cv::Mat mat = cv::Mat( rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes );

  rotation_180( mat );
  return mat;
}
