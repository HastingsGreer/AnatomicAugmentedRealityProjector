#include "FlyCapture2.h"

#include <QThread>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class CameraInput
{
public:
  CameraInput();
  ~CameraInput();

  bool Run();
  void SetCameraFrameRate( double framerate );
  double GetCameraFrameRate();

  void SetNbImages( int nbImages ) { this->NbImages = nbImages; };
  int GetNbImages() const { return this->NbImages; };

  bool RecordImages();
  cv::Mat DisplayImages();

  FlyCapture2::Camera Camera;

private:
  int NbImages;

};
