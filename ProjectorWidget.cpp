#include "ProjectorWidget.hpp"

#include <QApplication>
#include <QDesktopWidget>
#include <QPainter>

ProjectorWidget::ProjectorWidget(QWidget * parent, Qt::WindowFlags flags) :
  QWidget(parent, flags),
  Height(1080),
  Width(1920),
  LineThickness(1), 
  Row(100)
{}

ProjectorWidget::~ProjectorWidget()
{
}

// /!\ Use of usigned char to code the color between 0 (black) and 255 (white)
cv::Mat ProjectorWidget::CreateLineImage()
{
  cv::Mat image = cv::Mat::zeros(this->Height, this->Width, CV_8UC1); // use CV_32S for int
  for (int j = 0; j < image.cols; j++)
  {
    for (int t = 0; t < this->LineThickness; t++)
    {
      image.at<unsigned char>(this->Row + t, j) = 255;
    }
  }
  return image;
}

cv::Mat ProjectorWidget::CreateLinePattern()
{
  std::vector<unsigned char> pattern{ 0,1,2,1,2,0,2,0,1,0 };

  int size = pattern.size();
  std::cout << "size = " << size << std::endl;
  int step = this->GetWidth() / size;
  std::cout << "step = " << step << std::endl;
  this->SetStep(step);
  
  cv::Mat im = cv::Mat::zeros(this->Height, this->Width, CV_8UC3);
  /*if (size != this->Width)
  {
    std::cerr << "Error : the size of the projector and of the de Bruijn's pattern don't match." << std::endl;
    return im;
  }*/
  int pos = 0;
  bool reverse = false;
  for (int i = 0; i < size; i++)
  {
    cv::Vec3b color;
   
    if (pattern.at(i) == 0)
    {
      // Blue
      color = { 255,0,0 };
    }
    else if (pattern.at(i) == 1)
    {
      // Green
      color = { 0,255,0 };
    }
    else if (pattern.at(i) == 2)
    {
      // Red
      color = { 0,0,255 };
    }
    for (int j = 0; j < this->GetHeight(); j++)
    {
      //im.at<cv::Vec3b>(100, i) = color;
      for (pos = i * step; pos < (i + 1) * step; pos++)
      {
        im.at<cv::Vec3b>(j, pos) = color;
      }
    }
  }
  // The camera is upper-down, so we need to reverse the code displayed
  if (reverse == true)
  {
    std::reverse(pattern.begin(), pattern.end());
  }
  std::vector<unsigned char>::const_iterator it_p = pattern.cbegin(), it_p_end = pattern.cend();
  std::cout << "Code : ";
  for (; it_p != it_p_end; ++it_p)
  {
    std::cout << int(*it_p) << " ";
  }
  std::cout << std::endl;

  // Creation of a map to store the index number of a code
  std::unordered_map<cv::Vec3b, int, Vec3bHash> map_pattern;
  
  for (int i = 0; i < size - 2; i++)
  {
    //if (i < size - 2) 
    {
      if (reverse == true)
      {
        map_pattern.emplace(cv::Vec3b(pattern.at(i), pattern.at(i + 1), pattern.at(i + 2)), (size - i - 1)*step);
        std::cout << " i : " << i << " (size-i-1)*step : " << (size - i - 1)*step << " " << cv::Vec3b(pattern.at(i), pattern.at(i + 1), pattern.at(i + 2)) << std::endl;
      }
      else
      {
        map_pattern.emplace(cv::Vec3b(pattern.at(i), pattern.at(i + 1), pattern.at(i + 2)), i*step);
        std::cout << " i : " << i << " i*step : " << i*step << " " << cv::Vec3b(pattern.at(i), pattern.at(i + 1), pattern.at(i + 2)) << std::endl;
      }
    }
  }
  if (reverse == true)
  {

  }
  else
  {
    map_pattern.emplace(cv::Vec3b(pattern.at(size - 2), pattern.at(size - 1), 99), (size - 2)*step);
    map_pattern.emplace(cv::Vec3b(pattern.at(size - 1), 99, 99), (size - 1)*step);
    std::cout << " i : " << size-2 << " i*step : " << (size-2)*step << " " << cv::Vec3b(pattern.at(size-2), pattern.at(size-1), 99) << std::endl;
    std::cout << " i : " << size-1 << " i*step : " << (size-1)*step << " " << cv::Vec3b(pattern.at(size-1), 99, 99) << std::endl;
  }

    /*else if (i == size - 2)
    {
      map_pattern.emplace(cv::Vec3b(pattern.at(i), pattern.at(i + 1), pattern.at(0)), i*step);
    }
    else if (i == size - 1)
    {
      map_pattern.emplace(cv::Vec3b(pattern.at(i), pattern.at(0), pattern.at(1)), i*step);
    }*/
    // same code -> won't be saved in the unordered_map
  
  /*std::unordered_map<cv::Vec3b, int, Vec3bHash>::const_iterator it = map_pattern.cbegin(), it_end = map_pattern.cend();
  int i = 0;
  for (; it != it_end; ++it)
  {
    std::cout << "Element " << i << " : " << it->first << " " << it->second << std::endl;
    i++;
  }*/
  this->SetPattern(map_pattern);
  return im;
}

std::vector<cv::Point2i> ProjectorWidget::GetCoordLine(cv::Mat image)
{
  // TODO: condition on type of matrix 
  std::vector<cv::Point2i> coord;
  for (int i = 0; i < image.rows; i++)
  {
    unsigned char *row = image.ptr<unsigned char>(i);
    //std::cout << "ligne : " << i << std::endl;
    //std::cout << "row :" << *row << std::endl;
    for (int j = 0; j < image.cols; j++)
    {
      if ((int)row[j] != 0) // or =255
      {
        coord.push_back(cv::Point2i(i, j));
      }
    }
  }

  return coord;
}

void ProjectorWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);

  if (!this->Pixmap.isNull())
  {
    //QPixmap scale_pixmap = Pixmap.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    //QRectF rect = QRectF(QPointF(0, 0), QPointF(scale_pixmap.width(), scale_pixmap.height()));
    QRectF rect = QRectF(QPointF(0, 0), QPointF(width(), height()));
    painter.drawPixmap(rect, this->Pixmap, rect);
    emit new_image(this->Pixmap);
  }
  else
  {
    QRectF rect = QRectF(QPointF(0, 0), QPointF(width(), height()));
    painter.drawText(rect, Qt::AlignCenter, "No image");
  }
}

void ProjectorWidget::start()
{
  QDesktopWidget * desktop = QApplication::desktop();
  int screen = desktop->screenCount();
  std::cout << screen << std::endl;
  // We choose the last screen added (highest number) = the projector
  //display
  QRect screen_resolution = desktop->screenGeometry(screen - 1);
  move(QPoint(screen_resolution.x(), screen_resolution.y()));
  showFullScreen();
  QApplication::processEvents();
}