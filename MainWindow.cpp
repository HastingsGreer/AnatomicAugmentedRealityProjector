#include "io_util.hpp"
#include "MainWindow.hpp"
#include "ui_mainwindow.h"
#include "moc_MainWindow.cpp"
#include "moc_ProjectorWidget.cpp"

#include "FlyCapture2.h"

#include <opencv2/imgproc/imgproc.hpp>

#include <QtConcurrent>
#include <qtconcurrentrun.h>
#include <QDesktopWidget>
#include <QtGui>
#include <QThread>
#include <QGraphicsPixmapItem>
#include <QFileDialog>

#include <iostream>


MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  Projector(),
  CamInput()
{
  ui->setupUi(this);
  this->setWindowTitle("Camera Projector");

  connect(ui->proj_height, SIGNAL(valueChanged(int)), this, SLOT(SetProjectorHeight()));
  connect(ui->proj_width, SIGNAL(valueChanged(int)), this, SLOT(SetProjectorWidth()));
  connect(ui->proj_thickness, SIGNAL(valueChanged(int)), this, SLOT(SetProjectorLineThickness()));
  connect(ui->proj_row, SIGNAL(valueChanged(int)), this, SLOT(SetProjectorLineRow()));
  connect(ui->cam_framerate, SIGNAL(valueChanged(double)), this, SLOT(SetCameraFrameRate()));
  connect(ui->cam_nbimages, SIGNAL(valueChanged(int)), this, SLOT(SetCameraNbImages()));


  this->SetCameraFrameRate();

  // Timer
  this->timer = new QTimer(this);
  this->timer->setSingleShot(false);
  this->timer->setInterval(5);
  this->connect(timer, SIGNAL(timeout()), SLOT(DisplayCamera()));

  QString calibrationFile = "C:\\Camera_Projector_Calibration\\CameraProjector\\calibration.yml";
  this->Calib.LoadCalibration(calibrationFile);
  this->Calib.Display();
}

MainWindow::~MainWindow()
{
  delete ui;
}

inline QImage cvMatToQImage(const cv::Mat &mat)
{
  switch (mat.type())
  {
  // 8-bit, 3 channel
  case CV_8UC3:
    {
    QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_RGB888);
    return image.rgbSwapped();
    }
  // 8-bit, 1 channel
  case CV_8UC1:
    {
    // creating a color table only the first time
    static QVector<QRgb> sColorTable;

    if (sColorTable.isEmpty())
      {
      for (int i = 0; i < 256; i++)
        {
        sColorTable.append(qRgb(i, i, i));
        //NOTE : /!\ takes time
        }
      }
    QImage image(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Indexed8);
    image.setColorTable(sColorTable);
    return image;
    }
  default:
    qWarning() << "Type not handled : " << mat.type();
    break;
  }
  return QImage();
}

// Note : If we know that the lifetime of the cv::Mat is shorter than the QImage, then pass false for the inCloneImageData argument. This will share the QImage data.
inline cv::Mat QImageToCvMat(const QImage& image, bool inCloneImageData = true)
{
  switch (image.format())
  {
  case QImage::Format_Indexed8:
  {
    //8-bit, 1 channel
    cv::Mat mat(image.height(), image.width(), CV_8UC1, const_cast<uchar*>(image.bits()), static_cast<size_t>(image.bytesPerLine()));
    return (inCloneImageData ? mat.clone() : mat);
  }
  case QImage::Format_RGB888:
  {
    if (!inCloneImageData)
    {
      qWarning() << "ASM::QImageToCvMat() - Conversion requires cloning because we use a temporary QImage";
    }

    QImage   swapped;
    swapped = image.rgbSwapped();

    return cv::Mat(swapped.height(), swapped.width(), CV_8UC3, const_cast<uchar*>(swapped.bits()), static_cast<size_t>(swapped.bytesPerLine())).clone();
  }
  default:
    qWarning() << "Type not handled : " << image.format();
    break;
  }
  return cv::Mat();
}

void MainWindow::on_proj_display_clicked()
{
  //cv::Mat mat = this->Projector.CreateLineImage();
  cv::Mat mat = this->Projector.CreateLinedeBruijn();

  if (!mat.data)
    {
    std::cout << "Could not open or find the image" << std::endl;
    return;
    }
  //std::vector<cv::Point2i> proj_points = this->Projector.GetCoordLine(mat);
  //std::cout << proj_points << std::endl;
  QPixmap pixmap = QPixmap::fromImage(cvMatToQImage(mat));
  this->Projector.SetPixmap(pixmap);
  /*QGraphicsScene *Scene = new QGraphicsScene(this);
  Scene->addPixmap(pixmap);
  Scene->setSceneRect(0, 0, pixmap.width(), pixmap.height());
  ui->proj_image->fitInView(Scene->sceneRect(), Qt::KeepAspectRatio);
  ui->proj_image->setScene(Scene);*/

  connect(&(this->Projector), SIGNAL(new_image(QPixmap)), this, SLOT(_on_new_projector_image(QPixmap)));

  this->Projector.start();

  //disconnect projector display signal
  disconnect(&(this->Projector), SIGNAL(new_image(QPixmap)), this, SLOT(_on_new_projector_image(QPixmap)));
}

void MainWindow::DisplayCamera()
{
  QGraphicsScene *scene = new QGraphicsScene(this);
  ui->cam_image->setScene(scene);
  cv::Mat mat = this->CamInput.DisplayImages();
  QPixmap PixMap = QPixmap::fromImage(cvMatToQImage(mat));
  scene->clear();
  ui->cam_image->scene()->addItem(new QGraphicsPixmapItem(PixMap));
  scene->setSceneRect(0, 0, PixMap.width(), PixMap.height());
  ui->cam_image->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);

  cv::waitKey(1);

  /*error = CamInput.Camera.StopCapture();
  if (error != FlyCapture2::PGRERROR_OK)
  {
  // This may fail when the camera was removed, so don't show
  // an error message
  }*/
}

void MainWindow::on_cam_display_clicked()
{
  CamInput.Run(); 
  this->timer->start();
}

void MainWindow::on_cam_record_clicked()
{
  this->CamInput.RecordImages();
}

void MainWindow::SetProjectorHeight()
{
  this->Projector.SetHeight(ui->proj_height->value());
}

void MainWindow::SetProjectorWidth()
{
  this->Projector.SetWidth(ui->proj_width->value());
}

void MainWindow::SetProjectorLineThickness()
{
  this->Projector.SetLineThickness(ui->proj_thickness->value());
}

void MainWindow::SetProjectorLineRow()
{
  this->Projector.SetRow(ui->proj_row->value());
}

void MainWindow::SetCameraFrameRate()
{
  CamInput.SetCameraFrameRate(ui->cam_framerate->value());
}

void MainWindow::SetCameraNbImages()
{
  CamInput.SetNbImages(ui->cam_nbimages->value());
}

void MainWindow::_on_new_projector_image(QPixmap pixmap)
{
  this->Projector.SetPixmap(pixmap);
}

void MainWindow::on_analyze_clicked()
{
  // Substract 2 images to keep only the line illuminated by the projector
  char* dir = "Results\\";
  QString filename = QFileDialog::getOpenFileName(this, "Open decoded files", dir);
  if (filename.isEmpty())
    {
    return;
    }
  cv::Mat mat_color = cv::imread(qPrintable(filename), CV_LOAD_IMAGE_COLOR);

  filename = QFileDialog::getOpenFileName(this, "Open decoded files", dir);
  if (filename.isEmpty())
    {
    return;
    }
  cv::Mat mat_color_ref = cv::imread(qPrintable(filename), CV_LOAD_IMAGE_COLOR);

  cv::Mat mat_c = abs(mat_color - mat_color_ref);
  if (!mat_c.data || mat_c.type() != CV_8UC3)
  {
    qCritical() << "ERROR invalid cv::Mat data\n";
  }
  // TODO : check if the calib file is valid
  cv::resize(mat_c, mat_c, cv::Size(500, 500));
  cv::imshow("Image line", mat_c);
  cv::waitKey(0);

  std::vector<cv::Point2i> cam_points;
  std::vector<cv::Vec3b> cam_colors;
  cv::Mat res = cv::Mat::zeros(mat_c.rows, mat_c.cols, CV_8UC1);
  for (int i = 0; i < mat_c.rows; i++)
  {
    for (int j = 0; j < mat_c.cols; j++)
    {
      if (mat_c.at<cv::Vec3b>(i, j).val[0] > 100 || mat_c.at<cv::Vec3b>(i, j).val[1] > 100 || mat_c.at<cv::Vec3b>(i, j).val[2] > 100)
      {
        res.at<unsigned char>(i, j) = 255;
        cam_points.push_back(cv::Point2i(i, j));
        cam_colors.push_back(mat_c.at<cv::Vec3b>(i, j));
      }
    }
  }
  cv::resize(res, res, cv::Size(500, 500));
  cv::imshow("Image result", res);
  cv::waitKey(0);

  // find the coord of the projector point corresponding to a point on cam_points
  std::vector<cv::Vec2i> proj_coordinates;
  cv::Vec2i proj_result;
  std::vector<cv::Point2i>::iterator it_cam_points = cam_points.begin();
  std::vector<cv::Vec3b>::iterator it_cam_colors = cam_colors.begin();
  std::vector<cv::Vec3b>::iterator it;
  std::vector<int> code_colors;
  double distance;
  cv::Point3d p;
  cv::Mat pointcloud = cv::Mat(this->Projector.GetHeight(), this->Projector.GetWidth(), CV_32FC3);
  // imageTest is used to control which points have been used on the projector for the reconstruction
  cv::Mat imageTest = cv::Mat::zeros(this->Projector.GetHeight(), this->Projector.GetWidth(), CV_8UC1);
  for (it_cam_points; it_cam_points != cam_points.end(); ++it_cam_points, ++it_cam_colors)
  {
    // get the code of the current point and of the 7 next points
    int good_points = 0;
    int crt_code = getCode(*it_cam_colors);
    while (crt_code == 99)
    {
      it_cam_colors++;
      crt_code = getCode(*it_cam_colors);
    }
    int next_code;
    it = it_cam_colors;
    
    code_colors.clear();
    code_colors.push_back(crt_code);
    while (good_points != 7 && it!=cam_colors.end())
    {
      it++;
      next_code = getCode(*it);
      if (next_code == 99)
      {
        std::cerr << "Error in the detected color" << std::endl;
      }
      else if (next_code != crt_code)
      {
        code_colors.push_back(next_code);
        crt_code = next_code;
        good_points++;
      }
    }
    // We have the 8 points needed to have the correspondant point of the projector
    // parcours de la table pour trouver le meme code
    std::vector<int> deBruijn = this->Projector.GetdeBruijn();
    std::vector<int>::iterator it_deBruijn = deBruijn.begin();
    int i = 0;
    bool correct = false;
    while ((it_deBruijn != deBruijn.end()) || (correct = false))
    {
      if (*it_deBruijn == code_colors.at(0))
      {
        it_deBruijn++;
        if (*it_deBruijn == code_colors.at(1))
        {
          it_deBruijn++;
          if (*it_deBruijn == code_colors.at(2))
          {
             // good code ! 
             correct = true;
          }
        }
      }
      else
      {
        it_deBruijn++;
        i++;
      }
    }
    if (it_deBruijn == deBruijn.end())
    {
      std::cerr << "Error : fail to find the corresponding point in the projector coordinates" << std::endl;
      continue;
    }
    // i corresponds to the column of the projector point we are looking for
    triangulate_stereo(this->Calib.Cam_K, this->Calib.Cam_kc, this->Calib.Proj_K, this->Calib.Proj_kc, this->Calib.R.t(), this->Calib.T, *it_cam_points, cv::Point2i(this->Projector.GetRow(),i), p, &distance);
    cv::Vec3f & cloud_point = pointcloud.at<cv::Vec3f>(this->Projector.GetRow(), i);
    cloud_point[0] = p.x;
    cloud_point[1] = p.y;
    cloud_point[2] = p.z;
    imageTest.at<unsigned char>(this->Projector.GetRow(), i) = 255;
   
  }
  if (!pointcloud.data)
  {
    qCritical() << "ERROR, reconstruction failed\n";
  }
  cv::resize(imageTest, imageTest, cv::Size(500, 500));
  cv::imshow("Image line", imageTest);
  cv::waitKey(0);

  QString name = "pointcloud";
  filename = QFileDialog::getSaveFileName(this, "Save pointcloud", name + ".ply", "Pointclouds (*.ply)");
  if (!filename.isEmpty())
  {
    std::cout << "Saving the pointcloud" << std::endl;
    bool binary = false;
    unsigned ply_flags = io_util::PlyPoints | (binary ? io_util::PlyBinary : 0);
    bool success = io_util::write_ply(filename.toStdString(), pointcloud, ply_flags);
    if (success == false)
    {
      qCritical() << "ERROR, saving the pointcloud failed\n";
      return;
    }
  }


  /*std::vector<cv::Point2i> cam_points;
  
  for (int i = 0; i < mat.rows; i++)
    {
    for (int j = 0; j < mat.cols; j++)
      {
      if ((int)mat.at<unsigned char>(i, j) > 50)
        {
        cam_points.push_back(cv::Point2i(i, j));
        }
      }
    }
  */
/*  QImage im = this->Projector.GetPixmap().toImage().convertToFormat(QImage::Format_Indexed8, Qt::AvoidDither);
  cv::Mat mat_proj = QImageToCvMat(im);

  std::vector<cv::Point2i> proj_points = this->Projector.GetCoordLine(mat_proj);
  std::vector<cv::Point2i>::iterator it_proj = proj_points.begin();
  std::vector<cv::Point3d> vec_w2;
  std::vector<cv::Point3d> vec_v2;
  cv::Mat inp2(1, 1, CV_64FC2);
  cv::Mat outp2;
  cv::Point3d w2, v2;
  for (it_proj; it_proj != proj_points.end(); ++it_proj)
  {
    //to image camera coordinates
    inp2.at<cv::Vec2d>(0, 0) = cv::Vec2d(it_proj->x, it_proj->y);

    cv::undistortPoints(inp2, outp2, this->Calib.Proj_K, this->Calib.Proj_kc);
    assert(outp2.type() == CV_64FC2 && outp2.rows == 1 && outp2.cols == 1);
    const cv::Vec2d & outvec2 = outp2.at<cv::Vec2d>(0, 0);
    cv::Point3d u2(outvec2[0], outvec2[1], 1.0);

    //to world coordinates
    w2 = cv::Point3d(cv::Mat(this->Calib.R.t()*(cv::Mat(u2) - this->Calib.T)));
    vec_w2.push_back(w2);
    //world rays
    v2 = cv::Point3d(cv::Mat(this->Calib.R.t()*cv::Mat(u2)));
    vec_v2.push_back(v2);
  }
  assert(vec_v2.size() == vec_w2.size());
  assert(vec_v2.size() == proj_points.size());

  cv::Mat pointcloud = cv::Mat(this->Projector.GetHeight(), this->Projector.GetWidth(), CV_32FC3);
  // imageTest is used to control which points have been used on the projector for the reconstruction
  cv::Mat imageTest = cv::Mat::zeros(this->Projector.GetHeight(), this->Projector.GetWidth(), CV_8UC1);
  double distance_min, distance;
  cv::Point2i good_proj_point;
  cv::Point3d p, good_p;
  std::vector<cv::Point2i>::iterator it_cam = cam_points.begin();
  cv::Mat inp1(1, 1, CV_64FC2);
  cv::Mat outp1;
  cv::Point3d w1, v1;
  for (it_cam; it_cam != cam_points.end(); ++it_cam)
  {
    //to image camera coordinates
   
    inp1.at<cv::Vec2d>(0, 0) = cv::Vec2d(it_cam->x, it_cam->y);
    
    cv::undistortPoints(inp1, outp1, this->Calib.Cam_K, this->Calib.Cam_kc);
    assert(outp1.type() == CV_64FC2 && outp1.rows == 1 && outp1.cols == 1);
    const cv::Vec2d & outvec1 = outp1.at<cv::Vec2d>(0, 0);
    cv::Point3d u1(outvec1[0], outvec1[1], 1.0);

    //to world coordinates
    w1 = u1;
    //world rays
    v1 = w1;

    //it_proj = proj_points.begin();
    distance_min = 9999;
    for (int i = 0; i < vec_v2.size(); i++)
    {
      p = approximate_ray_intersection(v1, w1, vec_v2.at(i), vec_w2.at(i), &distance);
      if (distance < distance_min)
      {
        distance_min = distance;
        good_p = p;
        good_proj_point = proj_points.at(i);
      }
    }
    cv::Vec3f & cloud_point = pointcloud.at<cv::Vec3f>(good_proj_point.x, good_proj_point.y);
    cloud_point[0] = good_p.x;
    cloud_point[1] = good_p.y;
    cloud_point[2] = good_p.z;
    imageTest.at<unsigned char>(good_proj_point.x, good_proj_point.y) = 255;
  }
  if (!pointcloud.data)
  {
    qCritical() << "ERROR, reconstruction failed\n";
  }
  cv::resize(imageTest, imageTest, cv::Size(500, 500));
  cv::imshow("Image line", imageTest);
  cv::waitKey(0);

  QString name = "pointcloud";
  filename = QFileDialog::getSaveFileName(this, "Save pointcloud", name + ".ply", "Pointclouds (*.ply)");
  if (!filename.isEmpty())
  {
    std::cout << "Saving the pointcloud" << std::endl;
    bool binary = false;
    unsigned ply_flags = io_util::PlyPoints | (binary ? io_util::PlyBinary : 0);
    bool success = io_util::write_ply(filename.toStdString(), pointcloud, ply_flags);
    if (success == false)
    {
      qCritical() << "ERROR, saving the pointcloud failed\n";
      return;
    }
  }*/

  /*for (it_cam; it_cam != cam_points.end(); ++it_cam)
    {
    distance_min = 9999;
    std::vector<cv::Point2i>::iterator it_proj = proj_points.begin();
    for (it_proj; it_proj != proj_points.end(); ++it_proj)
      {
      triangulate_stereo(this->Calib.Cam_K, this->Calib.Cam_kc, this->Calib.Proj_K, this->Calib.Proj_kc, this->Calib.R.t(), this->Calib.T, *it_cam, *it_proj, p, &distance);
      if (distance < distance_min)
        {
        distance_min = distance;
        good_p = p;
        good_proj_point = *it_proj;
        }
      }
    cv::Vec3f & cloud_point = pointcloud.at<cv::Vec3f>(good_proj_point.x, good_proj_point.y);
    cloud_point[0] = good_p.x;
    cloud_point[1] = good_p.y;
    cloud_point[2] = good_p.z;
    imageTest.at<unsigned char>(good_proj_point.x, good_proj_point.y) = 255;
    }
   if (!pointcloud.data)
    {
    qCritical() << "ERROR, reconstruction failed\n";
    }
  cv::resize(imageTest, imageTest, cv::Size(500, 500));
  cv::imshow("Image line", imageTest);
  cv::waitKey(0);

  QString name = "pointcloud";
  filename = QFileDialog::getSaveFileName(this, "Save pointcloud", name + ".ply", "Pointclouds (*.ply)");
  if (!filename.isEmpty())
    {
    std::cout << "Saving the pointcloud" << std::endl;
    bool binary = false;
    unsigned ply_flags = io_util::PlyPoints | (binary ? io_util::PlyBinary : 0);
    bool success = io_util::write_ply(filename.toStdString(), pointcloud, ply_flags);
    if (success == false)
      {
      qCritical() << "ERROR, saving the pointcloud failed\n";
      return;
      }
    }*/
}

void MainWindow::triangulate_stereo(const cv::Mat & K1, const cv::Mat & kc1, const cv::Mat & K2, const cv::Mat & kc2,
  const cv::Mat & Rt, const cv::Mat & T, const cv::Point2i & p1, const cv::Point2i & p2,
  cv::Point3d & p3d, double * distance)
{
  //to image camera coordinates
  cv::Mat inp1(1, 1, CV_64FC2), inp2(1, 1, CV_64FC2);
  inp1.at<cv::Vec2d>(0, 0) = cv::Vec2d(p1.x, p1.y);
  inp2.at<cv::Vec2d>(0, 0) = cv::Vec2d(p2.x, p2.y);
  cv::Mat outp1, outp2;
  cv::undistortPoints(inp1, outp1, K1, kc1);
  cv::undistortPoints(inp2, outp2, K2, kc2);
  assert(outp1.type() == CV_64FC2 && outp1.rows == 1 && outp1.cols == 1);
  assert(outp2.type() == CV_64FC2 && outp2.rows == 1 && outp2.cols == 1);
  const cv::Vec2d & outvec1 = outp1.at<cv::Vec2d>(0, 0);
  const cv::Vec2d & outvec2 = outp2.at<cv::Vec2d>(0, 0);
  cv::Point3d u1(outvec1[0], outvec1[1], 1.0);
  cv::Point3d u2(outvec2[0], outvec2[1], 1.0);

  //to world coordinates
  cv::Point3d w1 = u1;
  cv::Point3d w2 = cv::Point3d(cv::Mat(Rt*(cv::Mat(u2) - T)));

  //world rays
  cv::Point3d v1 = w1;
  cv::Point3d v2 = cv::Point3d(cv::Mat(Rt*cv::Mat(u2)));

  //compute ray-ray approximate intersection
  p3d = approximate_ray_intersection(v1, w1, v2, w2, distance);
}

cv::Point3d MainWindow::approximate_ray_intersection(const cv::Point3d & v1, const cv::Point3d & q1,
  const cv::Point3d & v2, const cv::Point3d & q2, double * distance)
{
  cv::Mat v1mat = cv::Mat(v1);
  cv::Mat v2mat = cv::Mat(v2);

  double v1tv1 = cv::Mat(v1mat.t()*v1mat).at<double>(0, 0);
  double v2tv2 = cv::Mat(v2mat.t()*v2mat).at<double>(0, 0);
  double v1tv2 = cv::Mat(v1mat.t()*v2mat).at<double>(0, 0);
  double v2tv1 = cv::Mat(v2mat.t()*v1mat).at<double>(0, 0);

  cv::Mat Vinv(2, 2, CV_64FC1);
  double detV = v1tv1*v2tv2 - v1tv2*v2tv1;
  Vinv.at<double>(0, 0) = v2tv2 / detV;  Vinv.at<double>(0, 1) = v1tv2 / detV;
  Vinv.at<double>(1, 0) = v2tv1 / detV; Vinv.at<double>(1, 1) = v1tv1 / detV;

  cv::Point3d q2_q1 = q2 - q1;
  double Q1 = v1.x*q2_q1.x + v1.y*q2_q1.y + v1.z*q2_q1.z;
  double Q2 = -(v2.x*q2_q1.x + v2.y*q2_q1.y + v2.z*q2_q1.z);

  double lambda1 = (v2tv2 * Q1 + v1tv2 * Q2) / detV;
  double lambda2 = (v2tv1 * Q1 + v1tv1 * Q2) / detV;

  cv::Point3d p1 = lambda1*v1 + q1; //ray1
  cv::Point3d p2 = lambda2*v2 + q2; //ray2

  cv::Point3d p = 0.5*(p1 + p2);

  if (distance != NULL)
    {
    *distance = cv::norm(p2 - p1);
    }
  return p;
}

int MainWindow::getCode(cv::Vec3b cam_color)
{
  /*if (cam_color[0] < 70)
  {
    if (cam_color[1] < 70)
    {
      if (cam_color[2] < 70)
      {
        // black
        return 6;
      }
      else
      {
        // blue
        return 3;
      }
    }
    else
    {
      if (cam_color[2] < 70)
      {
        // green
        return 2;
      }
      else
      {
        // cyan
        return 4;
      }
    }
  }
  else
  {
    if (cam_color[1] < 70)
    {
      if (cam_color[2] < 70)
      {
        // red
        return 1;
      }
      else
      {
        // magenta
        return 5;
      }
    }
    else
    {
      if (cam_color[2] < 70)
      {
        // yellow
        return 0;
      }
      else
      {
        // white point, shouldn't exist
        std::cerr << "Error in the detected color";
        return 99;
      }
    }
  }*/
  if (cam_color[0] > 70 && cam_color[1] < 70 && cam_color[2] < 70)
  {
    return 0;
  }
  else if (cam_color[1] > 70 && cam_color[0] < 70 && cam_color[2] < 70)
  {
    return 1;
  }
  else if (cam_color[2] > 70 && cam_color[0] < 70 && cam_color[1] < 70)
  {
    return 2;
  }
  else
  {
    std::cout << "Color not valid : " << cam_color << std::endl;
    return 99;
  }
}



cv::Vec2i MainWindow::DecodeCoordinates(cv::Vec2i cam_point, cv::Vec3b cam_color)
{
  cv::Vec2i proj_point{ 0,0 };
  
  

  

  return proj_point;
}
