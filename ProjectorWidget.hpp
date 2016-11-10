#ifndef __PROJECTOR_HPP__
#define __PROJECTOR_HPP__

#include <opencv2/core/core.hpp>

#include <QWidget>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unordered_map>
#include <vector>


class Vec3bHash
{
public:
  std::size_t operator()(const cv::Vec3b &vec) const
  {
    std::size_t s = std::hash<unsigned long>()((vec(0) << 16) | (vec(1)<<8) | (vec(2)));
    //std::cout << "Hash code : " << s << std::endl;
    return s;
  }
};

class ProjectorWidget : public QWidget
{
  Q_OBJECT

public:
  ProjectorWidget(QWidget * parent = 0, Qt::WindowFlags flags = 0);
  ~ProjectorWidget();

  cv::Mat CreateLineImage();
  cv::Mat CreateLinePattern();
  std::vector<cv::Point2i> GetCoordLine(cv::Mat image);

  QPixmap GetPixmap() const { return this->Pixmap; };
  int GetWidth() const { return this->Width; };
  int GetHeight() const { return this->Height; };
  int GetLineThickness() const { return this->LineThickness; };
  int GetRow() const { return this->Row; };
  int GetStep() const { return this->Step; };
  std::unordered_map<cv::Vec3b, int, Vec3bHash> GetPattern() const { return this->Pattern; };
  
  void SetPixmap(QPixmap image) { this->Pixmap = image; };
  void SetWidth(int x) { this->Width = x; };
  void SetHeight(int y) { this->Height = y; };
  void SetLineThickness(int thickness) { this->LineThickness = thickness; };
  void SetRow(int r) { this->Row = r; };
  void SetStep(int step) { this->Step = step; };
  void SetPattern(std::unordered_map<cv::Vec3b, int, Vec3bHash> pattern) { this->Pattern = pattern; };

  void start();

signals:
  void new_image(QPixmap pixmap);

protected:
  virtual void paintEvent(QPaintEvent *);

private:
  QPixmap Pixmap;
  int Height;
  int Width;
  int LineThickness;
  int Row;
  int Step;
  std::unordered_map<cv::Vec3b, int, Vec3bHash> Pattern;
};


#endif  /* __PROJECTOR_HPP__ */