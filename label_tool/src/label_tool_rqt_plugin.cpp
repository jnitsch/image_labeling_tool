#include <label_tool/label_tool_rqt_plugin.h>

namespace label_tool {

LabelToolPlugin::LabelToolPlugin():
  rqt_gui_cpp::Plugin(),
  widget_(0),
  nh_()
{
  // give QObjects reasonable names
  setObjectName("LabelToolPlugin");
}

void LabelToolPlugin::initPlugin(qt_gui_cpp::PluginContext &context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  // getting labels
  std::vector<std::string> labels;
  QList<QString> labls_qt;
  // adding empty string at beginning
  QString empty_str = QString("");
  labls_qt.push_back(empty_str);

  nh_.getParam("labels", labels);
  for(auto &label : labels)
  {
    ROS_INFO_STREAM("label with name " << label << " available");
    QString qstr = QString::fromStdString(label);
    labls_qt.push_back(qstr);
  }

  // setting labels to dropdown menu
  ui_.comboBox->addItems(labls_qt);

  // reading in path to save img
  if(!nh_.getParam("path_for_img_destination",path_to_dir_))
  {
    path_to_dir_ = "~/classification/";
    ROS_ERROR_STREAM("path_for_img_destination not specified assuming " << path_to_dir_);
  }

  // setting up srv
  visualize_img_srv_ = nh_.advertiseService("/label_images",&LabelToolPlugin::visualizeImgCb, this);
  visualize_img_with_selection_srv_ = nh_.advertiseService("/label_images_with_selection",
                                                           &LabelToolPlugin::visualizeImgWithSelectionCb, this);

  // setting up my scene to handle click events
  scene_ = new label_tool::Scene();
  ui_.graphicsView->setScene(scene_);
  ui_.graphicsView->show();


  // connect GUI elements
  connect(scene_, SIGNAL(mousePressed(QGraphicsSceneMouseEvent*)),
          this, SLOT(mousePressEventOnGraphicsView(QGraphicsSceneMouseEvent*)), Qt::QueuedConnection);
  connect(scene_, SIGNAL(mouseRelease(QGraphicsSceneMouseEvent*)),
          this, SLOT(mouseReleaseEventOnGraphicsView(QGraphicsSceneMouseEvent*)), Qt::QueuedConnection);
  connect(scene_, SIGNAL(mouseMove(QGraphicsSceneMouseEvent*)),
          this, SLOT(mouseMoveEventOnGraphicsView(QGraphicsSceneMouseEvent*)), Qt::QueuedConnection);
  connect(ui_.nextButton, SIGNAL(pressed()), this, SLOT(pushNextButton()),Qt::QueuedConnection);
  connect(ui_.prevButton, SIGNAL(pressed()), this, SLOT(pushPrevButton()),Qt::QueuedConnection);
  connect(ui_.saveCurrentImg, SIGNAL(pressed()), this, SLOT(pushSaveCurrentImgButton()),Qt::QueuedConnection);
  connect(ui_.saveSelection, SIGNAL(pressed()), this, SLOT(pushSaveCurrentSelectionButton()),Qt::QueuedConnection);
  connect(ui_.deleteSelectionButton, SIGNAL(pressed()), this, SLOT(pushDeleteSelectionButton()),Qt::QueuedConnection);
  connect(this, SIGNAL(dataAvailable()), this, SLOT(visualizeSlot()),Qt::QueuedConnection);


  reset();
}

void LabelToolPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void LabelToolPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void LabelToolPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void LabelToolPlugin::reset()
{
  img_with_polygons_.clear();
  current_index_ = 0;
  ui_.nextButton->setEnabled(false);
  ui_.prevButton->setEnabled(false);
  ui_.deleteSelectionButton->setEnabled(false);
  ui_.saveCurrentImg->setEnabled(false);
  ui_.saveSelection->setEnabled(false);
  ui_.split_into_patches->setEnabled(false);
  ui_.down_up_sample->setEnabled(false);
  ui_.comboBox->setCurrentIndex(-1);
  ui_.widthTextBrowser->setEnabled(false);
  ui_.heightTextBrowser->setEnabled(false);
}

void LabelToolPlugin::checkIndex()
{
  if(img_with_polygons_.empty())
  {
    ui_.prevButton->setEnabled(false);
    ui_.nextButton->setEnabled(false);
  }
  else if(current_index_ <= 0)
  {
    current_index_ = 0;
    ui_.prevButton->setEnabled(false);
    ui_.nextButton->setEnabled(true);
  }
  else if(current_index_ >= (img_with_polygons_.size() - 1) )
  {
    current_index_ = img_with_polygons_.size() - 1;
    ui_.nextButton->setEnabled(false);
    ui_.prevButton->setEnabled(true);
  }
  else
  {
    ui_.prevButton->setEnabled(true);
    ui_.nextButton->setEnabled(true);
  }

  if(!img_with_polygons_.empty())
  {
    ui_.deleteSelectionButton->setEnabled(true);
    ui_.saveCurrentImg->setEnabled(true);
    ui_.saveSelection->setEnabled(true);
    ui_.down_up_sample->setEnabled(true);
    ui_.split_into_patches->setEnabled(true);
    ui_.widthTextBrowser->setEnabled(true);
    ui_.heightTextBrowser->setEnabled(true);
  }
}

void LabelToolPlugin::visualizeImg()
{
  checkIndex();

  std::stringstream ss;
  ss << (current_index_ + 1) << " of " << img_with_polygons_.size();
  ui_.imgShowLabel->setText(QString::fromStdString(ss.str()));

  clearScene();

  drawImgWithPolygon();

  ui_.graphicsView->show();
  ui_.graphicsView->viewport()->repaint();
}

bool LabelToolPlugin::visualizeImgCb(label_tool_msgs::ImagesToLabel::Request &req,
                                     label_tool_msgs::ImagesToLabel::Response &res)
{
  reset();

  // copy data on class structure
  for(auto &img : req.images)
  {
    // convert to opencv mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, img.encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }


    std::vector<cv::Point> polygon_edges;

    ImgWithPolygon img_with_poly_class;
    img_with_poly_class.img_ = cv_ptr->image;
    img_with_poly_class.polygon_edges_ = polygon_edges;
    img_with_polygons_.push_back(img_with_poly_class);
  }

  emit dataAvailable();
  return true;
}

bool LabelToolPlugin::visualizeImgWithSelectionCb(label_tool_msgs::ImagesWithSelection::Request &req,
                                                  label_tool_msgs::ImagesWithSelection::Response &res)
{
  reset();

  // copy data on class structure
  for(auto &img_with_poly : req.images)
  {
    // convert to opencv mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img_with_poly.image, img_with_poly.image.encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }


    std::vector<cv::Point> polygon_edges;
    for(auto &point : img_with_poly.polygon_edges)
    {
      cv::Point new_vertex(point.x, point.y);
      polygon_edges.push_back(new_vertex);
    }

    ImgWithPolygon img_with_poly_class;
    img_with_poly_class.img_ = cv_ptr->image;
    img_with_poly_class.polygon_edges_ = polygon_edges;
    img_with_polygons_.push_back(img_with_poly_class);
  }

  emit dataAvailable();
  return true;
}

void LabelToolPlugin::insertSelectedPoint(QPointF point)
{
  if(selected_points_.empty() || !isSimilar(selected_points_[0], point) )
  {
    selected_points_.push_back(point);
    drawQtPolygon(false);
    return;
  }
  else
  {
    drawQtPolygon(true);
    if(!img_with_polygons_.empty())
      setNewPolygonEdges();
    selected_points_.clear();
    return;
  }
}

bool LabelToolPlugin::isSimilar(QPointF point_a, QPointF point_b)
{
  double distance = std::sqrt(std::pow(point_a.x() - point_b.x(),2) +
                              std::pow(point_a.y() - point_b.y(),2));
  if(distance < 5)
    return true;
  else
    return false;
}

void LabelToolPlugin::drawQtPolygon(bool connect)
{
  //delete everything from scene
  clearScene();

  // add img to scene
  if(!img_with_polygons_.empty())
    drawImage();



  if(selected_points_.size() == 1)
  {
    // draw point
    double rad = 1;
    QPen pen(Qt::blue);
    QBrush brush(Qt::SolidPattern);
    scene_->addEllipse(selected_points_[0].x() - rad,
        selected_points_[0].y() - rad ,
        rad * 5.0, rad * 5.0, pen, brush);
  }
  else
  {
    if(!connect)
    {
      QPen pen_point(Qt::blue);
      QBrush brush_point(Qt::SolidPattern);
      QPen pen_line(Qt::red);
      pen_line.setWidth(4);

      double rad = 1;
      for(int idx = 0; idx < (selected_points_.size() - 1); idx++)
      {
        scene_->addEllipse(selected_points_[idx].x() - rad,
                           selected_points_[idx].y() - rad ,
                           rad * 5.0, rad * 5.0, pen_point, brush_point);
        scene_->addLine(selected_points_[idx].x(), selected_points_[idx].y(),
                        selected_points_[idx + 1].x(), selected_points_[idx + 1].y(),
            pen_line);
      }
      scene_->addEllipse(selected_points_[selected_points_.size() - 1].x() - rad,
          selected_points_[selected_points_.size() - 1].y() - rad ,
          rad * 5.0, rad * 5.0, pen_point, brush_point);

    }
    else
    {
      QPen pen_line(Qt::green);
      pen_line.setWidth(4);
      for(int idx = 0; idx < (selected_points_.size() - 1); idx++)
      {
        scene_->addLine(selected_points_[idx].x(), selected_points_[idx].y(),
                        selected_points_[idx + 1].x(), selected_points_[idx + 1].y(),
            pen_line);
      }
      scene_->addLine(selected_points_[0].x(), selected_points_[0].y(),
          selected_points_[selected_points_.size() - 1].x(), selected_points_[selected_points_.size() - 1].y(),
          pen_line);
    }
  }

  ui_.graphicsView->show();
  ui_.graphicsView->viewport()->repaint();
}


void LabelToolPlugin::clearScene()
{
  QList< QGraphicsItem* > items = scene_->items();
  for(auto & item : items)
  {
    scene_->removeItem(item);
    delete item;
  }
}

void LabelToolPlugin::drawImage()
{
  if(img_with_polygons_.empty())
    return;

  cv::Mat cv_image;
  img_with_polygons_[current_index_].img_.copyTo(cv_image);

  // create qt image for visualization
  QImage qt_image(cv_image.data, cv_image.cols, cv_image.rows, cv_image.step[0], QImage::Format_RGB888);

  QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(qt_image));
  scene_->addItem(item);
}

void LabelToolPlugin::drawImgWithPolygon()
{
  std::vector<cv::Point> edges = img_with_polygons_[current_index_].polygon_edges_;
  cv::Mat cv_image;
  img_with_polygons_[current_index_].img_.copyTo(cv_image);


  if(!edges.empty())
  {
    // connect polygon edges
    for(uint edge_idx = 0; edge_idx < (edges.size()-1); edge_idx++)
    {
      cv::Point start(edges[edge_idx].x, edges[edge_idx].y);
      cv::Point end(edges[edge_idx + 1].x, edges[edge_idx + 1].y);
      cv::line(cv_image, start, end, CV_RGB(255,0,0), 3);
    }
    // connect first with last edge
    cv::Point start(edges[0].x, edges[0].y);
    cv::Point end(edges[edges.size()-1].x, edges[edges.size()-1].y);
    cv::line(cv_image, start, end, CV_RGB(255,0,0), 3);
  }
  // create qt image for visualization
  QImage qt_image(cv_image.data, cv_image.cols, cv_image.rows, cv_image.step[0], QImage::Format_RGB888);

  QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(qt_image));
  scene_->addItem(item);
}

void LabelToolPlugin::drawRect(QPointF point_a, QPointF point_b)
{
  clearScene();
  drawImage();

  qreal width = point_b.x() - point_a.x();
  qreal height = point_b.y() - point_a.y();
  QPen pen_line(Qt::red);
  pen_line.setWidth(4);
  scene_->addRect(point_a.x(), point_a.y(), width, height, pen_line);
}

void LabelToolPlugin::setNewPolygonEdges()
{
  // clear old stuff
  img_with_polygons_[current_index_].polygon_edges_.clear();

  for(auto &qtpoint : selected_points_)
  {
    cv::Point point(qtpoint.x(), qtpoint.y());
    img_with_polygons_[current_index_].polygon_edges_.push_back(point);
  }
}

std::string LabelToolPlugin::getFilename()
{
  // getting label
  std::string label = ui_.comboBox->currentText().toUtf8().constData();
  if(label == "")
    label = "not_defined";

  // saving img
  std::string destination = path_to_dir_+label+"/";

  DIR *dir;
  struct dirent *entry;
  int files_found = 0;
  if ((dir = opendir(destination.c_str())) != NULL)
  {
    // print all the files and directories within directory
    while ((entry = readdir (dir)) != NULL)
    {
      if( strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 )
      {
        files_found++;
      }
    }
  }
  else
  {
    // directory doesn't exist -> create it
    mkdir(destination.c_str(), 0700);
  }
  //ROS_INFO_STREAM("Found " << files_found << " files in directory " << destination);
  std::stringstream ss;
  ss << "img_" << files_found << ".png";
  std::string filename = destination + ss.str();

  return filename;
}

void LabelToolPlugin::savePolygonImg(int idx)
{
  if(img_with_polygons_.empty())
    return;

  cv::Mat cv_image = img_with_polygons_[idx].img_;
  std::vector<cv::Point> polygon_edges = img_with_polygons_[idx].polygon_edges_;
  cv::Mat roi_output;
  if(polygon_edges.empty())
  {
    // save whole imag
    roi_output = cv_image;
  }
  else
  {
    // creating mask
    cv::Mat mask(cv_image.size(), CV_8UC1, cv::Scalar(0));
    std::vector<std::vector<cv::Point> > fill_mask;
    fill_mask.push_back(polygon_edges);
    cv::fillPoly( mask, fill_mask, cv::Scalar(255));

    // apply mask
    cv::Mat output;
    cv_image.copyTo(output,mask);

    // bounding rect around poly
    cv::Rect roi = cv::boundingRect(polygon_edges);
    cv::Rect img_rect = cv::Rect(cv::Point(0,0), cv_image.size());
    cv::Rect rects_intersecion = img_rect & roi;


    // cut output to size of bounding rect
    roi_output = output(rects_intersecion);
  }

  std::string filename = getFilename();
  save(filename, roi_output);
}

void LabelToolPlugin::saveImg(int idx)
{
  if(img_with_polygons_.empty())
    return;

  cv::Mat cv_image = img_with_polygons_[idx].img_;
  std::string filename = getFilename();
  save(filename, cv_image);
}

void LabelToolPlugin::save(std::string filename, cv::Mat img)
{
  if(!ui_.split_into_patches->isChecked() && !ui_.down_up_sample->isChecked())
  {
    cv::imwrite(filename, img);
  }
  else
  {
    // get size from text browser
    QTextDocument *doc_width = ui_.widthTextBrowser->document();
    QString string_width = doc_width->toHtml();
    int width = static_cast<int>(string_width.toDouble());
    QTextDocument *doc_height = ui_.heightTextBrowser->document();
    QString string_height = doc_height->toHtml();
    int height = static_cast<int>(string_height.toDouble());
    if(width <= 0)
      width = 100;
    if(height <= 0)
      height = 100;
    cv::Size size(width, height);

    if(ui_.split_into_patches->isChecked())
    {
      // split img into patches
      for(int width_idx = 0; width_idx < img.cols/width; width_idx++)
      {
          for(int height_idx = 0; height_idx < img.rows/height; height_idx++)
          {
            cv::Mat patch = img(cv::Rect(width_idx * width, height_idx * height, width, height));
            std::string name = getFilename();
            cv::imwrite(name, patch);
          }
      }
    }
    if(ui_.down_up_sample->isChecked())
    {
      // resize img
      cv::Mat img_to_save;
      cv::resize(img, img_to_save, size);
      cv::imwrite(filename, img_to_save);
    }
  }
}


void LabelToolPlugin::pushNextButton()
{
  current_index_++;
  emit dataAvailable();
}

void LabelToolPlugin::pushPrevButton()
{
  current_index_--;
  emit dataAvailable();
}


void LabelToolPlugin::pushSaveCurrentImgButton()
{
  saveImg(current_index_);
}

void LabelToolPlugin::pushSaveCurrentSelectionButton()
{
  savePolygonImg(current_index_);
}

void LabelToolPlugin::pushDeleteSelectionButton()
{
  img_with_polygons_[current_index_].polygon_edges_.clear();
  visualizeImg();

}

void LabelToolPlugin::visualizeSlot()
{
  visualizeImg();
}

void LabelToolPlugin::mousePressEventOnGraphicsView(QGraphicsSceneMouseEvent *e)
{
  // start drawing polygon when click on right button
  if(e->button() == Qt::RightButton)
  {
    insertSelectedPoint(e->scenePos());
  }
  if(e->button() == Qt::LeftButton)
  {
    drawImage();
    ui_.graphicsView->show();
    ui_.graphicsView->viewport()->repaint();
    start_point_ = e->scenePos();
    selecting_ = true;
  }
}

void LabelToolPlugin::mouseReleaseEventOnGraphicsView(QGraphicsSceneMouseEvent *e)
{
  if(e->button() == Qt::LeftButton)
  {
    selecting_ = false;
    selected_points_.clear();
    selected_points_.push_back(start_point_);
    QPointF edge(start_point_.x(), e->scenePos().y());
    selected_points_.push_back(edge);
    selected_points_.push_back(e->scenePos());
    QPointF edge_2(e->scenePos().x(), start_point_.y());
    selected_points_.push_back(edge_2);
    drawQtPolygon(true);
    if(!img_with_polygons_.empty())
      setNewPolygonEdges();
    selected_points_.clear();
  }
}

void LabelToolPlugin::mouseMoveEventOnGraphicsView(QGraphicsSceneMouseEvent *e)
{
  if(selecting_)
    drawRect(start_point_, e->scenePos());
}




}

PLUGINLIB_EXPORT_CLASS(label_tool::LabelToolPlugin, rqt_gui_cpp::Plugin);
