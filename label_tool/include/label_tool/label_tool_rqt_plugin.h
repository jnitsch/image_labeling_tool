/**
 * \class LabelToolPlugin
 *
 * \brief RQT Plugin for easily labeling images
 *
 * Class implements RQT plugin to visualize stored images with their already
 * selected polygons. It provides also the functionality to select a new region
 * using mouse selection. (Left mouse click: rectangular selection; Right mouse
 * click: single polygon point selection).
 *
 * \author Julia Nitsch
 *
 * Contact: julia.nitsch@gmail.com
 *
 */
#pragma once

// used for widget
#include <rqt_gui_cpp/plugin.h>
#include <ui_label_tool.h>
#include <QWidget>
#include <ros/ros.h>
#include <label_tool/rqt/scene.h>

// visualizing imgs
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>

// plugin
#include <pluginlib/class_list_macros.h>

// srv to set the vector
#include <label_tool_msgs/ImagesToLabel.h>
#include <label_tool_msgs/ImagesWithSelection.h>

// counting how many files are in dir
#include <stdio.h>
#include <fstream>
#include <dirent.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <QtGui>
namespace label_tool {

/*!
 * Storage container for images and their selection
 */

struct ImgWithPolygon
{
  cv::Mat img_; ///< image stored as opencv matrix */
  std::vector<cv::Point> polygon_edges_; ///< polygon edges of selection */
};

/*!
 * RQT plugin
 */
class LabelToolPlugin : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  /// Constructor
  LabelToolPlugin();

  /** @defgroup rqt_plugin
   *  This functions are overloaded from rqt_gui_cpp::Plugin
   *  @{
   */
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
  /** @} */

private:
  Ui::LabelToolWidget ui_; ///< graphical userinterface defined in .ui file */
  QWidget* widget_; ///< qt widget */

  ros::NodeHandle nh_; /**< ros nodehandle (ros helper) */

  /// \brief reset and clean everything
  /// sets all buttons to disabled and clears data structure
  void reset();

  
  std::vector<ImgWithPolygon> img_with_polygons_; /**< vector of img */
  int current_index_ = 0; /**< index of current visualized img */


  /// \brief checks index
  /// Ensures that index doesn't get negative and not larger than the size of
  /// stored image vector. If index is 0 or the last index possible,
  /// it deactivates respectively the next or previous button
  void checkIndex();

  /** @defgroup image_visualization
   *  This functions handle the visualization of images
   *  @{
   */
  label_tool::Scene* scene_ = NULL; /**< own scene class which signals
                                          mouse events, so that this class can
                                          react to it */
  /// \brief clears the scene
  /// removes all qt items from the scene_
  void clearScene();

  /// \brief draws current img
  /// gets current visualized img converts it to qtimage and adds it as qt item
  /// to the scene_
  void drawImage();

  /// \brief draws current img and its selection
  /// gets current visualized img with its selection, converts it to qtimage
  /// and adds it as qt item to the scene_
  void drawImgWithPolygon();

  /// \brief draws selected rectangle
  /// visualize selection of mouse
  void drawRect(QPointF point_a, QPointF point_b);

  /// \brief visualize img and selection on qt graphics view
  /// draws img and (if set) selection to the graphic view of qt and refreshes
  /// the scene
  void visualizeImg();

  /// \brief draws selected polygon
  /// visualizes the current selection which are saved as polygon edges
  /// @param [in] connect   if connect is set true first and last point are
  ///                       connected and polygon is drawn as green polygon
  void drawQtPolygon(bool connect = false);
  /** @} */

  /** @defgroup image_saveing
   *  This functions and variables are for saving the images to the correct
   *  folder
   *  @{
   */
  std::string path_to_dir_; /**< path where img should be saved basically, set
                              by ros param */
  std::string getFilename();
  /// \brief saves img selection
  /// saves whole selection of img to path/label/img_*.png
  /// @param [in] idx  index of img (selection) which should be saved
  void savePolygonImg(int idx);

  /// \brief saves whole img
  /// @param [in] idx  index of img which should be saved
  void saveImg(int idx);

  /// \brief performs saving itself
  /// checks if user wants to scale or divorce into patches
  void save(std::string filename, cv::Mat img);


  /** @} */


  /** @defgroup mouse selection
   *  This functions and variables handle the mouse selection within the img.
   *  Polygons is selected with right mouse button, rectangular selection with
   *  left mouse button.
   *  @{
   */
  // selection of rqt - points with RIGHT mouse button
  std::vector<QPointF> selected_points_; /**< currently selected points */

  /// \brief insert selected point in datastructure selected_points_
  /// insert point and calls functions to visualize polygon
  void insertSelectedPoint(QPointF point);

  /// \brief checks is similarity of points
  /// checks distance of 2 points and if distance is small enough points are
  /// assumed similar
  /// @param [in]  point_a  first point
  /// @param [in]  point_b  second point
  /// @reutrn true if similar false otherwise
  bool isSimilar(QPointF point_a, QPointF point_b);

  /// \brief saves current selected polygon to img data structure
  /// saves selected_points_ as opencv points to img data structure
  void setNewPolygonEdges();


  // selection rect with LEFT mouse button (hold it while selecting)
  bool selecting_ = false; /**< shows if user is selecting a rectangular region */
  QPointF start_point_; /**< start point of rectangular selection */
  /** @}


  /** @defgroup ros_interface
   *  This functions handle all the callbacks from other ros nodes
   *  @{
   */
  ros::ServiceServer visualize_img_srv_; /**< ros srv to img */
  /// \brief service cb to  set img
  /// sets up data structure and immediatly returns true if data is set up
  /// correctly false otherwise
  bool visualizeImgCb(label_tool_msgs::ImagesToLabel::Request &req,
                      label_tool_msgs::ImagesToLabel::Response &res);

  ros::ServiceServer visualize_img_with_selection_srv_;
  bool visualizeImgWithSelectionCb(label_tool_msgs::ImagesWithSelection::Request &req,
                                   label_tool_msgs::ImagesWithSelection::Response &res);

  /** @} */

private slots:
  virtual void mousePressEventOnGraphicsView(QGraphicsSceneMouseEvent* e);
  virtual void mouseReleaseEventOnGraphicsView(QGraphicsSceneMouseEvent* e);
  virtual void mouseMoveEventOnGraphicsView(QGraphicsSceneMouseEvent* e);
  virtual void pushNextButton();
  virtual void pushPrevButton();
  virtual void pushSaveCurrentImgButton();
  virtual void pushSaveCurrentSelectionButton();
  virtual void pushDeleteSelectionButton();
  virtual void visualizeSlot();

signals:
  void dataAvailable(); /**< signals that new data is available, emited after
                          data was successfully set in ros cb so that qt threads
                          can handle data visualization, and user input */

};

}
