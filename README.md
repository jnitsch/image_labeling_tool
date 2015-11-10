# image_labeling_tool
This repository contains ros packages to ease image labeling. It is basically
a rqt plugin developed under Ubuntu 14.04 ros indigo.

## Services provided
```
/label_images [label_tool_msgs/ImagesToLabel]
```
The service label_images  takes a list of sensor_msgs/Image and visualizes this images in 
the rqt plugin.

```
/label_images_with_selection [label_tool_msgs/ImagesWithSelection]
```
The service label_images_with_selection takes sensor_msgs/Image with a list of 
already identified points of the image which are visualized as polygon.

## GUI
![GUI](https://github.com/jnitsch/image_labeling_tool/blob/master/gui.png)


### Selection
Rectangular selection: left mouse click and drag rectangle
Polygon selection: right mouse click sets polygon vertices, if polygon closed (clicked on start vertex again) it appears green

### Labels
Labels are defined in the config/label.yaml file. They can be selected in the drop down menu.

### Saving the images
The rosparam **path_for_img_destination** defines the path (default: ~/classification/) 

1. Select a label 
2. Press "either save whole img" (whole img is saved) or "save current selection" 
  *  you can check split into multiple patches: img (or selection) is split into multiple patches of defined size and stored
  *  you can check "down/upsample to patchsize" which resizes the img to the defined size
