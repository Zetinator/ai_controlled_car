# write_images
This node is meant to detect the lanes in the car's environment.

# How to use
In the terminal run the following commands:

```bash
rosrun write_images write_images.py
```

The node will **subscribe** to the topic called `/app/camera/rgb/image_raw` and then write the images to the **hdd**
