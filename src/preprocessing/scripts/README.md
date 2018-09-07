# preprocessing
This node is meant to detect the lanes in the car's environment.

# How to use
In the terminal run the following commands:

```bash
rosrun preprocessing image_preprocessing.py
```

The node will **subscribe** to the topic called `/app/camera/rgb/image_raw` and the **publish** on the topic called `/image_preprocessed` the lanes detected.
