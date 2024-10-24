# Environments
This repo runs at omnigibson simulator and visualize the scene by Omniverse Streaming Client. You probably need to [install the omniverse](https://behavior.stanford.edu/omnigibson/getting_started/installation.html#explore-omnigibson) and the [Omniverse Launcher](https://www.nvidia.com/en-in/omniverse/).

By the way I have to say sth. bad on the omniverse download website and the documentation. we intend to install the Omniverse launcher but instead [the download website](https://www.nvidia.com/en-in/omniverse/) keeps saying "start developing", and lead you to the Omniverse SDK. Such a mess!

# Usage
After all installed, you can start to control the camera by keyboard and mouse in Omniverse Streaming Client and construct the top-down map! Something like [the example usage](./example_usage.mp4):

<video src="./example_usage.mp4" width="500" height="300"></video>

# Explanation
1. calculate the camera intrinsic parameters, initialize top-down map and world point cloud.
2. camera collects the depth image of current observation.
3. convert depth image to point cloud (with in image view)
4. update and connect the image point cloud to the world point cloud
5. squeeze the world cloud points for visualization