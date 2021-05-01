# SPAM
## Snow Pointcloud Alignmnent and Measurement

<!-- ABOUT THE PROJECT -->
## About The Project
The goal of this project was to capture a 3D map of an environment without snow and then a second capture of the same area with snow cover. The script will automate this process and output a cloud compare BIN filetype of a pointcloud scalar field. The results highlight the differences between each capture with a difference measurement in the scalar field data.

![Results](https://github.com/zaknye/snow-pointcloud-alignment-measurement/blob/main/images/result.png?raw=true)

This repo contains a jupyter notebook that was used for initial experimentation and a guide to explain and highlight each step made when processing pointclouds. All work was done with a Livox Mid-40 sensor using Livox-Mapping within Robot Operating System.

![Livox Mid40](https://github.com/zaknye/snow-pointcloud-alignment-measurement/blob/main/images/livox_mid40.jpg?raw=true)

<!-- Process and Results -->
## Process and Results

We first start by using the corners.pcd file output by the Livox-Mapping software. This is the most sparse cloud and only retains mostly corner information. This data showed to be the most accurate and easy to work with when creating the alignment.

![Unaligned](https://github.com/zaknye/snow-pointcloud-alignment-measurement/blob/main/images/unaligned.png?raw=true)

We then downsample the cloud, calculate normal vectors and align them with direction, remove all points that have a Z component greater than 0.3 (omits the ground and surfaces snow tends to collect on), and initially uses the RANSAC algorithm for global alignment.

![RANSAC](https://github.com/zaknye/snow-pointcloud-alignment-measurement/blob/main/images/RANSAC-result.png?raw=true)

After initial alignment we further refine this using Open3D's ICP registration. This uses far more points to further refine how close the alignment is.

![ICP Registration](https://github.com/zaknye/snow-pointcloud-alignment-measurement/blob/main/images/ICP-Refined.png?raw=true)

Once alignment is complete using the corners file, the all_points files are loaded and transformed with the transform found in the previous steps.

![ICP Refined](https://github.com/zaknye/snow-pointcloud-alignment-measurement/blob/main/images/all-points-transformed.png?raw=true)
![Fence Closeup](https://github.com/zaknye/snow-pointcloud-alignment-measurement/blob/main/images/fence-closeup.png?raw=true)
![Lamp Closeup](https://github.com/zaknye/snow-pointcloud-alignment-measurement/blob/main/images/lamp-closeup.png?raw=true)

Then the all_points files are downsampled and outliers removed using the radius outlier class in Open3D. This gets rid of extraneous points and simplifies the cloud to make it a bit easier to work with. This also increases the accuracy of the distance calculation as the consensus on the plane is more accurate without outliers.

Once initial processing is complete, the M3C2 distance calculation is used with CloudCompare CLI. More information on the calculation method ![here.](https://arxiv.org/pdf/1302.1183.pdf) Once the calculation is complete we are returned a new file in CloudCompare's "BIN" filetype that has been filtered for only scalar values larger than 0.06m --> 100m. This file is a point cloud using the points from the post snow pointcloud with scalar data representing the distance from the initial scan to the post snow scan, essentially showing only snow depth.

![Result](https://github.com/zaknye/snow-pointcloud-alignment-measurement/blob/main/images/result.png?raw=true)


<!-- GETTING STARTED -->
## Getting Started

All work was done on a linux system (POP OS 20.10) and assumes UNIX style folder paths and structure. To interact with the project use the Jupyter notebook to visualize each process.

To clone the data used for this project navigate to the clone repository and use:
  ```sh
  # install google drive downloader
  pip install gdown
  gdown https://drive.google.com/uc?id=1_QOOiXFULTxp_77xb7h_-JQsfKYmVIbv
  gdown https://drive.google.com/uc?id=1hMJtftMclTRqjVkyRmWtpvr-q93vrf5g
  ```

### Prerequisites
The following are required to be installed on the system to use both the notebook and script.
* Python 3.8.6
* Open3d
  ```sh
  pip install open3d
  ```
* Numpy
  ```sh
  pip install numpy
  ```
* CloudCompare
  ```sh
  sudo snap install cloudcompare
  ```

The following are required to be installed on the system to use the notebook.

* Jupyter Notebooks
  ```sh
  pip install notebook
  ```

<!-- USAGE EXAMPLES -->
## Usage

The Livox sensor will output 3 different pointclouds upon being stopped in robot operating system.
* corners.pcd - Only corner information of the environment is saved, clearly showing edges within the environment. This is used for initial alignment of the pointclouds.
* surf.pcd - This is the result and slightly downsampled 3D map using the SURF algorithm.
* all_points.pcd - All points recorded by the LiDAR. By far the most dense and largest in file size. This cloud is used for final distance calculation and visualization.
To use the script:
  ```sh
  python3 alignment_script.py <source folder> <target folder>
  ```

<!-- CONTACT -->
## Contact

Zak Nye - [zaknye.com](https://zaknye.com) - zaknye@gmail.com

Project Link: [https://github.com/zaknye/snow-pointcloud-alignment-measurement](https://github.com/zaknye/snow-pointcloud-alignment-measurement)
