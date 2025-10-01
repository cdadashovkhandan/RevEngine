# RevEngine
---

A tool for automatically recognizing primitives in a point cloud, based on the work of [Raffo et al. (2022)](https://doi.org/10.1016/j.cagd.2022.102123) and [Romanengo et al (2023)](https://doi.org/10.1016/j.cad.2023.103479).

RevEngine employes the Hough Transform in order to detect shapes in a parallel fashion. The project is currently hardcoded to run on 8 threads.

For additional information, see this project's report.

## Usage

<img width="1591" height="795" alt="revengineui" src="https://github.com/user-attachments/assets/cb30e9e2-79d9-401e-a327-37a09e2669a5" />
 
- Click the `Import` button and choose a point cloud from file.
- Click `Preprocess` to perform all the pre-processing steps.
- If needed, tweak the parameters and re-run individual steps (top right) to fine-tune the pre-processing.
    - clicking `Preprocess` again is NOT recommended. It's best to import the point cloud again instead.
- When ready, click `Recognize Shapes`.
- wait for the execution to finish
- Click `Finalize` to undo the pre-processing and bring the cloud back to its original scale.
- Click `Export` and choose a directory to save all the recognized shapes to.

### Buttons & Parameters


- **Display**
All the checkboxes are self explanatory, with one exception:
    - Use downsampled cloud: If activated the downsampled cloud is displayed and also used in every calculation. If deactivated, the full cloud is used in all calculations.

- **Main Controls**
    - Import: Read a point cloud from file
    - Preprocess: Apply the preprocessing step to the point cloud.
    - Recognize Shapes: Run the main shape recognition algorithm
    - Finalize: scale and rotate the model back to its original state, along with the shapes that were detected.
    - Export: Save the detected shapes to a file.

- **Clustering**
    - Minimum Cluster size: The minimum number of points a cluster should have to be recognized.
    - Distance Threshold: The maximum radius within which neighboring points can be searched.
    - Live Preview: Re-run the clustering algorithm after any change in parameters, for ease of visualization. Basically clicks the "Live Preview" button for you every time a parameter is changed.
    - Force RANSAC: Force the RANSAC algorithm to be run in every step.

- **Individual Steps**: Re-run individual pre-processing steps after tweaking parameters.
- **Scaling**
TODO
- **Normals**
    - Estimation Method: Choose between PCA-based and Nearest-Neighbor-based normal estimation methods.
    - Search radius: Determine the radius to search within for PCA.
    - \#Neighbors: Determine the number of neighbors to consider per point for Nearest Nighbors.

## Testing

RevEngine was primarily tested with the [Fit4CAD dataset](https://github.com/chiararomanengo/Fit4CAD) and the test data in [Raffo et al. (2022)'s implementation repository](https://github.com/chiararomanengo/fitting_geometric_primitives).

## Dependencies

This application fundamentally relies on the [Point Cloud Library (PCL)](https://pointclouds.org/) and the libraries PCL depends on, namely [Boost, Eigen, and FLANN](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html).

In addition, a [modified implementation of DBSCAN by Eleobert](https://github.com/Eleobert/dbscan) is used under the MIT license.

## Future work

Although the implementations this work is based on support various shapes, the application currently only supports planes. Each of these shapes needs to be implemented as a separate class, inheriting from `PrimitiveShape`.

Due to bugs, one part of the pre-process step is disabled, namely the rotation. This is not a big deal for individual primitives or any components in the Fit4CAD dataset as they are already z-aligned, but this issue should be addressed first before any further expansions.

