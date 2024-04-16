# Pinax-camera-model
This is a forked version of the original work by Tomasz Łuczyński, Max Pfingsthorn and Andreas Birk. 

The following changes are made to the MatLab from the original (the ROS code is unedited):
* All OpenCV functions are removed from the MatLab code and replaced with MatLab native equivalent functions - this allows for easier use.
* The MatLab README document has been updated.
* An example Python script showing the usage of the generated Pinax maps with the OpenCV function 'remap' for undistorting images is added.
* A MatLab script implementing the empiracle equation for finding water refraction index from [1] is added.

When using this code in scientific work please cite:
```
Tomasz Łuczyński, Max Pfingsthorn, Andreas Birk
The Pinax-model for accurate and efficient refraction correction of underwater cameras in flat-pane housings
Ocean Engineering, Volume 133, 2017, Pages 9-22, ISSN 0029-8018, http://dx.doi.org/10.1016/j.oceaneng.2017.01.029.
(http://www.sciencedirect.com/science/article/pii/S0029801817300434)
```

Abstract: 
```
The calibration and refraction correction process for underwater cameras with flat-pane interfaces is presented that is
very easy and convenient to use in real world applications while yielding very accurate results. The correction is derived
from an analysis of the axial camera model for underwater cameras, which is among others computationally hard to tackle.
It is shown how realistic constraints on the distance of the camera to the window can be exploited, which leads to an approach
dubbed Pinax Model as it combines aspects of a virtual pinhole model with the projection function from the axial camera model.
It allows the pre-computation of a lookup-table for very fast refraction correction of the flat-pane with high accuracy.
The model takes the refraction indices of water into account, especially with respect to salinity, and it is therefore
sufficient to calibrate the underwater camera only once in air. It is demonstrated by real world experiments with several
underwater cameras in different salt and sweet water conditions that the proposed process outperforms standard methods.
Among others, it is shown how the presented method leads to accurate results with single in-air calibration and even with
just estimated salinity values.
```

Reference:

[1] X. Quan and E. S. Fry, “Empirical equation for the index of refraction of seawater,” Applied Optics, vol. 34, no. 18, p. 3477, 1995. doi:10.1364/ao.34.003477
