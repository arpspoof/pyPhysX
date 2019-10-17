# pyPhysX

##
Important Notice: 
#####
Please use PhysX 4.1 up to the commit in Mar 20, 2019. The change in August introduces a bug which will cause large error in SPD acceleration prediction for dof[0,1,2,19,21]. They probably have introduced some inconsistency between FD and ID calculation.

#### Generate Full Documentation
```
doxygen Doxyfile
```

#### Wiki Index

##### Preperation
* [Build and set up](wiki/Setup.md)
* \[Experimental] [Build with linear time SPD](wiki/SPDABA.md)

##### Usage
* [Collision System](wiki/CollisionSystem.md)
* [Urdf Loader](wiki/UrdfLoader.md)
