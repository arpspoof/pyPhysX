# pyPhysX

##
Important Notice: 

#####
Please use PhysX 4.1 up to the commit in Mar 20, 2019 to ensure correctness for matrix fatorization based SPD controller. The change in August introduces a bug which will cause large error in SPD acceleration prediction for dof[0,1,2,19,21]. They probably have introduced some inconsistency between FD and ID calculation. However, though the visual artifact is noticable, it does not harm simulation too much. Since this change also brings great performance improvement, we need to seek a balancce between correctness and speed if we use matrix fatorization based SPD controller. However, a nicer solution will be to use newest version plus new ABA based SPD controller which will not involve ID calculation at all. This will provide both speedy and accurate control.

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
