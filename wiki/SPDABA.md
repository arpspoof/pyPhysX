# Linear Time SPD Controller

#### [Experimental]

### How to Enable 
* Apply resources/spd-aba.patch to PhysX git repo (not this one).
* Re-build PhysX (make && make install).
* Enable `BUILD_SPD_ABA` option when configuring this project.
* Do a **clean** build for this project. See [Build and set up](wiki/Setup.md) for details.
* Use AddSPDForcesABA instead of AddSPDForces to use linear time SPD controller

**Note**:

Old SPD controller via AddSPDForces will still work.
