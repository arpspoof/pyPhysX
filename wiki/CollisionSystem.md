# Collision System

### Overview
This project currently use a simple collision group / mask based collision reporting system. In its nature, each rigid object (or rigid actor) can be assigned a *collision group* (32-bit integer) and a *collisionMask* (also 32-bit integer). These two set of values together will decide what kind of collisions will be reported and what kind of collisions will be ignored. 

### Collision Group / Mask
##### Intuition
Intuitively, collision group can be think of the *Id* of an object while collision mask specifies a list of *Id* of objects each of which is allowed to be collided with this object. Here *allow colliding* simply means that such collisions will be reported. We cannot prevent any collision from happening in current implementation, instead, we just choose which pairs of objects in a collision that we care about.
##### Details
If you're familiar with bit operations then it's easy to come up with a simple scheme to implement this idea, in an efficient manner. The trick is that we can use binary bits to encode this concept of *Id*. More specifically, since we are using 32-bit integers, we say that the *Id* of this object is *i* iff the *ith* bit of the collision group is set to **1**. All other bits in the collision group must be set to **0** to avoid confusions. Then we can define collision mask naturally. If this object is *allowed* to collide with object with *Id* *j*, then the *jth* bit of the collision mask must be set to **1**. To be more specific, collisions involving object with *Id* *i* and *j* will be reported iff the *jth* bit of object *i* 's collision mask is set to **1**, **and**, *ith* bit of object *j* 's collision mask is set to **1**. I.e, they are mutually *allowed* to collide with each other. If we express the idea in code, we'll get the following:
```cpp
if ((collisionMask1 & collisionGroup2) && (collisionMask2 & collisionGroup1)) {
    reportCollision();
}
```
##### Set up collision groups and masks in pyPhysX
To enable collision filtering for a rigid actor, we can call the following:
```python
# set collision group to 1, and report collisions with object 0 and 2
actor.SetupCollisionFiltering(1 << 1, (1 << 0) | (1 << 2))
```
This is an interface call to RigidActor::SetupCollisionFiltering
##### Fetch collision pairs
After one step of simulation, we can fetch all collision pairs through the following: (python)
```python
contactList = scene.GetAllContactPairs()
for contact in contactList:
    print(contact[0], contact[1])
```
This function will return a list of tuples of integers. Each tuple represents a contact pair (two integers). The two integers represent the collision groups of the two objects involving in a contact. Please note that if contact information is not retrieved after one scene stepping, it will be lost after the next scene step.

This is an interface call to Scene::GetAllContactPairs

### Limitations
* Since we are using a 32-bit integer for collision group, there are at most 32 possible collision groups. Namely from `1 << 0` to `1 << 31`. 
* We only report collisions when it is first detected. Even if the colliding two objects remain touched with each other, no future reports will be issued.
