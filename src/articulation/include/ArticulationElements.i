%{
    #include "ArticulationElements.h"
%}

%include "std_vector.i"
namespace std {
   %template() vector<Link*>;
};

class Link;

class Joint {
public:
// API BEGIN
    Link* parentLink;
    Link* childLink;
    int nDof;
    int cacheIndex;
    Joint() {} // For API only
// API END
};

class Link :public RigidActor {
public:
// API BEGIN
    Link* parentLink;
    Joint* inboundJoint;
    std::vector<Link*> childLinks;
    Link() {} // For API only
// API END
};
