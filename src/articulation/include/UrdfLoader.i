%{
    #include "UrdfLoader.h"
%}

%include "std_string.i"

class UrdfLoader :public IDisposable
{
// API BEGIN
public:
    UrdfLoader();
    void Dispose() override;
    void LoadDescriptionFromFile(std::string path);
// API END
};
