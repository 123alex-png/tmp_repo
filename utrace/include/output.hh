#ifndef OUTPUT_HH
#define OUTPUT_HH

#include <common.hh>
#include <string>
#include <vector>
void cmdline(const int&, const std::string&);

class jsonData : public data {
private:
    std::string breakpoint;
    std::vector<std::string> vals;

public:
    jsonData(const std::string breakpoint,
             const std::vector<std::string>& vals);
    void output(const std::string& outputFile) const override;
};

#endif