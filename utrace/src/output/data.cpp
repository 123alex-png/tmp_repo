#include <common.hh>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <output.hh>

jsonData::jsonData(const std::string breakpoint,
                   const std::vector<std::string>& vals)
    : breakpoint(breakpoint), vals(vals) {}

void jsonData::output(const std::string& outputFile) const {
    nlohmann::json output;
    output["time"] = time(0);
    output["breakpoint"] = breakpoint;
    output["values"] = vals;
    if (!outputFile.empty()) {
        std::ofstream file(outputFile, std::ios::app);
        file << output.dump() << std::endl;
    } else {
        std::cout << output.dump() << std::endl;
    }
}