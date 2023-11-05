#include <iostream>
#include <toml.hpp>
#include <common.hpp>

vector<stream> streams;
string name,dbginfo;
vector<string> arguments;
string Processfilter;

void safety_chk(toml::table tbl){
    //TODO: check if the table structure is correct
}

void parse_file(const char * filename){
    toml::table tbl;
    try{
        tbl=toml::parse_file(filename);
    }catch(const toml::parse_error& err){
        std::cerr << "Parsing failed:\n" << err << "\n";
        exit(1);
    }
    safety_chk(tbl);
    
    const auto &table=tbl.at("Process").as_table();
    name=table->at("name").value_or("default");
    if(table->contains("argumentHas"))
    if(auto arr=table->at("argumentHas").as_array())
        for(const auto& arg:*arr){
            arguments.push_back(arg.as_string()->get());
        }
    if(table->contains("dbginfo"))
    dbginfo=table->at("dbginfo").value_or("default");

    Processfilter=table->at("filter").value_or("default");

    if(auto arr=tbl.at("Stream").as_array())
        for(const auto& stream:*arr){
            struct stream s;
            const auto &tbl=stream.as_table();
            s.name=tbl->at("name").value_or("default");
            s.breakpoint=tbl->at("breakpoint").value_or("default");
            s.handler.handler=tbl->at("handler").value_or("default");
            streams.push_back(s);
        }

}