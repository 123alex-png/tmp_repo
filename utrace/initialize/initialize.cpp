// this file is left to be done in the future. It is supposed to parse the
// config file and initialize the program. At present, it is not used. Attention
// that it should be carefully checked and must implement the missing part when
// it is used.

#include <common.hpp>
#include <iostream>
#include <toml.hpp>

void safety_chk(toml::table tbl) {
	// TODO: check if the table structure is correct
}

void parse_file(const char* filename) {
	using std::string;
	toml::table tbl;
	try {
		tbl = toml::parse_file(filename);
	} catch (const toml::parse_error& err) {
		std::cerr << "Parsing failed:\n" << err << "\n";
		exit(1);
	}
	safety_chk(tbl);

	const auto& table = tbl.at("Process").as_table();
	string name = table->at("name").value_or("default");
	std::vector<string> arguments;
	if (table->contains("argumentHas"))
		if (auto arr = table->at("argumentHas").as_array())
			for (const auto& arg : *arr) {
				arguments.push_back(arg.as_string()->get());
			}
	string Processfilter = table->at("filter").value_or("default");

	std::vector<stream> streams;
	/*if (auto arr = tbl.at("Stream").as_array())
		for (const auto& stream : *arr) {
			struct stream s;
			const auto& tbl = stream.as_table();
			s.name = tbl->at("name").value_or("default");
			s.breakpoint = tbl->at("breakpoint").value_or("default");
			s.handler.handler = tbl->at("handler").value_or("default");
			streams.push_back(s);
		}
	*/
	return;
	// return config(name, arguments, filter, streams);
}