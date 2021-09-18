#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <utility>

using std::string;
using std::ifstream;
using std::vector;
using std::istringstream;
using std::make_integer_sequence;

const string PCD_FILE{"PCD_FILE"};
const string FILTER_RES{"FILTER_RES"};
const string MIN_POINT{"MIN_POINT"};
const string MAX_POINT{"MAX_POINT"};

namespace Parameters {


  const string paramDoc{"../src/params"};

  // Methods
  template <typename T>
  vector<T> getValues(const string& targetKey) {
    string line, key;
    T value;
    vector<T> collector{};
    ifstream filestream(paramDoc);
    if (filestream.is_open()) {
      while (std::getline(filestream, line)) {
        istringstream linestream(line);
        while (linestream >> key) {
          if (key == targetKey) {
            while (linestream >> value) collector.push_back(value);
            return collector;
          }
        }
      }
    } else { std::cout << "cannot find parameter files." << std::endl; }
    return collector;
  }


  template <class OutputT, class InputT, std::size_t ...I>
  OutputT make(vector<InputT>&args, std::index_sequence<I...>) {
    return OutputT(args[I]...);
  }


  template <class OutputT, typename ...InputT>
  OutputT get(const string& targetKey) {
    vector<InputT...> values =  getValues<InputT...>(targetKey);
    if (values.size() == 1) return static_cast<OutputT>(values[0]);
    return make<OutputT>(values, std::make_index_sequence<3>{});
  }

};

#endif
