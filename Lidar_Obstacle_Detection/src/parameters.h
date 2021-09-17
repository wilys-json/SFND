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


namespace Parameters {

  struct empty {};

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

  template <class OutputT, typename ...InputT>
  OutputT get(const string& targetKey) {
    vector<InputT...> values =  getValues<InputT...>(targetKey);
    if (values.size() == 1) return static_cast<OutputT>(values[0]);
    template <std::size_t ...I>
    return [values](OutputT& outputClass, indices<I...>) { return outputClass(values[I]...); };
  }
  //
  // template <class OutputT, typename InputT, std::size_t ...I>
  // OutputT make(OutputT&& makeClass, vector<InputT>&args, indices<I...>) {
  //   return T(args[I]...);
  // }

};

#endif
